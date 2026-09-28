"""Tests for DualArm / MirrorMode / PyBulletArm.mirror_from.

None of these need hardware or the real SDK: the real-arm half is a scripted
stand-in that answers the same calls.  What is being tested is the *contract*
around the real arm — that every mirror read is a fresh one, that commands are
never interleaved with mirrored frames, that both arms are always told about a
command, and that a missing litearm-core says so instead of half-working.

``_compat.Arm`` is patched rather than the SDK: that module is the single place
that decides which implementation is in play, so patching it is the same knob
the package itself uses, and the tests then behave identically whether or not
litearm-core is installed.
"""
from __future__ import annotations

import time

import pytest

from litearm_pybullet import DualArm, MirrorMode, PyBulletArm
from litearm_pybullet import _compat
from litearm_pybullet._compat import InvalidCommandError, MotionTimeoutError, Msg, RobotState

MOTION_CALLS = ("movej", "movej_sync", "move_p", "move_l", "move_c", "move_path")


def _state(q):
    """A RobotState carrying ``q`` — what a status frame decodes to."""
    return RobotState(joints=[_compat.JointState(q=float(v)) for v in q])


def _await_q(sim, expected, timeout=2.5):
    """Wait for the simulation's joints to land on ``expected``.

    Returns whether they did, so a caller can assert on the outcome instead of
    on elapsed time.
    """
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if list(sim.get_state().value.q) == pytest.approx(list(expected), abs=1e-6):
            return True
        time.sleep(0.02)
    return False


class _FakeRealArm:
    """The ``litearm_core.Arm`` subset the mirror code uses, scripted.

    Records every call and, for ``get_state``, every ``refresh`` value — the
    mirror is required to ask for a *fresh* frame, and a silent regression to
    ``refresh=False`` would show up here and nowhere else.
    """

    def __init__(self, port=None, n=7, q=None):
        self.port = port
        self.n = n
        self._q = list(q if q is not None else [0.1] * n)
        self.calls = []           # (name, args, kwargs)
        self.refresh_seen = []    # every `refresh` passed to get_state
        self.timeouts_seen = []   # every `timeout` passed to get_state
        self.closes = 0
        self.enabled = False
        #: name -> exception to raise instead of returning.
        self.raises = {}
        #: What get_state returns; None means "build it from self._q".
        self.frame = None

    # ── Lifecycle ──────────────────────────────────────────────────────────────

    def connect(self, port=None):
        self._record("connect", port)
        return self

    def close(self):
        self.closes += 1

    def enable(self, attempts=12):
        self._record("enable", attempts)
        self.enabled = True

    def disable(self):
        self._record("disable")
        self.enabled = False

    def emergency_stop(self):
        self._record("emergency_stop")

    def reset(self):
        self._record("reset")

    # ── Reading ────────────────────────────────────────────────────────────────

    def get_state(self, refresh=False, timeout=0.5):
        self._record("get_state", refresh, timeout)
        self.refresh_seen.append(refresh)
        self.timeouts_seen.append(timeout)
        self._maybe_raise("get_state")
        if self.frame is not None:
            return self.frame
        return Msg(value=_state(self._q), hz=100.0, timestamp=time.monotonic())

    def get_tcp(self, timeout=0.6):
        self._record("get_tcp", timeout)
        self._maybe_raise("get_tcp")
        return Msg(value=(0.3, 0.0, 0.35, 3.14, 0.0, 0.0), hz=100.0,
                   timestamp=time.monotonic())

    # ── Commands ───────────────────────────────────────────────────────────────

    def _motion(self, name, *args, **kwargs):
        self._record(name, *args, **kwargs)
        self._maybe_raise(name)
        return f"real-{name}"

    def movej(self, q, speed=1.0):
        return self._motion("movej", q, speed=speed)

    def movej_sync(self, q, speed=1.0):
        return self._motion("movej_sync", q, speed=speed)

    def move_p(self, pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03):
        return self._motion("move_p", pose, speed=speed, pos_tol=pos_tol,
                            rpy_tol=rpy_tol)

    def move_l(self, pose, speed=1.0, wait=True):
        return self._motion("move_l", pose, speed=speed, wait=wait)

    def move_c(self, pose_start, pose_via, pose_goal, speed=1.0, wait=True):
        return self._motion("move_c", pose_start, pose_via, pose_goal,
                            speed=speed, wait=wait)

    def move_path(self, poses, speed=1.0, wait=True):
        return self._motion("move_path", poses, speed=speed, wait=wait)

    # ── Helpers ────────────────────────────────────────────────────────────────

    def _record(self, name, *args, **kwargs):
        self.calls.append((name, args, kwargs))

    def _maybe_raise(self, name):
        exc = self.raises.get(name)
        if exc is not None:
            raise exc

    def names(self):
        return [c[0] for c in self.calls]

    def kwargs_of(self, name):
        return [c[2] for c in self.calls if c[0] == name]

    def get_state_calls(self):
        return self.names().count("get_state")


@pytest.fixture
def fake_sdk(monkeypatch):
    """Point ``_compat`` at the fake arm, as if litearm-core were installed."""
    monkeypatch.setattr(_compat, "HAS_LITEARM_CORE", True)
    monkeypatch.setattr(_compat, "Arm", _FakeRealArm)
    return _FakeRealArm


@pytest.fixture
def make_fake(monkeypatch):
    """Install one *specific* fake arm as ``_compat.Arm`` and return it.

    Lets a test script the arm it will assert on (``fake.raises[...]``,
    ``fake.frame``, ``fake.q``) without reaching into ``DualArm``'s internals.
    """
    def _install(fake):
        monkeypatch.setattr(_compat, "HAS_LITEARM_CORE", True)
        monkeypatch.setattr(_compat, "Arm", lambda port=None, **kw: fake)
        return fake
    return _install


@pytest.fixture
def sim():
    a = PyBulletArm(render=False)
    a.start()
    yield a
    a.close()


@pytest.fixture
def dual(fake_sdk):
    d = DualArm(real_port="/dev/ttyFAKE", render=False)
    d.start()
    yield d
    d.close()


# ── The install hint ──────────────────────────────────────────────────────────

def test_dual_arm_without_sdk_says_how_to_get_it(monkeypatch):
    """No litearm-core: refuse, name the package, and name how to install it.

    It is not on PyPI, so the usual `pip install <name>` would send the reader
    to a dead end — the message has to say where it actually comes from.
    """
    monkeypatch.setattr(_compat, "HAS_LITEARM_CORE", False)
    monkeypatch.setattr(_compat, "Arm", None)
    with pytest.raises(ImportError) as exc:
        DualArm(render=False)
    assert "litearm-core" in str(exc.value)
    assert "pip install -e" in str(exc.value)


def test_mirror_mode_without_sdk_says_how_to_get_it(monkeypatch, sim):
    monkeypatch.setattr(_compat, "HAS_LITEARM_CORE", False)
    with pytest.raises(ImportError) as exc:
        MirrorMode(_FakeRealArm(), sim)
    assert "litearm-core" in str(exc.value)


# ── DualArm construction ──────────────────────────────────────────────────────

def test_dual_arm_connects_both_arms(fake_sdk):
    dual = DualArm(real_port="/dev/ttyACM9", render=False)
    try:
        assert isinstance(dual.real, _FakeRealArm)
        assert dual.real.port == "/dev/ttyACM9"
        assert isinstance(dual.sim, PyBulletArm)
        assert "connect" in dual.real.names()
        # Constructing does not mirror: nothing polls the arm until start().
        assert dual.mirroring is False
        # mirror_first default: start() puts the simulation at the real pose.
        assert dual.start() is dual
        assert dual.mirroring is True
        assert dual.mirror_error is None
    finally:
        dual.close()


def test_dual_arm_does_not_enable_the_real_arm(fake_sdk):
    """Energising a physical arm is never a side effect of building an object."""
    dual = DualArm(render=False)
    try:
        assert "enable" not in dual.real.names()
        assert dual.real.enabled is False
    finally:
        dual.close()


def test_dual_arm_mirror_first_off_leaves_sim_alone(fake_sdk):
    dual = DualArm(render=False, mirror_first=False)
    try:
        assert dual.mirroring is False
        assert dual.real.get_state_calls() == 0
        # start() is what turns it on.
        assert dual.start() is dual
        assert dual.mirroring is True
    finally:
        dual.close()


def test_dual_arm_failed_sim_construction_closes_the_real_arm(fake_sdk, monkeypatch):
    """A half-built DualArm must not leak an open serial port."""
    closed = []

    class _BoomArm(_FakeRealArm):
        def close(self):
            closed.append(True)
            super().close()

    monkeypatch.setattr(_compat, "Arm", _BoomArm)
    monkeypatch.setattr(
        "litearm_pybullet.mirror.PyBulletArm",
        lambda *a, **kw: (_ for _ in ()).throw(RuntimeError("no viewer")),
    )
    with pytest.raises(RuntimeError, match="no viewer"):
        DualArm(render=False)
    assert closed == [True], "the real arm was left open"


# ── Mirroring: fresh frames, at a rate ────────────────────────────────────────

def test_mirror_reads_are_always_fresh(dual):
    """`refresh=False` would return the caller's last frame forever.

    A mirror built on that reports one pose for the rest of the session and
    looks like a hung arm.  Every read must ask for a new frame.
    """
    time.sleep(0.3)
    assert dual.real.get_state_calls() > 0
    assert set(dual.real.refresh_seen) == {True}


def test_mirror_read_is_bounded_and_rate_limited(fake_sdk):
    """The sim loop runs at 500 Hz; the link must not be asked 500 times a second."""
    dual = DualArm(render=False, mirror_rate_hz=20.0)
    try:
        dual.start()
        time.sleep(0.5)
        calls = dual.real.get_state_calls()
        assert calls <= 16, f"20 Hz for 0.5s should not be {calls} reads"
        assert calls >= 3, f"mirroring looks stalled: {calls} reads in 0.5s"
        assert max(dual.real.timeouts_seen) <= 0.05
    finally:
        dual.close()


def test_mirror_teleports_the_simulation_to_the_real_pose(make_fake):
    make_fake(_FakeRealArm(q=[0.4, 0.0, 0.0, -0.8, 0.0, 0.0, 0.0]))
    dual = DualArm(render=False, mirror_first=False)
    try:
        dual.start()
        assert _await_q(dual.sim, [0.4, 0.0, 0.0, -0.8, 0.0, 0.0, 0.0])
    finally:
        dual.close()


def test_mirror_never_applies_a_short_joint_vector(make_fake):
    """A 5-joint arm against a 7-joint simulation must not half-teleport it."""
    make_fake(_FakeRealArm(n=5, q=[0.3] * 5))
    dual = DualArm(render=False)
    try:
        dual.start()
        assert isinstance(dual.mirror_error, InvalidCommandError)
        assert dual.sim.get_state().value.q == pytest.approx([0.0] * 7, abs=1e-9)
    finally:
        dual.close()


def test_mirror_reports_an_empty_frame_instead_of_going_quiet(make_fake):
    """No frame is normal on a quiet link, but it must be visible."""
    fake = make_fake(_FakeRealArm())
    fake.frame = Msg(value=None, hz=0.0, timestamp=0.0)
    dual = DualArm(render=False)
    try:
        dual.start()
        assert isinstance(dual.mirror_error, MotionTimeoutError)
        assert dual.mirroring is True      # still trying, not silently dead
    finally:
        dual.close()


# ── Commands go to both arms ──────────────────────────────────────────────────

@pytest.mark.parametrize("name", MOTION_CALLS)
def test_commands_reach_both_arms(dual, name, monkeypatch):
    sim_seen = []
    monkeypatch.setattr(
        dual.sim, name, lambda *a, **kw: sim_seen.append((a, kw)) or f"sim-{name}"
    )
    call = getattr(dual, name)
    real_out, sim_out = call(*_args_for(name), **_kwargs_for(name))

    assert real_out == f"real-{name}"
    assert sim_out == f"sim-{name}"
    assert sim_seen, f"the simulation was never told about {name}"
    assert name in dual.real.names()


def test_movej_forwards_speed_to_both_arms(dual, monkeypatch):
    seen = {}
    monkeypatch.setattr(
        dual.sim, "movej",
        lambda q, speed=1.0: seen.update(sim=speed) or "sim-movej",
    )
    dual.movej([0.1] * 7, speed=0.25)
    assert seen["sim"] == 0.25
    assert dual.real.kwargs_of("movej")[-1]["speed"] == 0.25


def test_move_p_forwards_tolerances(dual, monkeypatch):
    seen = {}
    monkeypatch.setattr(
        dual.sim, "move_p",
        lambda pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03:
            seen.update(pos_tol=pos_tol, rpy_tol=rpy_tol) or "sim",
    )
    dual.move_p([0.3, 0.0, 0.35, 3.14, 0.0, 0.0], pos_tol=0.02, rpy_tol=0.1)
    assert seen == {"pos_tol": 0.02, "rpy_tol": 0.1}
    assert dual.real.kwargs_of("move_p")[-1]["pos_tol"] == 0.02


def test_both_arms_are_told_even_when_the_first_one_fails(dual, monkeypatch):
    """A failure on one arm must not leave the other one never commanded."""
    sim_seen = []
    monkeypatch.setattr(
        dual.sim, "movej", lambda *a, **kw: sim_seen.append(1) or "sim-movej"
    )
    dual.real.raises["movej"] = MotionTimeoutError("real arm 没到位")
    with pytest.raises(MotionTimeoutError, match="real arm 没到位"):
        dual.movej([0.0] * 7)
    assert sim_seen == [1], "the simulation never got the command"


def test_real_arm_failure_wins_over_the_simulation_one(dual, monkeypatch):
    monkeypatch.setattr(
        dual.sim, "movej",
        lambda *a, **kw: (_ for _ in ()).throw(InvalidCommandError("sim 拒绝")),
    )
    dual.real.raises["movej"] = MotionTimeoutError("real 超时")
    with pytest.raises(MotionTimeoutError, match="real 超时"):
        dual.movej([0.0] * 7)


def test_simulation_failure_surfaces_when_the_real_arm_is_fine(dual, monkeypatch):
    monkeypatch.setattr(
        dual.sim, "movej",
        lambda *a, **kw: (_ for _ in ()).throw(InvalidCommandError("sim 拒绝")),
    )
    with pytest.raises(InvalidCommandError, match="sim 拒绝"):
        dual.movej([0.0] * 7)


def test_mirroring_is_paused_for_the_whole_command(dual, monkeypatch):
    """A mirrored frame landing mid-move would overwrite the command's target."""
    reads = []

    def slow_sim_movej(*a, **kw):
        reads.append(dual.real.get_state_calls())
        time.sleep(0.15)
        reads.append(dual.real.get_state_calls())
        return "sim"

    monkeypatch.setattr(dual.sim, "movej", slow_sim_movej)
    dual.movej([0.0] * 7)
    assert reads[0] == reads[1], "the mirror read the arm while a move was running"
    assert dual.mirroring is True, "mirroring never resumed"


def test_mirroring_resumes_after_a_failed_command(dual, monkeypatch):
    dual.real.raises["movej"] = MotionTimeoutError("boom")
    with pytest.raises(MotionTimeoutError):
        dual.movej([0.0] * 7)
    assert dual.mirroring is True


# ── State ─────────────────────────────────────────────────────────────────────

def test_get_real_state_is_fresh_by_default(dual):
    """Watching the real arm is the whole point; a cached frame is not watching."""
    msg = dual.get_real_state()
    assert isinstance(msg, Msg)
    assert msg.value.n == 7
    assert dual.real.refresh_seen[-1] is True


def test_get_real_state_can_ask_for_the_cached_frame(dual):
    dual.get_real_state(refresh=False)
    assert dual.real.refresh_seen[-1] is False


def test_get_sim_state_never_come_back_empty(dual):
    msg = dual.get_sim_state()
    assert isinstance(msg, Msg)
    assert msg.value is not None
    assert msg.value.n == 7


def test_get_tcp_both_sides(dual):
    real = dual.get_tcp()
    sim = dual.get_sim_tcp()
    assert isinstance(real, Msg) and isinstance(sim, Msg)
    assert len(real.value) == 6
    assert len(sim.value) == 6


# ── Lifecycle ─────────────────────────────────────────────────────────────────

def test_enable_real_arm_first_then_simulation(dual, monkeypatch):
    """Order is a safety property: nothing here should energise the sim first."""
    log = []
    monkeypatch.setattr(dual.sim, "enable", lambda attempts=12: log.append("sim"))
    real_enable = dual.real.enable

    def spy(attempts=12):
        log.append("real")
        real_enable(attempts=attempts)

    monkeypatch.setattr(dual.real, "enable", spy)
    dual.enable()
    assert log == ["real", "sim"]
    assert dual.real.enabled is True


def test_disable_lets_go_of_the_simulation_first(dual, monkeypatch):
    log = []
    monkeypatch.setattr(dual.sim, "disable", lambda: log.append("sim"))
    monkeypatch.setattr(dual.real, "disable", lambda: log.append("real"))
    dual.disable()
    assert log == ["sim", "real"]


def test_emergency_stop_reaches_the_real_arm_first(dual, monkeypatch):
    log = []
    monkeypatch.setattr(dual.sim, "emergency_stop", lambda: log.append("sim"))
    monkeypatch.setattr(dual.real, "emergency_stop", lambda: log.append("real"))
    dual.emergency_stop()
    assert log == ["real", "sim"]


def test_reset_reaches_both(dual, monkeypatch):
    log = []
    monkeypatch.setattr(dual.sim, "reset", lambda: log.append("sim"))
    monkeypatch.setattr(dual.real, "reset", lambda: log.append("real"))
    dual.reset()
    assert log == ["real", "sim"]


def test_close_stops_mirroring_and_closes_both(dual):
    dual.close()
    assert dual.mirroring is False
    assert dual.real.closes == 1
    reads_after = dual.real.get_state_calls()
    time.sleep(0.15)
    assert dual.real.get_state_calls() == reads_after, "the mirror outlived close()"


def test_close_is_idempotent(dual):
    dual.close()
    dual.close()
    assert dual.real.closes == 1


def test_dual_arm_context_manager(fake_sdk):
    with DualArm(render=False) as dual:
        assert dual.mirroring is True
    assert dual.mirroring is False
    assert dual.real.closes == 1


# ── MirrorMode ────────────────────────────────────────────────────────────────

def test_mirror_mode_drives_the_simulation(fake_sdk, sim):
    fake = _FakeRealArm(q=[0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
    mirror = MirrorMode(fake, sim, rate_hz=50.0)
    try:
        assert mirror.start() is True
        assert mirror.running is True
        assert _await_q(sim, [0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.0])
        assert set(fake.refresh_seen) == {True}
    finally:
        mirror.stop()
    assert mirror.running is False


def test_mirror_mode_stops_reading(fake_sdk, sim):
    mirror = MirrorMode(_FakeRealArm(), sim)
    mirror.start()
    mirror.stop()
    calls = mirror.real.get_state_calls()
    time.sleep(0.15)
    assert mirror.real.get_state_calls() == calls


def test_mirror_mode_is_rate_limited(fake_sdk, sim):
    fake = _FakeRealArm()
    mirror = MirrorMode(fake, sim, rate_hz=20.0)
    try:
        mirror.start()
        time.sleep(0.5)
        assert 3 <= fake.get_state_calls() <= 16, fake.get_state_calls()
    finally:
        mirror.stop()


def test_mirror_mode_context_manager(fake_sdk, sim):
    with MirrorMode(_FakeRealArm(), sim) as mirror:
        assert mirror.running is True
    assert mirror.running is False


def test_mirror_mode_start_reports_a_dead_link(fake_sdk, sim):
    fake = _FakeRealArm()
    fake.frame = Msg(value=None, hz=0.0, timestamp=0.0)
    mirror = MirrorMode(fake, sim)
    assert mirror.start(timeout=0.3) is False
    assert isinstance(mirror.error, MotionTimeoutError)
    mirror.stop()


# ── PyBulletArm.mirror_from ───────────────────────────────────────────────────

def test_mirror_from_follows_a_real_arm(sim):
    fake = _FakeRealArm(q=[0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0])
    sim.mirror_from(fake, rate_hz=50.0)
    try:
        assert sim.mirroring is True
        assert _await_q(sim, [0.0, 0.5, 0.0, -1.0, 0.0, 0.0, 0.0])
        assert set(fake.refresh_seen) == {True}
        assert sim.mirror_error is None
    finally:
        sim.stop_mirroring()
    assert sim.mirroring is False


def test_mirror_from_is_rate_limited(sim):
    fake = _FakeRealArm()
    sim.mirror_from(fake, rate_hz=20.0)
    try:
        time.sleep(0.5)
        assert 3 <= fake.get_state_calls() <= 16, fake.get_state_calls()
        assert max(fake.timeouts_seen) <= 0.05
    finally:
        sim.stop_mirroring()


def test_mirror_from_records_a_missing_frame(sim):
    fake = _FakeRealArm()
    fake.frame = Msg(value=None, hz=0.0, timestamp=0.0)
    sim.mirror_from(fake)
    try:
        time.sleep(0.2)
        assert isinstance(sim.mirror_error, MotionTimeoutError)
    finally:
        sim.stop_mirroring()


def test_stop_mirroring_stops_reading(sim):
    fake = _FakeRealArm()
    sim.mirror_from(fake)
    time.sleep(0.1)
    sim.stop_mirroring()
    calls = fake.get_state_calls()
    time.sleep(0.15)
    assert fake.get_state_calls() == calls
    assert sim.mirroring is False


def test_mirror_from_restarts_cleanly(sim):
    first, second = _FakeRealArm(), _FakeRealArm(q=[0.9] * 7)
    sim.mirror_from(first)
    time.sleep(0.1)
    sim.mirror_from(second)          # replaces, does not stack threads
    try:
        assert _await_q(sim, [0.9] * 7)
        calls = first.get_state_calls()
        time.sleep(0.15)
        assert first.get_state_calls() == calls, "the first mirror is still running"
    finally:
        sim.stop_mirroring()


def test_close_stops_mirroring_too(sim):
    fake = _FakeRealArm()
    sim.mirror_from(fake)
    time.sleep(0.1)
    sim.close()
    calls = fake.get_state_calls()
    time.sleep(0.15)
    assert fake.get_state_calls() == calls
    assert sim.mirroring is False


def test_mirror_from_needs_no_litearm_core(sim, monkeypatch):
    """A fake arm is enough — mirroring is plain duck typing, no SDK required."""
    monkeypatch.setattr(_compat, "HAS_LITEARM_CORE", False)
    sim.mirror_from(_FakeRealArm())
    try:
        assert sim.mirroring is True
    finally:
        sim.stop_mirroring()


# ── Argument shapes used by the parametrised command test ─────────────────────

_POSE = [0.3, 0.0, 0.35, 3.14, 0.0, 0.0]


def _args_for(name):
    """Positional arguments for one DualArm command, as a tuple."""
    if name in ("movej", "movej_sync"):
        return ([0.1] * 7,)
    if name in ("move_p", "move_l"):
        return (_POSE,)
    if name == "move_c":
        return (_POSE, [0.32, 0.0, 0.35, 3.14, 0.0, 0.0], _POSE)
    return ([_POSE, _POSE],)       # move_path


def _kwargs_for(name):
    return {} if name in ("movej", "movej_sync") else {"speed": 0.5}
