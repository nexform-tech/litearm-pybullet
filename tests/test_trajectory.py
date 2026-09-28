"""Tests for the recorded-trajectory types and their record/replay path.

``JointTrajectory``/``TrajectoryFrame`` are simulation-only (litearm-python has no
equivalent), and they are the on-disk format users hand between runs — so the
round trip through a file is a contract, not an implementation detail.
"""
from __future__ import annotations

import json

import pytest

from litearm_pybullet import JointTrajectory, PyBulletArm, TrajectoryFrame


def _traj(name="t", n=3):
    return JointTrajectory(
        frames=[TrajectoryFrame(t=i * 0.01, q=[float(i)] * 7) for i in range(n)],
        name=name,
        sample_rate_hz=100.0,
        filter_alpha=0.15,
    )


# ── Frame ─────────────────────────────────────────────────────────────────────

def test_frame_defaults_dq_to_zeros():
    f = TrajectoryFrame(t=0.5, q=[0.1, 0.2])
    assert f.dq == [0.0, 0.0]
    assert (f.t, f.q) == (0.5, [0.1, 0.2])


def test_frame_round_trips_through_a_dict():
    f = TrajectoryFrame(t=1.25, q=[0.1, 0.2], dq=[0.3, 0.4])
    assert TrajectoryFrame.from_dict(f.to_dict()).to_dict() == f.to_dict()


def test_frame_from_dict_tolerates_a_missing_dq():
    """Older files may not carry velocities; that must not be a crash."""
    f = TrajectoryFrame.from_dict({"t": 0.0, "q": [0.0, 0.0]})
    assert f.dq == [0.0, 0.0]


def test_frame_repr_names_the_first_joint():
    assert "0.500" in repr(TrajectoryFrame(t=0.5, q=[0.25, 0.0]))


# ── Trajectory ────────────────────────────────────────────────────────────────

def test_trajectory_q_and_t_are_the_frame_columns():
    traj = _traj(n=3)
    assert traj.q == [[0.0] * 7, [1.0] * 7, [2.0] * 7]
    assert traj.t == [0.0, 0.01, 0.02]


def test_trajectory_round_trips_through_a_dict():
    traj = _traj(name="pick")
    back = JointTrajectory.from_dict(traj.to_dict())
    assert back.name == "pick"
    assert back.sample_rate_hz == 100.0
    assert back.filter_alpha == 0.15
    assert [f.to_dict() for f in back.frames] == [f.to_dict() for f in traj.frames]


def test_trajectory_from_dict_defaults_missing_metadata():
    traj = JointTrajectory.from_dict({"frames": [{"t": 0.0, "q": [0.0]}]})
    assert (traj.name, traj.sample_rate_hz, traj.filter_alpha) == ("", 100.0, 0.15)
    assert len(traj.frames) == 1


def test_trajectory_save_and_load(tmp_path):
    path = tmp_path / "nested" / "take.json"
    traj = _traj(name="take")
    traj.save(str(path))

    assert path.exists(), "save() must create the parent directories too"
    assert json.loads(path.read_text())["name"] == "take"

    back = JointTrajectory.load(str(path))
    assert back.to_dict() == traj.to_dict()


def test_trajectory_repr_counts_frames():
    assert repr(_traj(name="x", n=4)) == "JointTrajectory(name='x', frames=4)"


# ── The public import path ────────────────────────────────────────────────────

def test_types_are_importable_from_the_package_root():
    """They used to live in a vendored SDK stub; the root export is the API."""
    import litearm_pybullet

    assert litearm_pybullet.JointTrajectory is JointTrajectory
    assert litearm_pybullet.TrajectoryFrame is TrajectoryFrame
    assert "JointTrajectory" in litearm_pybullet.__all__
    assert "TrajectoryFrame" in litearm_pybullet.__all__


def test_no_vendored_sdk_stub_is_left_behind():
    """`_litearm` was the old-SDK shim; nothing may import it any more."""
    import importlib

    with pytest.raises(ImportError):
        importlib.import_module("litearm_pybullet._litearm")
    with pytest.raises(ImportError):
        importlib.import_module("litearm_pybullet._litearm.types")


# ── Recording and replaying through the arm ───────────────────────────────────

@pytest.fixture
def arm():
    a = PyBulletArm(render=False)
    a.start()
    yield a
    a.close()


def test_record_trajectory_measures_the_simulation(arm):
    """Recording is a simulation-only ability: nothing else can sample a path."""
    traj = arm.record_trajectory(duration_s=0.2, sample_rate_hz=100.0, name="r")
    assert isinstance(traj, JointTrajectory)
    assert traj.name == "r"
    assert len(traj.frames) == pytest.approx(20, abs=1)
    assert all(len(f.q) == 7 and len(f.dq) == 7 for f in traj.frames)
    # The clock is the sample clock, whatever the wall clock did.
    assert traj.t[0] == pytest.approx(0.0)
    assert traj.t[-1] == pytest.approx(0.19, abs=1e-6)


def test_record_trajectory_leaves_the_arm_enabled(arm):
    """Recording disables the position loop while it samples; it must put it back.

    Otherwise the arm is left limp after a recording, which looks exactly like a
    successful recording until the next command fails.
    """
    arm.record_trajectory(duration_s=0.1)
    # refresh=True: the cached frame is up to one physics step old, and the last
    # one built landed while the position loop was still down.
    assert arm.get_state(refresh=True).value.enabled is True
    # And it still takes commands — this raises if it never arrived.
    state = arm.movej([0.3] * 7, speed=0.5)
    assert state.q == pytest.approx([0.3] * 7, abs=0.05)


def test_play_trajectory_accepts_an_object(arm, tmp_path):
    traj = JointTrajectory(
        frames=[TrajectoryFrame(t=i * 0.1, q=[0.0, 0.2 * (i % 2), 0.0, 0.0,
                                              0.0, 0.0, 0.0]) for i in range(4)],
        name="small",
    )
    state = arm.play_trajectory(traj, goto_start=True, goto_speed=0.5)
    # Replay ends on the last frame's pose, within the arrival tolerance.
    assert state.q == pytest.approx([0.0, 0.2, 0.0, 0.0, 0.0, 0.0, 0.0], abs=0.05)


def test_play_trajectory_accepts_a_path(arm, tmp_path):
    path = tmp_path / "take.json"
    JointTrajectory(
        frames=[TrajectoryFrame(t=0.0, q=[0.0] * 7)], name="still"
    ).save(str(path))
    state = arm.play_trajectory(str(path))
    assert len(state.q) == 7


def test_play_trajectory_accepts_a_dict(arm, tmp_path):
    d = JointTrajectory(
        frames=[TrajectoryFrame(t=0.0, q=[0.1] * 7)], name="d"
    ).to_dict()
    state = arm.play_trajectory(d, goto_speed=0.5)
    assert len(state.q) == 7


def test_replay_trajectory_accepts_a_plain_joint_path(arm):
    state = arm.replay_trajectory([[0.0] * 7, [0.1] * 7], goto_speed=0.5)
    assert len(state.q) == 7
