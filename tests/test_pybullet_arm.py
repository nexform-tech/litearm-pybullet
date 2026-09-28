"""Tests for PyBulletArm — standalone simulation (no hardware needed)."""
from __future__ import annotations

import pytest
import numpy as np

from litearm_pybullet import PyBulletArm
from litearm_pybullet._compat import (
    CartPlan,
    IKError,
    InvalidCommandError,
    Msg,
    RobotState,
    mat_to_rpy,
    rpy_to_mat,
)


def _assert_mat_close(R, expected, abs_=1e-6):
    """pytest.approx does not accept nested sequences; compare row by row."""
    assert len(R) == len(expected)
    for row, exp_row in zip(R, expected):
        assert row == pytest.approx(exp_row, abs=abs_)


@pytest.fixture
def arm():
    """Create a headless PyBulletArm for testing."""
    a = PyBulletArm(render=False)
    a.start()
    yield a
    a.close()


def test_init_and_close(arm):
    """Test creation, start, and close."""
    assert arm is not None
    assert repr(arm) == "PyBulletArm(n_joints=7, render=False)"


def test_get_state(arm):
    """get_state returns a Msg envelope around a litearm_core RobotState."""
    import time
    time.sleep(0.1)
    msg = arm.get_state()
    assert isinstance(msg, Msg)
    assert msg.hz > 0
    assert msg.timestamp > 0

    state = msg.value
    assert isinstance(state, RobotState)
    assert state.n == 7
    assert len(state.q) == len(state.dq) == len(state.tau) == 7
    assert state.mode_name == "INIT"
    assert state.enabled is True
    assert state.faulted is False
    assert state.fault_axes == []
    assert [j.err for j in state.joints] == [0] * 7
    # The simulated banner is not a real firmware string, by design.
    assert arm.firmware == "PyBulletSim-7J"
    assert arm.n == 7


def test_get_status_now(arm):
    """get_status_now answers with a frame of this instant, never None."""
    msg = arm.get_status_now()
    assert isinstance(msg, Msg)
    assert msg.value is not None
    assert msg.value.n == 7


def test_get_tcp(arm):
    """get_tcp returns Msg[(x,y,z,r,p,y)]; get_tcp_pose rebuilds (pos, R)."""
    msg = arm.get_tcp()
    assert isinstance(msg, Msg)
    rpy = msg.value
    assert len(rpy) == 6
    assert all(isinstance(x, float) for x in rpy)

    pos, R = arm.get_tcp_pose()
    assert len(pos) == 3
    assert len(R) == 3
    assert len(R[0]) == 3
    assert all(isinstance(x, float) for x in pos)
    # Both spellings describe the same pose.
    assert pos == pytest.approx(list(rpy[:3]))
    _assert_mat_close(R, rpy_to_mat(rpy[3:]), abs_=1e-12)
    # ... and that pose is the FK of the joints the arm is actually in.
    pos_fk, _ = arm.fk(arm.get_state().value.q)
    assert pos == pytest.approx(pos_fk, abs=1e-3)


def test_fk(arm):
    """Test forward kinematics."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    assert len(pos) == 3
    assert len(R) == 3
    assert len(R[0]) == 3


def test_ik(arm):
    """ik takes a pose and returns q; failure raises IKError."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    pose6 = list(pos) + mat_to_rpy(R)

    # The SDK's pose[6] spelling.
    q_sol = arm.ik(pose6, q_seed=q)
    assert len(q_sol) == 7
    err = np.max(np.abs(np.array(q_sol) - np.array(q)))
    assert err < 0.1

    # (pos, R) is the simulation's superset of that form. The solver is seeded
    # from the arm's live state, so the two solutions need not be bit-equal —
    # what must agree is the pose they reach.
    q_sol2 = arm.ik((pos, R), q_seed=q)
    assert len(q_sol2) == 7
    pos_a, R_a = arm.fk(q_sol)
    pos_b, R_b = arm.fk(q_sol2)
    assert pos_a == pytest.approx(pos_b, abs=1e-3)
    _assert_mat_close(R_a, R_b, abs_=1e-3)

    # No explicit seed: the arm seeds from its own measured state.
    assert len(arm.ik(pose6)) == 7

    with pytest.raises(IKError):
        arm.ik([10.0, 0.0, 0.0, 0.0, 0.0, 0.0], q_seed=q)
    with pytest.raises(InvalidCommandError):
        arm.ik([1.0, 2.0, 3.0], q_seed=q)
    with pytest.raises(InvalidCommandError):
        arm.ik(pose6, q_seed=[0.0, 0.0])


def test_movej(arm):
    """movej returns the state it arrived in, not a bool."""
    import time
    time.sleep(0.1)
    target = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    st = arm.movej(target, speed=0.5)
    assert isinstance(st, RobotState)
    # The returned frame is the arrival frame: already within tolerance.
    err = np.max(np.abs(np.array(st.q) - np.array(target)))
    assert err < arm._q_tol, f"movej reported arrival at err={err}"
    assert np.max(np.abs(st.dq)) < arm._dq_tol
    # ... and the live state agrees.
    err_live = np.max(np.abs(np.array(arm.get_state().value.q) - np.array(target)))
    assert err_live < 0.05, f"movej did not reach target: err={err_live}"


def test_movej_sync(arm):
    """movej_sync lands every axis on the same step."""
    target = [0.0, 0.5, 0.0, -1.0, 0.0, 0.6, 0.0]
    st = arm.movej_sync(target, speed=0.5)
    assert isinstance(st, RobotState)
    assert np.max(np.abs(np.array(st.q) - np.array(target))) < arm._q_tol


def test_movej_validates_speed_and_arity(arm):
    """speed is a 0..1 fraction; out of range and wrong arity both raise."""
    with pytest.raises(InvalidCommandError):
        arm.movej([0.0] * 7, speed=1.5)
    with pytest.raises(InvalidCommandError):
        arm.movej([0.0] * 7, speed=-0.1)
    with pytest.raises(InvalidCommandError):
        arm.movej([0.0] * 6)
    # speed=0.0 is legal input (0..1 inclusive) and means "don't move".
    with pytest.raises(InvalidCommandError):
        arm.movej_sync([0.0] * 7, speed=2.0)


def test_move_l(arm):
    """move_l follows a straight line and reports the plan."""
    import time
    time.sleep(0.1)
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)

    pos, R = arm.get_tcp_pose()
    target = [pos[0], pos[1], pos[2] - 0.05]
    plan = arm.move_l([target, R], speed=0.5)
    assert isinstance(plan, CartPlan)
    assert plan.ok is True
    assert plan.n_wp > 0
    assert plan.plan_us >= 0
    assert plan.started_busy is True
    assert plan.settled is True
    assert len(plan.q_final) == 7
    assert plan.settle_err_rad < arm._q_tol
    # It really went there.
    tcp = arm.get_tcp().value
    assert tcp[2] == pytest.approx(target[2], abs=0.005)


def test_move_l_no_wait(arm):
    """wait=False reports "did not wait", not "did not arrive"."""
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)
    pos, R = arm.get_tcp_pose()
    plan = arm.move_l([pos[0], pos[1], pos[2] - 0.03] + mat_to_rpy(R), wait=False)
    assert plan.ok is True
    assert plan.settled is False
    assert plan.started_busy is False
    assert plan.q_final == []
    assert plan.settle_err_rad == 0.0
    # The arm keeps moving after the call returns. Reading it takes
    # refresh=True: like the real SDK, a plain get_state() hands back the last
    # frame *you* read, which here predates this motion.
    assert arm.get_status_now().value.cart_busy is True
    arm._motion_serial.acquire()   # waits for the background play to finish
    assert arm.get_status_now().value.cart_busy is False


def test_move_p(arm):
    """move_p is a joint-space point-to-point judged on the TCP."""
    import time
    time.sleep(0.1)
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)

    pos, R = arm.get_tcp_pose()
    # Same attitude, 4 cm down. The sim's local IK leaves a few millimetres of
    # residual, so the arrival tolerance is opened up to cover it — the test is
    # about move_p's arrival criterion, not about the solver's limits.
    goal = [pos[0], pos[1], pos[2] - 0.04] + mat_to_rpy(R)
    st = arm.move_p(goal, speed=0.5, pos_tol=0.01, rpy_tol=0.05)
    assert isinstance(st, RobotState)
    tcp = arm.get_tcp().value
    assert tcp[0] == pytest.approx(goal[0], abs=0.01)
    assert tcp[2] == pytest.approx(goal[2], abs=0.01)


def test_move_p_reports_an_impossible_tolerance_before_moving(arm):
    """A tolerance the solver cannot meet is an IKError, not a 15 s timeout."""
    pos, R = arm.get_tcp_pose()
    goal = [pos[0], pos[1], pos[2] - 0.02] + mat_to_rpy(R)
    with pytest.raises(IKError, match="可达精度"):
        arm.move_p(goal, pos_tol=1e-4)


def test_move_p_rejects_pose_sequences(arm):
    """A sequence of poses is move_path's job, and says so."""
    pos, R = arm.get_tcp_pose()
    with pytest.raises(InvalidCommandError, match="move_path"):
        arm.move_p([[pos[0], pos[1], pos[2], 0.0, 0.0, 0.0]] * 2)


def test_move_c_checks_the_declared_start(arm):
    """move_c starts from the measured TCP, so a wrong start must raise."""
    import time
    time.sleep(0.1)
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)

    tcp = arm.get_tcp().value
    via = [tcp[0] + 0.02, tcp[1], tcp[2] - 0.04, 0.0, 0.0, 0.0]
    goal = [tcp[0] + 0.04, tcp[1], tcp[2], 0.0, 0.0, 0.0]

    with pytest.raises(InvalidCommandError, match="pose_start"):
        arm.move_c([tcp[0] + 0.5, tcp[1], tcp[2], 0.0, 0.0, 0.0], via, goal)

    plan = arm.move_c(tcp, via, goal, speed=0.5)
    assert plan.ok is True
    assert plan.settled is True


def test_move_path_limits_and_corners(arm):
    """move_path rejects an empty path and more waypoints than the firmware takes."""
    pos, _ = arm.get_tcp_pose()
    with pytest.raises(InvalidCommandError, match="路径为空"):
        arm.move_path([])
    with pytest.raises(InvalidCommandError, match="32"):
        arm.move_path([[pos[0], pos[1], pos[2], 0.0, 0.0, 0.0]] * 33)


def test_deprecated_motion_aliases(arm):
    """The old spellings still work, and still mean the same thing."""
    import time
    time.sleep(0.1)
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)
    pos, R = arm.get_tcp_pose()
    rpy = mat_to_rpy(R)

    def pose(dz):
        return [pos[0], pos[1], pos[2] + dz] + rpy

    with pytest.deprecated_call():
        plan = arm.movel([pose(-0.02)[:3], R], speed=0.5)
    assert isinstance(plan, CartPlan)
    assert plan.ok is True

    with pytest.deprecated_call():
        plan = arm.movec(pose(-0.03), pose(-0.02), speed=0.5)
    assert plan.ok is True

    with pytest.deprecated_call():
        plan = arm.movep([pose(-0.015), pose(-0.01)], speed=0.5)
    assert plan.ok is True

    # Deprecated spellings are pure forwards — no second implementation that
    # can drift away from the new one.
    with pytest.deprecated_call():
        old = arm.movel(pose(-0.02), speed=0.3)
    new = arm.move_l(pose(-0.02), speed=0.3)
    assert (old.ok, old.n_wp, old.settled) == (new.ok, new.n_wp, new.settled)


def test_home(arm):
    """home() returns to the zero configuration, keyword-only timeout."""
    import time
    time.sleep(0.1)
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)
    st = arm.home()
    assert np.max(np.abs(np.array(st.q))) < arm._q_tol


def test_plan_movel(arm):
    """Test plan_movel produces a valid path."""
    q_start = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q_start)
    path = arm.plan_movel(q_start, (pos, R))
    assert len(path) > 0
    assert len(path[0]) == 7


def test_request_stop(arm):
    """Stop shows up as mode=EMERGENCY, and clearing it restores idle."""
    import time
    arm.request_stop()
    time.sleep(0.05)
    state = arm.get_state().value
    assert state.mode_name == "EMERGENCY"
    assert state.faulted is True

    arm.clear_stop()
    time.sleep(0.05)
    state = arm.get_state().value
    assert state.mode_name == "INIT"
    assert state.faulted is False


def test_enable_disable(arm):
    """Test enable/disable cycle."""
    import time
    arm.disable()
    time.sleep(0.05)
    assert arm.get_state().value.enabled is False

    arm.enable()
    time.sleep(0.05)
    assert arm.get_state().value.enabled is True


def test_set_get_gains(arm):
    """Test set_gains and get_gains."""
    gains = arm.set_gains(kp=[300]*7, kd=[10]*7)
    assert gains["kp"] == [300]*7
    assert gains["kd"] == [10]*7

    gains = arm.get_gains()
    assert gains["kp"] == [300]*7


def test_context_manager():
    """Test context manager."""
    with PyBulletArm(render=False) as a:
        import time
        time.sleep(0.1)
        assert a.get_state().value.n == 7
    # Should be closed now


def test_device(arm):
    """Test simulated device proxy."""
    hand = arm.device("hand_0")
    assert hand.open() is True
    assert hand.close() is True
    assert hand.set_gesture("pinch") is True
    assert hand.list_gestures() == ["open", "close", "pinch"]

    # Test devices property
    assert "hand_0" in arm.devices
    assert arm.devices["hand_0"].device_id == "hand_0"


def test_set_joint_positions(arm):
    """Test direct joint position setting."""
    q = [0.0, 0.3, 0.0, -0.5, 0.0, 0.3, 0.0]
    arm.set_joint_positions(q)
    import time
    time.sleep(0.1)
    err = np.max(np.abs(np.array(arm.get_state().value.q) - np.array(q)))
    assert err < 0.01


def test_fk_ik_roundtrip(arm):
    """Test FK→IK roundtrip: ik(fk(q)) ≈ q."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    q_sol = arm.ik(list(pos) + mat_to_rpy(R), q_seed=q)
    err = np.max(np.abs(np.array(q_sol) - np.array(q)))
    assert err < 0.1