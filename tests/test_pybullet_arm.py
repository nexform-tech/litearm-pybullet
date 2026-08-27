"""Tests for PyBulletArm — standalone simulation (no hardware needed)."""
from __future__ import annotations

import pytest
import numpy as np

from litearm_pybullet import PyBulletArm


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
    """Test get_state returns valid dict."""
    import time
    time.sleep(0.1)
    state = arm.get_state()
    assert state is not None
    assert "q" in state
    assert "dq" in state
    assert "state" in state
    assert len(state["q"]) == 7
    assert state["robot_serial"] == "PYBULLET-SIM-001"


def test_get_tcp_pose(arm):
    """Test get_tcp_pose returns valid position and rotation."""
    pos, R = arm.get_tcp_pose()
    assert len(pos) == 3
    assert len(R) == 3
    assert len(R[0]) == 3
    assert all(isinstance(x, float) for x in pos)


def test_fk(arm):
    """Test forward kinematics."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    assert len(pos) == 3
    assert len(R) == 3
    assert len(R[0]) == 3


def test_ik(arm):
    """Test inverse kinematics."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    q_sol, success = arm.ik(pos, R, q_seed=q)
    assert success
    assert len(q_sol) == 7
    # IK should be close to the original
    err = np.max(np.abs(np.array(q_sol) - np.array(q)))
    assert err < 0.1


def test_movej(arm):
    """Test movej blocking motion."""
    import time
    time.sleep(0.1)
    ok = arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)
    assert ok
    state = arm.get_state()
    q = np.array(state["q"])
    target = np.array([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0])
    err = np.max(np.abs(q - target))
    assert err < 0.05, f"movej did not reach target: err={err}"


def test_movel(arm):
    """Test movel Cartesian line motion."""
    import time
    time.sleep(0.1)
    # Move to a known pose first
    arm.movej([0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0], speed=0.5)

    pos, R = arm.get_tcp_pose()
    target = [pos[0], pos[1], pos[2] - 0.05]
    ok = arm.movel([target, R], speed=0.2)
    assert ok


def test_plan_movel(arm):
    """Test plan_movel produces a valid path."""
    q_start = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q_start)
    path = arm.plan_movel(q_start, (pos, R))
    assert len(path) > 0
    assert len(path[0]) == 7


def test_request_stop(arm):
    """Test request_stop and clear_stop."""
    import time
    arm.request_stop()
    time.sleep(0.05)
    state = arm.get_state()
    assert state["state"] == "stopping"

    arm.clear_stop()
    time.sleep(0.05)
    state = arm.get_state()
    assert state["state"] == "ready"


def test_enable_disable(arm):
    """Test enable/disable cycle."""
    import time
    arm.disable()
    time.sleep(0.05)
    state = arm.get_state()
    assert state["state"] == "disconnected"

    arm.enable()
    time.sleep(0.05)
    state = arm.get_state()
    assert state["state"] == "ready"


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
        state = a.get_state()
        assert state is not None
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
    state = arm.get_state()
    err = np.max(np.abs(np.array(state["q"]) - np.array(q)))
    assert err < 0.01


def test_fk_ik_roundtrip(arm):
    """Test FK→IK roundtrip: ik(fk(q)) ≈ q."""
    q = [0.0, 0.6, 0.0, -1.2, 0.0, 0.7, 0.0]
    pos, R = arm.fk(q)
    q_sol, success = arm.ik(pos, R, q_seed=q)
    assert success
    err = np.max(np.abs(np.array(q_sol) - np.array(q)))
    assert err < 0.1