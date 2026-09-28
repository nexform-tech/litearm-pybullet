"""The litearm-core API contract, written where it can fail.

Three things are pinned here:

1. The 31 names ``PyBulletArm`` shares with ``litearm_core.Arm`` — name,
   parameter names, defaults — as a literal table.  A signature change on
   either side has to be a deliberate edit to this file.
2. The names the simulation is *supposed* to add (58 simulation-only, 8 legacy
   aliases), so the public surface cannot grow or shrink by accident.  The
   README's three tiers and the counts in the developer guide come from these
   same numbers.
3. Against the real SDK, when it is installed: that ``litearm_core.Arm`` still
   matches the table, that the fallback's dataclasses have the same fields in
   the same order, that the fallback's error classes mirror the SDK's, and
   that the re-exported constant tables are the SDK's own.

With ``litearm-core`` absent only the fallback half runs — which is the point,
since the fallback is what ships to people who have no arm yet.
"""
from __future__ import annotations

import dataclasses
import inspect

import pytest

from litearm_pybullet import HAS_LITEARM_CORE, PyBulletArm, _compat, _fallback

if HAS_LITEARM_CORE:
    try:
        from litearm_core import _protocol as _sdk_protocol
    except ImportError:  # pragma: no cover - SDK internals moved
        _sdk_protocol = None
else:
    _sdk_protocol = None

requires_sdk = pytest.mark.skipif(
    not HAS_LITEARM_CORE, reason="litearm-core is not installed"
)


# ── The 1:1 tier ──────────────────────────────────────────────────────────────

#: ``name -> rendered signature``.  Properties read as ``"property"``.
SHARED_API = {
    "clear_faults": "()",
    "close": "()",
    "connect": "(port=None)",
    "disable": "()",
    "disconnect": "()",
    "emergency_stop": "()",
    "enable": "(attempts=12)",
    "get_state": "(refresh=False, timeout=0.5)",
    "get_status_now": "(timeout=0.5)",
    "get_tcp": "(timeout=0.6)",
    "home": "(timeout=None)",
    "ik": "(pose, q_seed=None, timeout=3.0)",
    "move_c": "(pose_start, pose_via, pose_goal, speed=1.0, wait=True)",
    "move_js": "(q, dq=None, tau_ff=None)",
    "move_l": "(pose, speed=1.0, wait=True)",
    "move_p": "(pose, speed=1.0, pos_tol=0.006, rpy_tol=0.03)",
    "move_path": "(poses, speed=1.0, wait=True)",
    "movej": "(q, speed=1.0)",
    "movej_sync": "(q, speed=1.0)",
    "park": "()",
    "reset": "()",
    "send_mit": "(idx, q, dq, kp, kd, tau)",
    "send_mit_all": "(q, dq, kp, kd, tau)",
    "set_motion_mode": "(mode)",
    "set_payload": "(mass, com=(0.0, 0.0, 0.0))",
    "set_speed": "(percent)",
    "zero_g": "(period=0.04)",
    "zero_g_active": "property",
    "zero_g_error": "property",
    "zero_g_start": "(period=0.04)",
    "zero_g_stop": "(raise_on_lost=False)",
}

#: Everything the simulation adds that litearm-core has no counterpart for.
#: Documentation calls these "simulation-only"; there are 58.
SIM_ONLY = (
    "cartesian_impedance", "connect_device", "delete_trajectory", "device",
    "devices", "discard_recording", "disconnect_device", "enter_teleop",
    "exit_teleop", "firmware", "fk", "get_active_device",
    "get_cartesian_limits", "get_collision_config", "get_end_effector",
    "get_gains", "get_installation", "get_joint_limits", "get_logs",
    "get_payload", "get_playback_state", "get_recording_state",
    "get_system_stats", "get_teleop_status", "get_zero_offsets", "hand", "hold",
    "joint_follow", "joint_impedance", "list_device_types",
    "list_trajectories", "mirror_error", "mirror_from", "mirroring", "n",
    "plan_movec", "plan_movel", "plan_movep", "play_trajectory", "port",
    "record_trajectory", "recover_joint_limits", "replay_joint_path",
    "replay_timed_trajectory", "replay_trajectory", "restart_service",
    "save_trajectory", "set_cartesian_limits", "set_collision_config",
    "set_end_effector", "set_gains", "set_installation", "set_joint_limits",
    "set_joint_positions", "set_zero_offsets", "start_recording",
    "stop_mirroring", "stop_recording",
)

#: Kept for 0.1 callers.  The first seven emit a DeprecationWarning and forward
#: to exactly one new method; ``start()`` is a bare forward to ``connect()``.
LEGACY = (
    "movel", "movec", "movep", "get_tcp_pose", "request_stop", "clear_stop",
    "zero_gravity", "start",
)

#: On ``litearm_core.Arm``, deliberately absent here: entering DFU, reading the
#: firmware's own log, tuning feed-forward, the model/param dumps.  The
#: simulation must not pretend to have them.
SDK_ONLY = (
    "FF_SCALAR_ITEMS", "FF_SCALAR_RO_ITEMS", "FF_VEC_ITEMS", "diag",
    "enter_dfu", "ff_preset", "get_ff_mask", "get_ff_scalar", "get_ff_vec",
    "last_reset_reason", "log", "model", "params", "poll_cart", "reconnect",
    "save_params", "set_ff_mask", "set_ff_scalar", "set_ff_vec",
    "set_gravity_scale", "set_gravity_vector", "set_inertia_scale",
)


def _public(cls) -> set:
    """The names a caller sees — dunders and privates excluded."""
    return {n for n in dir(cls) if not n.startswith("_")}


def _render_signature(cls, name: str) -> str:
    """``"property"``, ``"attr"``, or the parameter list without ``self``."""
    try:
        raw = inspect.getattr_static(cls, name)
    except AttributeError:  # pragma: no cover - used by the tests below
        return "<missing>"
    if isinstance(raw, property):
        return "property"
    if not callable(raw):
        return "attr"
    parts = []
    for param in inspect.signature(raw).parameters.values():
        if param.name == "self":
            continue
        rendered = param.name
        if param.default is not inspect.Parameter.empty:
            rendered += "=" + repr(param.default)
        parts.append(rendered)
    return "(" + ", ".join(parts) + ")"


def test_the_shared_table_is_the_agreed_size():
    """31 — the number the README promises."""
    assert len(SHARED_API) == 31


def test_the_simulation_only_and_legacy_lists_are_unique_and_disjoint():
    for group in (SIM_ONLY, LEGACY, SDK_ONLY):
        assert len(set(group)) == len(group)
    assert set(SIM_ONLY).isdisjoint(LEGACY)
    assert set(SIM_ONLY).isdisjoint(SHARED_API)
    assert set(LEGACY).isdisjoint(SHARED_API)
    assert set(SDK_ONLY).isdisjoint(SHARED_API)


def test_pybullet_arm_implements_every_shared_name():
    for name in SHARED_API:
        assert hasattr(PyBulletArm, name), f"PyBulletArm.{name} is missing"


def test_pybullet_arm_shared_signatures_match_the_table():
    """The 1:1 claim, checked name by name and default by default."""
    for name, expected in SHARED_API.items():
        assert _render_signature(PyBulletArm, name) == expected, name


def test_public_surface_is_exactly_the_three_tiers():
    """No name leaks in, none goes missing.

    This is what keeps the documented counts honest: adding a method to
    ``PyBulletArm`` fails here until it is classified.
    """
    assert _public(PyBulletArm) == set(SHARED_API) | set(SIM_ONLY) | set(LEGACY)


def test_simulation_does_not_claim_sdk_only_names():
    """Absent by design — DFU entry and firmware log reads mean nothing here."""
    assert _public(PyBulletArm).isdisjoint(SDK_ONLY)


# ── Against the real SDK ──────────────────────────────────────────────────────

@requires_sdk
def test_real_sdk_still_matches_the_shared_table():
    """If litearm-core changes a signature, this fails and the table is stale."""
    for name, expected in SHARED_API.items():
        assert hasattr(_compat.Arm, name), f"litearm_core.Arm.{name} is gone"
        assert _render_signature(_compat.Arm, name) == expected, name


@requires_sdk
def test_every_sdk_only_name_still_exists_on_the_real_sdk():
    for name in SDK_ONLY:
        assert hasattr(_compat.Arm, name), f"litearm_core.Arm.{name} is gone"


@requires_sdk
def test_fallback_dataclasses_have_the_sdk_fields_in_order():
    """Field *order* matters: these are built positionally in places."""
    for name in ("Msg", "RobotState", "JointState", "CartPlan"):
        sdk = getattr(_compat, name)
        local = getattr(_fallback, name)
        assert dataclasses.is_dataclass(sdk) and dataclasses.is_dataclass(local)
        assert [f.name for f in dataclasses.fields(local)] == [
            f.name for f in dataclasses.fields(sdk)
        ], name


@requires_sdk
def test_fallback_errors_mirror_the_sdk_hierarchy():
    """Same classes, same inheritance — so one ``except`` covers both."""
    from litearm_core import errors as sdk_errors

    sdk_names = {
        n for n in dir(sdk_errors)
        if n.endswith("Error") and isinstance(getattr(sdk_errors, n), type)
    }
    local_names = {n for n in dir(_fallback) if n.endswith("Error")}

    assert local_names == sdk_names
    for name in sorted(sdk_names):
        sdk_mro = [c.__name__ for c in getattr(sdk_errors, name).__mro__]
        local_mro = [c.__name__ for c in getattr(_fallback, name).__mro__]
        assert local_mro == sdk_mro, name


@requires_sdk
@pytest.mark.skipif(_sdk_protocol is None, reason="litearm_core._protocol moved")
def test_compat_reexports_the_sdk_constant_tables():
    """The mode/flag tables are the SDK's, not a local copy that drifted."""
    assert _compat.MODE_NAMES == _sdk_protocol.MODE_NAMES
    assert _compat.FLAG_NAMES == _sdk_protocol.FLAG_NAMES
    assert _compat.FLAG_ENABLED_BIT == _sdk_protocol.FLAG_ENABLED_BIT
    assert _compat.MAX_JOINTS == _sdk_protocol.MAX_JOINTS
