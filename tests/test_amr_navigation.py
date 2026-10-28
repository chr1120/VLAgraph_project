import copy
import math
import time
from types import SimpleNamespace

import pytest

from backend.world_model.amr_navigation import (
    OccupancyGrid, differential_command, fingerprint, plan_path, validate_plan, wheel_speeds,
)
from examples.isaacsim.mobile_world_model_bridge import MobileWorldModelAdapter


def map_fixture():
    return {"frame": "right_handed_z_up_m", "stage_identifier": "cell.usd",
            "resolution_m": .25, "origin_m": [-2, -2], "cells": [[0]*32 for _ in range(32)]}


def plan(grid=None, **kwargs):
    options = {"robot_prim_path": "/World/AMR", "radius_m": .2,
               "episode_id": "episode", "observed_at": time.time(), **kwargs}
    return plan_path(grid or map_fixture(), [0, 0], [3, 0], **options)


def test_astar_detours_wall_and_every_segment_has_footprint_clearance():
    data = map_fixture()
    for row in range(0, 18):
        data["cells"][row][14] = 1
    original = copy.deepcopy(data)
    result = plan(data)
    grid = OccupancyGrid(data)
    assert len(result["waypoints_m"]) > 2
    assert all(grid.segment_clear(a, b, .3) for a, b in zip(result["waypoints_m"], result["waypoints_m"][1:]))
    assert data == original
    assert validate_plan(result) == result


def test_unknown_space_boundary_and_narrow_corridor_block_routes():
    data = map_fixture()
    for row in data["cells"]:
        row[14] = -1
    with pytest.raises(ValueError, match="No traversable"):
        plan(data)
    with pytest.raises(ValueError, match="clearance"):
        plan_path(data, [-1.9, 0], [0, 0], robot_prim_path="/World/AMR", radius_m=.2,
                  episode_id="a", observed_at=time.time())
    # A center-line-only planner could pass this gap, but a disk cannot.
    data = map_fixture()
    for y in range(32):
        if y != 8:
            data["cells"][y][14] = 1
    with pytest.raises(ValueError):
        plan(data)


@pytest.mark.parametrize("update", [{"resolution_m": True}, {"cells": [[False]]},
                                    {"cells": [[0], [0, 1]]}, {"frame": "odom"}])
def test_invalid_maps_rejected(update):
    with pytest.raises(ValueError):
        plan({**map_fixture(), **update})


def test_plan_tampering_and_unit_errors_rejected():
    result = plan()
    result["waypoints_m"][-1] = [999, 999]
    with pytest.raises(ValueError, match="digest"):
        validate_plan(result)
    with pytest.raises(ValueError):
        plan(radius_m=True)


def test_differential_drive_turns_before_moving_and_maps_wheels():
    speed, turn = differential_command((0, 0), math.pi/2, (1, 0))
    assert speed == 0 and turn < 0
    assert wheel_speeds(.2, 0, .1, .5) == (2, 2)
    left, right = wheel_speeds(0, .4, .1, .5)
    assert left < 0 < right


@pytest.fixture
def adapter():
    descriptor = {"prim_path": "/World/AMR", "robot_kind": "amr", "base_motion": "planar",
                  "mobile_base": {"drive_type": "differential", "footprint_radius_m": .2}}
    result = MobileWorldModelAdapter(SimpleNamespace(), {"robots": [descriptor]}, clock=lambda: tuple(result.test_clock))
    result.test_clock = [1., 60]
    result.test_pose = [0., 0., 0., 0., 0.]
    result.test_map = map_fixture()
    result.test_drives = []
    result._identity = lambda: {"stage_identifier": "cell.usd", "meters_per_unit": 1., "up_axis": "Z"}
    result.navigation_map = lambda _: copy.deepcopy(result.test_map)
    result._pose = lambda _: (tuple(result.test_pose[:2]), *result.test_pose[2:])
    result._drive = lambda path, v, w: result.test_drives.append((v, w))
    return result


def start(adapter, goal=(3, 0), command_id="test"):
    p = plan_path(adapter.test_map, adapter.test_pose[:2], goal, robot_prim_path="/World/AMR",
                  radius_m=.2, episode_id=adapter.episode_id, observed_at=time.time())
    request = {"command_id": command_id, "plan": p, "timeout_sec": 120}
    adapter.navigate(request)
    return request


def step(adapter):
    adapter.test_clock[0] += 1/60
    adapter.test_clock[1] += 1
    adapter.tick()


def test_controller_reaches_goal_and_checks_three_distinct_settled_steps(adapter):
    start(adapter, goal=(.1, 0))
    for _ in range(600):
        step(adapter)
        v, w = adapter.test_drives[-1]
        adapter.test_pose[0] += v/60
        adapter.test_pose[2] += w/60
        adapter.test_pose[3:] = [abs(v), abs(w)]
        if adapter.active_id is None:
            break
    assert adapter.public_record("test")["completion_verified"] is True
    assert adapter.public_record("test")["status"] == "completed"
    assert abs(adapter.test_pose[0]-.1) < .03
    assert adapter.test_drives[-1] == (0., 0.)


def test_idle_or_ack_is_not_completion_and_retry_never_restarts(adapter):
    request = start(adapter)
    for _ in range(10):
        adapter.tick()  # Same physics step, cannot settle the command.
    assert adapter.public_record("test")["completion_verified"] is False
    assert adapter.navigate(request)["status"] == "running"
    request["plan"]["goal_m"] = [4, 0]
    request["plan"]["waypoints_m"][-1] = [4, 0]
    request["plan"]["digest"] = fingerprint({k: v for k, v in request["plan"].items() if k != "digest"})
    with pytest.raises(ValueError, match="different plan"):
        adapter.navigate(request)


def test_live_obstacle_stops_command_and_requires_replanning(adapter):
    start(adapter)
    adapter.test_map["cells"][8][13] = 1
    step(adapter)
    assert adapter.public_record("test")["status"] == "blocked"
    assert adapter.test_drives[-1] == (0., 0.)


def test_reset_cancels_command_invalidates_articulations_and_episode(adapter):
    request = start(adapter)
    old_episode = adapter.episode_id
    adapter.articulations["old"] = object()
    adapter.test_clock[:] = [0., 0]
    adapter.tick()
    assert adapter.public_record("test")["status"] == "cancelled"
    assert adapter.episode_id != old_episode
    assert not adapter.articulations
    request["command_id"] = "new"
    with pytest.raises(ValueError, match="episode"):
        adapter.navigate(request)


def test_reset_discards_invalid_handles_before_attempting_stop(adapter):
    start(adapter)
    old_episode = adapter.episode_id
    adapter.articulations["invalid_view"] = object()
    def drive(path, v, w):
        assert not adapter.articulations, "An old-epoch physics view was reused"
        adapter.test_drives.append((v, w))
    adapter._drive = drive
    adapter.test_clock[:] = [0., 0]
    adapter.tick()
    assert adapter.public_record("test")["status"] == "cancelled"
    assert adapter.episode_id != old_episode
    assert adapter.test_drives[-1] == (0., 0.)


def test_reset_advances_episode_even_if_new_physics_is_not_ready(adapter):
    start(adapter)
    old_episode = adapter.episode_id
    adapter.articulations["invalid_view"] = object()
    def not_ready(*args):
        raise ValueError("New physics handles not ready")
    adapter._drive = not_ready
    adapter.test_clock[:] = [0., 0]
    adapter.tick()
    new_episode = adapter.episode_id
    assert new_episode != old_episode and not adapter.articulations
    assert adapter.public_record("test")["status"] == "stopping"
    for _ in range(3):
        adapter.tick()
    assert adapter.episode_id == new_episode
    adapter._drive = lambda path, v, w: adapter.test_drives.append((v, w))
    step(adapter)
    assert adapter.public_record("test")["status"] == "cancelled"
    assert adapter.test_drives[-1] == (0., 0.)


def test_reset_during_pending_explicit_stop_also_invalidates_old_handles(adapter):
    start(adapter)
    adapter._drive = lambda *a: (_ for _ in ()).throw(ValueError("invalid view"))
    adapter.apply({"command": "amr_stop", "command_id": "test"}, lambda _: None)
    assert adapter.public_record("test")["status"] == "stopping"
    adapter.articulations["invalid_view"] = object()
    old_episode = adapter.episode_id
    adapter.test_clock[:] = [0., 0]
    adapter.tick()
    assert not adapter.articulations and adapter.episode_id != old_episode


def test_missing_observation_stops_and_failed_stop_is_retried_without_motion(adapter):
    start(adapter)
    adapter._pose = lambda _: (_ for _ in ()).throw(ValueError("sensor unavailable"))
    def fail_stop(*args):
        raise ValueError("actuator unavailable")
    adapter._drive = fail_stop
    step(adapter)
    assert adapter.public_record("test")["status"] == "stopping"
    adapter._drive = lambda p, v, w: adapter.test_drives.append((v, w))
    step(adapter)
    assert adapter.public_record("test")["status"] == "failed"
    assert adapter.test_drives[-1] == (0., 0.)


def test_stop_and_status_require_known_command_and_stop_is_idempotent(adapter):
    start(adapter)
    original = lambda _: pytest.fail("AMR command reached humanoid fallback")
    response = adapter.apply({"command": "amr_stop", "command_id": "test", "request_id": "r"}, original)
    assert response["ok"] and response["execution"]["status"] == "cancelled"
    assert adapter.apply({"command": "amr_stop", "command_id": "test"}, original)["ok"]
    assert not adapter.apply({"command": "amr_status", "command_id": "unknown"}, original)["ok"]


def test_explicit_stop_failure_never_resumes_navigation_on_next_tick(adapter):
    start(adapter)
    def unavailable(*args):
        raise ValueError("wheel controller unavailable")
    adapter._drive = unavailable
    response = adapter.apply({"command": "amr_stop", "command_id": "test"}, lambda _: None)
    assert not response["ok"]
    assert adapter.public_record("test")["status"] == "stopping"
    adapter._drive = lambda p, v, w: adapter.test_drives.append((v, w))
    step(adapter)
    assert adapter.public_record("test")["status"] == "cancelled"
    assert adapter.test_drives[-1] == (0., 0.)


def test_stale_plan_and_mismatched_start_are_rejected_before_motion(adapter, monkeypatch):
    request = start(adapter)
    adapter.finish("cancelled", "test")
    request["command_id"] = "new"
    adapter.test_pose[0] = 1.
    with pytest.raises(ValueError, match="moved"):
        adapter.navigate(request)
    adapter.test_pose[0] = 0.
    monkeypatch.setattr("examples.isaacsim.mobile_world_model_bridge.time.time", lambda: request["plan"]["observed_at"]+10)
    with pytest.raises(ValueError, match="stale"):
        adapter.navigate(request)


def test_timeout_stall_and_high_measured_velocity_do_not_report_arrival(adapter, monkeypatch):
    start(adapter, goal=(0, 0))
    adapter.test_pose[3] = .3
    for _ in range(5):
        step(adapter)
    assert not adapter.public_record("test")["completion_verified"]
    deadline = adapter.records["test"]["_deadline"]
    monkeypatch.setattr("examples.isaacsim.mobile_world_model_bridge.time.monotonic", lambda: deadline+1)
    step(adapter)
    assert adapter.public_record("test")["reason"] == "navigation_timeout"
    assert adapter.test_drives[-1] == (0., 0.)


def test_pose_uses_measured_quaternion_and_scales_translation_not_angular_velocity(adapter):
    adapter._identity = lambda: {"meters_per_unit": .01, "up_axis": "Y", "stage_identifier": "cell.usd"}
    adapter.observe_robot = lambda _: {
        "position": [100, 0, -200], "orientation_xyzw": [-2**-.5, 0, 0, 2**-.5],
        "linear_velocity": [100, 0, 0], "angular_velocity_rad_s": [0, .5, 0]}
    position, yaw, speed, turn = MobileWorldModelAdapter._pose(adapter, "/World/AMR")
    assert position == (1, 2)
    assert yaw == pytest.approx(0)
    assert speed == 1 and turn == .5


def test_live_joint_read_uses_explicit_dof_mapping_and_fails_without_physics(adapter, monkeypatch):
    import sys
    class Matrix:
        def ExtractTranslation(self): return [1, 2, 3]
        def ExtractRotationQuat(self): return SimpleNamespace(GetImaginary=lambda: [0, 0, 0], GetReal=lambda: 1)
    monkeypatch.setitem(sys.modules, "pxr", SimpleNamespace(
        Usd=SimpleNamespace(TimeCode=SimpleNamespace(Default=lambda: 0)),
        UsdGeom=SimpleNamespace(Xformable=lambda _: SimpleNamespace(ComputeLocalToWorldTransform=lambda _: Matrix()))))
    adapter.bridge._get_prim = lambda _: object()
    articulation = SimpleNamespace(
        dof_names=["rail_dof", "joint2", "joint1"],
        get_world_pose=lambda: ([1, 2, 3], [1, 0, 0, 0]),
        get_linear_velocity=lambda: [0, 0, 0], get_angular_velocity=lambda: [0, 0, 0],
        get_joint_positions=lambda: [10, .2, .1], get_joint_velocities=lambda: [20, .4, .3])
    adapter._articulation = lambda _: articulation
    adapter._identity = lambda: {"meters_per_unit": .001}
    descriptor = {"prim_path": "/World/FANUC", "robot_kind": "articulated_arm", "joint_bindings": [
        {"name": "J1", "dof_name": "joint1", "kind": "revolute"},
        {"name": "J2", "dof_name": "joint2", "kind": "revolute"},
        {"name": "rail", "dof_name": "rail_dof", "kind": "prismatic"}]}
    result = adapter.observe_robot(descriptor)
    assert [j["position"] for j in result["joints"]] == [.1, .2, .01]
    assert [j["velocity"] for j in result["joints"]] == [.3, .4, .02]
    articulation.get_joint_velocities = lambda: None
    result = adapter.observe_robot(descriptor)
    assert "observation_error" in result
    assert "joints" not in result and "linear_velocity" not in result


def test_collider_overlay_scales_bounds_excludes_selected_and_preserves_unknown(adapter, monkeypatch):
    import sys
    class Prim:
        def __init__(self, path): self.path = path
        def GetPath(self): return self.path
        def IsActive(self): return True
        def HasAPI(self, _): return True
    class Collision:
        def __init__(self, prim): pass
        def GetCollisionEnabledAttr(self): return SimpleNamespace(Get=lambda: True)
    monkeypatch.setitem(sys.modules, "pxr", SimpleNamespace(UsdPhysics=SimpleNamespace(CollisionAPI=Collision)))
    adapter.config["navigation_map"] = map_fixture()
    adapter.config["navigation_map"]["cells"][0][0] = -1
    adapter.config["ground_prim_paths"] = ["/World/Floor"]
    adapter.bridge._get_stage = lambda: SimpleNamespace(Traverse=lambda: iter([
        Prim("/World/AMR/wheel"), Prim("/World/Floor"), Prim("/World/FANUC/link1")]))
    adapter._identity = lambda: {"stage_identifier": "cell.usd", "meters_per_unit": .001, "up_axis": "Z"}
    adapter.bridge._prim_world_bbox = lambda _: {"center": [1500, 0, 1000], "size": [500, 500, 2000]}
    data = MobileWorldModelAdapter.navigation_map(adapter, "/World/AMR")
    assert data["cells"][0][0] == -1
    assert data["cells"][8][14] == 1
    assert data["cells"][8][8] == 0
    assert adapter.config["navigation_map"]["cells"][8][14] == 0


def test_wheel_output_targets_only_configured_dofs_and_preserves_other_joints(adapter, monkeypatch):
    import sys
    emitted = []
    descriptor = adapter.robots["/World/AMR"]
    descriptor["wheel_config"] = {"radius_m": .1, "separation_m": .5,
                                  "dof_names": ["left", "right"], "signs": [1, -1]}
    adapter._articulation = lambda _: SimpleNamespace(dof_names=["arm_joint", "right", "left"], apply_action=emitted.append)
    monkeypatch.setitem(sys.modules, "isaacsim.core.utils.types", SimpleNamespace(ArticulationAction=lambda **kwargs: kwargs))
    MobileWorldModelAdapter._drive(adapter, "/World/AMR", .2, 0.)
    assert emitted[0]["joint_indices"].tolist() == [2, 1]
    assert emitted[0]["joint_velocities"].tolist() == [2., -2.]
    assert "joint_positions" not in emitted[0]
