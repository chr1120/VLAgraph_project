import copy
import math
import time

import pytest

from backend.executors.world_model_nodes import exec_execution_guard, exec_fanuc_collision_check, exec_fanuc_trajectory_validate
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.fanuc_preflight import validate_fanuc_trajectory
from test_fanuc_collision import scene_fixture
from test_fanuc_preflight import action_fixture


def snapshot_fixture():
    return {"schema": "vlagraph.fanuc_snapshot.v1", "episode_id": "synthetic-episode", "state_version": 1,
            "sim_time": 10, "physics_step": 600, "prim_path": "/World/FANUC", "observed_at": time.time(),
            "synchronized_joint_snapshot": True, "prediction_ready": False,
            "geometry": {"meters_per_unit": 1, "up_axis": "Z", "stage_identifier": "test", "physics_synchronized": False},
            "raw_state": {"prim_path": "/World/FANUC", "physics_ready": True, "motion_active": False},
            "axes": [{"name": f"J{i}", "position": 0, "target_position": 0,
                      "velocity": 0, "unit": "rad", "virtual": False} for i in range(1, 7)]}


def far_scene():
    scene = scene_fixture()
    scene["obstacles"] = [{"id": "far", "min": [20, 20, 20], "max": [21, 21, 21]}]
    return scene


def guard(action, snapshot, *, report=None, collision=None, props=None):
    context = {"a": {"action": action}, "p": {"report": report or validate_fanuc_trajectory(action, snapshot=snapshot)},
               "s": {"snapshot": snapshot}}
    edges = [{"source": "a", "sourceHandle": "action", "target": "g", "targetHandle": "action"},
             {"source": "p", "sourceHandle": "report", "target": "g", "targetHandle": "report"},
             {"source": "s", "sourceHandle": "snapshot", "target": "g", "targetHandle": "snapshot"}]
    if collision is not None:
        context["c"] = {"collision_report": collision}
        edges.append({"source": "c", "sourceHandle": "collision_report", "target": "g", "targetHandle": "collision_report"})
    return exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation", **(props or {})}}, context, edges)


def executable_action():
    action = action_fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    return action


def test_initial_sweep_collision_is_detected_before_first_target():
    action, snapshot, scene = action_fixture(), snapshot_fixture(), scene_fixture()
    for frame in action["joint_trajectory"]:
        frame["joint_positions"][0] = 90
    snapshot["axes"][0]["position"] = -math.pi / 2
    assert check_fanuc_collision(action, scene)["status"] == "clear"
    report = check_fanuc_collision(action, scene, snapshot=snapshot, max_step_sec=0.1)
    assert report["status"] == "collision"
    assert any(contact["time_sec"] == 1 for contact in report["contacts"])
    assert report["initial_segment_checked"] is True
    assert report["checked_time_range_sec"] == [0, 4]
    assert "initial_motion_before_first_frame" not in report["unchecked"]
    assert "physical_base_alignment" in report["unchecked"]
    assert report["physical_collision_verified"] is False


def test_snapshot_allows_one_target_and_counts_initial_interval_in_budget():
    action = action_fixture()
    action["joint_trajectory"] = action["joint_trajectory"][:1]
    with pytest.raises(ValueError, match="two trajectory"):
        check_fanuc_collision(action, far_scene())
    report = check_fanuc_collision(action, far_scene(), snapshot=snapshot_fixture())
    assert report["status"] == "clear" and report["samples_checked"] == 41
    with pytest.raises(ValueError, match="budget"):
        check_fanuc_collision(action, far_scene(), snapshot=snapshot_fixture(), max_samples=40)


def test_preflight_checks_peak_limits_from_actual_initial_position():
    snapshot = snapshot_fixture()
    snapshot["axes"][0]["position"] = -math.pi / 2
    report = validate_fanuc_trajectory(action_fixture(), snapshot=snapshot, config={"maxVelocity": {"J1": 0.5}})
    assert any(error["code"] == "joint_velocity_limit" and error["frame"] == 0 for error in report["errors"])
    assert report["segments"][0]["end_frame"] == 0


@pytest.mark.parametrize("case", ["active", "unsettled", "out_of_limits", "virtual_external", "axes_mismatch"])
def test_unsupported_initial_state_fails_preflight_and_collision(case):
    snapshot = snapshot_fixture()
    expected = {"active": "initial_motion_not_idle", "unsettled": "initial_axis_not_settled",
                "out_of_limits": "initial_joint_position_limit", "virtual_external": "virtual_initial_axis",
                "axes_mismatch": "snapshot_or_segment_axis_mismatch"}[case]
    if case == "active": snapshot["raw_state"]["motion_active"] = True
    if case == "unsettled": snapshot["axes"][0]["velocity"] = 0.02
    if case == "out_of_limits": snapshot["axes"][0]["position"] = 4
    if case in {"virtual_external", "axes_mismatch"}:
        snapshot["axes"].append({"name": "externalAxisJoint", "position": 0, "target_position": 0,
                                 "velocity": None if case == "virtual_external" else 0,
                                 "virtual": case == "virtual_external", "unit": "m"})
    report = validate_fanuc_trajectory(action_fixture(), snapshot=snapshot)
    assert expected in {error["code"] for error in report["errors"]}
    with pytest.raises(ValueError, match="Static preflight"):
        check_fanuc_collision(action_fixture(), far_scene(), snapshot=snapshot)


@pytest.mark.parametrize("case", ["robot", "stage", "units", "up"])
def test_usd_proxy_snapshot_identity_must_match(case):
    scene = far_scene()
    scene.update(inventory_digest="test-inventory", robot_prim_path="/World/FANUC",
                 source_meters_per_unit=1, source_up_axis="Z")
    snapshot = snapshot_fixture()
    if case == "robot": scene["robot_prim_path"] = "/World/OtherRobot"
    if case == "stage": scene["scene_id"] = "other-stage"
    if case == "units": scene["source_meters_per_unit"] = 0.001
    if case == "up": scene["source_up_axis"] = "Y"
    with pytest.raises(ValueError, match="robot/stage/units differ"):
        check_fanuc_collision(action_fixture(), scene, snapshot=snapshot)


def test_matching_snapshot_is_bound_to_preflight_collision_and_guard():
    action, snapshot = executable_action(), snapshot_fixture()
    preflight = validate_fanuc_trajectory(action, snapshot=snapshot)
    collision = check_fanuc_collision(action, far_scene(), snapshot=snapshot)
    assert collision["snapshot_ref"] == preflight["snapshot_ref"]
    result = guard(action, snapshot, report=preflight, collision=collision)
    assert result["approved"] is True  # Only the explicit static_simulation scope.
    assert result["decision"]["initial_snapshot_age_sec"] >= 0
    assert guard(action, snapshot, report=preflight, collision=collision, props={"validationPolicy": "full"})["approved"] is False


@pytest.mark.parametrize("case", ["preflight_content", "collision_content", "missing_initial_segment", "no_initial_collision"])
def test_same_identifiers_cannot_hide_changed_snapshot_content(case):
    action, snapshot = executable_action(), snapshot_fixture()
    preflight = validate_fanuc_trajectory(action, snapshot=snapshot)
    changed = copy.deepcopy(snapshot)
    changed["axes"][0]["position"] = 0.1
    collision = check_fanuc_collision(action, far_scene(), snapshot=snapshot)
    if case == "preflight_content": snapshot = changed
    if case == "collision_content": collision = check_fanuc_collision(action, far_scene(), snapshot=changed)
    if case == "missing_initial_segment": collision["initial_segment_checked"] = False
    if case == "no_initial_collision": collision = check_fanuc_collision(action, far_scene())
    assert guard(action, snapshot, report=preflight, collision=collision)["approved"] is False


@pytest.mark.parametrize("age", [6, -1])
def test_stale_and_future_snapshot_are_blocked_at_guard(age):
    snapshot = snapshot_fixture()
    snapshot["observed_at"] = time.time() - age
    assert guard(executable_action(), snapshot)["approved"] is False


def test_failed_connected_snapshot_is_not_treated_as_offline_no_snapshot():
    action = executable_action()
    context = {"a": {"action": action}, "s": {"snapshot": None}}
    edges = [{"source": "a", "sourceHandle": "action", "target": "n", "targetHandle": "action"},
             {"source": "s", "sourceHandle": "snapshot", "target": "n", "targetHandle": "snapshot"}]
    assert exec_fanuc_trajectory_validate({"id": "n", "data": {}}, context, edges)["ok"] is False
    assert exec_fanuc_collision_check({"id": "n", "data": {"enabled": True, "scene": far_scene()}}, context, edges)["ok"] is False
    assert guard(action, None)["approved"] is False
    report = validate_fanuc_trajectory(action)
    context["p"] = {"report": report}
    result = exec_execution_guard({"id": "n", "data": {"validationPolicy": "static_simulation", "requireInitialSnapshot": True}},
                                  context, edges[:1] + [{"source": "p", "sourceHandle": "report", "target": "n", "targetHandle": "report"}])
    assert result["approved"] is False
