import copy
import math

import pytest

from backend.world_model.fanuc_collision import segment_box_distance, segment_segment_distance, check_fanuc_collision
from backend.executors import vla_nodes
from backend.runtime.runner import run_graph
from test_fanuc_execution_guard import pipeline
from test_fanuc_preflight import action_fixture


def scene_fixture():
    return {
        "schema": "vlagraph.fanuc_collision_scene.v1", "scene_id": "test", "scene_version": "v1",
        "robot_model_version": "test_proxy", "geometry_source": "configured_proxy",
        "kinematics_source": "project_m710_configuration", "frame": "right_handed_z_up_m",
        "base_position": [0, 0, 0], "base_orientation_wxyz": [1, 0, 0, 0],
        "link_capsules": [{"id": "tool_proxy", "joint_index": 5, "start_local": [0.1, 0, 0],
                           "end_local": [0.1, 0, 0], "radius_m": 0.05}],
        "excluded_self_pairs": [],
        "obstacles": [{"id": "box", "min": [2, -0.05, 1.85], "max": [2.1, 0.05, 1.96]}],
    }


def sweep_action():
    action = action_fixture()
    action["joint_trajectory"][0]["joint_positions"][0] = -90
    action["joint_trajectory"][1]["joint_positions"][0] = 90
    return action


def test_segment_box_distance_handles_edges_and_euclidean_corners():
    lower, upper = [-1, -1, -1], [1, 1, 1]
    assert segment_box_distance([-2, 0, 0], [2, 0, 0], lower, upper) == 0
    assert segment_box_distance([2, 2, -3], [2, 2, 3], lower, upper) == pytest.approx(math.sqrt(2))
    assert segment_box_distance([2, 0, 0], [2, 0, 0], lower, upper) == 1
    assert segment_box_distance([-2, 1, 0], [2, 1, 0], lower, upper) == 0


def test_segment_segment_distance_handles_crossings_and_degenerate_segments():
    assert segment_segment_distance([-1, 0, 0], [1, 0, 0], [0, -1, 0], [0, 1, 0]) == pytest.approx(0)
    assert segment_segment_distance([-1, 0, 0], [1, 0, 0], [0, -1, 1], [0, 1, 1]) == pytest.approx(1)
    assert segment_segment_distance([0, 0, 0], [0, 0, 0], [3, 0, 0], [3, 0, 0]) == 3


def test_midpoint_collision_is_found_even_with_clear_endpoints():
    report = check_fanuc_collision(sweep_action(), scene_fixture(), max_step_sec=1)
    assert report["status"] == "collision"
    assert any(contact["time_sec"] == 3 for contact in report["contacts"])
    assert report["physical_collision_verified"] is False
    assert report["execution_ready"] is False


def test_sparse_samples_cannot_falsely_certify_clear_sweep():
    report = check_fanuc_collision(sweep_action(), scene_fixture(), max_step_sec=2)
    assert not report["contacts"]
    assert report["status"] == "indeterminate"
    assert report["between_samples_certified"] is False
    assert report["unresolved_intervals"]


def test_far_obstacle_can_be_certified_for_configured_proxies_only():
    scene = scene_fixture()
    scene["obstacles"] = [{"id": "far", "min": [20, 20, 20], "max": [21, 21, 21]}]
    original = copy.deepcopy(scene)
    report = check_fanuc_collision(sweep_action(), scene, max_step_sec=2)
    assert report["status"] == "clear"
    assert report["between_samples_certified"] is True
    assert "initial_motion_before_first_frame" in report["unchecked"]
    assert scene == original


def test_self_collision_and_explicit_exclusions():
    scene = scene_fixture()
    scene["obstacles"] = [{"id": "far", "min": [20, 20, 20], "max": [21, 21, 21]}]
    second = copy.deepcopy(scene["link_capsules"][0])
    second["id"] = "other"
    scene["link_capsules"].append(second)
    report = check_fanuc_collision(action_fixture(), scene)
    assert report["status"] == "collision"
    assert report["self_pairs_checked"] == 1
    scene["excluded_self_pairs"] = [["tool_proxy", "other"]]
    report = check_fanuc_collision(action_fixture(), scene)
    assert report["status"] == "clear"
    assert report["self_pairs_checked"] == 0
    assert report["excluded_self_pairs"] == [["other", "tool_proxy"]]


@pytest.mark.parametrize("case", ["visual_bounds", "units", "radius", "duplicate", "quaternion", "exclusion", "budget", "bad_action"])
def test_untrusted_or_incomplete_geometry_is_rejected(case):
    scene, action = scene_fixture(), action_fixture()
    if case == "visual_bounds": scene["geometry_source"] = "scene_scan_bbox"
    if case == "units": scene["frame"] = "millimeters"
    if case == "radius": scene["link_capsules"][0]["radius_m"] = float("nan")
    if case == "duplicate": scene["obstacles"][0]["id"] = "tool_proxy"
    if case == "quaternion": scene["base_orientation_wxyz"] = [2, 0, 0, 0]
    if case == "exclusion": scene["excluded_self_pairs"] = [["tool_proxy", "unknown"]]
    if case == "bad_action": action["joint_trajectory"][0]["joint_positions"][0] = 999
    with pytest.raises(ValueError):
        check_fanuc_collision(action, scene, max_samples=2 if case == "budget" else 10000)


@pytest.mark.parametrize("enabled, valid_scene, expected_calls", [(True, True, 0), (True, False, 0), (False, False, 1), ("true", True, 0)])
def test_integrated_guard_blocks_collision_and_missing_geometry(monkeypatch, enabled, valid_scene, expected_calls):
    graph = pipeline("static_simulation")
    graph["nodes"].append({"id": "collision", "type": "fanuc_collision_check", "data": {
        "enabled": enabled, "scene": scene_fixture() if valid_scene else {}}})
    graph["edges"].extend([
        {"source": "prepare", "target": "collision", "sourceHandle": "state", "targetHandle": "action"},
        {"source": "collision", "target": "guard", "sourceHandle": "collision_report", "targetHandle": "collision_report"},
    ])
    calls = []
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: calls.append(True) or {"ok": True})
    result = run_graph(graph)
    assert len(calls) == expected_calls
    assert result["context"]["guard"]["approved"] is bool(expected_calls)


def test_clear_proxy_report_never_unlocks_full_policy(monkeypatch):
    scene = scene_fixture()
    scene["obstacles"] = [{"id": "far", "min": [20, 20, 20], "max": [21, 21, 21]}]
    graph = pipeline("full")
    graph["nodes"].append({"id": "collision", "type": "fanuc_collision_check", "data": {"enabled": True, "scene": scene}})
    graph["edges"].extend([
        {"source": "prepare", "target": "collision", "sourceHandle": "state", "targetHandle": "action"},
        {"source": "collision", "target": "guard", "sourceHandle": "collision_report", "targetHandle": "collision_report"},
    ])
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: pytest.fail("Transport called"))
    result = run_graph(graph)
    assert result["context"]["collision"]["collision_report"]["status"] == "clear"
    assert result["context"]["guard"]["approved"] is False


def test_indeterminate_sweep_blocks_static_simulation(monkeypatch):
    graph = pipeline("static_simulation")
    action = graph["nodes"][0]["data"]["robotState"]
    action["joint_trajectory"][0]["joint_positions"][0] = -90
    action["joint_trajectory"][1]["joint_positions"][0] = 90
    graph["nodes"].append({"id": "collision", "type": "fanuc_collision_check", "data": {
        "enabled": True, "scene": scene_fixture(), "maxStepSec": 2}})
    graph["edges"].extend([
        {"source": "prepare", "target": "collision", "sourceHandle": "state", "targetHandle": "action"},
        {"source": "collision", "target": "guard", "sourceHandle": "collision_report", "targetHandle": "collision_report"},
    ])
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: pytest.fail("Transport called"))
    result = run_graph(graph)
    assert result["context"]["collision"]["collision_report"]["status"] == "indeterminate"
    assert result["context"]["execute"]["skipped"] is True
