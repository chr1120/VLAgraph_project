import copy
import json
import time

import pytest

from backend.executors import world_model_nodes as nodes
from backend.world_model.fanuc_world_context import reconcile_memory, prediction_handoff
from backend.world_model.fanuc_preflight import action_fingerprint, validate_fanuc_trajectory
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.usd_geometry import collision_inventory_digest
from test_fanuc_initial_state import snapshot_fixture, far_scene, executable_action


def fixtures():
    now = time.time()
    snapshot = snapshot_fixture()
    scan = {"ok": True, "scene_id": "cell", "stage_identifier": "test", "meters_per_unit": 1, "up_axis": "Z",
            "timestamp": now, "sim_time": snapshot["sim_time"], "truncated": False,
            "robot": {"prim_path": snapshot["prim_path"], "position": [0, 0, 0]},
            "objects": [{"prim_path": "/World/Box", "position": [20, 20, 20], "bbox_center": [20, 20, 20], "size": [2, 2, 2]}]}
    record = {"id": "collected", "scan": copy.deepcopy(scan), "metadata": {}}
    inventory = {"schema": "vlagraph.usd_collision_inventory.v1", "stage_identifier": "test",
                 "robot_prim_path": snapshot["prim_path"], "meters_per_unit": 1, "up_axis": "Z",
                 "colliders": [{"prim_path": "/World/Box/Collision", "role": "static_obstacle",
                                "world_aabb_m_zup": {"min": [19]*3, "max": [21]*3}}]}
    inventory["inventory_digest"] = collision_inventory_digest(inventory)
    return record, scan, snapshot, inventory, now


def test_matching_memory_is_only_geometry_context_and_does_not_mutate_inputs():
    record, scan, snapshot, inventory, now = fixtures()
    original = copy.deepcopy(record)
    report = reconcile_memory(record, scan, snapshot, inventory, now=now)
    assert report["status"] == "geometry_context_consistent" and report["object_count"] == 1
    assert not report["physical_context_verified"] and not report["execution_ready"]
    assert record == original


@pytest.mark.parametrize("case,code", [
    ("stage", "scene_identity_or_frame_mismatch"), ("robot", "scene_identity_or_frame_mismatch"),
    ("truncated", "scene_scan_coverage_unverified"), ("old", "memory_stale_or_time_unknown"),
    ("fresh", "scan_stale_or_time_unknown"), ("moved", "memory_geometry_changed_or_incomplete"),
    ("new", "scene_object_set_changed"), ("unmapped", "observed_objects_without_static_collision_geometry"),
    ("time", "scan_snapshot_time_mismatch"), ("bounds", "collision_bounds_disagree_with_observation"),
])
def test_stale_changed_or_unsupported_context_cannot_produce_consistent_report(case, code):
    record, scan, snapshot, inventory, now = fixtures()
    if case == "stage": record["scan"].pop("stage_identifier")
    if case == "robot": record["scan"]["robot"]["prim_path"] = "/World/Other"
    if case == "truncated": scan["truncated"] = True
    if case == "old": record["scan"]["timestamp"] -= 90000
    if case == "fresh": scan["timestamp"] -= 30
    if case == "moved": scan["objects"][0]["position"][0] += 1
    if case == "new": scan["objects"] = []
    if case == "unmapped": inventory["colliders"] = []
    if case == "time": scan["sim_time"] += 1
    if case == "bounds": inventory["colliders"][0]["world_aabb_m_zup"]["min"] = [0]*3
    inventory["inventory_digest"] = collision_inventory_digest(inventory)
    report = reconcile_memory(record, scan, snapshot, inventory, now=now)
    assert report["status"] == "blocked" and code in {item["code"] for item in report["reasons"]}


def invoke(executor, values, props=None):
    context = {name: {name: value} for name, value in values.items()}
    edges = [{"source": name, "sourceHandle": name, "target": "node", "targetHandle": name} for name in values]
    return executor({"id": "node", "data": props or {}}, context, edges)


def test_saved_collection_flows_into_sweep_prediction_and_guard_without_authorizing_motion(tmp_path, monkeypatch):
    record, scan, snapshot, inventory, now = fixtures()
    path = tmp_path / "memory.jsonl"
    path.write_text(json.dumps(record)+"\n", encoding="utf-8")
    scene = far_scene()
    scene["inventory_digest"] = inventory["inventory_digest"]
    scene.update(robot_prim_path=snapshot["prim_path"], source_meters_per_unit=1, source_up_axis="Z")
    monkeypatch.setattr(nodes, "build_fanuc_usd_proxy", lambda inv: {"scene": scene})
    reconciled = invoke(nodes.exec_fanuc_world_context, {"scan": scan, "snapshot": snapshot, "inventory": inventory},
                        {"memoryPath": str(path)})
    assert reconciled["ok"] and reconciled["scene"] == scene
    world = reconciled["context_report"]
    action = executable_action()
    collision = check_fanuc_collision(action, scene, snapshot=snapshot)
    assert collision["status"] == "clear"
    predicted = invoke(nodes.exec_fanuc_world_prediction, {"action": action, "context_report": world,
                       "collision_report": collision, "snapshot": snapshot, "scene": scene})
    assert predicted["prediction_report"]["status"] == "physical_prediction_unavailable"
    values = {"action": action, "report": validate_fanuc_trajectory(action, snapshot=snapshot), "snapshot": snapshot,
              "context_report": world, "collision_report": collision, "prediction_report": predicted["prediction_report"]}
    result = invoke(nodes.exec_execution_guard, values, {"validationPolicy": "static_simulation", "requireWorldContext": True})
    assert not result["approved"]
    assert "physical_prediction_worker_unavailable" in {reason["code"] for reason in result["decision"]["reasons"]}
    values["prediction_report"]["physical_rollout_verified"] = True
    forged = invoke(nodes.exec_execution_guard, values, {"validationPolicy": "static_simulation"})
    assert not forged["approved"]
    assert "physical_prediction_worker_unavailable" in {reason["code"] for reason in forged["decision"]["reasons"]}
    values["context_report"]["evidence"]["scan"]["objects"][0]["position"][0] = 100
    result = invoke(nodes.exec_execution_guard, values, {"validationPolicy": "static_simulation"})
    assert not result["approved"] and result["decision"]["reasons"][0]["code"] == "invalid_guard_input"


def test_missing_memory_and_disconnected_context_fail_closed(tmp_path):
    _, scan, snapshot, inventory, _ = fixtures()
    result = invoke(nodes.exec_fanuc_world_context, {"scan": scan, "snapshot": snapshot, "inventory": inventory},
                    {"memoryPath": str(tmp_path / "missing")})
    assert not result["ok"] and result["scene"] is None
    action = executable_action()
    result = invoke(nodes.exec_execution_guard, {"action": action, "report": validate_fanuc_trajectory(action)},
                    {"validationPolicy": "static_simulation", "requireWorldContext": True})
    assert not result["approved"]


def test_prediction_rejects_changed_action_or_snapshot():
    record, scan, snapshot, inventory, now = fixtures()
    world = reconcile_memory(record, scan, snapshot, inventory, now=now)
    action, scene = executable_action(), far_scene()
    scene["inventory_digest"] = inventory["inventory_digest"]
    scene.update(robot_prim_path=snapshot["prim_path"], source_meters_per_unit=1, source_up_axis="Z")
    collision = check_fanuc_collision(action, scene, snapshot=snapshot)
    action["joint_trajectory"][0]["joint_positions"][0] += 1
    with pytest.raises(ValueError): prediction_handoff(action, world, collision, snapshot, scene)


def test_template_wires_memory_to_collision_prediction_and_guard():
    from pathlib import Path
    from backend.schemas import GraphRunRequest
    graph = json.loads((Path(__file__).resolve().parents[1] / "examples/graphs/isaacsim_fanuc_m710ic20l_graph.json").read_text(encoding="utf-8"))
    GraphRunRequest.model_validate(graph)
    nodes_by_id = {n["id"]: n for n in graph["nodes"]}
    assert nodes_by_id["fanuc-m710-guard"]["data"]["requireWorldContext"] is True
    assert nodes_by_id["fanuc-m710-collision"]["data"]["enabled"] is True
    assert any(e["source"] == "fanuc-m710-world-context" and e["target"] == "fanuc-m710-collision" and e["targetHandle"] == "scene" for e in graph["edges"])
