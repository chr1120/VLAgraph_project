import copy
import json
from pathlib import Path

import pytest

from backend.executors import vla_nodes
from backend.executors.world_model_nodes import exec_execution_guard
from backend.world_model.fanuc_preflight import action_fingerprint, validate_fanuc_trajectory
from backend.runtime.runner import run_graph
from test_fanuc_preflight import action_fixture


def executable_action():
    action = action_fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    return action


def guard(action, *, report=None, policy="full"):
    return exec_execution_guard(
        {"id": "guard", "data": {"validationPolicy": policy}},
        {"prepare": {"action": action}, "validate": {"report": report if report is not None else validate_fanuc_trajectory(action)}},
        [{"source": "prepare", "target": "guard", "sourceHandle": "action", "targetHandle": "action"},
         {"source": "validate", "target": "guard", "sourceHandle": "report", "targetHandle": "report"}],
    )


def test_full_policy_blocks_unimplemented_physical_checks():
    result = guard(executable_action())
    assert result["approved"] is False
    assert result["action"]["type"] == "execution_blocked"
    assert any(item["code"] == "full_validation_unavailable" for item in result["decision"]["reasons"])


def test_static_simulation_can_forward_the_exact_enabled_action():
    action = executable_action()
    result = guard(action, policy="static_simulation")
    assert result["approved"] is True
    assert result["action"] == action
    assert result["action"] is not action
    assert result["decision"]["collision_checked"] is False


@pytest.mark.parametrize("case", ["missing_report", "stale_action", "forged_pass", "preview", "disabled", "not_ready", "unknown_policy"])
def test_bad_or_disabled_inputs_never_produce_approved_action(case):
    action = executable_action()
    report = validate_fanuc_trajectory(action)
    policy = "static_simulation"
    if case == "missing_report": report = {}
    if case == "stale_action": action["joint_trajectory"][1]["joint_positions"][0] += 1
    if case == "forged_pass":
        action["joint_trajectory"][1]["joint_positions"][0] = 999
        report["action_digest"] = action_fingerprint(action)
    if case == "preview": action["preview_only"] = True; report = validate_fanuc_trajectory(action)
    if case == "disabled": action["execution_enabled"] = False; report = validate_fanuc_trajectory(action)
    if case == "not_ready": action["execution_ready"] = False; report = validate_fanuc_trajectory(action)
    if case == "unknown_policy": policy = "skip_everything"
    assert guard(action, report=report, policy=policy)["approved"] is False


def pipeline(policy):
    return {
        "nodes": [
            {"id": "prepare", "type": "robot_state", "data": {"robotState": executable_action()}},
            {"id": "validate", "type": "fanuc_trajectory_validate", "data": {}},
            {"id": "guard", "type": "execution_guard", "data": {"validationPolicy": policy}},
            {"id": "execute", "type": "robot_execute", "data": {
                "mode": "omniverse", "dryRun": False, "requireExecutionGuard": True}},
        ],
        "edges": [
            {"source": "prepare", "target": "validate", "sourceHandle": "state", "targetHandle": "action"},
            {"source": "prepare", "target": "guard", "sourceHandle": "state", "targetHandle": "action"},
            {"source": "validate", "target": "guard", "sourceHandle": "report", "targetHandle": "report"},
            {"source": "guard", "target": "execute", "sourceHandle": "action", "targetHandle": "action"},
        ],
    }


@pytest.mark.parametrize("policy, expected_calls", [("full", 0), ("static_simulation", 1)])
def test_integrated_pipeline_only_dispatches_when_scope_allows(monkeypatch, policy, expected_calls):
    sent = []
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda data, action: sent.append(copy.deepcopy(action)) or {"ok": True})
    result = run_graph(pipeline(policy))
    assert len(sent) == expected_calls
    assert result["context"]["execute"]["ok"] is bool(expected_calls)


def test_bypassing_guard_is_blocked_before_transport(monkeypatch):
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: pytest.fail("Transport called"))
    graph = pipeline("static_simulation")
    graph["edges"][-1] = {"source": "prepare", "target": "execute", "sourceHandle": "state", "targetHandle": "action"}
    result = run_graph(graph)
    assert result["context"]["execute"]["skipped"] is True


def test_static_simulation_policy_cannot_dispatch_to_ros(monkeypatch):
    monkeypatch.setattr(vla_nodes, "_ros_execute", lambda *args: pytest.fail("ROS called"))
    graph = pipeline("static_simulation")
    graph["nodes"][-1]["data"]["mode"] = "ros"
    assert run_graph(graph)["context"]["execute"]["skipped"] is True


def test_action_changed_after_guard_cannot_reach_transport(monkeypatch):
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: pytest.fail("Transport called"))
    authorized = guard(executable_action(), policy="static_simulation")
    authorized["action"]["joint_trajectory"][1]["joint_positions"][0] += 1
    result = vla_nodes.exec_robot_execute(
        {"id": "execute", "data": {"mode": "omniverse", "requireExecutionGuard": True}},
        {"guard": authorized},
        [{"source": "guard", "target": "execute", "sourceHandle": "action", "targetHandle": "action"}],
    )
    assert result["skipped"] is True


def test_execute_json_template_has_no_prepare_to_execution_bypass():
    path = Path(__file__).resolve().parents[1] / "examples/graphs/isaacsim_fanuc_m710ic20l_graph.json"
    graph = json.loads(path.read_text(encoding="utf-8"))
    nodes = {node["id"]: node for node in graph["nodes"]}
    assert nodes["fanuc-m710-guard"]["data"]["validationPolicy"] == "full"
    assert nodes["fanuc-m710-execute"]["data"]["requireExecutionGuard"] is True
    action_inputs = [edge for edge in graph["edges"] if edge["target"] == "fanuc-m710-execute" and edge["targetHandle"] == "action"]
    assert len(action_inputs) == 1
    assert action_inputs[0]["source"] == "fanuc-m710-guard"
