import copy

import pytest

from backend.executors import world_model_nodes as nodes
from backend.executors.world_model_nodes import FANUC_M710_PRIM_PATH
from backend.runtime.runner import execute_node
from test_fanuc_world_model_bridge import adapter, start


def observe_node():
    return {"id": "observe", "data": {"waitForCompletion": True, "pollIntervalSec": .05}}


def source_result(command_id="command"):
    return {"ok": True, "endpoint": "ws://selected-sim:9910/omniverse", "request": {
        "command": "apply_fanuc_trajectory", "command_id": command_id, "prim_path": FANUC_M710_PRIM_PATH}}


EDGES = [{"source": "execute", "sourceHandle": "result", "target": "observe", "targetHandle": "result"}]


def test_observer_waits_for_verified_completion_using_submitted_id(adapter, monkeypatch):
    start(adapter, "command")
    running = adapter.public_record("command")
    completed = {**running, "status": "completed", "completion_verified": True, "actual_state": adapter.snapshot()}
    responses, calls = iter([running, completed]), []
    def request(props, command, **fields):
        calls.append((props["omniverseEndpoint"], command, fields["command_id"]))
        return {"execution": next(responses)}
    monkeypatch.setattr(nodes, "_fanuc_observation_request", request)
    monkeypatch.setattr(nodes.time, "sleep", lambda _: None)
    result = nodes.exec_execution_observe(observe_node(), {"execute": {"result": source_result()}}, EDGES)
    assert result["ok"] and result["completed"]
    assert calls == [("ws://selected-sim:9910/omniverse", "get_command_status", "command")]*2


@pytest.mark.parametrize("result", [{"ok": False}, {"ok": True, "preview_only": True}, {"ok": True, "dry_run": True}])
def test_blocked_or_preview_execution_never_becomes_completion(result, monkeypatch):
    monkeypatch.setattr(nodes, "_fanuc_observation_request", lambda *a, **k: pytest.fail("No command was submitted"))
    output = nodes.exec_execution_observe(observe_node(), {"execute": {"result": result}}, EDGES)
    assert output["status"] == "not_started" and not output["completed"]


def test_wrong_robot_result_does_not_trigger_status_or_stop(monkeypatch):
    result = source_result()
    result["request"]["prim_path"] = "/World/OtherRobot"
    monkeypatch.setattr(nodes, "_fanuc_observation_request", lambda *a, **k: pytest.fail("Wrong robot accessed"))
    output = nodes.exec_execution_observe(observe_node(), {"execute": {"result": result}}, EDGES)
    assert not output["ok"] and not output["completed"]


def test_runtime_cancellation_callback_reaches_observer_and_targets_only_submitted_id(monkeypatch):
    calls = []
    monkeypatch.setattr(nodes, "_fanuc_observation_request", lambda props, command, **fields:
                        calls.append((command, fields)) or {"execution": {"status": "cancelled"}})
    output = execute_node(nodes.exec_execution_observe, observe_node(), {"execute": {"result": source_result()}},
                          EDGES, stop_requested=lambda: True)
    assert not output["completed"]
    assert calls == [("stop_command", {"command_id": "command", "prim_path": FANUC_M710_PRIM_PATH})]


def test_completion_timeout_requests_targeted_stop(adapter, monkeypatch):
    start(adapter, "command")
    calls = []
    def request(props, command, **fields):
        calls.append(command)
        return {"execution": adapter.public_record("command")}
    clock = iter([0., 2.])
    monkeypatch.setattr(nodes.time, "monotonic", lambda: next(clock))
    monkeypatch.setattr(nodes, "_fanuc_observation_request", request)
    node = observe_node()
    node["data"]["completionTimeoutSec"] = 1
    result = nodes.exec_execution_observe(node, {"execute": {"result": source_result()}}, EDGES)
    assert not result["ok"] and not result["completed"]
    assert calls == ["get_command_status", "stop_command"]


def test_targeted_stop_of_old_record_does_not_interrupt_new_motion(adapter):
    start(adapter, "old")
    adapter._cancel("old_interrupted")
    adapter.bridge.scene._active_motion = None
    start(adapter, "new")
    output = adapter.apply({"command": "stop_command", "command_id": "old", "prim_path": FANUC_M710_PRIM_PATH},
                           lambda _: pytest.fail("New motion was stopped"))
    assert output["ok"] and adapter.active_id == "new"


def test_targeted_stop_rejects_wrong_robot_and_stops_matching_motion(adapter):
    start(adapter, "command")
    wrong = adapter.apply({"command": "stop_command", "command_id": "command", "prim_path": "/World/Wrong"},
                          lambda _: pytest.fail("Wrong robot stop forwarded"))
    assert not wrong["ok"] and adapter.active_id == "command"
    def stop(payload):
        assert payload["command"] == "stop"
        adapter.bridge.scene._active_motion = None
        return {"ok": True}
    result = adapter.apply({"command": "stop_command", "command_id": "command", "prim_path": FANUC_M710_PRIM_PATH}, stop)
    assert result["execution"]["status"] == "cancelled" and adapter.active_id is None


def test_template_exports_are_generated_from_the_ui_source():
    from scripts.export_graph_templates import export
    export(check=True)
