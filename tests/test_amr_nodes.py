import copy
import json
import time
from pathlib import Path
from types import SimpleNamespace

import pytest

from backend.executors import amr_nodes
from backend.runtime.runner import run_graph
from examples.isaacsim.mobile_world_model_bridge import MobileWorldModelAdapter
from test_amr_navigation import map_fixture, plan


@pytest.fixture
def live(monkeypatch):
    now = [1., 60]
    descriptors = [
        {"prim_path": "/World/AMR", "robot_kind": "amr", "base_motion": "planar",
         "mobile_base": {"drive_type": "differential", "footprint_radius_m": .2}},
        {"prim_path": "/World/FANUC", "robot_kind": "articulated_arm", "base_motion": "fixed",
         "manufacturer": "FANUC", "model": "M-710iC/20L"},
    ]
    bridge = SimpleNamespace(_scan_scene=lambda _: {
        "scene_id": "cell", "timestamp": time.time(), "truncated": False,
        "objects": [{"prim_path": "/World/FANUC/link1", "position": [1, 0, 0]},
                    {"prim_path": "/World/Box", "position": [5, 3, 0]}]})
    adapter = MobileWorldModelAdapter(bridge, {"robots": descriptors}, clock=lambda: tuple(now))
    adapter._identity = lambda: {"stage_identifier": "cell.usd", "meters_per_unit": 1., "up_axis": "Z"}
    def observe(descriptor):
        robot = {**copy.deepcopy(descriptor), "position": [0, 0, 0], "orientation_xyzw": [0, 0, 0, 1],
                 "linear_velocity": [0, 0, 0], "angular_velocity_rad_s": [0, 0, 0]}
        if robot.get("mobile_base"):
            robot["mobile_base"].update(localization_valid=True, navigation_frame="stage")
        return robot
    adapter.observe_robot = observe
    adapter._pose = lambda _: ((0, 0), 0, 0, 0)
    adapter._drive = lambda *args: None
    adapter.navigation_map = lambda _: map_fixture()
    adapter.config["navigation_map"] = map_fixture()
    def request(props, command, **fields):
        return adapter.apply({"command": command, "request_id": "test", **fields}, lambda _: pytest.fail("Legacy fallback"))
    monkeypatch.setattr(amr_nodes, "bridge_request", request)
    adapter.test_clock = now
    return adapter


def graph_fixture():
    path = Path(__file__).resolve().parents[1] / "examples/graphs/amr_live_navigation_graph.json"
    return json.loads(path.read_text(encoding="utf-8"))


def test_example_graph_collects_two_robots_plans_and_defaults_to_preview(live):
    result = run_graph(graph_fixture())["context"]
    assert result["observe"]["ok"]
    assert len(result["observe"]["world_state"]["robots"]) == 2
    assert result["observe"]["world_state"]["robots"][1]["model"] == "M-710iC/20L"
    assert result["observe"]["scan"]["objects"][0]["prim_path"] == "/World/Box"
    assert result["plan"]["ok"]
    assert result["navigate"]["started"] is False
    assert live.active_id is None


def test_enabled_graph_starts_async_motion_status_and_stop_are_connected(live):
    graph = graph_fixture()
    graph["nodes"][2]["data"].update(enableExecution=True, commandId="integration")
    result = run_graph(graph)["context"]
    assert result["navigate"]["ok"] and result["navigate"]["started"]
    assert not result["navigate"]["completed"]
    node = {"id": "status", "data": {"commandId": "integration"}}
    assert not amr_nodes.exec_amr_status(node, {}, [])["completed"]
    stopped = amr_nodes.exec_amr_stop(node, {}, [])
    assert stopped["execution"]["status"] == "cancelled"
    assert live.active_id is None


def test_failed_live_observation_does_not_become_zero_motion_or_valid_plan(live):
    live.observe_robot = lambda descriptor: {**descriptor, "position": [0, 0, 0], "observation_error": "physics missing"}
    result = run_graph(graph_fixture())["context"]
    assert result["observe"]["ok"]
    assert result["observe"]["world_state"]["robot"]["linear_velocity_m_s"] is None
    assert not result["plan"]["ok"]
    assert not result["navigate"]["started"]


def test_capture_rejects_mixed_physics_steps_and_idle_reset_changes_episode(live):
    original = live.navigation_map
    def advanced(path):
        live.test_clock[1] += 1
        return original(path)
    live.navigation_map = advanced
    with pytest.raises(ValueError, match="advanced"):
        live.capture("/World/AMR")
    old_episode = live.episode_id
    live.test_clock[:] = [0., 0]
    live.navigation_map = original
    assert live.capture("/World/AMR")["episode_id"] != old_episode


def test_transport_correlates_responses_and_closes_sockets(monkeypatch):
    import websocket
    seen = []
    class Socket:
        def send(self, value): self.request = json.loads(value)
        def recv(self): return json.dumps({"ok": True, "request_id": self.request["request_id"], "command": self.request["command"]})
        def close(self): seen.append("closed")
    monkeypatch.setattr(websocket, "create_connection", lambda *a, **k: Socket())
    # Call the real transport, independently of fixture replacement.
    assert amr_nodes.bridge_request({}, "amr_status", command_id="test")["ok"]
    assert seen == ["closed"]


def test_uncertain_mutation_is_not_replayed_on_another_endpoint(monkeypatch):
    import websocket
    calls = []
    class Socket:
        def send(self, value): calls.append(json.loads(value))
        def recv(self): raise websocket.WebSocketTimeoutException("lost reply")
        def close(self): pass
    monkeypatch.setattr(websocket, "create_connection", lambda *a, **k: Socket())
    with pytest.raises(ValueError, match="uncertain"):
        amr_nodes.bridge_request({}, "amr_navigate", command_id="test", plan=plan())
    assert len(calls) == 1


def test_mismatched_response_does_not_count_as_success(monkeypatch):
    import websocket
    class Socket:
        def send(self, value): pass
        def recv(self): return '{"ok": true, "command": "amr_status", "request_id": "other"}'
        def close(self): pass
    monkeypatch.setattr(websocket, "create_connection", lambda *a, **k: Socket())
    with pytest.raises(ValueError, match="identity mismatch"):
        amr_nodes.bridge_request({}, "amr_status", command_id="test")
