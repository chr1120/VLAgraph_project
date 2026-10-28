import copy
import threading
import time
from types import SimpleNamespace

import pytest

from backend.world_model.amr_navigation import fingerprint, plan_path, routes_conflict
from backend.world_model.multi_robot import build_inference_batch, infer_batch, select_candidate
from backend.world_model.state_builder import build_world_state
from backend.executors.multi_robot_nodes import exec_robot_candidate_select
from examples.isaacsim.mobile_world_model_bridge import MobileWorldModelAdapter
from test_amr_navigation import map_fixture


def scene():
    robots = []
    for name, y in (("AMR1", 0), ("AMR2", 3)):
        robots.append({"prim_path": f"/World/{name}", "position": [0, y, 0],
                       "robot_kind": "amr", "base_motion": "planar",
                       "orientation_xyzw": [0, 0, 0, 1], "linear_velocity": [0, 0, 0],
                       "angular_velocity_rad_s": [0, 0, 0],
                       "mobile_base": {"drive_type": "differential", "footprint_radius_m": .2,
                                       "navigation_frame": "stage", "localization_valid": True}})
    for name, y in (("FANUC1", 0), ("FANUC2", 3)):
        robots.append({"prim_path": f"/World/{name}", "position": [5, y, 0],
                       "robot_kind": "articulated_arm", "manufacturer": "FANUC", "model": "M-710iC/20L",
                       "base_motion": "fixed", "orientation_xyzw": [0, 0, 0, 1],
                       "joints": [{"name": f"J{i}", "kind": "revolute", "unit": "rad", "position": 0., "velocity": 0.}
                                  for i in range(1, 7)]})
    return {"scene_id": "cell", "stage_identifier": "cell.usd", "meters_per_unit": 1,
            "up_axis": "Z", "timestamp": time.time(), "sim_time": 1., "truncated": False,
            "episode_id": "episode", "robots": robots, "objects": []}


def world():
    return build_world_state(scene(), episode_id="episode", scene_version="v1", state_version=1).model_dump(mode="json")


def assignments():
    bindings, tasks, sensors = [], [], {}
    for name, y in (("AMR1", 0), ("AMR2", 3)):
        path = f"/World/{name}"
        bindings.append({"robot_prim_path": path, "robot_kind": "amr", "provider": "amr_grid",
                         "action_schema": "vlagraph.amr_plan.v1"})
        tasks.append({"robot_prim_path": path, "instruction": "Navigate to assigned goal", "goal_m": [3, y]})
    for index in (1, 2):
        path = f"/World/FANUC{index}"
        bindings.append({"robot_prim_path": path, "robot_kind": "articulated_arm", "provider": "openvla",
                         "model_name": "arm-model", "endpoint": "http://arm-policy",
                         "action_schema": "arm_policy_candidate.v1"})
        tasks.append({"robot_prim_path": path, "instruction": f"Inspect station {index}"})
        sensors[path] = {"robot_prim_path": path, "episode_id": "episode", "scene_version": "v1",
                         "sim_time": 1., "observed_at": time.time(), "image": f"image-{index}"}
    return bindings, tasks, sensors


def make_batch(state=None):
    bindings, tasks, sensors = assignments()
    return build_inference_batch(state or world(), bindings, tasks, sensors=sensors,
                                 navigation_maps={f"/World/AMR{i}": map_fixture() for i in (1, 2)})


def test_parallel_mixed_inference_keeps_instance_tasks_images_and_shared_world(monkeypatch):
    barrier, payloads = threading.Barrier(2), []
    def request(url, *, json, timeout):
        payloads.append(copy.deepcopy(json))
        barrier.wait(timeout=2)  # Proves the two arm policies overlap in flight.
        return SimpleNamespace(raise_for_status=lambda: None, json=lambda: {
            "action": {"robot_prim_path": json["robot_prim_path"], "joint_targets": [0]*6}})
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", request)
    state = world()
    batch = make_batch(state)
    original = copy.deepcopy(batch)
    result = infer_batch(batch)
    assert result["ok"] and len(result["candidates"]) == 4
    assert batch == original
    by_path = {payload["robot_prim_path"]: payload for payload in payloads}
    for index in (1, 2):
        payload = by_path[f"/World/FANUC{index}"]
        assert payload["image"] == f"image-{index}"
        assert payload["task_instruction"] == f"Inspect station {index}"
        assert f"Task for robot /World/FANUC{index}: Inspect station {index}" in payload["prompt"]
        assert "/World/AMR1" in payload["prompt"] and "/World/AMR2" in payload["prompt"]
        assert payload["state"]["robot"]["prim_path"] == f"/World/FANUC{index}"
        assert len(payload["state"]["world_state"]["robots"]) == 4
        assert "robot" not in payload["state"]["world_state"]
    assert all(not item["execution_ready"] for item in result["candidates"])
    candidate, selected = select_candidate(result, state, "/World/AMR2")
    assert selected.robot.prim_path == "/World/AMR2"
    assert candidate["action"]["robot_prim_path"] == "/World/AMR2"
    assert candidate["action"]["goal_m"] == [3., 3.]


@pytest.mark.parametrize("change", ["wrong_kind", "duplicate_task", "wrong_camera", "stale_camera", "wrong_step", "missing_binding"])
def test_context_rejects_ambiguous_or_mismatched_robot_inputs(change):
    bindings, tasks, sensors = assignments()
    if change == "wrong_kind": bindings[0]["robot_kind"] = "humanoid"
    if change == "duplicate_task": tasks.append(copy.deepcopy(tasks[0]))
    if change == "wrong_camera": sensors["/World/FANUC1"]["robot_prim_path"] = "/World/FANUC2"
    if change == "stale_camera": sensors["/World/FANUC1"]["observed_at"] -= 20
    if change == "wrong_step": sensors["/World/FANUC1"]["sim_time"] = 20
    if change == "missing_binding": bindings.pop()
    with pytest.raises(ValueError):
        build_inference_batch(world(), bindings, tasks, sensors=sensors,
                              navigation_maps={f"/World/AMR{i}": map_fixture() for i in (1, 2)})


def test_one_provider_failure_is_reported_per_robot_and_blocks_coordinated_selection(monkeypatch):
    def request(url, *, json, timeout):
        if json["robot_prim_path"] == "/World/FANUC2":
            raise RuntimeError("policy unavailable")
        return SimpleNamespace(raise_for_status=lambda: None, json=lambda: {"action": {"joint_targets": [0]*6}})
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", request)
    state = world()
    result = infer_batch(make_batch(state))
    assert not result["ok"]
    assert [item["ok"] for item in result["candidates"]] == [True, True, True, False]
    output = exec_robot_candidate_select({"id": "select", "data": {"robotPrimPath": "/World/AMR1"}},
        {"source": {"candidates": result}, "world": {"world_state": state}},
        [{"source": "source", "sourceHandle": "candidates", "target": "select", "targetHandle": "candidates"},
         {"source": "world", "sourceHandle": "world_state", "target": "select", "targetHandle": "world_state"}])
    assert not output["ok"] and output["plan"] is None


def test_candidates_cannot_be_applied_to_another_world_version(monkeypatch):
    monkeypatch.setattr("backend.world_model.multi_robot.infer_one", lambda *a: {"action": {"joint_targets": [0]*6}})
    batch = make_batch()
    # Use only the arm jobs because the mock deliberately returns arm actions.
    batch["jobs"] = batch["jobs"][2:]
    batch["digest"] = fingerprint({k: v for k, v in batch.items() if k != "digest"})
    result = infer_batch(batch)
    changed = world()
    changed["state_version"] += 1
    with pytest.raises(ValueError, match="current shared world"):
        select_candidate(result, changed, "/World/FANUC1")


def test_model_cannot_redirect_its_candidate_to_another_robot(monkeypatch):
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", lambda *a, **k: SimpleNamespace(
        raise_for_status=lambda: None, json=lambda: {"action": {"robot_prim_path": "/World/AMR1"}}))
    result = infer_batch(make_batch())
    assert not result["candidates"][2]["ok"]
    assert "different robot" in result["candidates"][2]["message"]


def fleet():
    config = {"robots": [r for r in scene()["robots"] if r["robot_kind"] == "amr"]}
    clock, drives = [1., 60], []
    adapter = MobileWorldModelAdapter(SimpleNamespace(), config, clock=lambda: tuple(clock))
    adapter.episode_id = "episode"
    adapter._identity = lambda: {"stage_identifier": "cell.usd", "meters_per_unit": 1., "up_axis": "Z"}
    adapter.navigation_map = lambda _: map_fixture()
    adapter._pose = lambda p: ((0, 0 if p.endswith("1") else 3), 0, 0, 0)
    adapter._drive = lambda p, v, w: drives.append((p, v, w))
    return adapter, clock, drives


def start_two(adapter):
    for index, y in ((1, 0), (2, 3)):
        path = f"/World/AMR{index}"
        plan = plan_path(map_fixture(), [0, y], [3, y], robot_prim_path=path,
                         radius_m=.2, episode_id=adapter.episode_id, observed_at=time.time())
        adapter.navigate({"command_id": f"command-{index}", "plan": plan})


def test_two_amrs_advance_same_step_and_stop_is_instance_specific():
    adapter, clock, drives = fleet()
    start_two(adapter)
    assert len(adapter.active_commands) == 2
    clock[:] = [1.02, 61]
    adapter.tick()
    assert {p for p, v, _ in drives if v > 0} == {"/World/AMR1", "/World/AMR2"}
    adapter.apply({"command": "amr_stop", "command_id": "command-1"}, lambda _: None)
    assert adapter.public_record("command-1")["status"] == "cancelled"
    assert adapter.public_record("command-2")["status"] == "running"
    assert adapter.active_commands == {"/World/AMR2": "command-2"}


def test_one_amr_observation_failure_does_not_stop_other_robots():
    adapter, clock, drives = fleet()
    start_two(adapter)
    original = adapter._pose
    def pose(path):
        if path.endswith("1"):
            raise ValueError("sensor failed")
        return original(path)
    adapter._pose = pose
    clock[:] = [1.02, 61]
    adapter.tick()
    assert adapter.public_record("command-1")["status"] == "failed"
    assert adapter.public_record("command-2")["status"] == "running"
    assert any(p == "/World/AMR2" and v > 0 for p, v, _ in drives)


def test_fanuc_command_does_not_cancel_amr_commands_and_reset_cancels_all():
    adapter, clock, _ = fleet()
    start_two(adapter)
    response = adapter.apply({"command": "apply_fanuc_trajectory", "prim_path": "/World/FANUC1"}, lambda _: {"ok": True})
    assert response["ok"] and len(adapter.active_commands) == 2
    clock[:] = [0., 0]
    adapter.tick()
    assert not adapter.active_commands
    assert all(adapter.public_record(f"command-{i}")["status"] == "cancelled" for i in (1, 2))


def test_crossing_route_reservation_rejects_second_amr():
    first = plan_path(map_fixture(), [0, 0], [3, 0], robot_prim_path="/World/AMR1", radius_m=.2,
                      episode_id="episode", observed_at=time.time())
    second = plan_path(map_fixture(), [1, -1], [1, 1], robot_prim_path="/World/AMR2", radius_m=.2,
                       episode_id="episode", observed_at=time.time())
    assert routes_conflict(first, second)
    adapter, _, _ = fleet()
    adapter.navigate({"command_id": "first", "plan": first})
    with pytest.raises(ValueError, match="reserved route"):
        adapter.navigate({"command_id": "second", "plan": second})
    assert len(adapter.active_commands) == 1


def test_live_example_graph_runs_world_through_four_inferences_and_robot_routes(monkeypatch):
    import json
    from pathlib import Path
    from backend.executors import amr_nodes
    from backend.runtime.runner import run_graph
    from test_fanuc_preflight import action_fixture

    scan = scene()
    scan["navigation_maps"] = {f"/World/AMR{i}": map_fixture() for i in (1, 2)}
    monkeypatch.setattr(amr_nodes, "bridge_request", lambda props, command, **fields: {"ok": True, "scan": scan})
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", lambda *a, **k: SimpleNamespace(
        raise_for_status=lambda: None, json=action_fixture_response))
    graph = json.loads((Path(__file__).resolve().parents[1]/"examples/graphs/multi_robot_world_inference_graph.json").read_text())
    output = run_graph(graph)["context"]
    assert output["infer"]["ok"]
    for name in ("AMR1", "AMR2", "FANUC1", "FANUC2"):
        assert output["select-"+name]["ok"]
        assert output["select-"+name]["world_state"]["robot"]["prim_path"] == "/World/"+name
    assert not output["navigate-AMR1"]["started"]
    assert not output["navigate-AMR2"]["started"]
    assert output["select-FANUC1"]["action"]["preview_only"] is True


def action_fixture_response():
    from test_fanuc_preflight import action_fixture
    return {"action": action_fixture()}


def policy_inputs(monkeypatch):
    from backend.executors.robot_setup_nodes import exec_robot_setup
    from test_fanuc_preflight import action_fixture
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", lambda *a, **k: SimpleNamespace(
        raise_for_status=lambda: None, json=action_fixture_response))
    state = world()
    batch = make_batch(state)
    for job in batch["jobs"]:
        if job["robot"]["robot_kind"] == "articulated_arm":
            job["binding"]["action_schema"] = "vlagraph.fanuc_m710ic20l.action.v1"
    batch["digest"] = fingerprint({k: v for k, v in batch.items() if k != "digest"})
    result = infer_batch(batch)
    candidate, selected = select_candidate(result, state, "/World/FANUC1")
    config = exec_robot_setup({"id": "setup", "data": {
        "instanceId": "fanuc_1", "robotPrimPath": "/World/FANUC1",
        "omniverseEndpoint": "ws://localhost:9910/omniverse"}}, {}, [])["robot_config"]
    context = {"candidate": {"candidate": candidate}, "world": {"world_state": selected.model_dump(mode="json")},
               "setup": {"robot_config": config}}
    edges = [{"source": source, "sourceHandle": handle, "target": "prepare", "targetHandle": handle}
             for source, handle in (("candidate", "candidate"), ("world", "world_state"), ("setup", "robot_config"))]
    return context, edges


def test_fanuc_policy_adapter_preserves_preview_and_existing_execution_guard(monkeypatch):
    from backend.executors.multi_robot_nodes import exec_fanuc_policy_prepare
    from backend.executors.world_model_nodes import exec_execution_guard
    context, edges = policy_inputs(monkeypatch)
    context["candidate"]["candidate"]["action"].update(execution_enabled=True, execution_ready=True, preview_only=False)
    preview = exec_fanuc_policy_prepare({"id": "prepare", "data": {}}, context, edges)
    assert preview["ok"], preview
    assert preview["action"]["preview_only"] is True
    enabled = exec_fanuc_policy_prepare({"id": "prepare", "data": {"enableExecution": True}}, context, edges)
    assert enabled["ok"] and enabled["requires_execution_guard"]
    assert enabled["action"]["robot_prim_path"] == "/World/FANUC1"
    guard_context = {"prepare": enabled, "setup": context["setup"]}
    guard_edges = [{"source": "prepare", "sourceHandle": handle, "target": "guard", "targetHandle": handle}
                   for handle in ("action", "report")]
    guard_edges.append({"source": "setup", "sourceHandle": "robot_config", "target": "guard", "targetHandle": "robot_config"})
    full = exec_execution_guard({"id": "guard", "data": {"validationPolicy": "full"}}, guard_context, guard_edges)
    assert not full["approved"]
    static = exec_execution_guard({"id": "guard", "data": {"validationPolicy": "static_simulation"}}, guard_context, guard_edges)
    assert static["approved"]


def test_fanuc_policy_adapter_rejects_wrong_instance_and_joint_limit_violation(monkeypatch):
    from backend.executors.multi_robot_nodes import exec_fanuc_policy_prepare
    context, edges = policy_inputs(monkeypatch)
    context["candidate"]["candidate"]["robot_prim_path"] = "/World/FANUC2"
    wrong = exec_fanuc_policy_prepare({"id": "prepare", "data": {"enableExecution": True}}, context, edges)
    assert not wrong["ok"] and wrong["action"] is None
    context["candidate"]["candidate"]["robot_prim_path"] = "/World/FANUC1"
    context["candidate"]["candidate"]["action"]["joint_trajectory"][0]["joint_positions"][0] = 999
    invalid = exec_fanuc_policy_prepare({"id": "prepare", "data": {"enableExecution": True}}, context, edges)
    assert not invalid["ok"] and invalid["action"] is None


def test_whole_fleet_stop_attempts_every_robot_even_if_one_controller_is_unavailable():
    adapter, _, drives = fleet()
    start_two(adapter)
    def drive(path, v, w):
        if path.endswith("1"):
            raise ValueError("AMR1 controller unavailable")
        drives.append((path, v, w))
    adapter._drive = drive
    with pytest.raises(ValueError):
        adapter.finish("cancelled", "shutdown")
    assert adapter.public_record("command-1")["status"] == "stopping"
    assert adapter.public_record("command-2")["status"] == "cancelled"


def test_two_sessions_share_scene_but_cancel_only_their_own_amr(monkeypatch):
    import asyncio
    import json
    from pathlib import Path
    from backend.executors import amr_nodes
    from backend.runtime.session_manager import GraphRuntimeManager

    adapter, _, _ = fleet()
    scan = scene()
    scan["navigation_maps"] = {f"/World/AMR{i}": map_fixture() for i in (1, 2)}
    def request(props, command, **fields):
        if command == "capture_robot_observations":
            return {"ok": True, "scan": copy.deepcopy(scan)}
        return adapter.apply({"command": command, **fields}, lambda _: None)
    monkeypatch.setattr(amr_nodes, "bridge_request", request)
    template = json.loads((Path(__file__).resolve().parents[1]/"examples/graphs/multi_robot_world_inference_graph.json").read_text())
    def graph(name):
        value = copy.deepcopy(template)
        allowed = {"observe", "context", "infer", "select-"+name, "navigate-"+name}
        value["nodes"] = [n for n in value["nodes"] if n["id"] in allowed]
        value["edges"] = [e for e in value["edges"] if e["source"] in allowed and e["target"] in allowed]
        for node in value["nodes"]:
            if node["id"] == "context":
                for key in ("robotBindings", "robotTasks"):
                    node["data"][key] = [v for v in node["data"][key] if v["robot_prim_path"] == "/World/"+name]
            if node["type"] == "amr_navigate":
                node["data"]["enableExecution"] = True
        return value
    async def until(predicate):
        deadline = time.monotonic()+3
        while not predicate():
            assert time.monotonic() < deadline
            await asyncio.sleep(.005)
    async def run():
        manager = GraphRuntimeManager()
        first = (await manager.start_session(graph("AMR1")))["session_id"]
        first_task = manager._sessions[first]["task"]
        await until(lambda: "/World/AMR1" in adapter.active_commands)
        second = (await manager.start_session(graph("AMR2")))["session_id"]
        second_task = manager._sessions[second]["task"]
        try:
            await until(lambda: len(adapter.active_commands) == 2)
            await manager.cancel_session(first)
            await asyncio.wait_for(first_task, 3)
            assert "/World/AMR1" not in adapter.active_commands
            assert "/World/AMR2" in adapter.active_commands
            assert manager.get_session(first)["status"] == "cancelled"
        finally:
            await manager.cancel_session(second)
            await asyncio.wait_for(second_task, 3)
        assert not adapter.active_commands
    asyncio.run(run())


def test_gr00t_receives_explicit_measured_joint_groups_without_g1_defaults(monkeypatch):
    state = world()
    state["robots"][2]["joints"][0]["position"] = .2
    binding = {"robot_prim_path": "/World/FANUC1", "robot_kind": "articulated_arm", "provider": "gr00t",
               "model_name": "fanuc-trained-policy", "endpoint": "http://fanuc-policy",
               "embodiment_tag": "fanuc_joint_space", "action_schema": "arm_policy_candidate.v1",
               "state_joint_groups": {"joint_positions": ["J2", "J1"]}, "require_image": False}
    batch = build_inference_batch(state, [binding], [{"robot_prim_path": "/World/FANUC1", "instruction": "Inspect"}])
    payloads = []
    def request(url, *, json, timeout):
        payloads.append(copy.deepcopy(json))
        return SimpleNamespace(raise_for_status=lambda: None, json=lambda: {"action": [0]*6})
    monkeypatch.setattr("backend.world_model.multi_robot.requests.post", request)
    assert infer_batch(batch)["ok"]
    assert payloads[0]["state"]["joint_positions"] == [0., .2]
    assert payloads[0]["embodiment_tag"] == "fanuc_joint_space"
    assert payloads[0]["require_complete_state"] is True
    assert "REAL_G1" not in str(payloads[0])
