import copy
from types import SimpleNamespace

import pytest

from examples.isaacsim.fanuc_world_model_bridge import FanucWorldModelAdapter
from scripts.install_fanuc_world_model_adapter import install
from backend.world_model.fanuc import FanucPhysicsSnapshot
from backend.executors.world_model_nodes import exec_execution_observe
from test_fanuc_observation import state_fixture


@pytest.fixture
def adapter():
    state = state_fixture()
    state["joint_positions_rad"] = [0.0] * 7
    state["joint_target_positions_rad"] = [0.0] * 7
    scene = SimpleNamespace(
        _active_motion=None, command_joint_names=state["joint_names"],
        get_vlagraph_state=lambda: copy.deepcopy(state),
        _articulation=SimpleNamespace(get_joint_velocities=lambda: [0.0] * 7),
    )
    clock = [1.0, 60]
    result = FanucWorldModelAdapter(SimpleNamespace(scene=scene), clock=lambda: tuple(clock))
    result._geometry = lambda: {"meters_per_unit": 1, "up_axis": "Z", "stage_identifier": "cell",
                                "physics_synchronized": False, "tcp_calibrated": False}
    result.test_clock = clock
    result.test_state = state
    return result


def start(adapter, command_id="test"):
    def original(payload):
        adapter.bridge.scene._active_motion = {"keyframes": [(0, [0] * 7), (2, [0] * 7)],
                                                "elapsed": 0, "duration": 2}
        return {"ok": True, "applied": True}
    request = {"command": "apply_fanuc_trajectory", "command_id": command_id, "action": {"joint_trajectory": [1]}}
    return adapter.apply(request, original), request


def test_snapshot_uses_command_order_and_physics_dof_velocity(adapter):
    adapter.bridge.scene._articulation.get_joint_velocities = lambda: [10, 1, 2, 3, 4, 5, 6]
    snapshot = adapter.snapshot()
    validated = FanucPhysicsSnapshot.model_validate(snapshot)
    assert [axis.velocity for axis in validated.axes] == [1, 2, 3, 4, 5, 6, 10]
    assert validated.sim_time == 1
    assert validated.geometry["physics_synchronized"] is False
    assert validated.prediction_ready is False


def test_scene_scan_retains_stage_identity_without_claiming_physics_synchronization(adapter):
    response = adapter.apply({"command": "scan_scene", "max_objects": 100},
                             lambda payload: {"ok": True, "scan": {"objects": []}})
    scan = response["scan"]
    assert scan["stage_identifier"] == "cell" and scan["sim_time"] == 1
    assert scan["meters_per_unit"] == 1 and scan["up_axis"] == "Z"
    assert scan["physics_synchronized"] is False and scan["truncated"] is False


def test_rejects_physics_step_changing_during_capture(adapter):
    def changed_geometry():
        adapter.test_clock[1] += 1
        return {"stage_identifier": "cell"}
    adapter._geometry = changed_geometry
    with pytest.raises(ValueError, match="advanced during capture"):
        adapter.snapshot()
    assert adapter.version == 0


def test_completion_requires_three_distinct_settled_physics_steps(adapter):
    start(adapter)
    motion = adapter.bridge.scene._active_motion
    motion["elapsed"] = 2
    adapter.bridge.scene._active_motion = None
    adapter.tick()
    adapter.tick()
    assert adapter.public_record("test")["status"] == "running"
    for _ in range(2):
        adapter.test_clock[1] += 1
        adapter.tick()
    record = adapter.public_record("test")
    assert record["status"] == "completed"
    assert record["completion_verified"] is True
    assert record["outcome"] == "unknown"
    assert record["actual_state"]["episode_id"] == record["episode_id"]


def test_idle_after_interruption_is_not_completed(adapter):
    start(adapter)
    adapter.bridge.scene._active_motion = None
    adapter.tick()
    assert adapter.public_record("test")["status"] == "cancelled"


def test_high_velocity_prevents_completion(adapter):
    start(adapter)
    adapter.bridge.scene._active_motion["elapsed"] = 2
    adapter.bridge.scene._active_motion = None
    adapter.bridge.scene._articulation.get_joint_velocities = lambda: [0.5] * 7
    for _ in range(5):
        adapter.test_clock[1] += 1
        adapter.tick()
    assert adapter.public_record("test")["status"] == "running"


def test_command_retry_does_not_execute_twice_and_changed_payload_rejected(adapter):
    _, request = start(adapter)
    response = adapter.apply(request, lambda _: pytest.fail("Duplicate executed"))
    assert response["duplicate"] is True
    assert response["applied"] is False
    request["action"] = {"different": True}
    with pytest.raises(ValueError, match="different action"):
        adapter.apply(request, lambda _: None)


def test_reset_cancels_command_and_starts_new_episode(adapter):
    start(adapter)
    old_episode = adapter.episode_id
    adapter.test_clock[:] = [0, 0]
    adapter.tick()
    assert adapter.public_record("test")["status"] == "cancelled"
    assert adapter.episode_id != old_episode


def test_unknown_old_command_is_not_inferred_from_idle(adapter):
    with pytest.raises(ValueError, match="Unknown command_id"):
        adapter.public_record("old")


def test_execution_node_rejects_unverified_completion(adapter, monkeypatch):
    start(adapter)
    record = adapter.public_record("test")
    record["status"] = "completed"
    monkeypatch.setattr("backend.executors.world_model_nodes._fanuc_observation_request",
                        lambda *args, **kwargs: {"execution": record})
    result = exec_execution_observe({"id": "observe", "data": {"commandId": "test"}}, {}, [])
    assert result["ok"] is False
    assert result["completed"] is False


def test_installer_preserves_backup_and_is_idempotent(tmp_path):
    package = tmp_path / "python/isaac_fanuc_m710"
    package.mkdir(parents=True)
    original = b'class Bridge:\n    def __init__(self):\n        self._last_state_ok_at = 0.0\n'
    target = package / "vlagraph_bridge.py"
    target.write_bytes(original)
    first = install(tmp_path)
    installed = target.read_bytes()
    second = install(tmp_path)
    assert first == second
    assert target.read_bytes() == installed
    assert target.with_suffix('.py.vlagraph-world-model.bak').read_bytes() == original
    assert installed.count(b'attach_world_model(self)') == 1
    assert (package / "vlagraph_usd_geometry.py").exists()
    assert len(first["geometry_sha256"]) == 64


def test_robot_execute_forwards_explicit_command_id(monkeypatch):
    from backend.executors import vla_nodes

    sent = []
    class Socket:
        def settimeout(self, value): pass
        def send(self, value):
            import json
            sent.append(json.loads(value))
        def close(self): pass
    monkeypatch.setattr(vla_nodes, "create_connection", lambda *args, **kwargs: Socket())
    result = vla_nodes._omniverse_execute(
        {"omniverseEndpoint": "ws://localhost:9910/omniverse", "omniverseWaitForResponse": False,
         "omniverseCommand": "apply_fanuc_trajectory", "commandId": "episode-1-step-1"},
        {"command": "apply_fanuc_trajectory", "joint_trajectory": [{"time_sec": 2, "joint_positions": [0] * 6}]},
    )
    assert result["ok"] is True
    assert sent[0]["command_id"] == "episode-1-step-1"
