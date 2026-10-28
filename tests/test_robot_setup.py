import copy
import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import numpy as np
import pytest

from backend.executors.robot_setup_nodes import exec_robot_setup
from backend.executors.fanuc_nodes import _m710_fk, _solve_m710_ik, exec_fanuc_ls_parse, exec_fanuc_m710_prepare
from backend.executors.world_model_nodes import exec_execution_guard
from backend.executors import vla_nodes
from backend.world_model.robot_profiles import M710_PROFILE, validate_profile, validate_robot_config, configured_data
from backend.world_model.fanuc_preflight import validate_fanuc_trajectory, action_fingerprint
from backend.world_model.fanuc_collision import check_fanuc_collision
from test_fanuc_preflight import action_fixture
from test_fanuc_collision import scene_fixture
from test_fanuc_nodes import JOINT_ONLY_LS


def setup(**extra):
    props = {"robotModel": "fanuc_m710ic20l", "settingsMode": "preset", "instanceId": "arm-a",
             "omniverseEndpoint": "ws://localhost:9910/omniverse", "robotPrimPath": "/World/Robot"}
    return exec_robot_setup({"id": "setup", "data": {**props, **extra}}, {}, [])


def custom():
    profile = copy.deepcopy(M710_PROFILE)
    profile.update(id="synthetic_six_axis", model="Synthetic test arm", source="synthetic_fixture")
    profile["origins_m"][4][0] = 1.2
    profile["limits_deg"][0] = [-5, 5]
    return profile


def inputs(node_id, values):
    context = {key: {key: value} for key, value in values.items()}
    edges = [{"source": key, "sourceHandle": key, "target": node_id, "targetHandle": key} for key in values]
    return context, edges


def test_builtin_and_exported_profile_match_and_connection_overrides_old_node_values():
    config = setup()["robot_config"]
    assert config["profile"] == M710_PROFILE and validate_robot_config(config) == config
    path = Path(__file__).resolve().parents[1] / "frontend/public/robot_profiles/fanuc_m710ic20l.json"
    assert json.loads(path.read_text(encoding="utf-8")) == M710_PROFILE
    ctx, edges = inputs("read", {"robot_config": config})
    data, _ = configured_data({"id": "read", "data": {"omniverseEndpoint": "ws://wrong"}}, ctx, edges, "read")
    assert data["omniverseEndpoint"] == config["connection"]["endpoint"]
    assert data["robotPrimPath"] == "/World/Robot"


@pytest.mark.parametrize("change", [
    {"robotModel": "custom_fanuc_6axis"}, {"settingsMode": "unknown"},
    {"settingsMode": "manual", "manualProfile": {}},
    {"prepareSettings": {"enableExecution": True}}, {"prepareSettings": {"validationPolicy": "static_simulation"}},
    {"omniverseEndpoint": "http://localhost"}, {"robotPrimPath": "relative/path"},
    {"settingsMode": "auto", "usdPath": "no-such-model.usd"},
    {"prepareSettings": {"externalAxisRootMotionEnabled": "false"}},
    {"prepareSettings": {"trajectoryTimeScale": -1}},
    {"prepareSettings": {"uframeTransforms": {"1": {"xyzMm": [0, 0, 0]}}}},
])
def test_incomplete_or_execution_overriding_settings_do_not_fall_back(change):
    result = setup(**change)
    assert result["ok"] is False and result["robot_config"] is None


@pytest.mark.parametrize("field,value", [("axes", [[0, 0, 0]]*6), ("joint_names", ["J1"]*6),
                                         ("tool_offset_m", [True, 0, 0]), ("limits_deg", [[2, 1]]*6)])
def test_invalid_custom_structures_are_rejected(field, value):
    profile = custom()
    profile[field] = value
    with pytest.raises(ValueError): validate_profile(profile)


def test_builtin_identity_cannot_be_reused_for_different_geometry():
    profile = copy.deepcopy(M710_PROFILE)
    profile["origins_m"][0][2] += .1
    with pytest.raises(ValueError): validate_profile(profile)


def test_fk_and_ik_use_each_profile_without_global_mutation():
    profile = custom()
    q = np.zeros(6)
    with ThreadPoolExecutor() as pool:
        builtin, changed = list(pool.map(lambda p: _m710_fk(q, p), [M710_PROFILE, profile]))
    assert builtin[0][0] - changed[0][0] == pytest.approx(.6)
    solved = _solve_m710_ik(*changed, q, profile=profile)
    assert solved["ok"] and solved["position_error_m"] < 1e-9
    np.testing.assert_allclose(_m710_fk(q)[0], builtin[0])


def test_prepare_action_preflight_collision_and_execution_use_selected_profile(tmp_path, monkeypatch):
    profile = custom()
    config = setup(robotModel="custom_fanuc_6axis", settingsMode="manual", manualProfile=json.dumps(profile),
                   prepareSettings={"timingMode": "fixed_total", "totalDurationSec": 4})["robot_config"]
    path = tmp_path / "motion.ls"
    path.write_text(JOINT_ONLY_LS.replace("CNT50", "FINE"), encoding="utf-8")
    parsed = exec_fanuc_ls_parse({"id": "p", "data": {"filePath": str(path)}}, {}, [])
    ctx, edges = inputs("prepare", {"robot_config": config, "program": parsed["program"]})
    prepared = exec_fanuc_m710_prepare({"id": "prepare", "data": {}}, ctx, edges)
    action = prepared["action"]
    assert action["robot_profile"] == profile and action["schema"] == "vlagraph.fanuc_serial_arm.action.v1"
    report = validate_fanuc_trajectory(action)
    assert not report["static_checks_passed"]  # source J1=10/-15 exceeds the selected +/-5 degree limits
    legacy = copy.deepcopy(action)
    legacy.pop("robot_profile")
    legacy["schema"] = "vlagraph.fanuc_m710ic20l.action.v1"
    assert validate_fanuc_trajectory(legacy)["static_checks_passed"]
    with pytest.raises(ValueError, match="different robot profiles"):
        check_fanuc_collision(action, scene_fixture())
    ctx, edges = inputs("g", {"robot_config": config, "action": action, "report": report})
    decision = exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, ctx, edges)
    assert not decision["approved"]
    assert any(r["code"] == "robot_profile_execution_unverified" for r in decision["decision"]["reasons"])
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *a: pytest.fail("Unverified robot dispatched"))
    ctx, edges = inputs("e", {"robot_config": config, "action": action})
    assert vla_nodes.exec_robot_execute({"id": "e", "data": {"mode": "omniverse"}}, ctx, edges)["skipped"]
    ctx, edges = inputs("e", {"action": action})
    assert vla_nodes.exec_robot_execute({"id": "e", "data": {"mode": "omniverse"}}, ctx, edges)["skipped"]


def test_config_change_or_disconnected_config_invalidates_prepared_action():
    config = setup()["robot_config"]
    action = action_fixture()
    action.update(robot_profile=M710_PROFILE, robot_config_digest=config["digest"], robot_instance_id="arm-a",
                  robot_prim_path="/World/Robot", preview_only=False, execution_enabled=True, execution_ready=True)
    report = validate_fanuc_trajectory(action)
    changed = setup(instanceId="arm-b")["robot_config"]
    for selected in (None, changed):
        values = {"action": action, "report": report}
        if selected: values["robot_config"] = selected
        ctx, edges = inputs("g", values)
        result = exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, ctx, edges)
        assert not result["approved"]
    bad = copy.deepcopy(config)
    bad["connection"]["robot_prim_path"] = "/World/Other"
    with pytest.raises(ValueError): validate_robot_config(bad)


def test_custom_scene_changes_collision_result_instead_of_using_m710_chain():
    action = action_fixture()
    profile = custom()
    profile["limits_deg"][0] = [-180, 180]
    action["robot_profile"] = profile
    scene = scene_fixture()
    scene.update(robot_profile=profile, kinematics_source="serial_arm_profile")
    result = check_fanuc_collision(action, scene)
    assert result["status"] == "clear"
    assert result["robot_profile_digest"] == action_fingerprint(profile)
    assert check_fanuc_collision(action_fixture(), scene_fixture())["status"] == "collision"


def test_usd_auto_read_extracts_geometry_and_refuses_unknown_tcp(tmp_path):
    pytest.importorskip("pxr")
    from test_fanuc_usd_proxy import stage_fixture
    path = tmp_path / "robot.usda"
    stage = stage_fixture()
    stage.GetRootLayer().Export(str(path))
    result = setup(settingsMode="auto", usdPath=str(path))
    assert result["ok"] and result["robot_config"]["profile"] == M710_PROFILE
    params = {"robotModel": "custom_fanuc_6axis", "settingsMode": "auto", "usdPath": str(path),
              "customProfileId": "derived_fixture", "customModelName": "USD fixture"}
    assert not setup(**params)["ok"]
    result = setup(**params, toolOffsetM=[.1, 0, 0])
    assert result["ok"] and result["summary"]["execution_verified"] is False
    assert result["profile"]["origins_m"][2][2] == pytest.approx(1.15)


def test_connected_missing_setup_never_uses_legacy_defaults():
    ctx, edges = inputs("read", {"robot_config": None})
    with pytest.raises(ValueError): configured_data({"id": "read", "data": {}}, ctx, edges, "read")


def test_verified_builtin_static_scope_routes_mock_transport_to_selected_instance(monkeypatch):
    config = setup()["robot_config"]
    action = action_fixture()
    action.update(robot_profile=M710_PROFILE, robot_config_digest=config["digest"], robot_instance_id="arm-a",
                  robot_prim_path="/World/Robot", preview_only=False, execution_enabled=True, execution_ready=True)
    ctx, edges = inputs("g", {"robot_config": config, "action": action, "report": validate_fanuc_trajectory(action)})
    guarded = exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, ctx, edges)
    assert guarded["approved"]
    calls = []
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda data, action: calls.append(data) or {"ok": True})
    result = vla_nodes.exec_robot_execute({"id": "e", "data": {"mode": "omniverse", "omniverseEndpoint": "ws://wrong"}},
        {"setup": {"robot_config": config}, "guard": guarded},
        [{"source": "setup", "sourceHandle": "robot_config", "target": "e", "targetHandle": "robot_config"},
         {"source": "guard", "sourceHandle": "action", "target": "e", "targetHandle": "action"}])
    assert result["ok"] and len(calls) == 1
    assert calls[0]["omniverseEndpoint"] == "ws://localhost:9910/omniverse"
    assert calls[0]["omniversePrimPath"] == "/World/Robot" and calls[0]["clientId"] == "arm-a"
