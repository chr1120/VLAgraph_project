import copy
import itertools

import numpy as np
import pytest

pytest.importorskip("pxr")
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from backend.executors.fanuc_nodes import M710_JOINT_ORIGINS, M710_JOINT_LIMITS
from backend.executors.world_model_nodes import exec_fanuc_usd_proxy_build, exec_fanuc_collision_check
from backend.world_model.usd_geometry import collision_inventory_digest, extract_collision_inventory
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy, validate_fanuc_usd_binding, _rotation
from backend.world_model.fanuc_collision import check_fanuc_collision, segment_segment_distance
from test_fanuc_preflight import action_fixture
from test_fanuc_collision import scene_fixture


def stage_fixture(*, units=1, external=False, up="Z"):
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, units)
    UsdGeom.SetStageUpAxis(stage, up)
    root = UsdGeom.Xform.Define(stage, "/World/Robot")
    root.AddTranslateOp().Set(Gf.Vec3d(10 / units, 20 / units, 30 / units))
    root.AddRotateZOp().Set(90)
    base = UsdGeom.Xform.Define(stage, "/World/Robot/base").GetPrim()
    UsdPhysics.RigidBodyAPI.Apply(base)
    rotations = [Gf.Quatf(1), Gf.Quatf(1), Gf.Quatf(0, -1, 0, 0),
                 Gf.Quatf(0, 0, -1, 0), Gf.Quatf(0, -1, 0, 0), Gf.Quatf(0, 0, -1, 0)]
    axes = ["Z", "Y", "Y", "X", "Y", "X"]
    previous = base.GetPath()
    for index in range(6):
        body = UsdGeom.Xform.Define(stage, f"/World/Robot/link{index+1}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        cube = UsdGeom.Cube.Define(stage, f"{body.GetPath()}/shape")
        cube.CreateSizeAttr(0.1 / units)
        cube.AddScaleOp().Set(Gf.Vec3f(1, 2, 3))
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Robot/J{index+1}")
        joint.CreateBody0Rel().SetTargets([previous])
        joint.CreateBody1Rel().SetTargets([body.GetPath()])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*(M710_JOINT_ORIGINS[index] / units)))
        joint.CreateLocalRot0Attr(rotations[index])
        joint.CreateLocalRot1Attr(rotations[index])
        joint.CreateAxisAttr(axes[index])
        lower, upper = np.degrees(M710_JOINT_LIMITS[index])
        joint.CreateLowerLimitAttr(float(lower))
        joint.CreateUpperLimitAttr(float(upper))
        previous = body.GetPath()
    if external:
        rail = UsdGeom.Xform.Define(stage, "/World/Robot/rail").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(rail)
        fixed = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/world_rail")
        fixed.CreateBody0Rel().SetTargets([root.GetPath()])
        fixed.CreateBody1Rel().SetTargets([rail.GetPath()])
        joint = UsdPhysics.PrismaticJoint.Define(stage, "/World/Robot/external")
        joint.CreateAxisAttr("Y")
        joint.CreateBody0Rel().SetTargets([rail.GetPath()])
        joint.CreateBody1Rel().SetTargets([base.GetPath()])
    return stage


def inventory(stage=None):
    return extract_collision_inventory(stage or stage_fixture(), robot_prim_path="/World/Robot")


def rehash(value):
    value["inventory_digest"] = collision_inventory_digest(value)


@pytest.mark.parametrize("units,up", [(1, "Z"), (0.001, "Z"), (1, "Y")])
def test_signed_joint_frames_match_project_fk_and_convert_world_pose(units, up):
    data = inventory(stage_fixture(units=units, up=up))
    original = copy.deepcopy(data)
    result = build_fanuc_usd_proxy(data)
    report, scene = result["binding_report"], result["scene"]
    assert report["kinematics_match"] is True
    assert len(report["composition_samples"]) == 3
    assert max(sample["max_transform_error"] for sample in report["composition_samples"]) < 1e-6
    assert report["base_position"] == pytest.approx([10, 20, 30] if up == "Z" else [10, -30, 20])
    assert len(scene["link_capsules"]) == 6
    assert scene["excluded_self_pairs"] == []
    assert scene["inventory_digest"] == data["inventory_digest"]
    assert scene["execution_ready"] is False and scene["physical_collision_verified"] is False
    assert report["physics_synchronized"] is False
    expected = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])
    if up == "Y": expected = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) @ expected
    assert np.allclose(_rotation(scene["base_orientation_wxyz"]), expected, atol=1e-6)
    assert data == original


def test_capsules_enclose_all_authored_collision_box_corners():
    data = inventory()
    result = build_fanuc_usd_proxy(data, padding_m=0.005)
    shapes = {item["prim_path"]: item for item in data["colliders"]}
    for capsule in result["scene"]["link_capsules"]:
        bounds = shapes[capsule["id"]]["body_local_aabb_stage"]
        for corner in itertools.product(*zip(bounds["min"], bounds["max"])):
            distance = segment_segment_distance(corner, corner, capsule["start_local"], capsule["end_local"])
            assert distance <= capsule["radius_m"] - 0.005 + 1e-10


@pytest.mark.parametrize("case", ["axis", "origin", "child_position", "child_rotation", "parent", "limits", "scale", "mirror", "shear"])
def test_binding_mismatch_blocks_proxy_generation(case):
    data = inventory()
    joint = data["joint_bindings"]["J3"]
    if case == "axis": joint["axis"] = "X"
    if case == "origin": joint["local_pos0_stage"][2] += 0.01
    if case == "child_position": joint["local_pos1_stage"][0] = 0.01
    if case == "child_rotation": joint["local_rot1_wxyz"] = [1, 0, 0, 0]
    if case == "parent": joint["body0"] = [data["joint_bindings"]["J1"]["body1"]]
    if case == "limits": joint["upper_limit_deg"] = 120
    if case in {"scale", "mirror", "shear"}:
        frame = data["body_world_transforms_stage"][joint["body1"]]
        if case == "scale": frame[2][2] = 2
        if case == "mirror": frame[2][2] = -1
        if case == "shear": frame[1][2] += 0.2
    rehash(data)
    assert validate_fanuc_usd_binding(data)["kinematics_match"] is False
    with pytest.raises(ValueError, match="binding failed"):
        build_fanuc_usd_proxy(data)


def test_nonzero_external_reference_and_world_direction_are_derived_from_joint_frames():
    stage = stage_fixture(external=True)
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot/base")).AddTranslateOp().Set(Gf.Vec3d(0, 0.25, 0))
    result = build_fanuc_usd_proxy(inventory(stage))
    external = result["scene"]["external_axis"]
    assert external["reference_position_m"] == pytest.approx(0.25)
    assert external["direction_world"] == pytest.approx([-1, 0, 0])
    assert result["scene"]["base_position"] == pytest.approx([9.75, 20, 30])


@pytest.mark.parametrize("case", ["off_axis", "rotation", "disabled", "multiple", "cycle"])
def test_unsupported_external_chain_is_rejected(case):
    data = inventory(stage_fixture(external=True))
    joint = next(item for item in data["auxiliary_joint_bindings"] if item["type"] == "PhysicsPrismaticJoint")
    if case == "off_axis": data["body_world_transforms_stage"]["/World/Robot/base"][3][2] += 0.1
    if case == "rotation": joint["local_rot0_wxyz"] = [0, 1, 0, 0]
    if case == "disabled": joint["enabled"] = False
    if case == "multiple": data["auxiliary_joint_bindings"].append(copy.deepcopy(joint))
    if case == "cycle": joint["body0"] = ["/World/Robot/base"]
    rehash(data)
    with pytest.raises(ValueError, match="binding failed"):
        build_fanuc_usd_proxy(data)


def test_explicit_exclusions_and_provenance_reach_collision_report():
    data = inventory()
    first = build_fanuc_usd_proxy(data)["scene"]
    pair = [item["id"] for item in first["link_capsules"][:2]]
    result = build_fanuc_usd_proxy(data, excluded_self_pairs=[pair])
    assert result["scene"]["excluded_self_pairs"] == [pair]
    report = check_fanuc_collision(action_fixture(), result["scene"])
    assert report["inventory_digest"] == data["inventory_digest"]
    assert report["source_coverage"] == "authored_mapped_robot_collision_bounds"
    assert "environment_collision_coverage" in report["unchecked"]
    assert report["execution_ready"] is False
    with pytest.raises(ValueError, match="Exclusions"):
        build_fanuc_usd_proxy(data, excluded_self_pairs=[[pair[0], "unknown"]])


def test_missing_and_tampered_inventory_never_build_a_scene():
    data = inventory()
    data["joint_bindings"]["J1"]["axis"] = "X"
    for value in (None, data):
        result = exec_fanuc_usd_proxy_build({"id": "proxy", "data": {}}, {"g": {"inventory": value}},
                                         [{"source": "g", "sourceHandle": "inventory", "target": "proxy", "targetHandle": "inventory"}])
        assert result["ok"] is False and result["scene"] is None


def test_failed_connected_proxy_cannot_fall_back_to_node_scene():
    context = {"proxy": {"scene": None}, "action": {"action": action_fixture()}}
    edges = [{"source": "proxy", "sourceHandle": "scene", "target": "check", "targetHandle": "scene"},
             {"source": "action", "sourceHandle": "action", "target": "check", "targetHandle": "action"}]
    result = exec_fanuc_collision_check({"id": "check", "data": {"enabled": True, "scene": scene_fixture()}}, context, edges)
    assert result["ok"] is False and result["collision_report"] is None


@pytest.mark.parametrize("broken_binding", [False, True])
def test_file_to_proxy_to_existing_execute_preserves_full_guard(tmp_path, monkeypatch, broken_binding):
    from backend.executors import vla_nodes
    from backend.runtime.runner import run_graph
    from test_fanuc_execution_guard import pipeline

    stage = stage_fixture()
    if broken_binding:
        stage.GetPrimAtPath("/World/Robot/J3").GetAttribute("physics:axis").Set("X")
    path = tmp_path / "robot.usda"
    stage.GetRootLayer().Export(str(path))
    graph = pipeline("full")
    graph["nodes"].extend([
        {"id": "geometry", "type": "fanuc_usd_geometry_read", "data": {
            "source": "file", "usdPath": str(path), "robotPrimPath": "/World/Robot"}},
        {"id": "proxy", "type": "fanuc_usd_proxy_build", "data": {}},
        {"id": "collision", "type": "fanuc_collision_check", "data": {"enabled": True}},
    ])
    graph["edges"].extend([
        {"source": "geometry", "sourceHandle": "inventory", "target": "proxy", "targetHandle": "inventory"},
        {"source": "geometry", "sourceHandle": "inventory", "target": "collision", "targetHandle": "inventory"},
        {"source": "proxy", "sourceHandle": "scene", "target": "collision", "targetHandle": "scene"},
        {"source": "prepare", "sourceHandle": "state", "target": "collision", "targetHandle": "action"},
        {"source": "collision", "sourceHandle": "collision_report", "target": "guard", "targetHandle": "collision_report"},
    ])
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *args: pytest.fail("Transport called"))
    result = run_graph(graph)["context"]
    assert result["geometry"]["ok"] is True
    assert result["proxy"]["ok"] is (not broken_binding)
    assert result["collision"]["ok"] is (not broken_binding)
    assert result["guard"]["approved"] is False
    assert result["execute"]["skipped"] is True


@pytest.mark.parametrize("case", ["success", "bridge_rejects", "binding_mismatch"])
def test_read_only_validation_cli_propagates_geometry_and_binding_failures(tmp_path, monkeypatch, case):
    import json
    from scripts import validate_fanuc_observation as cli

    data = inventory()
    if case == "binding_mismatch":
        data["joint_bindings"]["J1"]["axis"] = "X"
        rehash(data)
    def observe(graph):
        assert all(node["type"] in {"fanuc_state_read", "scene_scan", "print"} for node in graph["nodes"])
        return {"error_count": 0, "context": {"fanuc-state": {"state": {}}, "fanuc-scan": {"scan": {}}}}
    monkeypatch.setattr(cli, "run_graph", observe)
    monkeypatch.setattr(cli, "exec_fanuc_snapshot_read", lambda *args: {"ok": True, "snapshot": {}})
    geometry = {"ok": False, "inventory": None, "message": "Unsupported command"} if case == "bridge_rejects" else {"ok": True, "inventory": data}
    monkeypatch.setattr(cli, "exec_fanuc_usd_geometry_read", lambda *args: geometry)
    output = tmp_path / "runtime.json"
    monkeypatch.setattr("sys.argv", ["validate_fanuc_observation.py", "--snapshot", "--geometry", "--output", str(output)])
    assert cli.main() == (0 if case == "success" else 1)
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["motion_requested"] is False
    assert result["commands_sent"] == ["get_state", "scan_scene", "capture_world_state", "get_collision_geometry"]
    assert result["ok"] is (case == "success")
