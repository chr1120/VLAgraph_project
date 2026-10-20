import copy
import json

import pytest

pytest.importorskip("pxr")
from pxr import Gf, Usd, UsdGeom, UsdPhysics

from backend.world_model.usd_geometry import extract_collision_inventory
from backend.executors.world_model_nodes import exec_fanuc_usd_geometry_read, exec_fanuc_collision_check
from examples.isaacsim.fanuc_world_model_bridge import FanucWorldModelAdapter
from test_fanuc_preflight import action_fixture
from test_fanuc_collision import scene_fixture


def stage_fixture():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.SetStageMetersPerUnit(stage, 1)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.Xform.Define(stage, "/World/Robot")
    for index in range(1, 7):
        body = UsdGeom.Xform.Define(stage, f"/World/Robot/link{index}").GetPrim()
        UsdPhysics.RigidBodyAPI.Apply(body)
        shape = UsdGeom.Cube.Define(stage, f"{body.GetPath()}/shape")
        shape.CreateSizeAttr(0.1)
        UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
        joint = UsdPhysics.RevoluteJoint.Define(stage, f"/World/Robot/J{index}")
        joint.CreateBody1Rel().SetTargets([body.GetPath()])
    return stage


def extract(stage, **kwargs):
    return extract_collision_inventory(stage, robot_prim_path="/World/Robot", **kwargs)


def test_inventory_is_read_only_and_joint_coverage_is_not_execution_readiness():
    stage = stage_fixture()
    before = stage.GetRootLayer().ExportToString()
    result = extract(stage)
    assert len(result["colliders"]) == 6
    assert result["robot_joint_collision_coverage_complete"] is True
    assert result["proxy_scene_ready"] is False
    assert result["execution_ready"] is False
    assert result["physical_collision_verified"] is False
    assert "environment_collision_coverage" in result["unchecked"]
    assert stage.GetRootLayer().ExportToString() == before
    assert result == extract(stage)
    json.dumps(result, allow_nan=False)


def test_instance_proxy_collision_shapes_are_included():
    stage = stage_fixture()
    prototype = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(prototype, "/Asset")
    shape = UsdGeom.Cube.Define(prototype, "/Asset/collider")
    UsdPhysics.CollisionAPI.Apply(shape.GetPrim())
    instance = UsdGeom.Xform.Define(stage, "/World/Robot/link6/instance").GetPrim()
    instance.GetReferences().AddReference(prototype.GetRootLayer().identifier, "/Asset")
    instance.SetInstanceable(True)
    assert not any(str(prim.GetPath()).endswith("instance/collider") for prim in stage.Traverse())
    result = extract(stage)
    item = next(item for item in result["colliders"] if item["prim_path"].endswith("instance/collider"))
    assert item["robot_joint"] == "J6"
    assert item["body_local_aabb_stage"]["min"] == [-1, -1, -1]


def test_disabled_geometry_and_missing_joint_fail_coverage():
    stage = stage_fixture()
    UsdPhysics.CollisionAPI(stage.GetPrimAtPath("/World/Robot/link3/shape")).CreateCollisionEnabledAttr(False)
    stage.RemovePrim("/World/Robot/J4")
    result = extract(stage)
    assert result["robot_joint_collision_coverage_complete"] is False
    assert result["missing_joint_bindings"] == ["J4"]
    assert result["missing_joint_colliders"] == ["J3", "J4"]
    assert result["disabled_collision_prims"] == ["/World/Robot/link3/shape"]


def test_world_bounds_convert_units_up_axis_and_distinguish_dynamic_obstacles():
    stage = stage_fixture()
    UsdGeom.SetStageMetersPerUnit(stage, 0.001)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.y)
    box = UsdGeom.Cube.Define(stage, "/World/Static")
    box.CreateSizeAttr(2)
    box.AddTranslateOp().Set(Gf.Vec3d(1000, 2000, 3000))
    UsdPhysics.CollisionAPI.Apply(box.GetPrim())
    moving = UsdGeom.Cube.Define(stage, "/World/Moving")
    UsdPhysics.CollisionAPI.Apply(moving.GetPrim())
    UsdPhysics.RigidBodyAPI.Apply(moving.GetPrim())
    visual = UsdGeom.Cube.Define(stage, "/World/VisualOnly")
    result = extract(stage)
    candidate = result["static_obstacle_candidates"][0]
    assert candidate["id"] == "/World/Static"
    assert candidate["min"] == pytest.approx([0.999, -3.001, 1.999])
    assert candidate["max"] == pytest.approx([1.001, -2.999, 2.001])
    assert next(c for c in result["colliders"] if c["prim_path"] == "/World/Moving")["role"] == "moving_obstacle"
    assert not any(c["prim_path"] == str(visual.GetPath()) for c in result["colliders"])


def test_truncation_is_explicit_and_cannot_pass_coverage():
    result = extract(stage_fixture(), max_colliders=2)
    assert len(result["colliders"]) == 2
    assert result["truncated"] is True
    assert result["robot_joint_collision_coverage_complete"] is False


def test_file_node_and_bridge_adapter_read_same_inventory_without_motion(tmp_path):
    from types import SimpleNamespace
    stage = stage_fixture()
    path = tmp_path / "robot.usda"
    stage.GetRootLayer().Export(str(path))
    node = {"id": "geometry", "data": {"source": "file", "usdPath": str(path), "robotPrimPath": "/World/Robot"}}
    result = exec_fanuc_usd_geometry_read(node, {}, [])
    assert result["ok"] is True and result["coverage_complete"] is True
    adapter = FanucWorldModelAdapter(SimpleNamespace(_stage=lambda: stage))
    reply = adapter.apply({"command": "get_collision_geometry", "prim_path": "/World/Robot"},
                          lambda _: pytest.fail("Read command reached motion handler"))
    assert reply["ok"] is True
    assert reply["inventory"] == extract(stage)
    assert not adapter.records


def test_file_node_reports_unavailable_or_wrong_stage(tmp_path):
    result = exec_fanuc_usd_geometry_read({"id": "g", "data": {"source": "file", "usdPath": str(tmp_path / "missing.usd")}}, {}, [])
    assert result["ok"] is False and result["inventory"] is None


@pytest.mark.parametrize("case", ["missing", "incomplete", "stale", "tampered", "valid"])
def test_connected_inventory_is_required_and_bound_to_proxy_scene(case):
    inventory = extract(stage_fixture())
    scene = scene_fixture()
    scene["inventory_digest"] = inventory["inventory_digest"]
    if case == "missing": inventory = None
    if case == "incomplete": inventory["robot_joint_collision_coverage_complete"] = False
    if case == "stale": scene["inventory_digest"] = "different"
    if case == "tampered": inventory["colliders"][0]["world_aabb_m_zup"]["min"][0] = 999
    context = {"action": {"action": action_fixture()}, "geometry": {"inventory": inventory}}
    edges = [{"source": "action", "sourceHandle": "action", "target": "check", "targetHandle": "action"},
             {"source": "geometry", "sourceHandle": "inventory", "target": "check", "targetHandle": "inventory"}]
    result = exec_fanuc_collision_check({"id": "check", "data": {"enabled": True, "scene": copy.deepcopy(scene)}}, context, edges)
    assert result["ok"] is (case == "valid")
