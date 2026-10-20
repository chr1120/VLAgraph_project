import itertools

import numpy as np
import pytest

pytest.importorskip("pxr")
from pxr import Gf, UsdGeom, UsdPhysics

from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
from backend.world_model.fanuc_collision import check_fanuc_collision, segment_segment_distance
from test_fanuc_usd_proxy import stage_fixture, inventory
from test_fanuc_collision import scene_fixture
from test_fanuc_preflight import action_fixture


def add_box(stage, path, size=0.1):
    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(size)
    UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube


@pytest.mark.parametrize("units,up", [(1, "Z"), (0.001, "Z"), (1, "Y")])
def test_base_and_rail_are_enclosed_in_distinct_moving_and_world_frames(units, up):
    stage = stage_fixture(units=units, external=True, up=up)
    add_box(stage, "/World/Robot/base/shape", 0.1 / units)
    add_box(stage, "/World/Robot/rail/shape", 0.1 / units)
    data = inventory(stage)
    scene = build_fanuc_usd_proxy(data)["scene"]
    coverage = scene["geometry_coverage"]
    assert len(scene["link_capsules"]) == 8
    assert coverage["capsules_by_attachment"] == {"joint": 6, "base": 1, "world": 1}
    assert coverage["all_enabled_robot_colliders_mapped"] is True
    assert coverage["full_cell_coverage_verified"] is False
    assert scene["omitted_robot_colliders"] == []
    for capsule in scene["link_capsules"]:
        if capsule["attachment"] == "joint": continue
        assert "joint_index" not in capsule
        item = next(item for item in data["colliders"] if item["prim_path"] == capsule["id"])
        bounds = item["world_aabb_m_zup"] if capsule["attachment"] == "world" else item["body_local_aabb_stage"]
        scale = 1 if capsule["attachment"] == "world" else units
        for corner in itertools.product(*zip(bounds["min"], bounds["max"])):
            corner = np.asarray(corner) * scale
            assert segment_segment_distance(corner, corner, capsule["start_local"], capsule["end_local"]) <= capsule["radius_m"]


def fixed_tool_stage(*, enabled=True, wrong_pose=False, duplicate_parent=False):
    stage = stage_fixture()
    tool = UsdGeom.Xform.Define(stage, "/World/Robot/tool")
    UsdPhysics.RigidBodyAPI.Apply(tool.GetPrim())
    tool.AddTranslateOp().Set(Gf.Vec3d(0.2 if not wrong_pose else 0.3, 0, 0))
    add_box(stage, "/World/Robot/tool/shape")
    joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/tool_fixed")
    joint.CreateBody0Rel().SetTargets(["/World/Robot/link6"])
    joint.CreateBody1Rel().SetTargets([tool.GetPath()])
    joint.CreateLocalPos0Attr(Gf.Vec3f(0.2, 0, 0))
    joint.CreateJointEnabledAttr(enabled)
    if duplicate_parent:
        other = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/tool_fixed2")
        other.CreateBody0Rel().SetTargets(["/World/Robot/link5"])
        other.CreateBody1Rel().SetTargets([tool.GetPath()])
    return stage


def test_explicit_fixed_tool_is_attached_after_j6_and_transformed_conservatively():
    scene = build_fanuc_usd_proxy(inventory(fixed_tool_stage()))["scene"]
    capsule = next(item for item in scene["link_capsules"] if item["id"].endswith("tool/shape"))
    assert capsule["attachment"] == "joint" and capsule["joint_index"] == 5
    assert (np.asarray(capsule["start_local"]) + capsule["end_local"])[0] / 2 == pytest.approx(0.2)
    assert scene["geometry_coverage"]["tool_coverage"] == "unverified"


def test_rotated_fixed_tool_bounds_are_transformed_to_the_joint_frame():
    stage = fixed_tool_stage()
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot/tool")).AddRotateZOp().Set(90)
    joint = UsdPhysics.FixedJoint(stage.GetPrimAtPath("/World/Robot/tool_fixed"))
    joint.CreateLocalRot0Attr(Gf.Quatf(2 ** -0.5, 0, 0, 2 ** -0.5))
    UsdGeom.Xformable(stage.GetPrimAtPath("/World/Robot/tool/shape")).AddScaleOp().Set(Gf.Vec3f(1, 3, 2))
    scene = build_fanuc_usd_proxy(inventory(stage))["scene"]
    capsule = next(item for item in scene["link_capsules"] if item["id"].endswith("tool/shape"))
    for x, y, z in itertools.product([-0.05, 0.05], [-0.15, 0.15], [-0.1, 0.1]):
        point = [0.2 - y, x, z]
        assert segment_segment_distance(point, point, capsule["start_local"], capsule["end_local"]) <= capsule["radius_m"]


def test_actuated_tool_and_conflicting_fixed_parent_are_not_treated_as_rigid():
    stage = fixed_tool_stage()
    joint = UsdPhysics.RevoluteJoint.Define(stage, "/World/Robot/gripper_joint")
    joint.CreateBody0Rel().SetTargets(["/World/Robot/link6"])
    joint.CreateBody1Rel().SetTargets(["/World/Robot/tool"])
    data = inventory(stage)
    assert any(item["type"] == "PhysicsRevoluteJoint" for item in data["auxiliary_joint_bindings"])
    assert build_fanuc_usd_proxy(data)["scene"]["omitted_robot_colliders"] == ["/World/Robot/tool/shape"]


def test_extra_constraint_on_arm_link_blocks_fk_binding():
    stage = stage_fixture()
    joint = UsdPhysics.FixedJoint.Define(stage, "/World/Robot/extra_constraint")
    joint.CreateBody0Rel().SetTargets(["/World/Robot/link2"])
    joint.CreateBody1Rel().SetTargets(["/World/Robot/link6"])
    with pytest.raises(ValueError, match="additional joint"):
        build_fanuc_usd_proxy(inventory(stage))


@pytest.mark.parametrize("kwargs", [{"enabled": False}, {"wrong_pose": True}, {"duplicate_parent": True}])
def test_unverified_tool_connections_are_reported_as_unmapped(kwargs):
    scene = build_fanuc_usd_proxy(inventory(fixed_tool_stage(**kwargs)))["scene"]
    assert scene["omitted_robot_colliders"] == ["/World/Robot/tool/shape"]
    coverage = scene["geometry_coverage"]
    assert coverage["all_enabled_robot_colliders_mapped"] is False
    assert coverage["unmapped_robot_colliders"][0]["reason"] == "unmapped_or_unverified_rigid_body_attachment"


def test_dynamic_environment_and_visual_only_geometry_are_not_silently_marked_covered():
    stage = stage_fixture()
    UsdGeom.Cube.Define(stage, "/World/VisualOnly")
    dynamic = add_box(stage, "/World/MovingObstacle")
    UsdPhysics.RigidBodyAPI.Apply(dynamic.GetPrim())
    scene = build_fanuc_usd_proxy(inventory(stage))["scene"]
    coverage = scene["geometry_coverage"]
    assert coverage["boundables_without_collision_api"] >= 1
    assert coverage["unmodeled_moving_obstacles"] == ["/World/MovingObstacle"]
    assert coverage["static_obstacle_count"] == 0
    assert coverage["environment_coverage"] == "unverified"


def rail_action():
    action = action_fixture()
    action["joint_names"].append("externalAxisJoint")
    action["joint_trajectory"][0]["joint_positions"].append(-1000)
    action["joint_trajectory"][1]["joint_positions"].append(1000)
    return action


def rail_scene(attachment):
    scene = scene_fixture()
    scene["base_position"] = [0, 0, 0]
    scene["external_axis"] = {"direction_world": [1, 0, 0], "reference_position_m": 0}
    scene["link_capsules"] = [{"id": "body", "attachment": attachment, "start_local": [0, 0, 0],
                               "end_local": [0, 0, 0], "radius_m": 0.05}]
    scene["obstacles"] = [{"id": "box", "min": [-0.1, -0.1, -0.1], "max": [0.1, 0.1, 0.1]}]
    return scene


def test_base_follows_external_axis_and_collides_between_clear_endpoints():
    report = check_fanuc_collision(rail_action(), rail_scene("base"), max_step_sec=1)
    assert report["status"] == "collision"
    assert any(item["time_sec"] == 3 for item in report["contacts"])
    assert report["capsules_by_attachment"] == {"joint": 0, "base": 1, "world": 0}


def test_world_rail_stays_fixed_and_has_zero_external_motion_bound():
    scene = rail_scene("world")
    scene["link_capsules"][0]["start_local"] = [0.5, 0, 0]
    scene["link_capsules"][0]["end_local"] = [0.5, 0, 0]
    report = check_fanuc_collision(rail_action(), scene, max_step_sec=2)
    assert report["status"] == "clear" and report["between_samples_certified"] is True
    assert not report["unresolved_intervals"]
    assert report["capsules_by_attachment"]["world"] == 1


@pytest.mark.parametrize("attachment", ["base", "world"])
def test_arm_self_collision_with_base_or_fixed_rail_is_checked(attachment):
    scene = rail_scene(attachment)
    scene["link_capsules"][0]["radius_m"] = 1.1
    scene["link_capsules"].append({"id": "arm", "joint_index": 0, "start_local": [0, 0, -0.565],
                                   "end_local": [0, 0, -0.565], "radius_m": 0.05})
    scene["obstacles"] = []
    report = check_fanuc_collision(rail_action(), scene)
    assert report["status"] == "collision" and report["self_pairs_checked"] == 1
    scene["excluded_self_pairs"] = [["body", "arm"]]
    assert check_fanuc_collision(rail_action(), scene)["status"] == "indeterminate"


@pytest.mark.parametrize("attachment,joint", [("invalid", None), ("base", 0), ("world", 2)])
def test_invalid_or_conflicting_attachment_is_rejected(attachment, joint):
    scene = rail_scene(attachment)
    if joint is not None: scene["link_capsules"][0]["joint_index"] = joint
    with pytest.raises(ValueError):
        check_fanuc_collision(rail_action(), scene)


@pytest.mark.parametrize("missing", ["unmapped_robot_colliders", "unmodeled_moving_obstacles"])
def test_known_unmodeled_geometry_cannot_be_reported_clear_or_pass_execute_guard(missing):
    from backend.executors.world_model_nodes import exec_execution_guard
    from backend.world_model.fanuc_preflight import validate_fanuc_trajectory

    scene = rail_scene("world")
    scene["link_capsules"][0]["start_local"] = [10, 0, 0]
    scene["link_capsules"][0]["end_local"] = [10, 0, 0]
    scene["geometry_coverage"] = {"all_enabled_robot_colliders_mapped": missing != "unmapped_robot_colliders",
                                  "unmapped_robot_colliders": [], "unmodeled_moving_obstacles": []}
    scene["geometry_coverage"][missing] = ["missing-geometry"]
    action = rail_action()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    report = check_fanuc_collision(action, scene)
    assert report["status"] == "indeterminate"
    assert not report["contacts"] and missing in report["unmodeled_geometry"]
    context = {"a": {"action": action}, "p": {"report": validate_fanuc_trajectory(action)}, "c": {"collision_report": report}}
    edges = [{"source": source, "sourceHandle": handle, "target": "guard", "targetHandle": handle}
             for source, handle in [("a", "action"), ("p", "report"), ("c", "collision_report")]]
    result = exec_execution_guard({"id": "guard", "data": {"validationPolicy": "static_simulation"}}, context, edges)
    assert result["approved"] is False
