"""Read composed USD collision geometry without changing the stage or physics.

Uses only stdlib + pxr, so the same file can be copied into the Isaac extension.
Authored collision bounds are inventory data, not a ready kinematic proxy scene.
"""
from __future__ import annotations

import hashlib
import itertools
import json
import math


def collision_inventory_digest(inventory):
    """Hash the complete inventory payload, excluding its digest field."""
    payload = {key: value for key, value in inventory.items() if key != "inventory_digest"}
    return hashlib.sha256(json.dumps(payload, sort_keys=True, allow_nan=False).encode()).hexdigest()


def extract_collision_inventory(stage, *, robot_prim_path, root_path="/World", max_colliders=4096):
    from pxr import Gf, Usd, UsdGeom, UsdPhysics

    if stage is None:
        raise ValueError("USD stage is unavailable")
    if not stage.GetPrimAtPath(root_path) or not stage.GetPrimAtPath(robot_prim_path):
        raise ValueError("Scene root and selected robot prim must exist")
    if not robot_prim_path.startswith(root_path.rstrip("/") + "/"):
        raise ValueError("Robot must be inside the selected scene root")
    if isinstance(max_colliders, bool) or not isinstance(max_colliders, int) or not 1 <= max_colliders <= 4096:
        raise ValueError("maxColliders must be 1..4096")
    units, up = float(UsdGeom.GetStageMetersPerUnit(stage)), str(UsdGeom.GetStageUpAxis(stage))
    if not math.isfinite(units) or units <= 0 or up not in {"Y", "Z"}:
        raise ValueError("USD stage units and up-axis must be known")

    def xyz(value):
        parsed = [float(component) for component in value]
        if len(parsed) != 3 or not all(math.isfinite(v) for v in parsed):
            raise ValueError("Invalid geometry vector")
        return parsed

    def world_m(value):
        x, y, z = [component * units for component in xyz(value)]
        if not all(math.isfinite(v) for v in (x, y, z)):
            raise ValueError("Transformed geometry exceeds finite range")
        return [x, -z, y] if up == "Y" else [x, y, z]

    def body_owner(prim):
        current = prim
        while current and current.GetPath() != stage.GetPseudoRoot().GetPath():
            if current.HasAPI(UsdPhysics.RigidBodyAPI):
                api = UsdPhysics.RigidBodyAPI(current)
                return str(current.GetPath()), bool(api.GetRigidBodyEnabledAttr().Get()), bool(api.GetKinematicEnabledAttr().Get())
            current = current.GetParent()
        return None, None, None

    prims = [prim for prim in Usd.PrimRange.Stage(stage, Usd.TraverseInstanceProxies())
             if str(prim.GetPath()) == root_path or str(prim.GetPath()).startswith(root_path.rstrip("/") + "/")]
    robot_prefix = robot_prim_path.rstrip("/") + "/"
    bindings = {}
    issues = []
    for prim in prims:
        if not str(prim.GetPath()).startswith(robot_prefix) or not prim.IsA(UsdPhysics.RevoluteJoint):
            continue
        name = prim.GetName()
        if name not in {f"J{i}" for i in range(1, 7)}:
            continue
        joint = UsdPhysics.RevoluteJoint(prim)
        body0 = list(map(str, joint.GetBody0Rel().GetTargets()))
        body1 = list(map(str, joint.GetBody1Rel().GetTargets()))
        if name in bindings or len(body1) != 1 or not stage.GetPrimAtPath(body1[0]) or not joint.GetJointEnabledAttr().Get():
            issues.append({"code": "ambiguous_or_disabled_joint_binding", "joint": name, "prim_path": str(prim.GetPath())})
            continue
        q0, q1 = joint.GetLocalRot0Attr().Get(), joint.GetLocalRot1Attr().Get()
        bindings[name] = {
            "joint_path": str(prim.GetPath()), "body0": body0, "body1": body1[0],
            "axis": str(joint.GetAxisAttr().Get()), "local_pos0_stage": xyz(joint.GetLocalPos0Attr().Get()),
            "local_pos1_stage": xyz(joint.GetLocalPos1Attr().Get()),
            "local_rot0_wxyz": [float(q0.GetReal()), *xyz(q0.GetImaginary())],
            "local_rot1_wxyz": [float(q1.GetReal()), *xyz(q1.GetImaginary())],
            "lower_limit_deg": float(joint.GetLowerLimitAttr().Get()),
            "upper_limit_deg": float(joint.GetUpperLimitAttr().Get()),
        }
        for limit_key in ("lower_limit_deg", "upper_limit_deg"):
            if not math.isfinite(bindings[name][limit_key]):
                bindings[name][limit_key] = None
    owner_to_joint = {}
    for name, binding in bindings.items():
        if binding["body1"] in owner_to_joint:
            issues.append({"code": "multiple_joints_share_body", "body": binding["body1"]})
        owner_to_joint[binding["body1"]] = name
    colliders, disabled, unsupported, unloaded = [], [], [], []
    no_collision_count = 0
    truncated = False
    time_code = Usd.TimeCode.Default()
    for prim in prims:
        path = str(prim.GetPath())
        if prim.HasPayload() and not prim.IsLoaded():
            unloaded.append(path)
        if not prim.HasAPI(UsdPhysics.CollisionAPI):
            no_collision_count += int(bool(UsdGeom.Boundable(prim)))
            continue
        if not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
            disabled.append(path)
            continue
        if len(colliders) + len(unsupported) >= max_colliders:
            truncated = True
            break
        try:
            boundable = UsdGeom.Boundable(prim)
            if not boundable:
                raise ValueError("Collision prim is not a supported Boundable")
            extent = UsdGeom.Boundable.ComputeExtentFromPlugins(boundable, time_code)
            if extent is None or len(extent) != 2:
                raise ValueError("Cannot compute intrinsic collision geometry extent")
            lower, upper = xyz(extent[0]), xyz(extent[1])
            if any(a > b for a, b in zip(lower, upper)):
                raise ValueError("Collision extent is empty or inverted")
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(time_code)
            corners = [Gf.Vec3d(*values) for values in itertools.product(*zip(lower, upper))]
            world = [world_m(matrix.Transform(point)) for point in corners]
            owner, body_enabled, kinematic = body_owner(prim)
            is_robot = path == robot_prim_path or path.startswith(robot_prefix)
            if is_robot and owner and body_enabled is False:
                issues.append({"code": "robot_rigid_body_disabled", "prim_path": path, "body": owner})
            role = "robot" if is_robot else "static_obstacle" if owner is None else "moving_obstacle"
            approximation = str(UsdPhysics.MeshCollisionAPI(prim).GetApproximationAttr().Get()) if prim.HasAPI(UsdPhysics.MeshCollisionAPI) else None
            record = {
                "prim_path": path, "type": prim.GetTypeName(), "role": role,
                "rigid_body_path": owner, "rigid_body_enabled": body_enabled, "kinematic_enabled": kinematic,
                "robot_joint": owner_to_joint.get(owner) if is_robot else None,
                "local_aabb_stage": {"min": lower, "max": upper},
                "world_aabb_m_zup": {"min": [min(v[i] for v in world) for i in range(3)],
                                     "max": [max(v[i] for v in world) for i in range(3)]},
                "collision_approximation": approximation, "bound_source": "collision_api_intrinsic_extent",
                "world_transform_stage": [[float(matrix[i][j]) for j in range(4)] for i in range(4)],
                "contact_offset_stage": None,
            }
            offset = prim.GetAttribute("physxCollision:contactOffset")
            if offset and offset.HasAuthoredValueOpinion():
                offset_value = float(offset.Get())
                if math.isfinite(offset_value):
                    record["contact_offset_stage"] = offset_value
            if owner:
                owner_matrix = UsdGeom.Xformable(stage.GetPrimAtPath(owner)).ComputeLocalToWorldTransform(time_code)
                if abs(float(owner_matrix.GetDeterminant())) < 1e-12:
                    raise ValueError("Rigid body has a singular transform")
                relative = matrix * owner_matrix.GetInverse()
                local_points = [xyz(relative.Transform(point)) for point in corners]
                record["body_local_aabb_stage"] = {"min": [min(v[i] for v in local_points) for i in range(3)],
                                                    "max": [max(v[i] for v in local_points) for i in range(3)]}
            colliders.append(record)
        except (ValueError, TypeError, RuntimeError) as exc:
            unsupported.append({"prim_path": path, "message": str(exc)})
    missing_joints = [f"J{i}" for i in range(1, 7) if f"J{i}" not in bindings]
    coverage = {f"J{i}": [item["prim_path"] for item in colliders if item["robot_joint"] == f"J{i}"] for i in range(1, 7)}
    missing_shapes = [name for name, paths in coverage.items() if not paths]
    composition_errors = [str(error) for error in stage.GetCompositionErrors()]
    complete = not (missing_joints or missing_shapes or issues or unsupported or unloaded or truncated or composition_errors)
    result = {
        "schema": "vlagraph.usd_collision_inventory.v1", "source": "composed_usd_default_time",
        "stage_identifier": stage.GetRootLayer().identifier, "robot_prim_path": robot_prim_path,
        "root_path": root_path, "meters_per_unit": units, "up_axis": up,
        "output_world_frame": "right_handed_z_up_m", "colliders": colliders,
        "joint_bindings": bindings, "joint_collider_coverage": coverage,
        "missing_joint_bindings": missing_joints, "missing_joint_colliders": missing_shapes,
        "disabled_collision_prims": disabled, "unsupported_collision_prims": unsupported,
        "unloaded_payloads": unloaded, "composition_errors": composition_errors, "issues": issues,
        "truncated": truncated, "boundables_without_collision_api": no_collision_count,
        "robot_joint_collision_coverage_complete": complete,
        "proxy_scene_ready": False, "physical_collision_verified": False, "execution_ready": False,
        "unchecked": ["project_fk_to_usd_joint_frame_binding", "base_and_tool_coverage", "contact_skin_defaults",
                      "environment_collision_coverage", "physical_snapshot_synchronization", "physical_rollout"],
        "static_obstacle_candidates": [{"id": item["prim_path"], **item["world_aabb_m_zup"]}
                                       for item in colliders if item["role"] == "static_obstacle"],
    }
    result["inventory_digest"] = collision_inventory_digest(result)
    return result
