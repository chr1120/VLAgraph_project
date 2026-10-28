"""Read-only point-convex-hull diagnostics at recorded review poses.

This is inspection evidence, not PhysX cooking, continuous collision checking,
or an automatic change to the execution collision report/exclusion policy.
"""
from __future__ import annotations

import itertools

import numpy as np

from .convex_distance import convex_distance_bounds
from .fanuc_geometry_review import build_geometry_review
from .fanuc_preflight import action_fingerprint, validate_fanuc_trajectory
from .fanuc_usd_proxy import _attachment_frames, _body_frame, _rotation
from .usd_geometry import extract_collision_inventory


def _points_in_anchor(stage, inventory, source, anchors):
    from pxr import Usd, UsdGeom, UsdPhysics

    prim = stage.GetPrimAtPath(source["prim_path"])
    if not prim or not prim.HasAPI(UsdPhysics.CollisionAPI) or not UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get():
        raise ValueError("Active collision prim is unavailable")
    if prim.IsA(UsdGeom.Mesh) and source.get("collision_approximation") == "convexHull":
        raw = UsdGeom.Mesh(prim).GetPointsAttr().Get(Usd.TimeCode.Default())
        kind = "convex_hull_of_authored_mesh_points"
    elif prim.IsA(UsdGeom.Cube):
        extent = UsdGeom.Boundable.ComputeExtentFromPlugins(UsdGeom.Boundable(prim), Usd.TimeCode.Default())
        raw = list(itertools.product(*zip(*extent)))
        kind = "authored_cube_vertices"
    else:
        raise ValueError("Only Cube and Mesh with authored convexHull approximation are supported")
    points = np.asarray(raw, dtype=float)
    if (points.ndim != 2 or points.shape[1:] != (3,) or not 1 <= len(points) <= 20000
            or not np.all(np.isfinite(points))):
        raise ValueError("Unsupported or incomplete collision point cloud")
    matrix = np.asarray(source["world_transform_stage"], dtype=float)
    world = (points @ matrix[:3, :3] + matrix[3, :3]) * inventory["meters_per_unit"]
    owner = source.get("rigid_body_path")
    if owner is None:
        anchor = {"attachment": "world"}
        anchored = world[:, [0, 2, 1]] * [1, -1, 1] if inventory["up_axis"] == "Y" else world
    else:
        if owner not in anchors:
            raise ValueError("Collision body has no verified kinematic attachment")
        anchor = anchors[owner]
        body = _body_frame(inventory, owner)
        local = (world - body[:3, 3]) @ body[:3, :3]
        transform = anchor["body_to_anchor"]
        anchored = local @ transform[:3, :3].T + transform[:3, 3]
    if not np.all(np.isfinite(anchored)) or np.max(np.abs(anchored)) > 1e6:
        raise ValueError("Collision point cloud exceeds supported coordinate range")
    return anchored, anchor, {"prim_path": source["prim_path"], "representation": kind,
                             "point_count": len(points), "anchor_points_digest": action_fingerprint(anchored.tolist())}


def _world_points(points, anchor, scene, positions):
    from backend.executors.fanuc_nodes import _rotation as rotate_axis
    from .robot_profiles import profile_arrays
    M710_JOINT_ORIGINS, M710_JOINT_AXES, _, _ = profile_arrays(scene.get("robot_profile"))

    if anchor["attachment"] == "world":
        return points.copy()
    rotation = _rotation(scene["base_orientation_wxyz"])
    position = np.asarray(scene["base_position"], dtype=float)
    external = scene.get("external_axis")
    if external is not None:
        if "externalAxisJoint" not in positions:
            raise ValueError("Inspection pose must specify the scene external axis")
        position = position + np.asarray(external["direction_world"]) * (positions["externalAxisJoint"] - external["reference_position_m"])
    elif "externalAxisJoint" in positions:
        raise ValueError("Inspection scene lacks external axis mapping")
    if anchor["attachment"] == "joint":
        for index in range(anchor["joint_index"] + 1):
            position = position + rotation @ M710_JOINT_ORIGINS[index]
            rotation = rotation @ rotate_axis(M710_JOINT_AXES[index], positions[f"J{index+1}"])
    return points @ rotation.T + position


def _positions_at(trajectory, at_time):
    if isinstance(at_time, bool) or not isinstance(at_time, (int, float)) or not np.isfinite(at_time):
        raise ValueError("Inspection time must be finite")
    if not trajectory[0]["time_sec"] <= at_time <= trajectory[-1]["time_sec"]:
        raise ValueError("Inspection time is outside the checked trajectory")
    for frame in trajectory:
        if at_time == frame["time_sec"]:
            return frame["positions"]
    for left, right in zip(trajectory, trajectory[1:]):
        if left["time_sec"] < at_time < right["time_sec"]:
            alpha = (at_time - left["time_sec"]) / (right["time_sec"] - left["time_sec"])
            blend = alpha * alpha * (3 - 2 * alpha)
            return {name: value + blend * (right["positions"][name] - value) for name, value in left["positions"].items()}
    raise ValueError("Inspection pose is unavailable")


def _inspection_context(stage, inventory, scene, action, collision_report, snapshot):
    current = extract_collision_inventory(stage, robot_prim_path=inventory["robot_prim_path"],
                                          root_path=inventory["root_path"])
    if current["inventory_digest"] != inventory["inventory_digest"]:
        raise ValueError("Current composed USD inventory differs; regenerate proxy and review evidence")
    plan = build_geometry_review(inventory, scene, collision_report=collision_report)
    if collision_report.get("action_digest") != action_fingerprint(action):
        raise ValueError("Collision report does not match the inspected action")
    preflight = validate_fanuc_trajectory(action, snapshot=snapshot)
    if not preflight["static_checks_passed"] or collision_report.get("snapshot_ref") != preflight["snapshot_ref"]:
        raise ValueError("Inspection action/snapshot must match a passing static preflight")
    trajectory = list(preflight["normalized_trajectory"])
    if snapshot is not None:
        from .fanuc import FanucPhysicsSnapshot
        observed = FanucPhysicsSnapshot.model_validate(snapshot)
        if (observed.prim_path != inventory["robot_prim_path"]
                or observed.geometry.get("stage_identifier") != inventory["stage_identifier"]
                or observed.geometry.get("meters_per_unit") != inventory["meters_per_unit"]
                or observed.geometry.get("up_axis") != inventory["up_axis"]):
            raise ValueError("Inspection snapshot robot/stage/units must match the inventory")
        trajectory.insert(0, {"time_sec": 0, "positions": {axis.name: axis.position for axis in observed.axes}})
    anchors = _attachment_frames(inventory)
    sources = {item["prim_path"]: item for item in inventory["colliders"]}
    capsules = {item["id"]: item for item in scene["link_capsules"]}
    candidates = [item for item in plan["pair_decisions"] if item["usd_relation"] == "no_direct_joint_connection"
                  and item["sampled_diagnostic"] and item["sampled_diagnostic"].get("margin_violation_samples", 0)]
    if len(candidates) > 64:
        raise ValueError("At most 64 review pairs may be inspected per request")
    return trajectory, anchors, sources, capsules, candidates


def inspect_review_hulls(stage, inventory, scene, action, collision_report, *, snapshot=None):
    trajectory, anchors, sources, capsules, candidates = _inspection_context(
        stage, inventory, scene, action, collision_report, snapshot)
    results, cache = [], {}
    for item in candidates:
        at_time = item["sampled_diagnostic"]["min_time_sec"]
        entry = {"pair": item["pair"], "body_paths": item["body_paths"], "time_sec": at_time,
                 "capsule_min_sampled_clearance_m": item["sampled_diagnostic"]["min_sampled_clearance_m"]}
        try:
            positions = _positions_at(trajectory, at_time)
            clouds, descriptions = [], []
            for identifier in item["pair"]:
                source_path = capsules[identifier]["source_prim_path"]
                if source_path not in cache:
                    cache[source_path] = _points_in_anchor(stage, inventory, sources[source_path], anchors)
                points, anchor, description = cache[source_path]
                clouds.append(_world_points(points, anchor, scene, positions))
                descriptions.append(description)
            entry.update(convex_distance_bounds(*clouds, margin_m=collision_report["margin_m"]))
            entry.update(geometry=descriptions, joint_positions=positions)
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as exc:
            entry.update(status="indeterminate", message=str(exc), margin_separation_certified=False)
        results.append(entry)
    result = {"schema": "vlagraph.fanuc_hull_inspection.v1", "inventory_digest": inventory["inventory_digest"],
              "scene_digest": action_fingerprint(scene), "action_digest": action_fingerprint(action),
              "collision_report_digest": action_fingerprint(collision_report), "pairs": results,
              "scope": "nonadjacent_capsule_violations_at_recorded_minimum_sample_pose_only",
              "motion_requested": False, "physical_collision_verified": False, "execution_ready": False,
              "unchecked": ["physx_cooked_collision_shapes", "contact_skin_defaults", "other_trajectory_poses",
                            "between_sample_motion", "live_physics_state", "tool_and_environment_completeness"],
              "exclusion_policy": "unchanged"}
    result["inspection_digest"] = action_fingerprint(result)
    return result
