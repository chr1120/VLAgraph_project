"""Conservative collision diagnostics for explicitly configured geometric proxies.

Not an Isaac Sim physical rollout. No geometry is inferred from visual bounds.
"""
from __future__ import annotations

import math
import numpy as np

from .fanuc_preflight import action_fingerprint, validate_fanuc_trajectory


def segment_box_distance(start, end, lower, upper):
    """Exact Euclidean segment/AABB distance via piecewise quadratic minima."""
    start, end, lower, upper = [np.asarray(value, dtype=float) for value in (start, end, lower, upper)]
    direction = end - start
    breaks = {0.0, 1.0}
    for index in range(3):
        if direction[index] != 0:
            for boundary in (lower[index], upper[index]):
                parameter = (boundary - start[index]) / direction[index]
                if 0 < parameter < 1:
                    breaks.add(float(parameter))
    breaks = sorted(breaks)
    candidates = list(breaks)
    for left, right in zip(breaks, breaks[1:]):
        point = start + direction * ((left + right) / 2)
        active = (point < lower) | (point > upper)
        boundary = np.where(point < lower, lower, upper)
        slope = direction[active]
        intercept = (start - boundary)[active]
        quadratic = float(slope @ slope)
        if quadratic > 0:
            candidates.append(float(np.clip(-float(slope @ intercept) / quadratic, left, right)))
    return min(float(np.linalg.norm(np.maximum(np.maximum(lower - (start + t * direction), start + t * direction - upper), 0)))
               for t in candidates)


def segment_segment_distance(a, b, c, d):
    a, b, c, d = [np.asarray(value, dtype=float) for value in (a, b, c, d)]
    u, v = b - a, d - c
    candidates = []
    for point, start, end in ((a, c, d), (b, c, d), (c, a, b), (d, a, b)):
        direction = end - start
        length_sq = float(direction @ direction)
        t = float(np.clip((point - start) @ direction / length_sq, 0, 1)) if length_sq else 0
        candidates.append(float(np.linalg.norm(point - start - t * direction)))
    solution = np.linalg.lstsq(np.column_stack((u, -v)), c - a, rcond=None)[0]
    if np.all(solution >= 0) and np.all(solution <= 1):
        candidates.append(float(np.linalg.norm(a + solution[0] * u - c - solution[1] * v)))
    return min(candidates)


def check_fanuc_collision(action, scene, *, max_step_sec=0.05, margin_m=0.01, max_samples=10000, snapshot=None):
    from backend.executors.fanuc_nodes import M710_JOINT_ORIGINS, M710_JOINT_AXES, _rotation

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("Geometry values must be finite numbers")
        return float(value)

    def vector(value):
        if not isinstance(value, (list, tuple)) or len(value) != 3:
            raise ValueError("Expected a three-component vector")
        parsed = np.asarray([number(v) for v in value])
        if np.max(np.abs(parsed)) > 1e6:
            raise ValueError("Geometry exceeds the supported coordinate range")
        return parsed

    max_step_sec, margin_m = number(max_step_sec), number(margin_m)
    if not 0.001 <= max_step_sec <= 2 or not 0 <= margin_m <= 1:
        raise ValueError("maxStepSec must be 0.001..2; marginM must be 0..1")
    if not isinstance(max_samples, int) or isinstance(max_samples, bool) or not 2 <= max_samples <= 10000:
        raise ValueError("maxSamples must be an integer from 2 to 10000")
    if not isinstance(scene, dict) or scene.get("schema") != "vlagraph.fanuc_collision_scene.v1":
        raise ValueError("Explicit FANUC collision scene is required")
    if scene.get("frame") != "right_handed_z_up_m" or scene.get("geometry_source") != "configured_proxy":
        raise ValueError("Only configured meter/Z-up proxies are supported")
    if scene.get("kinematics_source") != "project_m710_configuration":
        raise ValueError("Proxy geometry must explicitly use project_m710_configuration kinematics")
    source_coverage = scene.get("geometry_coverage")
    unmodeled_geometry = []
    if source_coverage is not None:
        if not isinstance(source_coverage, dict):
            raise ValueError("geometry_coverage must be an object")
        for key in ("unmapped_robot_colliders", "unmodeled_moving_obstacles"):
            if not isinstance(source_coverage.get(key), list):
                raise ValueError(f"geometry_coverage.{key} must be a list")
            if source_coverage[key]:
                unmodeled_geometry.append(key)
        if source_coverage.get("all_enabled_robot_colliders_mapped") is not True:
            unmodeled_geometry.append("robot_collider_mapping_incomplete")
        if source_coverage.get("declared_geometry_incomplete") is True:
            unmodeled_geometry.append("declared_geometry_incomplete")
    if not scene.get("scene_id") or not scene.get("scene_version") or not scene.get("robot_model_version"):
        raise ValueError("Scene and robot model versions are required")
    base = vector(scene.get("base_position"))
    quaternion = scene.get("base_orientation_wxyz")
    if not isinstance(quaternion, list) or len(quaternion) != 4:
        raise ValueError("Explicit base quaternion wxyz is required")
    w, x, y, z = [number(v) for v in quaternion]
    if not math.isclose(w*w+x*x+y*y+z*z, 1, abs_tol=1e-6):
        raise ValueError("Base quaternion must have unit length")
    norm = math.sqrt(w*w+x*x+y*y+z*z)
    w, x, y, z = [value / norm for value in (w, x, y, z)]
    base_rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                              [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                              [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    raw_capsules, raw_boxes = scene.get("link_capsules"), scene.get("obstacles")
    if not isinstance(raw_capsules, list) or not 1 <= len(raw_capsules) <= 64 or not isinstance(raw_boxes, list) or len(raw_boxes) > 256:
        raise ValueError("Require 1..64 link capsules and an explicit list of at most 256 obstacles")
    capsules, boxes = [], []
    identifiers = set()
    for item in raw_capsules:
        if not isinstance(item, dict):
            raise ValueError("Capsule must be an object")
        identifier, joint = item.get("id"), item.get("joint_index")
        attachment = item.get("attachment", "joint")
        if not isinstance(identifier, str) or not identifier or identifier in identifiers:
            raise ValueError("Geometry IDs must be unique nonempty strings")
        identifiers.add(identifier)
        if attachment not in {"joint", "base", "world"}:
            raise ValueError("Capsule attachment must be joint, base or world")
        if attachment == "joint" and (isinstance(joint, bool) or not isinstance(joint, int) or not 0 <= joint <= 5):
            raise ValueError("Capsule joint_index must be 0..5 (after that joint's rotation)")
        if attachment != "joint" and joint is not None:
            raise ValueError("Base/world capsules cannot specify joint_index")
        radius = number(item.get("radius_m"))
        if not 0 < radius <= 10:
            raise ValueError("Capsule radius must be positive and at most 10m")
        start, end = vector(item.get("start_local")), vector(item.get("end_local"))
        levers = ([sum(float(np.linalg.norm(origin)) for origin in M710_JOINT_ORIGINS[i+1:joint+1])
                   + max(float(np.linalg.norm(start)), float(np.linalg.norm(end))) for i in range(joint+1)]
                  if attachment == "joint" else [])
        capsules.append({"id": identifier, "joint": joint, "start": start, "end": end,
                         "radius": radius, "levers": levers, "attachment": attachment})
    for item in raw_boxes:
        if not isinstance(item, dict) or item.get("id") in identifiers or not isinstance(item.get("id"), str) or not item["id"]:
            raise ValueError("Obstacle IDs must be unique nonempty strings")
        identifiers.add(item["id"])
        lower, upper = vector(item.get("min")), vector(item.get("max"))
        if np.any(lower >= upper):
            raise ValueError("Obstacle min must be strictly less than max")
        boxes.append({"id": item["id"], "lower": lower, "upper": upper})
    exclusions = set()
    capsule_ids = {item["id"] for item in capsules}
    for pair in scene.get("excluded_self_pairs", []):
        if not isinstance(pair, list) or len(pair) != 2 or pair[0] == pair[1] or not set(pair).issubset(capsule_ids):
            raise ValueError("Excluded self pairs must name two distinct configured capsules")
        exclusions.add(tuple(sorted(pair)))
    pairs = [(i, j) for i in range(len(capsules)) for j in range(i+1, len(capsules))
             if tuple(sorted((capsules[i]["id"], capsules[j]["id"]))) not in exclusions]
    preflight = validate_fanuc_trajectory(action, snapshot=snapshot)
    if not preflight["static_checks_passed"]:
        raise ValueError("Static preflight must pass before geometric collision analysis")
    trajectory = list(preflight["normalized_trajectory"])
    if snapshot is not None:
        from .fanuc import FanucPhysicsSnapshot
        observed = FanucPhysicsSnapshot.model_validate(snapshot)
        if scene.get("inventory_digest"):
            if (scene.get("robot_prim_path") != observed.prim_path
                    or scene.get("scene_id") != observed.geometry.get("stage_identifier")
                    or scene.get("source_meters_per_unit") != observed.geometry.get("meters_per_unit")
                    or scene.get("source_up_axis") != observed.geometry.get("up_axis")):
                raise ValueError("USD proxy scene and initial snapshot robot/stage/units differ")
        trajectory.insert(0, {"frame": "initial_snapshot", "time_sec": 0.0,
                              "positions": {axis.name: axis.position for axis in observed.axes}})
    if len(trajectory) < 2:
        raise ValueError("At least two trajectory frames are required")
    axis_names = set(trajectory[0]["positions"])
    external = scene.get("external_axis")
    direction, reference = np.zeros(3), 0.0
    if "externalAxisJoint" in axis_names:
        if not isinstance(external, dict):
            raise ValueError("External axis requires explicit world direction and reference position")
        direction = vector(external.get("direction_world"))
        if not math.isclose(float(np.linalg.norm(direction)), 1, abs_tol=1e-6):
            raise ValueError("External axis world direction must be a unit vector")
        reference = number(external.get("reference_position_m"))
    elif external is not None:
        raise ValueError("Scene external axis is absent from the trajectory")
    durations = [b["time_sec"]-a["time_sec"] for a, b in zip(trajectory, trajectory[1:])]
    if any(duration / max_step_sec > max_samples for duration in durations):
        raise ValueError("Sample budget exceeded")
    sample_count = 1 + sum(math.ceil(duration / max_step_sec) for duration in durations)
    if sample_count > max_samples:
        raise ValueError("Sample budget exceeded; reduce duration or increase maxStepSec explicitly")
    comparison_count = len(capsules) * len(boxes) + len(pairs)
    if sample_count * comparison_count > 250000:
        raise ValueError("Geometry comparison budget exceeded")

    def poses(positions):
        moving_base = base + direction * (positions.get("externalAxisJoint", reference) - reference)
        rotation, position = base_rotation.copy(), moving_base.copy()
        frames = []
        for index in range(6):
            position = position + rotation @ M710_JOINT_ORIGINS[index]
            rotation = rotation @ _rotation(M710_JOINT_AXES[index], positions[f"J{index+1}"])
            frames.append((position.copy(), rotation.copy()))
        geometry = []
        for item in capsules:
            if item["attachment"] == "world":
                origin, rotation = np.zeros(3), np.eye(3)
            elif item["attachment"] == "base":
                origin, rotation = moving_base, base_rotation
            else:
                origin, rotation = frames[item["joint"]]
            geometry.append((origin + rotation @ item["start"], origin + rotation @ item["end"]))
        return geometry

    def clearances(geometry):
        values = {}
        for i, capsule in enumerate(capsules):
            for box in boxes:
                values[(capsule["id"], box["id"])] = segment_box_distance(*geometry[i], box["lower"], box["upper"]) - capsule["radius"]
        for i, j in pairs:
            values[(capsules[i]["id"], capsules[j]["id"])] = segment_segment_distance(*geometry[i], *geometry[j]) - capsules[i]["radius"] - capsules[j]["radius"]
        return values

    contacts, unresolved = [], []
    pair_diagnostics = {}
    tolerance = 1e-7
    minimum = None
    previous_values, previous_positions = None, None
    checked = 0
    for segment_index, (left, right) in enumerate(zip(trajectory, trajectory[1:])):
        count = math.ceil((right["time_sec"] - left["time_sec"]) / max_step_sec)
        for step in range(0 if segment_index == 0 else 1, count + 1):
            alpha = step / count
            blend = alpha * alpha * (3 - 2 * alpha)
            positions = {name: left["positions"][name] + blend * (right["positions"][name] - left["positions"][name]) for name in axis_names}
            at_time = left["time_sec"] + alpha * (right["time_sec"] - left["time_sec"])
            values = clearances(poses(positions))
            checked += 1
            if values:
                current_min = min(values.values())
                minimum = current_min if minimum is None else min(minimum, current_min)
            for pair, distance in values.items():
                detail = pair_diagnostics.setdefault(pair, {
                    "pair": list(pair), "kind": "self" if pair[1] in capsule_ids else "environment",
                    "min_sampled_clearance_m": distance, "min_time_sec": at_time,
                    "samples_checked": 0, "margin_violation_samples": 0,
                })
                detail["samples_checked"] += 1
                if distance < detail["min_sampled_clearance_m"]:
                    detail["min_sampled_clearance_m"], detail["min_time_sec"] = distance, at_time
                if distance <= margin_m + tolerance:
                    detail["margin_violation_samples"] += 1
                if distance <= margin_m + tolerance and len(contacts) < 100:
                    contacts.append({"pair": list(pair), "time_sec": at_time, "clearance_m": distance})
            if previous_values is not None:
                bounds = {}
                rail_bound = abs(positions.get("externalAxisJoint", 0) - previous_positions.get("externalAxisJoint", 0))
                for item in capsules:
                    bounds[item["id"]] = (0 if item["attachment"] == "world" else rail_bound) + sum(
                        lever * abs(positions[f"J{i+1}"] - previous_positions[f"J{i+1}"])
                        for i, lever in enumerate(item["levers"]))
                for pair, distance in previous_values.items():
                    bound = bounds.get(pair[0], 0) + bounds.get(pair[1], 0)
                    if distance <= margin_m + bound + tolerance and len(unresolved) < 100:
                        unresolved.append({"pair": list(pair), "end_time_sec": at_time, "motion_bound_m": bound})
            previous_values, previous_positions = values, positions
    status = "collision" if contacts else "indeterminate" if unresolved or not comparison_count or unmodeled_geometry else "clear"
    return {"schema": "vlagraph.fanuc_collision_report.v1", "status": status,
            "action_digest": action_fingerprint(action), "scene_digest": action_fingerprint(scene),
            "scene_id": scene["scene_id"], "scene_version": scene["scene_version"],
            "robot_model_version": scene["robot_model_version"], "geometry_source": "configured_proxy",
            "inventory_digest": scene.get("inventory_digest"), "source_coverage": scene.get("coverage", "configured_proxies"),
            "snapshot_ref": preflight["snapshot_ref"], "initial_segment_checked": snapshot is not None,
            "checked_time_range_sec": [trajectory[0]["time_sec"], trajectory[-1]["time_sec"]],
            "kinematics_source": "project_m710_configuration", "geometry_tolerance_m": tolerance,
            "coverage": "configured_capsules_and_static_aabbs", "samples_checked": checked,
            "min_sampled_clearance_m": minimum, "margin_m": margin_m,
            "contacts": contacts, "unresolved_intervals": unresolved,
            "pair_diagnostics": list(pair_diagnostics.values()), "contacts_limit": 100,
            "self_pairs_checked": len(pairs), "excluded_self_pairs": [list(pair) for pair in sorted(exclusions)],
            "capsules_by_attachment": {attachment: sum(item["attachment"] == attachment for item in capsules)
                                       for attachment in ("joint", "base", "world")},
            "source_geometry_coverage": scene.get("geometry_coverage"),
            "unmodeled_geometry": unmodeled_geometry,
            "between_samples_certified": status == "clear",
            "physical_collision_verified": False, "execution_ready": False,
            "unchecked": list(dict.fromkeys([item for item in ["initial_motion_before_first_frame", "actual_mesh_coverage",
                                                "dynamic_obstacles", "physical_rollout", "physical_base_alignment", "live_state_at_dispatch"]
                                             + list(scene.get("unchecked", []))
                                             if item != "initial_motion_before_first_frame" or snapshot is None]))}
