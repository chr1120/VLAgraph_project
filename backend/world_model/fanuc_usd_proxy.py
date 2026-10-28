"""Validate USD joint contracts and construct conservative robot proxies.

Default-time USD transforms are not synchronized physics observations. This
module never starts physics, moves bodies, or grants execution permission.
"""
from __future__ import annotations

import math
import itertools

import numpy as np

from .usd_geometry import collision_inventory_digest


POSITION_TOLERANCE_M = 1e-6
ROTATION_TOLERANCE = 1e-6


def _array(value, shape):
    result = np.asarray(value, dtype=float)
    if result.shape != shape or not np.all(np.isfinite(result)):
        raise ValueError(f"Expected finite array of shape {shape}")
    return result


def _rotation(quaternion):
    w, x, y, z = _array(quaternion, (4,))
    if not math.isclose(w*w+x*x+y*y+z*z, 1, abs_tol=ROTATION_TOLERANCE):
        raise ValueError("Joint quaternion must have unit length")
    w, x, y, z = np.asarray([w, x, y, z]) / np.linalg.norm([w, x, y, z])
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def _quaternion(rotation):
    # Symmetric eigenproblem avoids singular branches at 180-degree rotations.
    r = rotation
    matrix = np.array([
        [r[0, 0]-r[1, 1]-r[2, 2], r[0, 1]+r[1, 0], r[0, 2]+r[2, 0], r[2, 1]-r[1, 2]],
        [r[0, 1]+r[1, 0], r[1, 1]-r[0, 0]-r[2, 2], r[1, 2]+r[2, 1], r[0, 2]-r[2, 0]],
        [r[0, 2]+r[2, 0], r[1, 2]+r[2, 1], r[2, 2]-r[0, 0]-r[1, 1], r[1, 0]-r[0, 1]],
        [r[2, 1]-r[1, 2], r[0, 2]-r[2, 0], r[1, 0]-r[0, 1], np.trace(r)],
    ])
    _, vectors = np.linalg.eigh(matrix)
    x, y, z, w = vectors[:, -1]
    result = np.array([w, x, y, z])
    return (result if w >= 0 else -result).tolist()


def _transform(position, rotation):
    result = np.eye(4)
    result[:3, :3], result[:3, 3] = rotation, position
    return result


def _body_frame(inventory, path):
    # USD/Gf uses row vectors; the validator uses column vectors throughout.
    matrix = _array(inventory["body_world_transforms_stage"][path], (4, 4)).T.copy()
    if not np.allclose(matrix[3], [0, 0, 0, 1], atol=1e-9, rtol=0):
        raise ValueError("Body transform must be affine")
    rotation = matrix[:3, :3]
    if (not np.allclose(rotation.T @ rotation, np.eye(3), atol=ROTATION_TOLERANCE, rtol=0)
            or not math.isclose(float(np.linalg.det(rotation)), 1, abs_tol=ROTATION_TOLERANCE)):
        raise ValueError("Scaled, reflected or sheared body frames are unsupported")
    matrix[:3, 3] *= inventory["meters_per_unit"]
    return matrix


def _joint_frame(joint, side, units):
    return _transform(_array(joint[f"local_pos{side}_stage"], (3,)) * units,
                      _rotation(joint[f"local_rot{side}_wxyz"]))


def _axis(joint):
    if joint.get("axis") not in {"X", "Y", "Z"}:
        raise ValueError("Unsupported joint axis")
    return np.eye(3)["XYZ".index(joint["axis"])]


def _check_inventory(inventory):
    if not isinstance(inventory, dict) or inventory.get("schema") != "vlagraph.usd_collision_inventory.v1":
        raise ValueError("USD collision inventory is required")
    if inventory.get("inventory_digest") != collision_inventory_digest(inventory):
        raise ValueError("USD inventory digest mismatch")
    if inventory.get("robot_joint_collision_coverage_complete") is not True:
        raise ValueError("USD joint collision coverage must be complete")
    units = inventory.get("meters_per_unit")
    if isinstance(units, bool) or not isinstance(units, (int, float)) or not math.isfinite(units) or units <= 0:
        raise ValueError("Invalid USD units")
    if inventory.get("up_axis") not in {"Y", "Z"}:
        raise ValueError("Unsupported world up axis")
    return units


def _external_axis(inventory, base_path):
    """Follow explicit fixed parents to at most one prismatic rail joint."""
    units = inventory["meters_per_unit"]
    auxiliaries = inventory.get("auxiliary_joint_bindings", [])
    current, seen, external = base_path, set(), None
    while True:
        if current in seen:
            raise ValueError("Cycle in base/external-axis joint chain")
        seen.add(current)
        parents = [joint for joint in auxiliaries if current in joint.get("body1", [])]
        if not parents:
            if current != inventory["robot_prim_path"]:
                # A fixed-base robot may have no auxiliary joints at all.
                if current != base_path:
                    raise ValueError("Base chain must terminate at the selected robot root")
            break
        if len(parents) != 1:
            raise ValueError("Ambiguous base joint chain")
        joint = parents[0]
        if joint.get("enabled") is not True or len(joint["body0"]) != 1 or joint["body1"] != [current]:
            raise ValueError("Invalid or disabled base joint")
        parent = joint["body0"][0]
        frame0 = _body_frame(inventory, parent) @ _joint_frame(joint, 0, units)
        frame1 = _body_frame(inventory, current) @ _joint_frame(joint, 1, units)
        relative = np.linalg.inv(frame0) @ frame1
        if not np.allclose(relative[:3, :3], np.eye(3), atol=ROTATION_TOLERANCE, rtol=0):
            raise ValueError("Authored base joint rotations do not satisfy joint frames")
        displacement = relative[:3, 3]
        if joint["type"] == "PhysicsFixedJoint":
            if np.linalg.norm(displacement) > POSITION_TOLERANCE_M:
                raise ValueError("Authored fixed joint positions disagree")
        elif joint["type"] == "PhysicsPrismaticJoint" and external is None:
            axis = _axis(joint)
            reference = float(displacement @ axis)
            if np.linalg.norm(displacement - axis * reference) > POSITION_TOLERANCE_M:
                raise ValueError("Authored external-axis displacement is off axis")
            external = {"direction_world": (frame0[:3, :3] @ axis).tolist(),
                        "reference_position_m": reference,
                        "source_joint_path": joint["joint_path"]}
        else:
            raise ValueError("Only fixed joints and one prismatic external axis are supported")
        current = parent
    return external


def validate_fanuc_usd_binding(inventory, *, profile=None):
    from backend.executors.fanuc_nodes import _rotation as rotate
    from .robot_profiles import profile_arrays
    M710_JOINT_ORIGINS, M710_JOINT_AXES, M710_JOINT_LIMITS, _ = profile_arrays(profile)

    units = _check_inventory(inventory)
    joints = inventory["joint_bindings"]
    errors, details = [], []
    base_path = None
    for index in range(6):
        name = f"J{index+1}"
        try:
            joint = joints[name]
            if any(item.get("enabled") is True and joint["body1"] in item.get("body1", [])
                   for item in inventory.get("auxiliary_joint_bindings", [])):
                raise ValueError("Arm body is constrained by an additional joint")
            if len(joint["body0"]) != 1 or not joint["body1"].startswith(inventory["robot_prim_path"].rstrip("/") + "/"):
                raise ValueError("Joint bodies must belong to the selected robot")
            if index and joint["body0"] != [joints[f"J{index}"]["body1"]]:
                raise ValueError("Joint parent does not match the previous arm link")
            if not index:
                base_path = joint["body0"][0]
                if not base_path.startswith(inventory["robot_prim_path"].rstrip("/") + "/"):
                    raise ValueError("Arm base must belong to the selected robot")
            _body_frame(inventory, joint["body0"][0])
            _body_frame(inventory, joint["body1"])
            frame0, frame1 = _joint_frame(joint, 0, units), _joint_frame(joint, 1, units)
            origin_error = float(np.linalg.norm(frame0[:3, 3] - M710_JOINT_ORIGINS[index]))
            axis_error = float(np.linalg.norm(frame0[:3, :3] @ _axis(joint) - M710_JOINT_AXES[index]))
            if origin_error > POSITION_TOLERANCE_M or axis_error > ROTATION_TOLERANCE:
                raise ValueError("USD joint origin or signed axis differs from project FK")
            if (np.linalg.norm(frame1[:3, 3]) > POSITION_TOLERANCE_M
                    or not np.allclose(frame0[:3, :3], frame1[:3, :3], atol=ROTATION_TOLERANCE, rtol=0)):
                raise ValueError("USD child joint frame differs from project FK")
            limits = _array([joint["lower_limit_deg"], joint["upper_limit_deg"]], (2,))
            if not np.allclose(np.radians(limits), M710_JOINT_LIMITS[index], atol=ROTATION_TOLERANCE, rtol=0):
                raise ValueError("USD joint limits differ from project preflight limits")
            details.append({"joint": name, "origin_error_m": origin_error, "signed_axis_error": axis_error})
        except (ValueError, TypeError, KeyError) as exc:
            errors.append({"joint": name, "message": str(exc)})
    base, external = None, None
    samples = []
    if not errors:
        try:
            base = _body_frame(inventory, base_path)
            external = _external_axis(inventory, base_path)
            # Independently compose the USD joint relation T0*R(q)*inverse(T1).
            for angles in ([0]*6, [0.2, -0.3, 0.4, -0.5, 0.6, -0.7], [-0.4, 0.5, -0.6, 0.7, -0.8, 0.9]):
                usd, project = np.eye(4), np.eye(4)
                maximum = 0.0
                for index, angle in enumerate(angles):
                    joint = joints[f"J{index+1}"]
                    usd = usd @ _joint_frame(joint, 0, units) @ _transform(np.zeros(3), rotate(_axis(joint), angle)) @ np.linalg.inv(_joint_frame(joint, 1, units))
                    project = project @ _transform(M710_JOINT_ORIGINS[index], rotate(M710_JOINT_AXES[index], angle))
                    maximum = max(maximum, float(np.max(np.abs(usd - project))))
                samples.append({"joint_angles_rad": angles, "max_transform_error": maximum})
                if maximum > 5e-6:
                    raise ValueError("USD joint composition and project FK disagree")
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as exc:
            errors.append({"joint": "base_chain", "message": str(exc)})
    conversion = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) if inventory["up_axis"] == "Y" else np.eye(3)
    if external:
        external["direction_world"] = (conversion @ external["direction_world"]).tolist()
    return {"schema": "vlagraph.fanuc_usd_binding.v1", "inventory_digest": inventory["inventory_digest"],
            "kinematics_match": not errors, "errors": errors, "joints": details, "composition_samples": samples,
            "position_tolerance_m": POSITION_TOLERANCE_M, "rotation_tolerance": ROTATION_TOLERANCE,
            "base_position": (conversion @ base[:3, 3]).tolist() if base is not None else None,
            "base_orientation_wxyz": _quaternion(conversion @ base[:3, :3]) if base is not None else None,
            "external_axis": external, "pose_source": "usd_default_time", "physics_synchronized": False,
            "physical_collision_verified": False, "execution_ready": False}


def _attachment_frames(inventory):
    """Map validated arm/base bodies and rigid fixed attachments to anchors."""
    units = inventory["meters_per_unit"]
    conversion = np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]]) if inventory["up_axis"] == "Y" else np.eye(3)
    world_conversion = _transform(np.zeros(3), conversion)
    joints = inventory["joint_bindings"]
    anchors = {joint["body1"]: {"attachment": "joint", "joint_index": index,
                               "body_to_anchor": np.eye(4)} for index, joint in
               enumerate(joints[f"J{i}"] for i in range(1, 7))}
    auxiliaries = inventory.get("auxiliary_joint_bindings", [])
    current = joints["J1"]["body0"][0]
    base = _body_frame(inventory, current)
    attachment, seen = "base", set()
    while current not in seen:
        seen.add(current)
        body = _body_frame(inventory, current)
        anchors[current] = {"attachment": attachment,
                            "body_to_anchor": np.linalg.inv(base) @ body if attachment == "base" else world_conversion @ body}
        parents = [joint for joint in auxiliaries if joint["body1"] == [current]]
        if not parents:
            break
        joint = parents[0]  # The base-chain validator already checked uniqueness.
        if joint["type"] == "PhysicsPrismaticJoint":
            attachment = "world"
        current = joint["body0"][0]
    # Additional tools/fixtures must have an explicit enabled fixed joint.
    # A visual hierarchy alone never binds a moving collider to an arm link.
    for _ in range(len(auxiliaries)):
        changed = False
        for joint in auxiliaries:
            if (joint["type"] != "PhysicsFixedJoint" or joint.get("enabled") is not True
                    or len(joint["body0"]) != 1 or len(joint["body1"]) != 1):
                continue
            parent, child = joint["body0"][0], joint["body1"][0]
            if parent not in anchors or child in anchors:
                continue
            parents = [item for item in auxiliaries if child in item["body1"]]
            if len(parents) != 1 or child in {item["body1"] for item in joints.values()}:
                continue
            relative = _joint_frame(joint, 0, units) @ np.linalg.inv(_joint_frame(joint, 1, units))
            expected = _body_frame(inventory, parent) @ relative
            actual = _body_frame(inventory, child)
            if (np.linalg.norm(expected[:3, 3] - actual[:3, 3]) > POSITION_TOLERANCE_M
                    or not np.allclose(expected[:3, :3], actual[:3, :3], atol=ROTATION_TOLERANCE, rtol=0)):
                continue
            anchors[child] = {**anchors[parent], "body_to_anchor": anchors[parent]["body_to_anchor"] @ relative}
            changed = True
        if not changed:
            break
    return anchors


def build_fanuc_usd_proxy(inventory, *, padding_m=0.0, excluded_self_pairs=None, profile=None):
    report = validate_fanuc_usd_binding(inventory, profile=profile)
    if not report["kinematics_match"]:
        raise ValueError(f"USD/FK binding failed: {report['errors']}")
    if isinstance(padding_m, bool) or not isinstance(padding_m, (int, float)) or not math.isfinite(padding_m) or not 0 <= padding_m <= 0.1:
        raise ValueError("paddingM must be a finite number from 0 to 0.1m")
    units, capsules, ignored = inventory["meters_per_unit"], [], []
    anchors = _attachment_frames(inventory)
    for item in inventory["colliders"]:
        name = item.get("robot_joint")
        if item["role"] != "robot":
            continue
        owner = item.get("rigid_body_path")
        anchor = anchors.get(owner)
        if owner is not None and anchor is None:
            ignored.append({"prim_path": item["prim_path"], "reason": "unmapped_or_unverified_rigid_body_attachment"})
            continue
        if name in inventory["joint_bindings"] and owner != inventory["joint_bindings"][name]["body1"]:
            raise ValueError("Collider owner differs from the validated joint body")
        if item.get("collision_approximation") not in {None, "none", "convexHull"}:
            raise ValueError("Unsupported cooked mesh approximation for proxy generation")
        if owner is None:
            anchor = {"attachment": "world"}
            bounds = item["world_aabb_m_zup"]
            lower, upper = _array(bounds["min"], (3,)), _array(bounds["max"], (3,))
        else:
            bounds = item["body_local_aabb_stage"]
            lower, upper = _array(bounds["min"], (3,)) * units, _array(bounds["max"], (3,)) * units
            if np.any(lower > upper):
                raise ValueError("Invalid body-local collider bounds")
            corners = np.asarray(list(itertools.product(*zip(lower, upper))))
            matrix = anchor["body_to_anchor"]
            transformed = corners @ matrix[:3, :3].T + matrix[:3, 3]
            lower, upper = transformed.min(axis=0), transformed.max(axis=0)
        if np.any(lower > upper):
            raise ValueError("Invalid body-local collider bounds")
        center, half = (lower + upper) / 2, (upper - lower) / 2
        axis = int(np.argmax(half))
        start, end = center.copy(), center.copy()
        start[axis], end[axis] = lower[axis], upper[axis]
        # The full longest-axis span plus the other two half extents encloses
        # every box corner and therefore the box's convex interior.
        radius = float(np.linalg.norm(np.delete(half, axis))) + float(padding_m) + 1e-6
        if (not math.isfinite(radius) or not np.all(np.isfinite([start, end]))
                or radius > 10 or max(np.max(np.abs(start)), np.max(np.abs(end))) > 1e6):
            raise ValueError("Generated capsule exceeds collision engine limits")
        capsule = {"id": item["prim_path"], "attachment": anchor["attachment"],
                         "start_local": start.tolist(), "end_local": end.tolist(), "radius_m": radius,
                         "source_prim_path": item["prim_path"], "enclosure": "transformed_collision_aabb"}
        if anchor["attachment"] == "joint":
            capsule["joint_index"] = anchor["joint_index"]
        capsules.append(capsule)
    if not 6 <= len(capsules) <= 64:
        raise ValueError("Require 6..64 robot capsules")
    exclusions = [] if excluded_self_pairs is None else excluded_self_pairs
    ids = {item["id"] for item in capsules}
    if not isinstance(exclusions, list) or any(not isinstance(pair, list) or len(pair) != 2
                                               or not all(isinstance(value, str) for value in pair)
                                               or pair[0] == pair[1] or not set(pair).issubset(ids) for pair in exclusions):
        raise ValueError("Exclusions must explicitly identify two generated robot capsules")
    obstacles = inventory["static_obstacle_candidates"]
    if len(obstacles) > 256:
        raise ValueError("Static obstacle count exceeds collision engine limit")
    scene = {"schema": "vlagraph.fanuc_collision_scene.v1", "scene_id": inventory["stage_identifier"],
             "scene_version": inventory["inventory_digest"], "robot_model_version": "usd_robot_aabb_capsules_v2",
             **({"robot_profile": profile} if profile is not None else {}),
             "geometry_source": "configured_proxy", "kinematics_source": "serial_arm_profile" if profile is not None else "project_m710_configuration",
             "frame": "right_handed_z_up_m", "inventory_digest": inventory["inventory_digest"],
             "robot_prim_path": inventory["robot_prim_path"], "source_meters_per_unit": units,
             "source_up_axis": inventory["up_axis"],
             "base_position": report["base_position"], "base_orientation_wxyz": report["base_orientation_wxyz"],
             "link_capsules": capsules, "obstacles": [dict(item) for item in obstacles],
             "excluded_self_pairs": [list(pair) for pair in exclusions],
             "coverage": "authored_mapped_robot_collision_bounds", "padding_m": float(padding_m),
             "omitted_robot_colliders": [item["prim_path"] for item in ignored], "physics_synchronized": False,
             "geometry_coverage": {
                 "scope": "active_enabled_collision_prims_in_inventory",
                 "enabled_robot_colliders": sum(item["role"] == "robot" for item in inventory["colliders"]),
                 "included_robot_colliders": len(capsules), "unmapped_robot_colliders": ignored,
                 "all_enabled_robot_colliders_mapped": not ignored,
                 "capsules_by_attachment": {kind: sum(item["attachment"] == kind for item in capsules)
                                            for kind in ("joint", "base", "world")},
                 "static_obstacle_count": len(obstacles),
                 "unmodeled_moving_obstacles": [item["prim_path"] for item in inventory["colliders"] if item["role"] == "moving_obstacle"],
                 "boundables_without_collision_api": inventory["boundables_without_collision_api"],
                 "disabled_collision_prims": inventory["disabled_collision_prims"],
                 "tool_coverage": "unverified", "environment_coverage": "unverified",
                 "full_cell_coverage_verified": False,
             },
             "physical_collision_verified": False, "execution_ready": False,
             "unchecked": ["base_and_tool_coverage", "environment_collision_coverage", "contact_skin_defaults",
                           "initial_motion_before_first_frame", "dynamic_obstacles", "physical_rollout"]}
    if report["external_axis"]:
        scene["external_axis"] = dict(report["external_axis"])
    return {"scene": scene, "binding_report": report}
