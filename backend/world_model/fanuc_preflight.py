"""Offline checks for the dedicated FANUC bridge's joint interpolation.

This produces a diagnostic preview, never an execution authorization.
"""
from __future__ import annotations

import copy
import json
import hashlib
import math
from typing import Any


def action_fingerprint(action):
    return hashlib.sha256(json.dumps(action, sort_keys=True, allow_nan=False, separators=(",", ":")).encode()).hexdigest()


def snapshot_reference(snapshot):
    from .fanuc import FanucPhysicsSnapshot
    observed = FanucPhysicsSnapshot.model_validate(snapshot)
    return {"episode_id": observed.episode_id, "state_version": observed.state_version,
            "sim_time": observed.sim_time, "physics_step": observed.physics_step, "prim_path": observed.prim_path,
            "snapshot_digest": action_fingerprint(observed.model_dump(mode="json", by_alias=True))}


def validate_fanuc_trajectory(action: dict[str, Any], *, config=None, snapshot=None):
    from .robot_profiles import validate_profile, profile_arrays
    from .fanuc import FanucPhysicsSnapshot

    config = config or {}
    errors, warnings, normalized, segments = [], [], [], []
    required = [f"J{i}" for i in range(1, 7)]
    profile = validate_profile(action.get("robot_profile") if isinstance(action, dict) else None)
    profile_limits = profile_arrays(profile)[2]
    limits = {name: list(profile_limits[i]) for i, name in enumerate(required)}
    limits["externalAxisJoint"] = [-10.0, 10.0]
    unchecked = ["environment_collision", "self_collision", "singularity", "physical_rollout", "live_state_freshness"]

    def error(code, frame=None, **detail):
        errors.append({"code": code, "frame": frame, **detail})

    def number(value):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError("Expected a finite number")
        return float(value)

    def canonical(name):
        return "externalAxisJoint" if name in ("E1", "externalAxisJoint") else name

    def metric_map(raw, kind):
        if not isinstance(raw, dict):
            raise ValueError(f"{kind} must be a joint-name map")
        result = {}
        for name, value in raw.items():
            if name not in limits:
                raise ValueError(f"Unknown axis in {kind}: {name}")
            parsed = number(value)
            if parsed <= 0:
                raise ValueError(f"{kind} must be positive")
            result[name] = parsed
        return result

    velocities = metric_map(config.get("maxVelocity", {}), "maxVelocity")
    accelerations = metric_map(config.get("maxAcceleration", {}), "maxAcceleration")
    for key, values in (("maxVelocity", velocities), ("maxAcceleration", accelerations)):
        missing = sorted(set(limits) - set(values))
        if missing:
            unchecked.append({"check": key, "unconfigured_axes": missing})
    if not isinstance(action, dict):
        raise ValueError("FANUC action must be an object")
    if (action.get("schema") not in {"vlagraph.fanuc_m710ic20l.action.v1", "vlagraph.fanuc_serial_arm.action.v1"}
            or action.get("command") != "apply_fanuc_trajectory"
            or action.get("schema") == "vlagraph.fanuc_serial_arm.action.v1" and not action.get("robot_profile")):
        error("unsupported_action_contract")
    try:
        json.dumps(action, allow_nan=False)
    except (ValueError, TypeError):
        error("non_json_or_non_finite_action")
    for key in ("ik_failures", "skipped_cartesian_frames", "skipped_unresolved_frames"):
        if action.get(key, 0) != 0:
            error("incomplete_source_program", field=key, value=str(action[key]))
    for key in ("skipped_cartesian_details", "skipped_unresolved_details"):
        if action.get(key):
            error("incomplete_source_program", field=key, count=len(action[key]))
    if action.get("frame_time_sanitization", {}).get("adjusted_frames", 0):
        error("source_times_were_repaired")
    frames = action.get("joint_trajectory")
    if not isinstance(frames, list) or not frames:
        error("empty_trajectory")
        frames = []
    if len(frames) > 10000:
        raise ValueError("Preflight supports at most 10000 frames")
    previous, previous_time = None, 0.0
    if snapshot is not None:
        observed = FanucPhysicsSnapshot.model_validate(snapshot)
        previous = {axis.name: axis.position for axis in observed.axes}
        snapshot_ref = snapshot_reference(snapshot)
        if observed.raw_state.get("motion_active") is not False:
            error("initial_motion_not_idle")
        for axis in observed.axes:
            if axis.virtual:
                error("virtual_initial_axis", axis=axis.name)
            if not limits[axis.name][0] <= axis.position <= limits[axis.name][1]:
                error("initial_joint_position_limit", axis=axis.name, value=axis.position)
            threshold = 0.001 if axis.name == "externalAxisJoint" else 0.01
            if axis.velocity is None or abs(axis.velocity) > threshold:
                error("initial_axis_not_settled", axis=axis.name, velocity=axis.velocity, threshold=threshold)
    else:
        snapshot_ref = None
        unchecked.append("initial_motion_without_snapshot")

    for index, frame in enumerate(frames):
        try:
            if not isinstance(frame, dict):
                raise ValueError("Frame must be an object")
            time_sec = number(frame.get("time_sec"))
            if time_sec <= previous_time:
                error("non_increasing_time", index, time_sec=time_sec, previous_time=previous_time)
            elif time_sec - previous_time < 0.001:
                error("segment_shorter_than_bridge_minimum", index)
            unit = frame.get("unit", action.get("unit"))
            if unit not in ("degree", "deg", "degrees", "rad", "radian", "radians"):
                raise ValueError("Unknown joint unit")
            degree = unit in ("degree", "deg", "degrees")
            mapping = frame.get("joint_target_map")
            if mapping is not None:
                if not isinstance(mapping, dict) or not mapping:
                    raise ValueError("joint_target_map must be a nonempty object")
                parsed = {}
                for key, value in mapping.items():
                    name = canonical(key)
                    if name not in limits:
                        raise ValueError(f"Unknown target axis: {key}")
                    value = number(value)
                    # Dedicated bridge always uses mm for external command values.
                    value = value * 0.001 if name == "externalAxisJoint" else math.radians(value) if degree else value
                    if name in parsed and not math.isclose(parsed[name], value, abs_tol=1e-9, rel_tol=1e-9):
                        raise ValueError("E1 and externalAxisJoint alias targets conflict")
                    parsed[name] = value
            else:
                values = frame.get("joint_positions")
                names = frame.get("joint_names", action.get("joint_names"))
                if (not isinstance(names, list) or not all(isinstance(name, str) for name in names)
                        or len(set(names)) != len(names) or not isinstance(values, list) or len(values) != len(names)):
                    raise ValueError("Joint array requires matching unique names")
                parsed = {}
                for name, value in zip(names, values):
                    name = canonical(name)
                    if name not in limits or name in parsed:
                        raise ValueError("Unknown or duplicate target axis")
                    value = number(value)
                    parsed[name] = value * 0.001 if name == "externalAxisJoint" else math.radians(value) if degree else value
            if not set(required).issubset(parsed):
                raise ValueError("Each frame must contain J1..J6")
            if normalized and set(parsed) != set(normalized[-1]["positions"]):
                error("axis_set_changed", index)
            if previous is not None and set(previous) != set(parsed):
                error("snapshot_or_segment_axis_mismatch", index)
            for name, value in parsed.items():
                lower, upper = limits[name]
                if not lower <= value <= upper:
                    error("joint_position_limit", index, axis=name, value=value, bounds=[float(lower), float(upper)])
            if mapping is not None and isinstance(frame.get("joint_positions"), list):
                # Prepare's joint_positions array is J1..J6 only even when its
                # joint_names contains E1 aliases. Named targets govern execution.
                raw = frame["joint_positions"]
                if len(raw) != 6:
                    raise ValueError("Prepared joint_positions must contain six arm values")
                for name, value in zip(required, raw):
                    value = math.radians(number(value)) if degree else number(value)
                    if not math.isclose(value, parsed[name], abs_tol=1e-9, rel_tol=1e-9):
                        error("named_and_array_targets_conflict", index, axis=name)
            external = frame.get("external_axis_targets_m", {})
            if isinstance(external, dict) and "E1" in external:
                if "externalAxisJoint" not in parsed or not math.isclose(number(external["E1"]), parsed["externalAxisJoint"], abs_tol=1e-9):
                    error("external_axis_unit_conflict", index)
            mode = frame.get("cartesian_mode")
            if frame.get("ik_solved") or mode:
                uf = frame.get("position_uf") if frame.get("position_uf") is not None else frame.get("active_uf")
                ut = frame.get("position_ut") if frame.get("position_ut") is not None else frame.get("active_ut")
                if (mode != "calibrated_uf_ik" or uf not in action.get("calibrated_uframes", [])
                        or ut not in action.get("calibrated_utools", [])):
                    error("uf_ut_calibration_missing", index, uf=uf, ut=ut)
            motion_type = frame.get("motion_type")
            if motion_type in ("L", "C"):
                error("cartesian_path_fidelity_unverified", index, motion_type=motion_type)
            termination = str(frame.get("termination") or "").upper()
            if termination.startswith("CNT") or number(frame.get("cnt_percent", 0)) > 0:
                error("cnt_blending_unverified", index)
            for key in ("root_translation", "rootTranslation", "external_axis_root_translation"):
                root = frame.get(key)
                if root is not None:
                    if not isinstance(root, (list, tuple)) or len(root) != 3:
                        raise ValueError("Root translation requires three finite values")
                    if any(abs(number(v)) > 1e-9 for v in root):
                        error("external_axis_root_transform_unverified", index, field=key)
            duration = time_sec - previous_time
            if previous is not None and duration >= 0.001:
                peaks = {}
                for name in set(previous) & set(parsed):
                    delta = abs(parsed[name] - previous[name])
                    velocity, acceleration = 1.5 * delta / duration, 6 * delta / duration ** 2
                    if not math.isfinite(velocity) or not math.isfinite(acceleration):
                        error("non_finite_segment_metrics", index, axis=name)
                        continue
                    peaks[name] = {"velocity": velocity, "acceleration": acceleration}
                    if name in velocities and velocity > velocities[name]:
                        error("joint_velocity_limit", index, axis=name, peak=velocity, limit=velocities[name])
                    if name in accelerations and acceleration > accelerations[name]:
                        error("joint_acceleration_limit", index, axis=name, peak=acceleration, limit=accelerations[name])
                segments.append({"end_frame": index, "duration_sec": duration, "smoothstep_peaks": peaks})
            normalized.append({"frame": index, "time_sec": time_sec, "positions": parsed})
            previous, previous_time = parsed, time_sec
        except (ValueError, TypeError, KeyError, OverflowError) as exc:
            error("invalid_frame", index, message=str(exc))
            previous = None
    if normalized and normalized[-1]["time_sec"] < 2:
        error("bridge_minimum_duration", duration_sec=normalized[-1]["time_sec"])
    if not velocities or not accelerations:
        warnings.append("Velocity/acceleration limits must be configured before execution assessment")
    preview = None
    if not errors:
        preview = copy.deepcopy(action)
        preview.update(preview_only=True, execution_enabled=False, execution_ready=False)
    return {
        "schema": "vlagraph.fanuc_preflight.v1", "static_checks_passed": not errors,
        "action_digest": action_fingerprint(action) if not any(item["code"] == "non_json_or_non_finite_action" for item in errors) else None,
        "validation_config": {"maxVelocity": velocities, "maxAcceleration": accelerations},
        "execution_ready": False, "collision_checked": False,
        "limit_source": profile["source"], "robot_profile_digest": action_fingerprint(profile),
        "interpolation": "dedicated_bridge_joint_smoothstep",
        "errors": errors, "warnings": warnings, "unchecked": unchecked,
        "snapshot_ref": snapshot_ref, "normalized_trajectory": normalized, "segments": segments,
        "preview_action": preview,
    }
