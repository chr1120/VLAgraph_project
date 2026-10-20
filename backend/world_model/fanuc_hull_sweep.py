"""Conservative interval diagnostics for selected authored convex hull pairs.

No report from this module changes execution policy or collision exclusions.
"""
from __future__ import annotations

import math

import numpy as np

from .convex_distance import convex_distance_bounds
from .fanuc_hull_inspection import _inspection_context, _points_in_anchor, _world_points
from .fanuc_preflight import action_fingerprint


def _motion_levers(points, anchor):
    from backend.executors.fanuc_nodes import M710_JOINT_ORIGINS

    if anchor["attachment"] != "joint":
        return []
    joint = anchor["joint_index"]
    radius = float(np.max(np.linalg.norm(points, axis=1)))
    return [sum(float(np.linalg.norm(origin)) for origin in M710_JOINT_ORIGINS[index+1:joint+1]) + radius
            for index in range(joint+1)]


def _motion_bound(anchor, levers, left, right, *, rail_scale=1.0):
    if anchor["attachment"] == "world":
        return 0.0
    rail = abs(right.get("externalAxisJoint", 0) - left.get("externalAxisJoint", 0))
    return rail_scale * rail + sum(lever * abs(right[f"J{index+1}"] - left[f"J{index+1}"])
                      for index, lever in enumerate(levers))


def inspect_review_hull_sweep(stage, inventory, scene, action, collision_report, *, snapshot=None,
                              max_step_sec=0.05, max_samples=10000):
    if (isinstance(max_step_sec, bool) or not isinstance(max_step_sec, (int, float)) or not math.isfinite(max_step_sec)
            or not 0.001 <= max_step_sec <= 2):
        raise ValueError("max_step_sec must be 0.001..2")
    if isinstance(max_samples, bool) or not isinstance(max_samples, int) or not 2 <= max_samples <= 10000:
        raise ValueError("max_samples must be 2..10000")
    trajectory, anchors, sources, capsules, candidates = _inspection_context(
        stage, inventory, scene, action, collision_report, snapshot)
    if len(trajectory) < 2:
        raise ValueError("Hull sweep requires two trajectory poses, including an optional initial snapshot")
    counts = [math.ceil((right["time_sec"] - left["time_sec"]) / max_step_sec)
              for left, right in zip(trajectory, trajectory[1:])]
    sample_count = 1 + sum(counts)
    if sample_count > max_samples or sample_count * len(candidates) > 10000:
        raise ValueError("Hull sweep sample or 10000 pair/sample comparison budget exceeded")
    margin = collision_report.get("margin_m")
    if isinstance(margin, bool) or not isinstance(margin, (int, float)) or not math.isfinite(margin) or not 0 <= margin <= 1:
        raise ValueError("Collision report margin must be finite and 0..1m")
    rail_scale = 1.0
    if scene.get("external_axis") is not None:
        direction = np.asarray(scene["external_axis"]["direction_world"], dtype=float)
        if direction.shape != (3,) or not np.all(np.isfinite(direction)):
            raise ValueError("External axis direction must be a finite three-component vector")
        rail_scale = float(np.linalg.norm(direction))
        if not math.isclose(rail_scale, 1, rel_tol=0, abs_tol=1e-6):
            raise ValueError("External axis direction must have unit length")
    cache, failures = {}, {}
    selected_ids = {identifier for item in candidates for identifier in item["pair"]}
    for identifier in sorted(selected_ids):
        try:
            source_path = capsules[identifier]["source_prim_path"]
            points, anchor, description = _points_in_anchor(stage, inventory, sources[source_path], anchors)
            cache[identifier] = points, anchor, description, _motion_levers(points, anchor)
        except (ValueError, TypeError, KeyError, np.linalg.LinAlgError) as exc:
            failures[identifier] = str(exc)
    states = []
    for item in candidates:
        states.append({"pair": item["pair"], "body_paths": item["body_paths"],
                       "status": "indeterminate", "samples_checked": 0, "margin_violation_samples": 0,
                       "indeterminate_samples": 0, "unresolved_interval_count": 0, "unresolved_intervals": [],
                       "min_sample_distance_lower_bound_m": None, "min_sample_distance_upper_bound_m": None,
                       "min_upper_bound_time_sec": None, "min_interval_distance_lower_bound_m": None,
                       "geometry": [cache[key][2] for key in item["pair"] if key in cache],
                       "unsupported_geometry": {key: failures[key] for key in item["pair"] if key in failures}})
    previous = None
    for segment, (left, right) in enumerate(zip(trajectory, trajectory[1:])):
        for index in range(0 if segment == 0 else 1, counts[segment] + 1):
            alpha = index / counts[segment]
            blend = alpha * alpha * (3 - 2 * alpha)
            positions = {name: value + blend * (right["positions"][name] - value) for name, value in left["positions"].items()}
            at_time = left["time_sec"] + alpha * (right["time_sec"] - left["time_sec"])
            world = {key: _world_points(data[0], data[1], scene, positions) for key, data in cache.items()}
            current = {}
            for pair_index, state in enumerate(states):
                if state["unsupported_geometry"]:
                    continue
                first, second = state["pair"]
                try:
                    bounds = convex_distance_bounds(world[first], world[second], margin_m=margin)
                except (ValueError, TypeError, np.linalg.LinAlgError):
                    state["indeterminate_samples"] += 1
                    continue
                state["samples_checked"] += 1
                epsilon = bounds["tolerance_m"] + bounds["numerical_guard_m"]
                lower, upper = bounds["distance_lower_bound_m"], bounds["distance_upper_bound_m"]
                if state["min_sample_distance_lower_bound_m"] is None or lower < state["min_sample_distance_lower_bound_m"]:
                    state["min_sample_distance_lower_bound_m"] = lower
                if state["min_sample_distance_upper_bound_m"] is None or upper < state["min_sample_distance_upper_bound_m"]:
                    state["min_sample_distance_upper_bound_m"], state["min_upper_bound_time_sec"] = upper, at_time
                if upper <= margin + epsilon:
                    state["margin_violation_samples"] += 1
                elif lower <= margin + epsilon:
                    state["indeterminate_samples"] += 1
                current[pair_index] = (lower, epsilon)
                if previous is not None:
                    # Smoothstep is monotonic inside each original trajectory
                    # segment; endpoint angle deltas bound all intermediate poses.
                    movement = sum(_motion_bound(cache[key][1], cache[key][3], previous[1], positions, rail_scale=rail_scale)
                                   for key in (first, second))
                    prior = previous[2].get(pair_index)
                    interval_lower = max(0.0, max(prior[0], lower) - movement) if prior else 0.0
                    if state["min_interval_distance_lower_bound_m"] is None or interval_lower < state["min_interval_distance_lower_bound_m"]:
                        state["min_interval_distance_lower_bound_m"] = interval_lower
                    if prior is None or interval_lower <= margin + max(prior[1], epsilon):
                        state["unresolved_interval_count"] += 1
                        if len(state["unresolved_intervals"]) < 100:
                            state["unresolved_intervals"].append({"start_time_sec": previous[0], "end_time_sec": at_time,
                                                                  "motion_bound_m": movement,
                                                                  "distance_lower_bound_m": interval_lower})
            previous = at_time, positions, current
    for state in states:
        state["between_samples_certified"] = bool(not state["unsupported_geometry"]
            and state["samples_checked"] == sample_count and not state["margin_violation_samples"]
            and not state["indeterminate_samples"] and not state["unresolved_interval_count"])
        state["status"] = ("margin_violation" if state["margin_violation_samples"] else
                            "separated_over_interval" if state["between_samples_certified"] else "indeterminate")
    status = ("not_applicable" if not states else "margin_violation" if any(state["margin_violation_samples"] for state in states)
              else "separated_over_interval" if all(state["between_samples_certified"] for state in states) else "indeterminate")
    result = {"schema": "vlagraph.fanuc_hull_sweep.v1", "status": status, "pairs": states,
              "inventory_digest": inventory["inventory_digest"], "scene_digest": action_fingerprint(scene),
              "action_digest": action_fingerprint(action), "collision_report_digest": action_fingerprint(collision_report),
              "snapshot_ref": collision_report.get("snapshot_ref"), "initial_segment_checked": snapshot is not None,
              "checked_time_range_sec": [trajectory[0]["time_sec"], trajectory[-1]["time_sec"]],
              "sample_count": sample_count, "max_step_sec": max_step_sec, "margin_m": margin,
              "scope": "selected_nonadjacent_capsule_violation_pairs_only",
              "all_selected_pairs_certified": bool(states) and all(state["between_samples_certified"] for state in states),
              "motion_requested": False, "physical_collision_verified": False, "execution_ready": False,
              "exclusion_policy": "unchanged",
              "unchecked": ["other_collision_pairs", "physx_cooked_collision_shapes", "contact_skin_defaults",
                            "live_physics_state", "tool_and_environment_completeness"]}
    result["sweep_digest"] = action_fingerprint(result)
    return result
