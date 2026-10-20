"""Reconcile persisted scene observations with fresh geometry; never invent colliders."""
from __future__ import annotations

import math

from .fanuc import FanucPhysicsSnapshot
from .fanuc_preflight import action_fingerprint, snapshot_reference
from .state_builder import build_world_state
from .usd_geometry import collision_inventory_digest


def reconcile_memory(record, scan, snapshot, inventory, *, now, max_memory_age_sec=86400,
                     max_live_age_sec=5, position_tolerance_m=.02):
    for value, lower, upper in ((now, 0, 1e12), (max_memory_age_sec, 1, 604800),
                                (max_live_age_sec, .1, 60), (position_tolerance_m, 0, .1)):
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not lower <= value <= upper:
            raise ValueError("Invalid memory reconciliation time or tolerance")
    observed = FanucPhysicsSnapshot.model_validate(snapshot)
    if not isinstance(record, dict) or not isinstance(record.get("scan"), dict):
        raise ValueError("A persisted Collection record with its original scan is required")
    if not isinstance(scan, dict) or scan.get("ok") is False:
        raise ValueError("A fresh scene scan is required")
    if (not isinstance(inventory, dict) or inventory.get("schema") != "vlagraph.usd_collision_inventory.v1"
            or inventory.get("inventory_digest") != collision_inventory_digest(inventory)):
        raise ValueError("Invalid collision inventory digest")
    reasons = []
    saved = record["scan"]
    identity = observed.geometry.get("stage_identifier")
    for source in (saved, scan):
        if (not identity or source.get("stage_identifier") != identity
                or source.get("meters_per_unit") != observed.geometry["meters_per_unit"]
                or source.get("up_axis") != observed.geometry["up_axis"]
                or (source.get("robot") or {}).get("prim_path") != observed.prim_path):
            reasons.append({"code": "scene_identity_or_frame_mismatch"})
        if source.get("truncated") is not False:
            reasons.append({"code": "scene_scan_coverage_unverified"})
    if (inventory.get("stage_identifier") != identity or inventory.get("robot_prim_path") != observed.prim_path
            or inventory.get("meters_per_unit") != observed.geometry["meters_per_unit"]
            or inventory.get("up_axis") != observed.geometry["up_axis"]):
        reasons.append({"code": "inventory_snapshot_identity_mismatch"})
    for label, stamp, maximum in (("memory", saved.get("timestamp"), max_memory_age_sec),
                                   ("scan", scan.get("timestamp"), max_live_age_sec),
                                   ("snapshot", observed.observed_at, max_live_age_sec)):
        if (isinstance(stamp, bool) or not isinstance(stamp, (float, int)) or not math.isfinite(stamp)
                or not -.5 <= now - stamp <= maximum):
            reasons.append({"code": f"{label}_stale_or_time_unknown"})
    if scan.get("scene_id") != saved.get("scene_id") or not scan.get("scene_id"):
        reasons.append({"code": "memory_scene_id_mismatch"})
    sim_time = scan.get("sim_time")
    if (isinstance(sim_time, bool) or not isinstance(sim_time, (float, int)) or not math.isfinite(sim_time)
            or abs(sim_time - observed.sim_time) > .1):
        reasons.append({"code": "scan_snapshot_time_mismatch"})
    if observed.raw_state.get("motion_active") is not False:
        reasons.append({"code": "robot_not_idle"})

    def objects(source):
        world = build_world_state(source, scene_version="1", episode_id=observed.episode_id, state_version=1)
        return {obj.prim_path: obj for obj in world.objects
                if obj.prim_path != observed.prim_path and not obj.prim_path.startswith(observed.prim_path + "/")}

    previous, current = objects(saved), objects(scan)
    changed = []
    if previous.keys() != current.keys():
        reasons.append({"code": "scene_object_set_changed", "added": sorted(current.keys() - previous.keys()),
                        "removed": sorted(previous.keys() - current.keys())})
    for path in previous.keys() & current.keys():
        a, b = previous[path], current[path]
        if (a.size is None or b.size is None or a.bbox_center is None or b.bbox_center is None
                or any(abs(x-y) > position_tolerance_m for x, y in zip(a.position, b.position))
                or any(abs(x-y) > position_tolerance_m for x, y in zip(a.bbox_center, b.bbox_center))
                or any(abs(x-y) > position_tolerance_m for x, y in zip(a.size, b.size))):
            changed.append(path)
    if changed:
        reasons.append({"code": "memory_geometry_changed_or_incomplete", "paths": sorted(changed)})
    colliders = inventory.get("colliders", [])
    unmapped = [path for path in current if not any(
        item.get("role") == "static_obstacle" and
        (item.get("prim_path") == path or str(item.get("prim_path", "")).startswith(path + "/")) for item in colliders)]
    if unmapped:
        reasons.append({"code": "observed_objects_without_static_collision_geometry", "paths": sorted(unmapped)})
    for path, obj in current.items():
        if obj.bbox_center is None or obj.size is None:
            continue
        for collider in colliders:
            if collider.get("prim_path") != path and not str(collider.get("prim_path", "")).startswith(path + "/"):
                continue
            bounds = collider.get("world_aabb_m_zup") or {}
            lower, upper = bounds.get("min"), bounds.get("max")
            if (not isinstance(lower, list) or not isinstance(upper, list) or len(lower) != 3 or len(upper) != 3
                    or any(isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) for v in lower + upper)
                    or any(lo > hi or lo < center-size/2-position_tolerance_m or hi > center+size/2+position_tolerance_m
                           for lo, hi, center, size in zip(lower, upper, obj.bbox_center, obj.size))):
                reasons.append({"code": "collision_bounds_disagree_with_observation", "path": path})
    return {"schema": "vlagraph.fanuc_world_context.v1", "status": "blocked" if reasons else "geometry_context_consistent",
            "reasons": reasons, "record_id": record.get("id"), "memory_digest": action_fingerprint(record),
            "scan_digest": action_fingerprint(scan), "inventory_digest": inventory["inventory_digest"],
            "snapshot_ref": snapshot_reference(snapshot), "evaluated_at": now,
            "config": {"max_memory_age_sec": max_memory_age_sec, "max_live_age_sec": max_live_age_sec,
                       "position_tolerance_m": position_tolerance_m},
            "object_count": len(current), "physical_context_verified": False,
            "unchecked": ["default_time_geometry_physics_alignment", "unobserved_environment", "physical_rollout"],
            "execution_ready": False}


def prediction_handoff(action, context_report, collision, snapshot, scene):
    if not isinstance(context_report, dict) or context_report.get("schema") != "vlagraph.fanuc_world_context.v1":
        raise ValueError("World context report is required")
    reference = snapshot_reference(snapshot)
    if context_report.get("snapshot_ref") != reference:
        raise ValueError("Prediction context uses a different snapshot")
    if (not isinstance(collision, dict) or collision.get("schema") != "vlagraph.fanuc_collision_report.v1"
            or collision.get("action_digest") != action_fingerprint(action)
            or collision.get("snapshot_ref") != reference or not isinstance(scene, dict)
            or collision.get("scene_digest") != action_fingerprint(scene)
            or scene.get("inventory_digest") != context_report.get("inventory_digest")):
        raise ValueError("Collision sweep must match the action, context inventory, scene and initial snapshot")
    return {"schema": "vlagraph.fanuc_world_prediction.v1", "status": "physical_prediction_unavailable",
            "action_digest": action_fingerprint(action), "snapshot_ref": reference,
            "context_digest": action_fingerprint(context_report), "scene_digest": action_fingerprint(scene),
            "collision_digest": action_fingerprint(collision), "nominal_sweep_status": collision.get("status"),
            "physical_rollout_verified": False, "execution_ready": False,
            "message": "Geometry sweep is diagnostic; an isolated physical prediction worker is not implemented."}
