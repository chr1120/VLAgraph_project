"""Prepare and apply explicit geometry reviews without granting execution.

USD adjacency suggests inspection; it never proves that a collision can be
excluded. Expected geometry is a declaration to check against the inventory.
"""
from __future__ import annotations

import copy
import itertools

from .fanuc_preflight import action_fingerprint
from .usd_geometry import collision_inventory_digest


SCHEMA = "vlagraph.fanuc_geometry_review.v1"


def _sources(inventory, scene):
    if (not isinstance(inventory, dict) or inventory.get("schema") != "vlagraph.usd_collision_inventory.v1"
            or inventory.get("inventory_digest") != collision_inventory_digest(inventory)):
        raise ValueError("Invalid USD inventory or digest")
    if (not isinstance(scene, dict) or scene.get("schema") != "vlagraph.fanuc_collision_scene.v1"
            or scene.get("inventory_digest") != inventory["inventory_digest"]):
        raise ValueError("Proxy scene must match the source inventory")
    capsules = scene.get("link_capsules")
    if not isinstance(capsules, list) or not 1 <= len(capsules) <= 64:
        raise ValueError("Require 1..64 robot capsules")
    ids = [item["id"] for item in capsules]
    if len(set(ids)) != len(ids) or not all(isinstance(value, str) for value in ids):
        raise ValueError("Capsule IDs must be unique strings")
    sources = {item["prim_path"]: item for item in inventory["colliders"]}
    if any(item.get("source_prim_path") not in sources for item in capsules):
        raise ValueError("Every capsule must identify a source collision prim")
    return capsules, sources


def _pair(value, ids):
    if (not isinstance(value, list) or len(value) != 2 or not all(isinstance(item, str) for item in value)
            or value[0] == value[1] or not set(value).issubset(ids)):
        raise ValueError("Review pair must name two distinct robot capsule IDs")
    return tuple(sorted(value))


def build_geometry_review(inventory, scene, *, collision_report=None, hull_inspection=None, hull_sweep=None):
    capsules, sources = _sources(inventory, scene)
    ids = {item["id"] for item in capsules}
    scene_digest = action_fingerprint(scene)
    observed = {}
    hull_details = {}
    sweep_details = {}
    if collision_report is not None:
        if (not isinstance(collision_report, dict) or collision_report.get("schema") != "vlagraph.fanuc_collision_report.v1"
                or collision_report.get("scene_digest") != scene_digest):
            raise ValueError("Collision report does not match the reviewed scene")
        if not isinstance(collision_report.get("pair_diagnostics"), list):
            raise ValueError("Regenerate the collision report with complete pair diagnostics")
        for detail in collision_report["pair_diagnostics"]:
            if detail.get("kind") == "self":
                pair = _pair(detail.get("pair"), ids)
                if pair in observed:
                    raise ValueError("Duplicate collision pair diagnostics")
                observed[pair] = copy.deepcopy(detail)
    if hull_inspection is not None:
        if (collision_report is None or not isinstance(hull_inspection, dict)
                or hull_inspection.get("schema") != "vlagraph.fanuc_hull_inspection.v1"
                or hull_inspection.get("inventory_digest") != inventory["inventory_digest"]
                or hull_inspection.get("scene_digest") != scene_digest
                or hull_inspection.get("action_digest") != collision_report.get("action_digest")
                or hull_inspection.get("collision_report_digest") != action_fingerprint(collision_report)
                or hull_inspection.get("inspection_digest") != action_fingerprint({key: value for key, value in hull_inspection.items()
                                                                                  if key != "inspection_digest"})):
            raise ValueError("Hull inspection must match the inventory, scene and collision report")
        if not isinstance(hull_inspection.get("pairs"), list):
            raise ValueError("Hull inspection pairs must be a list")
        for detail in hull_inspection["pairs"]:
            pair = _pair(detail.get("pair"), ids)
            if pair in hull_details or detail.get("status") not in {"separated_hulls", "overlap_or_within_tolerance", "indeterminate"}:
                raise ValueError("Invalid or duplicate hull inspection pair")
            if pair not in observed or detail.get("time_sec") != observed[pair].get("min_time_sec"):
                raise ValueError("Hull inspection must use the recorded pair diagnostic pose")
            hull_details[pair] = copy.deepcopy(detail)
    if hull_sweep is not None:
        if (collision_report is None or not isinstance(hull_sweep, dict)
                or hull_sweep.get("schema") != "vlagraph.fanuc_hull_sweep.v1"
                or hull_sweep.get("inventory_digest") != inventory["inventory_digest"]
                or hull_sweep.get("scene_digest") != scene_digest
                or hull_sweep.get("action_digest") != collision_report.get("action_digest")
                or hull_sweep.get("snapshot_ref") != collision_report.get("snapshot_ref")
                or hull_sweep.get("collision_report_digest") != action_fingerprint(collision_report)
                or hull_sweep.get("sweep_digest") != action_fingerprint({key: value for key, value in hull_sweep.items() if key != "sweep_digest"})):
            raise ValueError("Hull sweep must match the inventory, scene and collision report")
        if not isinstance(hull_sweep.get("pairs"), list):
            raise ValueError("Hull sweep pairs must be a list")
        for detail in hull_sweep["pairs"]:
            pair = _pair(detail.get("pair"), ids)
            if pair in sweep_details or pair not in observed or detail.get("status") not in {
                    "separated_over_interval", "margin_violation", "indeterminate"}:
                raise ValueError("Invalid or duplicate hull sweep pair")
            sweep_details[pair] = copy.deepcopy(detail)
    joint_links = {}
    for binding in inventory["joint_bindings"].values():
        if len(binding["body0"]) == 1:
            joint_links[frozenset([binding["body0"][0], binding["body1"]])] = "adjacent_revolute_joint"
    for binding in inventory.get("auxiliary_joint_bindings", []):
        if binding.get("enabled") is True and len(binding["body0"]) == len(binding["body1"]) == 1:
            joint_links[frozenset(binding["body0"] + binding["body1"])] = {
                "PhysicsFixedJoint": "direct_fixed_joint", "PhysicsPrismaticJoint": "adjacent_prismatic_joint"
            }.get(binding["type"], "other_joint_connection")
    excluded = {_pair(pair, ids) for pair in scene.get("excluded_self_pairs", [])}
    decisions = []
    for first, second in itertools.combinations(sorted(capsules, key=lambda item: item["id"]), 2):
        pair = (first["id"], second["id"])
        owners = [sources[item["source_prim_path"]].get("rigid_body_path") for item in (first, second)]
        relation = ("same_rigid_body" if owners[0] is not None and owners[0] == owners[1]
                    else joint_links.get(frozenset(owners), "no_direct_joint_connection"))
        diagnostic = observed.get(pair)
        decisions.append({"pair": list(pair), "body_paths": owners, "usd_relation": relation,
                          "inspection_priority": "inspect" if relation != "no_direct_joint_connection"
                          or (diagnostic and diagnostic.get("margin_violation_samples", 0)) else "normal",
                          "sampled_diagnostic": diagnostic,
                          "hull_inspection": hull_details.get(pair),
                          "hull_sweep": sweep_details.get(pair),
                          "decision": "exclude" if pair in excluded else "keep",
                          "reviewed": False, "reviewed_by": "", "rationale": ""})
    return {"schema": SCHEMA, "inventory_digest": inventory["inventory_digest"], "scene_digest": scene_digest,
            "pair_decisions": decisions,
            "geometry_expectations": {
                "tool": {"declared": False, "required_prim_paths": []},
                "environment": {"declared": False, "required_prim_paths": []},
            },
            "diagnostic_action_digest": collision_report.get("action_digest") if collision_report else None,
            "hull_inspection_digest": hull_inspection.get("inspection_digest") if hull_inspection else None,
            "hull_sweep_digest": hull_sweep.get("sweep_digest") if hull_sweep else None,
            "physical_collision_verified": False, "execution_ready": False}


def _expectations(inventory, scene, raw):
    if not isinstance(raw, dict) or set(raw) != {"tool", "environment"}:
        raise ValueError("geometry_expectations must contain tool and environment")
    mapped = {item["source_prim_path"] for item in scene["link_capsules"]}
    obstacles = {item["id"] for item in scene["obstacles"]}
    results, incomplete = {}, False
    for kind in ("tool", "environment"):
        declaration = raw[kind]
        if not isinstance(declaration, dict) or not isinstance(declaration.get("declared"), bool):
            raise ValueError("Geometry declaration requires an explicit declared boolean")
        paths = declaration.get("required_prim_paths")
        if (not isinstance(paths, list) or len(paths) > 256
                or not all(isinstance(path, str) and path.startswith("/") and path != "/"
                           and not path.endswith("/") and "//" not in path and "." not in path.split("/")
                           and ".." not in path.split("/") for path in paths) or len(set(paths)) != len(paths)):
            raise ValueError("Required prim paths must be unique absolute USD paths (at most 256)")
        if paths and not declaration["declared"]:
            raise ValueError("Nonempty expectations must be explicitly declared")
        entries = []
        for path in paths:
            matches = [item for item in inventory["colliders"]
                       if item["prim_path"] == path or item["prim_path"].startswith(path + "/")]
            relevant = [item for item in matches if (item["role"] == "robot") == (kind == "tool")]
            included = mapped if kind == "tool" else obstacles
            missing = [item["prim_path"] for item in relevant if item["prim_path"] not in included]
            status = "mapped_declared_geometry" if relevant and not missing else "unmodeled_geometry" if relevant else "missing_enabled_collision_geometry"
            incomplete |= status != "mapped_declared_geometry"
            entries.append({"prim_path": path, "status": status, "matching_collision_prims": [item["prim_path"] for item in relevant],
                            "unmodeled_collision_prims": missing})
        results[kind] = {"declared": declaration["declared"], "entries": entries,
                         "status": "undeclared" if not declaration["declared"] else "declared_empty" if not paths
                         else "declared_geometry_mapped" if all(item["status"] == "mapped_declared_geometry" for item in entries)
                         else "declared_geometry_incomplete"}
    return results, incomplete


def apply_geometry_review(inventory, scene, review):
    capsules, _ = _sources(inventory, scene)
    if (not isinstance(review, dict) or review.get("schema") != SCHEMA
            or review.get("inventory_digest") != inventory["inventory_digest"]
            or review.get("scene_digest") != action_fingerprint(scene)):
        raise ValueError("Geometry review does not match the current inventory and scene")
    ids = {item["id"] for item in capsules}
    expected = set(itertools.combinations(sorted(ids), 2))
    decisions = review.get("pair_decisions")
    if not isinstance(decisions, list) or len(decisions) != len(expected):
        raise ValueError("Review must contain every self-collision pair exactly once")
    seen, exclusions, reviewed_count, exclusion_audit = set(), [], 0, []
    for item in decisions:
        if not isinstance(item, dict):
            raise ValueError("Review decisions must be objects")
        pair = _pair(item.get("pair"), ids)
        if pair in seen:
            raise ValueError("Duplicate review pair")
        seen.add(pair)
        if item.get("decision") not in {"keep", "exclude"} or not isinstance(item.get("reviewed"), bool):
            raise ValueError("Review decision must be keep/exclude with an explicit reviewed boolean")
        reviewed_count += item["reviewed"]
        if item["decision"] == "exclude":
            if (item["reviewed"] is not True
                    or not all(isinstance(item.get(key), str) and 0 < len(item[key].strip()) <= 2000
                               for key in ("reviewed_by", "rationale"))):
                raise ValueError("Each exclusion requires reviewed=true, reviewed_by and rationale")
            exclusions.append(list(pair))
            exclusion_audit.append({key: copy.deepcopy(item[key]) for key in ("pair", "reviewed_by", "rationale")})
    expectations, incomplete = _expectations(inventory, scene, review.get("geometry_expectations"))
    result = copy.deepcopy(scene)
    result["excluded_self_pairs"] = sorted(exclusions)
    result["geometry_coverage"]["declared_geometry_expectations"] = expectations
    result["geometry_coverage"]["declared_geometry_incomplete"] = bool(incomplete)
    result["geometry_review"] = {"review_digest": action_fingerprint(review), "source_scene_digest": review["scene_digest"],
                                 "hull_inspection_digest": review.get("hull_inspection_digest"),
                                 "hull_sweep_digest": review.get("hull_sweep_digest"),
                                 "reviewed_pair_count": reviewed_count, "total_pair_count": len(expected),
                                 "excluded_pair_count": len(exclusions), "exclusion_audit": exclusion_audit,
                                 "physical_collision_verified": False, "execution_ready": False}
    return result
