"""Read-only applicability review of controller TOOLDATA, without applying transforms."""
from __future__ import annotations

import csv
import hashlib
import math
import re

from backend.executors.fanuc_nodes import M710_JOINT_LIMITS, _parse_motion_lines, _parse_positions


def review_calibration_candidate(ls_source: str, candidate_source: str) -> dict:
    tables = {"UF": {}, "UT": {}, "POSE": {}}
    metadata, issues, ignored = {}, [], []
    for line_number, line in enumerate(candidate_source.splitlines(), 1):
        if not line.strip():
            continue
        try:
            row = [value.strip() for value in next(csv.reader([line], strict=True))]
        except csv.Error:
            issues.append({"code": "malformed_csv", "line": line_number})
            continue
        kind = row[0].upper()
        if kind in {"CONTROLLER_VERSION", "RRS_VERSION", "ROBOT_NAME"}:
            if len(row) != 2 or not row[1] or kind in metadata:
                issues.append({"code": "invalid_or_duplicate_metadata", "line": line_number})
            else:
                metadata[kind] = row[1]
            continue
        if kind not in tables:
            ignored.append({"line": line_number, "record": row[0]})
            continue
        try:
            if len(row) not in ({8, 9} if kind != "POSE" else {8}):
                raise ValueError()
            key = row[1] if kind == "POSE" else str(int(row[1]))
            if not key or (kind != "POSE" and int(key) < 0):
                raise ValueError()
            values = [float(value) for value in row[2:8]]
            if not all(math.isfinite(value) for value in values):
                raise ValueError()
        except (ValueError, IndexError):
            issues.append({"code": "invalid_record", "line": line_number, "record": kind})
            continue
        if key in tables[kind]:
            issues.append({"code": "duplicate_record", "line": line_number, "record": kind, "key": key})
        else:
            tables[kind][key] = values

    robot_name = metadata.get("ROBOT_NAME", "")
    model_matches = bool(re.match(r"^(?:FANUC[ _-]+)?M[-_ ]?710IC[/_ -]?20L(?:$|[_ -])", robot_name, re.I))
    if not model_matches:
        issues.append({"code": "robot_model_mismatch" if robot_name else "robot_model_missing"})

    required = {"UF": set(), "UT": set()}
    references = []
    frames = _parse_motion_lines(ls_source, _parse_positions(ls_source))
    for frame in frames:
        position = frame["position"]
        if position.get("kind") != "cartesian":
            continue
        ref = {"line": frame["line"], "position_id": frame["position_id"]}
        for kind in required:
            key = kind.lower()
            value = position.get(key)
            if value is None:
                value = frame.get("active_" + key)
            ref[key] = value
            if value is None:
                issues.append({"code": "unknown_frame_selection", "line": frame["line"], "record": kind})
            else:
                required[kind].add(str(value))
        references.append(ref)
    if not frames:
        issues.append({"code": "no_supported_motion_frames"})
    missing = {kind: sorted(required[kind] - tables[kind].keys(), key=int) for kind in required}
    for kind, keys in missing.items():
        if keys:
            issues.append({"code": "missing_frame_definitions", "record": kind, "keys": keys})
    poses = tables["POSE"]
    seed_name = "start" if "start" in poses else "START" if "START" in poses else next(iter(poses), None)
    seed = poses.get(seed_name)
    if seed is None:
        issues.append({"code": "joint_seed_missing"})
    else:
        for index, value in enumerate(seed):
            lower, upper = M710_JOINT_LIMITS[index]
            if not math.degrees(lower) - 1e-9 <= value <= math.degrees(upper) + 1e-9:
                issues.append({"code": "joint_seed_out_of_limits", "joint": f"J{index+1}", "value_deg": value})
    return {
        "schema": "vlagraph.fanuc_calibration_candidate_review.v1",
        "status": "rejected" if issues else "requires_coordinate_alignment",
        "source_sha256": hashlib.sha256(ls_source.encode("utf-8")).hexdigest(),
        "candidate_sha256": hashlib.sha256(candidate_source.encode("utf-8")).hexdigest(),
        "digest_encoding": "decoded_text_utf8",
        "metadata": metadata, "robot_model_matches": model_matches,
        "available_frames": {kind: sorted(tables[kind], key=int) for kind in required},
        "required_frames": {kind: sorted(required[kind], key=int) for kind in required},
        "missing_frames": missing, "cartesian_references": references,
        "seed": {"name": seed_name, "joints_deg": seed},
        "issues": issues, "uninterpreted_records": ignored,
        "pending_evidence": ["robot_instance_and_program_provenance", "controller_base_to_ik_base_transform",
                             "uf_wpr_convention_and_reference_pose", "flange_to_tcp_convention_and_reference_pose",
                             "external_axis_frame_and_zero", "calibration_revision_and_measured_residuals"],
        "transforms_applied": False, "calibration_verified": False, "execution_ready": False,
    }
