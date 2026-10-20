"""Trace prepared LS motion segments and their blockers without exporting actions."""
from __future__ import annotations

import copy
from collections import Counter
import math
import re

from .fanuc_preflight import action_fingerprint, validate_fanuc_trajectory


def audit_ls_instructions(source):
    """Inventory /MN statements omitted or only partially parsed by the adapter."""
    if not isinstance(source, str):
        raise ValueError("LS source must be text")
    match = re.search(r"/MN\s*(.*?)/POS", source, re.S)
    if match is None:
        return {"schema": "vlagraph.fanuc_ls_instruction_audit.v1", "unmodeled": [],
                "issues": ["mn_section_unavailable"], "source_sha256": action_fingerprint(source)}
    motion = re.compile(r"[JL]\s+P\[\d+(?::[^\]]*)?\]\s+[0-9.]+(?:%|mm/sec)\s+(?:FINE|CNT\d+)", re.I)
    frame = re.compile(r"(?:UFRAME_NUM|UTOOL_NUM)\s*=\s*\d+", re.I)
    unmodeled, recognized = [], Counter()
    statement_pattern = re.compile(r"^\s*(\d+):\s*(.*?)\s*;\s*$")
    issues = ["unparsed_mn_statements"] if any(line.strip() and not statement_pattern.fullmatch(line)
                                              for line in match.group(1).splitlines()) else []
    for statement in re.finditer(r"^\s*(\d+):\s*(.*?)\s*;\s*$", match.group(1), re.M):
        line, body = int(statement.group(1)), statement.group(2).strip()
        if not body or body.startswith("!"):
            continue
        if frame.fullmatch(body):
            recognized["frame_selection"] += 1
            continue
        if motion.fullmatch(body):
            recognized["motion"] += 1
            continue
        if re.match(r"^(?:IF|JMP|LBL|CALL|RUN|SELECT|FOR|ENDFOR|WHILE|ENDWHILE|SKIP|END|ABORT|PAUSE)\b", body, re.I):
            kind = "control_flow"
        elif motion.match(body):
            kind = "motion_modifier"
        elif re.match(r"^[JLC]\s+(?:P|PR)\[", body, re.I):
            kind = "unsupported_motion"
        elif re.match(r"^WAIT\b", body, re.I):
            kind = "wait"
        else:
            kind = "process_or_io"
        unmodeled.append({"source_line": line, "kind": kind, "statement": body})
    return {"schema": "vlagraph.fanuc_ls_instruction_audit.v1", "source_sha256": action_fingerprint(source),
            "recognized_counts": dict(recognized), "unmodeled": unmodeled, "issues": issues,
            "source_program_semantics_verified": False}


def review_fanuc_segments(action, *, config=None, snapshot=None):
    report = validate_fanuc_trajectory(action, config=config, snapshot=snapshot)
    frames = action.get("joint_trajectory", [])
    if not isinstance(frames, list):
        frames = []
    normalized = {item["frame"]: item for item in report["normalized_trajectory"]}
    globals_ = [copy.deepcopy(error) for error in report["errors"] if error.get("frame") is None]
    by_frame = {}
    for error in report["errors"]:
        if error.get("frame") is not None:
            by_frame.setdefault(error["frame"], []).append(error)
    audit = action.get("source_instruction_audit")
    instructions = []
    if audit is not None:
        if not isinstance(audit, dict) or audit.get("schema") != "vlagraph.fanuc_ls_instruction_audit.v1" or not isinstance(audit.get("unmodeled"), list):
            raise ValueError("Invalid LS instruction audit")
        instructions = audit["unmodeled"]
        if audit.get("issues"):
            globals_.append({"code": "source_instruction_audit_incomplete", "issues": audit["issues"]})
        if any(item.get("kind") in {"control_flow", "unsupported_motion"} for item in instructions):
            globals_.append({"code": "source_sequence_unverified"})

    def origin(index):
        if index is None:
            return {"frame": None, "source": "initial_snapshot" if snapshot is not None else "unknown_initial_state"}
        frame = frames[index] if isinstance(frames[index], dict) else {}
        return {"frame": index, "source_line": frame.get("source_line"), "source_position_id": frame.get("source_position_id"),
                "motion_type": frame.get("motion_type"), "termination": frame.get("termination"), "source": frame.get("source", "ls_motion")}

    segments = []
    for index, raw in enumerate(frames):
        left_index = index - 1 if index else None
        start, end = origin(left_index), origin(index)
        issues = copy.deepcopy(globals_)
        for endpoint in ([left_index, index] if index else [index]):
            issues.extend(copy.deepcopy(by_frame.get(endpoint, [])))
        for endpoint in ([left_index, index] if index else [index]):
            if endpoint not in normalized:
                issues.append({"code": "missing_normalized_endpoint", "frame": endpoint})
        start_time = normalized[left_index]["time_sec"] if left_index in normalized else 0 if index == 0 else None
        end_time = normalized[index]["time_sec"] if index in normalized else None
        duration = end_time - start_time if start_time is not None and end_time is not None else None
        if duration is None or not math.isfinite(duration) or duration <= 0:
            issues.append({"code": "invalid_segment_duration"})
        first_line, last_line = start.get("source_line"), end.get("source_line")
        local_instructions = [copy.deepcopy(item) for item in instructions if isinstance(item.get("source_line"), int)
                              and isinstance(last_line, int)
                              and ((first_line if isinstance(first_line, int) else -1) < item["source_line"] <= last_line
                                   or item.get("kind") == "motion_modifier" and item["source_line"] == first_line)]
        if any(item.get("kind") == "motion_modifier" for item in local_instructions):
            issues.append({"code": "source_motion_modifier_unverified"})
        status = ("blocked" if issues else "requires_initial_snapshot" if index == 0 and snapshot is None
                  else "requires_source_instruction_review" if local_instructions else "static_segment_checks_passed")
        segments.append({"segment": index, "start": start, "end": end,
                         "original_start_time_sec": start_time, "original_end_time_sec": end_time, "duration_sec": duration,
                         "status": status, "issues": issues, "unmodeled_instructions": local_instructions,
                         "geometry_diagnostic_candidate": status == "static_segment_checks_passed",
                         "execution_ready": False})
    return {"schema": "vlagraph.fanuc_segment_review.v1", "action_digest": report["action_digest"],
            "snapshot_ref": report["snapshot_ref"], "source_program": action.get("source_program"),
            "program_static_checks_passed": report["static_checks_passed"], "global_issues": globals_,
            "segment_count": len(segments), "status_counts": dict(Counter(item["status"] for item in segments)),
            "error_counts": dict(Counter(item["code"] for item in report["errors"])),
            "segments": segments, "source_instruction_audit": copy.deepcopy(audit),
            "scope": "nominal_dedicated_bridge_motion_segments_only", "time_rebased": False,
            "partial_program_execution_supported": False, "physical_collision_verified": False, "execution_ready": False}
