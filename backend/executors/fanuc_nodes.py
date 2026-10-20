from __future__ import annotations

import json
import math
import re
import csv
from pathlib import Path
from typing import Any, Dict, List
import numpy as np

from backend.executors.omy3m_nodes import _dataset_root, _host_path, _json_safe, _safe_name


# Official ROBOTIS OMY-3M URDF kinematic chain (link0 -> link6).
# Source: ROBOTIS-GIT/open_manipulator, Apache-2.0.
OMY3M_JOINT_ORIGINS = np.asarray(
    [
        [0.0, 0.0, 0.1715],
        [0.0, -0.1215, 0.0],
        [0.0, 0.0, 0.2470],
        [0.0, 0.1215, 0.2195],
        [0.0, -0.1130, 0.0],
        [0.0, 0.0, 0.1155],
    ],
    dtype=float,
)
OMY3M_JOINT_AXES = np.asarray(
    [[0, 0, 1], [0, 1, 0], [0, 1, 0], [0, 1, 0], [0, 0, 1], [0, 1, 0]],
    dtype=float,
)

# Kinematic chain extracted from:
# C:\Users\0\Desktop\fanuc_robot\fanuc_description\m710ic_20l_4\payloads\Physics\physics.usda
# and cross-checked with m710ic_20l.urdf. Origins are physics:localPos0.
# Axes are the effective parent-frame axes after applying physics:localRot0
# to each USD physics:axis token. The J6 flange/tool0 offset comes from
# payloads/base.usda and the URDF fixed joint J6-flange.
# Cartesian LS records are retargeted relative to the first pose in each
# UFRAME/UTOOL group because the uploaded controller frames are not Stage
# transforms. This preserves programmed deltas without pretending that the
# controller's calibrated world frame is known by Isaac Sim.
M710_KINEMATICS_SOURCE = (
    "fanuc_robot/fanuc_description/m710ic_20l_4/payloads/Physics/physics.usda"
)
M710_JOINT_ORIGINS = np.asarray(
    [[0, 0, 0.565], [0.15, 0, 0], [0, 0, 1.15], [0, 0, 0.19], [1.8, 0, 0], [0, 0, 0]],
    dtype=float,
)
M710_JOINT_AXES = np.asarray(
    [[0, 0, 1], [0, 1, 0], [0, -1, 0], [-1, 0, 0], [0, -1, 0], [-1, 0, 0]],
    dtype=float,
)
M710_TOOL_OFFSET = np.asarray([0.1, 0, 0], dtype=float)
M710_ZERO_TCP_POSITION = np.asarray([2.05, 0.0, 1.905], dtype=float)
M710_JOINT_LIMITS = np.radians(
    np.asarray([[-180, 180], [-90, 135], [-80.5, 205], [-200, 200], [-140, 140], [-450, 450]])
)


def _incoming_values(node_id: str, edges: List[Dict[str, Any]], context: Dict[str, Any]) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for edge in edges:
        if edge.get("target") != node_id:
            continue
        payload = context.get(edge.get("source"))
        if payload is None:
            continue
        source_handle = edge.get("sourceHandle")
        value = payload
        if source_handle:
            if not isinstance(payload, dict) or source_handle not in payload:
                continue
            value = payload[source_handle]
        result[str(edge.get("targetHandle") or source_handle or edge.get("source"))] = value
    return result


def _props(node: Dict[str, Any]) -> Dict[str, Any]:
    value = node.get("data")
    return value if isinstance(value, dict) else {}


def _path_from_dataset(value: Any) -> Path:
    if isinstance(value, dict):
        value = value.get("path") or value.get("dataset_path") or value.get("file_path")
    path = Path(str(value or "").strip())
    if not str(path):
        raise ValueError("FANUC LS dataset path is empty")
    if not path.exists():
        raise FileNotFoundError(f"FANUC LS file not found: {path}")
    return path


def _read_ls(path: Path) -> str:
    raw = path.read_bytes()
    for encoding in ("utf-8-sig", "cp949", "latin-1"):
        try:
            return raw.decode(encoding)
        except UnicodeDecodeError:
            continue
    return raw.decode("latin-1", errors="replace")


def _number(text: str, key: str) -> float | None:
    match = re.search(rf"\b{re.escape(key)}\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))", text, re.I)
    return float(match.group(1)) if match else None

def _external_axes_mm(text: str) -> Dict[str, float]:
    axes: Dict[str, float] = {}
    for match in re.finditer(r"\bE(\d+)\s*=\s*([-+]?(?:\d+(?:\.\d*)?|\.\d+))\s*(?:mm|deg)?", text, re.I):
        axes[f"E{int(match.group(1))}"] = float(match.group(2))
    return dict(sorted(axes.items(), key=lambda item: int(item[0][1:])))

def _parse_config(body: str) -> Dict[str, Any] | None:
    match = re.search(r"\bCONFIG\s*:\s*'([^']+)'", body, re.I)
    if not match:
        return None
    raw = match.group(1).strip()
    parts = [part.strip() for part in raw.split(",")]
    flags = [part for part in re.split(r"\s+", parts[0]) if part] if parts else []
    turns: List[int] = []
    for part in parts[1:]:
        try:
            turns.append(int(part))
        except ValueError:
            turns.append(0)
    return {
        "raw": raw,
        "shoulder": flags[0] if len(flags) > 0 else "",
        "elbow": flags[1] if len(flags) > 1 else "",
        "wrist": flags[2] if len(flags) > 2 else "",
        "turns": turns,
    }


def _parse_positions(source: str) -> Dict[int, Dict[str, Any]]:
    positions: Dict[int, Dict[str, Any]] = {}
    pattern = re.compile(
        r'P\[(\d+)(?::"([^"]*)")?\]\s*\{(.*?)\r?\n\};',
        re.S,
    )
    for match in pattern.finditer(source):
        position_id = int(match.group(1))
        label = str(match.group(2) or "")
        body = match.group(3)
        uf_match = re.search(r"\bUF\s*:\s*(\d+)", body)
        ut_match = re.search(r"\bUT\s*:\s*(\d+)", body)
        config = _parse_config(body)
        joints = [_number(body, f"J{index}") for index in range(1, 7)]
        xyz = [_number(body, key) for key in ("X", "Y", "Z")]
        wpr = [_number(body, key) for key in ("W", "P", "R")]
        external_axes = _external_axes_mm(body)
        if all(value is not None for value in joints):
            kind = "joint"
        elif all(value is not None for value in xyz + wpr):
            kind = "cartesian"
        else:
            kind = "unknown"
        positions[position_id] = {
            "position_id": position_id,
            "label": label,
            "kind": kind,
            "uf": int(uf_match.group(1)) if uf_match else None,
            "ut": int(ut_match.group(1)) if ut_match else None,
            "config": config,
            "joints_deg": joints if kind == "joint" else [],
            "xyz_mm": xyz if kind == "cartesian" else [],
            "wpr_deg": wpr if kind == "cartesian" else [],
            "external_axes_mm": external_axes,
            "external_axis_names": list(external_axes.keys()),
        }
    return positions


def _parse_motion_lines(source: str, positions: Dict[int, Dict[str, Any]]) -> List[Dict[str, Any]]:
    mn_match = re.search(r"/MN\s*(.*?)/POS", source, re.S)
    mn = mn_match.group(1) if mn_match else ""
    line_pattern = re.compile(r"^\s*(\d+):\s*(.*?)\s*;\s*$", re.M)
    motion_pattern = re.compile(
        r"^([JL])\s+P\[(\d+)(?::[^\]]*)?\]\s+([0-9.]+)(%|mm/sec)\s+(FINE|CNT\d+)",
        re.I,
    )
    frame_pattern = re.compile(r"^(UFRAME_NUM|UTOOL_NUM)\s*=\s*(\d+)$", re.I)
    frames: List[Dict[str, Any]] = []
    active_uf: int | None = None
    active_ut: int | None = None
    for line_match in line_pattern.finditer(mn):
        line_number = int(line_match.group(1))
        body = line_match.group(2).strip()
        frame_match = frame_pattern.match(body)
        if frame_match:
            value = int(frame_match.group(2))
            if frame_match.group(1).upper() == "UFRAME_NUM":
                active_uf = value
            else:
                active_ut = value
            continue
        match = motion_pattern.match(body)
        if not match:
            continue
        position_id = int(match.group(2))
        position = dict(positions.get(position_id) or {"position_id": position_id, "kind": "missing"})
        frames.append(
            {
                "sequence": len(frames),
                "line": line_number,
                "raw": body,
                "active_uf": active_uf,
                "active_ut": active_ut,
                "motion_type": match.group(1).upper(),
                "position_id": position_id,
                "speed": float(match.group(3)),
                "speed_unit": match.group(4),
                "termination": match.group(5).upper(),
                "position": position,
            }
        )
    return frames


def _tooldata_transform(values: List[str]) -> Dict[str, Any] | None:
    if len(values) < 6:
        return None
    try:
        numeric = [float(value) for value in values[:6]]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in numeric):
        return None
    return {"xyzMm": numeric[:3], "wprDeg": numeric[3:6]}


def _parse_tooldata_initial_settings(source: str, path: Path) -> Dict[str, Any] | None:
    poses: Dict[str, List[float]] = {}
    uframe_transforms: Dict[str, Dict[str, Any]] = {}
    utool_transforms: Dict[str, Dict[str, Any]] = {}
    metadata: Dict[str, Any] = {}
    for raw_line in source.splitlines():
        line = raw_line.strip()
        if not line:
            continue
        try:
            row = next(csv.reader([line]))
        except csv.Error:
            continue
        row = [item.strip() for item in row]
        if not row:
            continue
        record_type = row[0].upper()
        if record_type in {"CONTROLLER_VERSION", "RRS_VERSION", "ROBOT_NAME"} and len(row) > 1:
            metadata[record_type.lower()] = row[1]
            continue
        if record_type in {"UF", "UT"} and len(row) >= 8:
            transform = _tooldata_transform(row[2:8])
            if transform is None:
                continue
            transform["name"] = row[8] if len(row) > 8 else ""
            if record_type == "UF":
                uframe_transforms[row[1]] = transform
            else:
                utool_transforms[row[1]] = transform
            continue
        if record_type == "POSE" and len(row) >= 8:
            try:
                joints = [float(value) for value in row[2:8]]
            except ValueError:
                continue
            if all(math.isfinite(value) for value in joints):
                poses[row[1]] = joints

    if not poses and not uframe_transforms and not utool_transforms:
        return None

    pose_name = "start" if "start" in poses else "START" if "START" in poses else next(iter(poses), "")
    joints_deg = poses.get(pose_name, [])
    return {
        "schema": "vlagraph.fanuc_tooldata.initial_settings.v1",
        "program_name": path.stem,
        "source_path": str(path),
        "format": "tooldata_csv",
        "metadata": metadata,
        "pose_name": pose_name,
        "poses": poses,
        "joints_deg": joints_deg,
        "external_axes_mm": {},
        "uframe_transforms": uframe_transforms,
        "utool_transforms": utool_transforms,
        "summary": {
            "position_count": len(poses),
            "motion_frame_count": 0,
            "joint_position_count": len(poses),
            "has_joint_seed": len(joints_deg) >= 6,
            "external_axis_names": [],
            "uframe_count": len(uframe_transforms),
            "utool_count": len(utool_transforms),
        },
    }



def _parse_initial_settings(path: Path) -> Dict[str, Any]:
    source = _read_ls(path)
    tooldata_settings = _parse_tooldata_initial_settings(source, path)
    if tooldata_settings is not None:
        return tooldata_settings

    program_match = re.search(r"^/PROG\s+(\S+)", source, re.M)
    positions = _parse_positions(source)
    frames = _parse_motion_lines(source, positions)
    ordered_positions = [positions[key] for key in sorted(positions)]
    joint_position = next(
        (position for position in ordered_positions if position.get("kind") == "joint"),
        None,
    )
    reference_position = joint_position or (ordered_positions[0] if ordered_positions else None)
    external_axes_mm = (
        reference_position.get("external_axes_mm")
        if isinstance(reference_position, dict) and isinstance(reference_position.get("external_axes_mm"), dict)
        else {}
    )
    joints_deg = joint_position.get("joints_deg") if isinstance(joint_position, dict) else []
    return {
        "schema": "vlagraph.fanuc_ls.initial_settings.v1",
        "program_name": program_match.group(1) if program_match else path.stem,
        "source_path": str(path),
        "position": reference_position,
        "joint_position": joint_position,
        "joints_deg": joints_deg if isinstance(joints_deg, list) else [],
        "external_axes_mm": external_axes_mm,
        "summary": {
            "position_count": len(positions),
            "motion_frame_count": len(frames),
            "joint_position_count": sum(1 for value in positions.values() if value.get("kind") == "joint"),
            "has_joint_seed": isinstance(joint_position, dict),
            "external_axis_names": sorted(external_axes_mm.keys()),
        },
    }


def exec_fanuc_ls_parse(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    from backend.world_model.fanuc_segment_review import audit_ls_instructions
    inputs = _incoming_values(node["id"], edges, context)
    data = _props(node)
    path = _path_from_dataset(inputs.get("dataset") or data.get("filePath"))
    source = _read_ls(path)
    program_match = re.search(r"^/PROG\s+(\S+)", source, re.M)
    positions = _parse_positions(source)
    frames = _parse_motion_lines(source, positions)
    joint_count = sum(1 for value in positions.values() if value["kind"] == "joint")
    cartesian_count = sum(1 for value in positions.values() if value["kind"] == "cartesian")
    initial_settings = None
    initial_settings_input = inputs.get("initialSettings") or inputs.get("initial_settings") or data.get("initialSettings")
    if initial_settings_input:
        initial_settings_path = _path_from_dataset(initial_settings_input)
        initial_settings = _parse_initial_settings(initial_settings_path)
    program = {
        "schema": "vlagraph.fanuc_ls.v1",
        "instruction_audit": audit_ls_instructions(source),
        "program_name": program_match.group(1) if program_match else path.stem,
        "source_path": str(path),
        "positions": list(positions.values()),
        "frames": frames,
        "summary": {
            "position_count": len(positions),
            "motion_frame_count": len(frames),
            "joint_position_count": joint_count,
            "cartesian_position_count": cartesian_count,
            "requires_uf_ut_calibration": cartesian_count > 0,
        },
    }
    if initial_settings is not None:
        program["initial_settings"] = initial_settings
        program["summary"].update(
            {
                "has_initial_settings": True,
                "initial_settings_source_path": initial_settings.get("source_path"),
                "initial_settings_has_joint_seed": bool(initial_settings.get("summary", {}).get("has_joint_seed")),
                "initial_settings_external_axis_names": initial_settings.get("summary", {}).get("external_axis_names", []),
            }
        )
    return {"ok": True, "program": program, "trajectory": frames, "summary": program["summary"], "initial_settings": initial_settings}


def exec_fanuc_m710_prepare(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Convert FANUC joint and relative Cartesian records to M-710 targets."""
    inputs = _incoming_values(node["id"], edges, context)
    data = _props(node)
    program = inputs.get("program")
    if not isinstance(program, dict):
        raise ValueError("FANUC program input is required")
    frames = program.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("FANUC program has no motion frames")

    seconds_per_frame = float(data.get("secondsPerFrame", 3.0))
    if not math.isfinite(seconds_per_frame) or seconds_per_frame <= 0.0:
        raise ValueError("FANUC secondsPerFrame must be positive")
    total_duration_sec = float(data.get("totalDurationSec", 30.0))
    if not math.isfinite(total_duration_sec) or total_duration_sec < 0.0:
        raise ValueError("FANUC totalDurationSec must be zero or positive")
    timing_mode = str(data.get("timingMode") or "source_speed").strip().lower()
    use_source_timing = timing_mode not in {"fixed_total", "uniform", "legacy"}
    joint_percent_seconds = float(data.get("jointPercentReferenceSec", 2.0))
    trajectory_time_scale = float(data.get("trajectoryTimeScale", 1.0))
    if not math.isfinite(trajectory_time_scale) or trajectory_time_scale <= 0.0:
        raise ValueError("FANUC trajectoryTimeScale must be positive")
    initial_pose_duration_sec = float(data.get("initialPoseDurationSec", 2.0))
    if not math.isfinite(initial_pose_duration_sec) or initial_pose_duration_sec < 0.0:
        raise ValueError("FANUC initialPoseDurationSec must be zero or positive")
    initial_pose_hold_sec = float(data.get("initialPoseHoldSec", 2.0))
    if not math.isfinite(initial_pose_hold_sec) or initial_pose_hold_sec < 0.0:
        raise ValueError("FANUC initialPoseHoldSec must be zero or positive")
    return_to_initial_pose = bool(data.get("returnToInitialPoseOnComplete", True))
    initial_pose_return_sec = float(data.get("initialPoseReturnSec", 2.0))
    if not math.isfinite(initial_pose_return_sec) or initial_pose_return_sec < 0.0:
        raise ValueError("FANUC initialPoseReturnSec must be zero or positive")
    enable_execution = bool(data.get("enableExecution", False))
    preview_only = bool(data.get("previewOnly", True))
    joint_names = [f"J{index}" for index in range(1, 7)]
    joint_trajectory: List[Dict[str, Any]] = []
    skipped_unresolved_details: List[Dict[str, Any]] = []
    skipped_cartesian_details: List[Dict[str, Any]] = []
    skipped_cartesian = 0
    skipped_unresolved = 0
    ik_failures = 0
    solve_cartesian_ik = bool(data.get("solveCartesianIk", True))
    require_all_ik = bool(data.get("requireAllIk", False))
    position_scale = float(data.get("cartesianPositionScale", 1.0))
    cartesian_axis_signs = np.asarray(_float_vector(data.get("cartesianAxisSigns"), 3, [1.0, 1.0, 1.0]), dtype=float)
    joint_axis_signs = np.asarray(
        _float_vector(data.get("jointAxisSigns"), 6, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        dtype=float,
    )
    joint_axis_signs = np.where(joint_axis_signs < 0.0, -1.0, 1.0)
    initial_joint_axis_signs = np.asarray(
        _float_vector(data.get("initialJointAxisSigns"), 6, [1.0, 1.0, 1.0, 1.0, 1.0, 1.0]),
        dtype=float,
    )
    initial_joint_axis_signs = np.where(initial_joint_axis_signs < 0.0, -1.0, 1.0)
    orientation_scale = float(data.get("cartesianOrientationScale", 1.0))
    orientation_weight = float(data.get("ikOrientationWeight", 0.75))
    ik_max_iterations = int(data.get("ikMaxIterations", 220))
    ik_position_tolerance = float(data.get("ikPositionToleranceM", 0.001))
    ik_orientation_tolerance = math.radians(float(data.get("ikOrientationToleranceDeg", 1.0)))
    ik_max_joint_delta = math.radians(float(data.get("ikMaxJointDeltaDeg", 140.0)))
    external_axis_vectors = _external_axis_vectors(inputs.get("externalAxisVectors") or data.get("externalAxisVectors"))
    external_axis_joint_names = _external_axis_joint_names(inputs.get("externalAxisJointNames") or data.get("externalAxisJointNames"))
    external_axis_root_motion_enabled = bool(data.get("externalAxisRootMotionEnabled", True))
    external_axis_affects_ik = bool(data.get("externalAxisAffectsIk", False))
    initial_settings = program.get("initial_settings") if isinstance(program.get("initial_settings"), dict) else {}
    initial_uframe_transforms = _indexed_pose_transforms(initial_settings.get("uframe_transforms"))
    initial_utool_transforms = _indexed_pose_transforms(initial_settings.get("utool_transforms"))
    configured_uframe_transforms = _indexed_pose_transforms(inputs.get("uframeTransforms") or data.get("uframeTransforms"))
    configured_utool_transforms = _indexed_pose_transforms(inputs.get("utoolTransforms") or data.get("utoolTransforms"))
    uframe_transforms = {**initial_uframe_transforms, **configured_uframe_transforms}
    utool_transforms = {**initial_utool_transforms, **configured_utool_transforms}
    initial_joints_deg = initial_settings.get("joints_deg") if isinstance(initial_settings, dict) else None
    initial_external_axes_mm = (
        initial_settings.get("external_axes_mm")
        if isinstance(initial_settings, dict) and isinstance(initial_settings.get("external_axes_mm"), dict)
        else {}
    )
    external_axis_origin_targets_m = _external_axis_targets_m(initial_external_axes_mm)
    if not external_axis_origin_targets_m:
        external_axis_origin_targets_m = {axis_name: 0.0 for axis_name in external_axis_joint_names}
    seed_source = initial_joints_deg if isinstance(initial_joints_deg, list) and len(initial_joints_deg) >= 6 else data.get("ikSeedDeg")
    seed_source_values = np.asarray(_float_vector(seed_source, 6, [0, 0, 0, 0, 0, 0]), dtype=float)
    if isinstance(initial_joints_deg, list) and len(initial_joints_deg) >= 6:
        seed_source_values = seed_source_values * initial_joint_axis_signs
    seed = np.radians(seed_source_values)
    anchor_by_frame: Dict[tuple[Any, Any], tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]] = {}
    elapsed_time = 0.0
    previous_joints: np.ndarray | None = seed.copy() if initial_joints_deg else None
    previous_tcp: np.ndarray | None = _m710_fk(seed)[0] if initial_joints_deg else None
    external_axis_root_origin_m: np.ndarray | None = (
        _external_axis_offset_m(initial_external_axes_mm, external_axis_vectors)
        if initial_external_axes_mm
        else None
    )

    def record_unresolved(frame: Dict[str, Any], position: Dict[str, Any] | None, reason: str) -> None:
        skipped_unresolved_details.append(
            {
                "line": frame.get("line"),
                "position_id": frame.get("position_id"),
                "motion_type": frame.get("motion_type"),
                "termination": frame.get("termination"),
                "active_uf": frame.get("active_uf"),
                "active_ut": frame.get("active_ut"),
                "position_kind": position.get("kind") if isinstance(position, dict) else None,
                "position_uf": position.get("uf") if isinstance(position, dict) else None,
                "position_ut": position.get("ut") if isinstance(position, dict) else None,
                "reason": reason,
            }
        )

    def record_cartesian_skip(frame: Dict[str, Any], reason: str, solution: Dict[str, Any] | None = None) -> None:
        detail = {
            "line": frame.get("line"),
            "position_id": frame.get("position_id"),
            "motion_type": frame.get("motion_type"),
            "termination": frame.get("termination"),
            "active_uf": frame.get("active_uf"),
            "active_ut": frame.get("active_ut"),
            "reason": reason,
        }
        if solution:
            detail.update(
                {
                    "position_error_m": solution.get("position_error_m"),
                    "orientation_error_rad": solution.get("orientation_error_rad"),
                    "configuration_branch_preserved": solution.get("configuration_branch_preserved"),
                    "max_joint_delta_rad": solution.get("max_joint_delta_rad"),
                }
            )
        skipped_cartesian_details.append(detail)

    for frame in frames:
        position = frame.get("position") if isinstance(frame, dict) else None
        if not isinstance(position, dict):
            skipped_unresolved += 1
            record_unresolved(frame if isinstance(frame, dict) else {}, None, "missing_position_payload")
            continue
        if position.get("kind") == "cartesian":
            if not solve_cartesian_ik:
                skipped_cartesian += 1
                record_cartesian_skip(frame, "cartesian_ik_disabled")
                continue
            xyz = np.asarray(position.get("xyz_mm", []), dtype=float)
            wpr = position.get("wpr_deg", [])
            if xyz.shape != (3,) or not isinstance(wpr, list) or len(wpr) != 3:
                skipped_unresolved += 1
                record_unresolved(frame, position, "invalid_cartesian_xyz_wpr")
                continue
            frame_uf = position.get("uf") if position.get("uf") is not None else frame.get("active_uf")
            frame_ut = position.get("ut") if position.get("ut") is not None else frame.get("active_ut")
            frame_key = (frame_uf, frame_ut)
            source_rotation = _wpr_rotation([math.radians(float(value)) for value in wpr])
            external_axes_mm = position.get("external_axes_mm") if isinstance(position.get("external_axes_mm"), dict) else {}
            external_axis_offset_m = _external_axis_offset_m(external_axes_mm, external_axis_vectors)
            if external_axis_root_origin_m is None:
                external_axis_root_origin_m = external_axis_offset_m.copy()
            external_axis_root_translation = (
                (external_axis_offset_m - external_axis_root_origin_m) * cartesian_axis_signs
                if external_axis_root_motion_enabled
                else np.zeros(3)
            )
            source_position_m = xyz * 0.001 + (external_axis_offset_m if external_axis_affects_ik else np.zeros(3))
            if isinstance(frame_uf, int) and frame_uf in uframe_transforms:
                target_position, target_rotation = _apply_transform(
                    source_position_m,
                    source_rotation,
                    uframe_transforms[frame_uf],
                )
                cartesian_mode = "calibrated_uf_ik"
            elif frame_key not in anchor_by_frame:
                current_position, current_rotation = _m710_fk(seed)
                anchor_by_frame[frame_key] = (source_position_m.copy(), source_rotation.copy(), current_position, current_rotation)
                source_origin, source_orientation, target_origin, target_orientation = anchor_by_frame[frame_key]
                target_position = target_origin + (source_position_m - source_origin) * position_scale * cartesian_axis_signs
                relative_rotation = source_orientation.T @ source_rotation
                target_rotation = target_orientation @ _scale_rotation(relative_rotation, orientation_scale)
                cartesian_mode = "relative_uf_ut_ik"
            else:
                source_origin, source_orientation, target_origin, target_orientation = anchor_by_frame[frame_key]
                target_position = target_origin + (source_position_m - source_origin) * position_scale * cartesian_axis_signs
                relative_rotation = source_orientation.T @ source_rotation
                target_rotation = target_orientation @ _scale_rotation(relative_rotation, orientation_scale)
                cartesian_mode = "relative_uf_ut_ik"
            if isinstance(frame_ut, int) and frame_ut in utool_transforms:
                tool_origin, tool_rotation = utool_transforms[frame_ut]
                target_position = target_position - target_rotation @ tool_origin + target_rotation @ M710_TOOL_OFFSET
                target_rotation = target_rotation @ tool_rotation.T
            solution = _solve_m710_ik(
                target_position,
                target_rotation,
                seed,
                max_iterations=ik_max_iterations,
                position_tolerance=ik_position_tolerance,
                orientation_tolerance=ik_orientation_tolerance,
                orientation_weight=orientation_weight,
                max_joint_delta=ik_max_joint_delta,
            )
            if not solution["ok"]:
                ik_failures += 1
                skipped_cartesian += 1
                record_cartesian_skip(frame, "ik_not_converged_or_branch_changed", solution)
                continue
            seed = np.asarray(solution["joint_positions"])
            values = np.degrees(seed).tolist()
            next_tcp, _ = _m710_fk(seed)
            if use_source_timing:
                elapsed_time += _motion_duration_seconds(
                    frame,
                    previous_joints,
                    seed,
                    previous_tcp,
                    next_tcp,
                    seconds_per_frame,
                    joint_percent_seconds,
                )
                time_sec = elapsed_time
            else:
                time_sec = len(joint_trajectory) * seconds_per_frame + seconds_per_frame
            previous_joints = seed.copy()
            previous_tcp = next_tcp.copy()
            external_axis_targets = {
                **external_axis_origin_targets_m,
                **_external_axis_targets_m(external_axes_mm),
            }
            external_joint_targets = _external_axis_joint_target_map(external_axis_targets, external_axis_joint_names)
            frame_joint_names = [*joint_names, *external_joint_targets.keys()]
            frame_joint_target_map = {**dict(zip(joint_names, values)), **external_joint_targets}
            joint_trajectory.append(
                {
                    "time_sec": time_sec,
                    "unit": "degree",
                    "joint_names": frame_joint_names,
                    "joint_positions": values,
                    "joint_target_map": frame_joint_target_map,
                    "external_axis_targets_m": external_axis_targets,
                    "external_axis_root_translation": external_axis_root_translation.tolist(),
                    "root_translation": external_axis_root_translation.tolist(),
                    "rootTranslation": external_axis_root_translation.tolist(),
                    "root_translation_mode": "offset_from_initial",
                    "rootTranslationMode": "offset_from_initial",
                    "source_position_id": frame.get("position_id"),
                    "source_line": frame.get("line"),
                    "motion_type": frame.get("motion_type"),
                    "termination": frame.get("termination"),
                    "active_uf": frame.get("active_uf"),
                    "active_ut": frame.get("active_ut"),
                    "position_uf": position.get("uf"),
                    "position_ut": position.get("ut"),
                    "config": position.get("config"),
                    "cartesian_mode": cartesian_mode,
                    "speed": frame.get("speed"),
                    "speed_unit": frame.get("speed_unit"),
                    "cnt_percent": _cnt_percent(frame.get("termination")),
                    "ik_solved": True,
                    "ik_position_error_m": solution["position_error_m"],
                    "ik_orientation_error_rad": solution["orientation_error_rad"],
                    "external_axes_mm": external_axes_mm,
                }
            )
            continue
        joints = position.get("joints_deg")
        if position.get("kind") != "joint" or not isinstance(joints, list) or len(joints) != 6:
            skipped_unresolved += 1
            record_unresolved(frame, position, "missing_or_unknown_position_record")
            continue
        values = (
            np.asarray([float(value) for value in joints], dtype=float)
            * joint_axis_signs
        ).tolist()
        if not all(math.isfinite(value) for value in values):
            raise ValueError(
                f"FANUC P[{frame.get('position_id')}] contains non-finite joint values"
            )
        next_joints = np.radians(np.asarray(values))
        next_tcp, _ = _m710_fk(next_joints)
        if use_source_timing:
            elapsed_time += _motion_duration_seconds(
                frame,
                previous_joints,
                next_joints,
                previous_tcp,
                next_tcp,
                seconds_per_frame,
                joint_percent_seconds,
            )
            time_sec = elapsed_time
        else:
            time_sec = len(joint_trajectory) * seconds_per_frame + seconds_per_frame
        previous_joints = next_joints.copy()
        previous_tcp = next_tcp.copy()
        external_axes_mm = position.get("external_axes_mm") if isinstance(position.get("external_axes_mm"), dict) else {}
        external_axis_targets = {
            **external_axis_origin_targets_m,
            **_external_axis_targets_m(external_axes_mm),
        }
        external_joint_targets = _external_axis_joint_target_map(external_axis_targets, external_axis_joint_names)
        frame_joint_names = [*joint_names, *external_joint_targets.keys()]
        frame_joint_target_map = {**dict(zip(joint_names, values)), **external_joint_targets}
        joint_trajectory.append(
            {
                "time_sec": time_sec,
                "unit": "degree",
                "joint_names": frame_joint_names,
                "joint_positions": values,
                "joint_target_map": frame_joint_target_map,
                "external_axis_targets_m": external_axis_targets,
                "source_position_id": frame.get("position_id"),
                "source_line": frame.get("line"),
                "motion_type": frame.get("motion_type"),
                "termination": frame.get("termination"),
                "active_uf": frame.get("active_uf"),
                "active_ut": frame.get("active_ut"),
                "position_uf": position.get("uf"),
                "position_ut": position.get("ut"),
                "config": position.get("config"),
                "speed": frame.get("speed"),
                "speed_unit": frame.get("speed_unit"),
                "cnt_percent": _cnt_percent(frame.get("termination")),
                "external_axes_mm": external_axes_mm,
            }
        )
        seed = next_joints

    trailing_external_axis_return = {"trimmed_frames": 0}
    if use_source_timing and joint_trajectory:
        first_time_sec = (
            0.0
            if isinstance(initial_joints_deg, list) and len(initial_joints_deg) >= 6
            else float(joint_trajectory[0].get("time_sec", 0.0))
        )
        for trajectory_frame in joint_trajectory:
            normalized_time = max(0.0, float(trajectory_frame.get("time_sec", 0.0)) - first_time_sec)
            trajectory_frame["time_sec"] = normalized_time / trajectory_time_scale
        total_duration_sec = float(joint_trajectory[-1].get("time_sec", 0.0))
        trailing_external_axis_return = _trim_trailing_external_axis_return(joint_trajectory)
        if trailing_external_axis_return.get("trimmed_frames"):
            total_duration_sec = joint_trajectory[-1]["time_sec"]
            seconds_per_frame = total_duration_sec / max(1, len(joint_trajectory) - 1)
    elif joint_trajectory and total_duration_sec > 0.0:
        seconds_per_frame = total_duration_sec / len(joint_trajectory)
        if len(joint_trajectory) == 1:
            joint_trajectory[0]["time_sec"] = total_duration_sec
        else:
            # Keep the first target at two seconds for compatibility with an
            # already-running pre-0.3 bridge, then distribute the remaining
            # targets through the requested total duration.
            first_target_sec = min(2.0, total_duration_sec / 2.0)
            remaining_step = (total_duration_sec - first_target_sec) / (len(joint_trajectory) - 1)
            for index, trajectory_frame in enumerate(joint_trajectory):
                trajectory_frame["time_sec"] = first_target_sec + index * remaining_step
        trailing_external_axis_return = _trim_trailing_external_axis_return(joint_trajectory)
        if trailing_external_axis_return.get("trimmed_frames"):
            total_duration_sec = joint_trajectory[-1]["time_sec"]
            seconds_per_frame = total_duration_sec / max(1, len(joint_trajectory) - 1)
    elif joint_trajectory:
        trailing_external_axis_return = _trim_trailing_external_axis_return(joint_trajectory)
        total_duration_sec = len(joint_trajectory) * seconds_per_frame

    initial_pose_frame: Dict[str, Any] | None = None
    if isinstance(initial_joints_deg, list) and len(initial_joints_deg) >= 6:
        initial_values = (
            np.asarray([float(value) for value in initial_joints_deg[:6]], dtype=float)
            * initial_joint_axis_signs
        ).tolist()
        if all(math.isfinite(value) for value in initial_values):
            initial_external_targets = dict(external_axis_origin_targets_m)
            initial_external_joint_targets = _external_axis_joint_target_map(
                initial_external_targets,
                external_axis_joint_names,
            )
            initial_pose_frame = {
                "time_sec": max(0.001, initial_pose_duration_sec),
                "unit": "degree",
                "joint_names": [*joint_names, *initial_external_joint_targets.keys()],
                "joint_positions": initial_values,
                "joint_target_map": {
                    **dict(zip(joint_names, initial_values)),
                    **initial_external_joint_targets,
                },
                "external_axis_targets_m": initial_external_targets,
                "source": "initial_settings",
                "source_pose_name": initial_settings.get("pose_name") if isinstance(initial_settings, dict) else None,
                "source_path": initial_settings.get("source_path") if isinstance(initial_settings, dict) else None,
            }
            if initial_pose_duration_sec > 0.0:
                initial_total_delay_sec = initial_pose_duration_sec + initial_pose_hold_sec
                for trajectory_frame in joint_trajectory:
                    trajectory_frame["time_sec"] = (
                        float(trajectory_frame.get("time_sec", 0.0))
                        + initial_total_delay_sec
                    )
                if initial_pose_hold_sec > 0.0:
                    initial_hold_frame = dict(initial_pose_frame)
                    initial_hold_frame["time_sec"] = initial_total_delay_sec
                    initial_hold_frame["source"] = "initial_settings_hold"
                    joint_trajectory.insert(0, initial_hold_frame)
                joint_trajectory.insert(0, initial_pose_frame)
                total_duration_sec = (
                    float(joint_trajectory[-1].get("time_sec", 0.0))
                    if joint_trajectory
                    else initial_pose_duration_sec
                )
            if return_to_initial_pose and initial_pose_return_sec > 0.0 and joint_trajectory:
                initial_return_frame = dict(initial_pose_frame)
                initial_return_frame["time_sec"] = (
                    float(joint_trajectory[-1].get("time_sec", 0.0))
                    + initial_pose_return_sec
                )
                initial_return_frame["source"] = "initial_settings_return"
                joint_trajectory.append(initial_return_frame)
                total_duration_sec = float(initial_return_frame["time_sec"])

    frame_time_sanitization = _ensure_strictly_increasing_frame_times(joint_trajectory)
    if joint_trajectory:
        total_duration_sec = float(joint_trajectory[-1].get("time_sec", total_duration_sec))

    execution_ready = bool(joint_trajectory) and (
        not require_all_ik or (ik_failures == 0 and skipped_cartesian == 0 and skipped_unresolved == 0)
    )
    effective_preview_only = preview_only or not enable_execution or not execution_ready
    action = {
        "schema": "vlagraph.fanuc_m710ic20l.action.v1",
        "source_program": {"name": program.get("program_name"), "path": program.get("source_path")},
        "source_instruction_audit": program.get("instruction_audit"),
        "type": "fanuc_m710ic20l_trajectory",
        "command": "apply_fanuc_trajectory",
        "target_embodiment": "FANUC_M710IC_20L",
        "unit": "degree",
        "joint_names": joint_names,
        "seconds_per_frame": seconds_per_frame,
        "total_duration_sec": total_duration_sec,
        "joint_trajectory": joint_trajectory,
        "initial_pose": initial_pose_frame,
        "initial_pose_duration_sec": initial_pose_duration_sec if initial_pose_frame else 0.0,
        "initial_pose_hold_sec": initial_pose_hold_sec if initial_pose_frame else 0.0,
        "return_to_initial_pose_on_complete": return_to_initial_pose,
        "initial_pose_return_sec": initial_pose_return_sec if initial_pose_frame and return_to_initial_pose else 0.0,
        "preview_only": effective_preview_only,
        "execution_enabled": enable_execution,
        "execution_ready": execution_ready,
        "ik_failures": ik_failures,
        "skipped_cartesian_frames": skipped_cartesian,
        "skipped_unresolved_frames": skipped_unresolved,
        "requires_uf_ut_calibration": bool(program.get("summary", {}).get("requires_uf_ut_calibration")),
        "cartesian_mode": (
            "calibrated_uf_ik"
            if solve_cartesian_ik and uframe_transforms
            else "relative_uf_ut_ik"
            if solve_cartesian_ik
            else "disabled"
        ),
        "cartesian_position_scale": position_scale,
        "cartesian_axis_signs": cartesian_axis_signs.tolist(),
        "joint_axis_signs": joint_axis_signs.tolist(),
        "initial_joint_axis_signs": initial_joint_axis_signs.tolist(),
        "external_axis_vectors": {name: vector.tolist() for name, vector in external_axis_vectors.items()},
        "external_axis_joint_names": external_axis_joint_names,
        "external_axis_root_motion_enabled": external_axis_root_motion_enabled,
        "external_axis_affects_ik": external_axis_affects_ik,
        "cartesian_orientation_scale": orientation_scale,
        "timing_mode": "source_speed" if use_source_timing else "fixed_total",
        "trajectory_time_scale": trajectory_time_scale,
        "calibrated_uframes": sorted(uframe_transforms.keys()),
        "calibrated_utools": sorted(utool_transforms.keys()),
        "ik_position_tolerance_m": ik_position_tolerance,
        "ik_orientation_tolerance_deg": math.degrees(ik_orientation_tolerance),
        "ik_orientation_weight": orientation_weight,
        "ik_max_joint_delta_deg": math.degrees(ik_max_joint_delta),
        "direct_joint_frames": sum(1 for frame in joint_trajectory if not frame.get("ik_solved")),
        "skipped_unresolved_details": skipped_unresolved_details,
        "skipped_cartesian_details": skipped_cartesian_details,
        "initial_settings_applied": bool(initial_settings),
        "initial_settings_source_path": initial_settings.get("source_path") if isinstance(initial_settings, dict) else None,
        "initial_joint_seed_applied": bool(initial_joints_deg),
        "initial_pose_frame_applied": bool(initial_pose_frame),
        "initial_external_axes_mm": initial_external_axes_mm,
        "initial_pose_hold_sec": initial_pose_hold_sec if initial_pose_frame else 0.0,
        "return_to_initial_pose_on_complete": return_to_initial_pose,
        "initial_pose_return_sec": initial_pose_return_sec if initial_pose_frame and return_to_initial_pose else 0.0,
        "trailing_external_axis_return": trailing_external_axis_return,
        "frame_time_sanitization": frame_time_sanitization,
    }
    calibrated_cartesian_frames = sum(
        1 for frame in joint_trajectory if frame.get("cartesian_mode") == "calibrated_uf_ik"
    )
    relative_cartesian_frames = sum(
        1 for frame in joint_trajectory if frame.get("cartesian_mode") == "relative_uf_ut_ik"
    )
    action["calibrated_cartesian_frames"] = calibrated_cartesian_frames
    action["relative_cartesian_frames"] = relative_cartesian_frames
    summary = {
        "source_frames": len(frames),
        "executable_joint_frames": len(joint_trajectory),
        "direct_joint_frames": sum(1 for frame in joint_trajectory if not frame.get("ik_solved")),
        "calibrated_cartesian_frames": calibrated_cartesian_frames,
        "relative_cartesian_frames": relative_cartesian_frames,
        "skipped_cartesian_frames": skipped_cartesian,
        "skipped_unresolved_frames": skipped_unresolved,
        "ik_solved_frames": sum(1 for frame in joint_trajectory if frame.get("ik_solved")),
        "ik_failures": ik_failures,
        "skipped_unresolved_details": skipped_unresolved_details,
        "skipped_cartesian_details": skipped_cartesian_details,
        "execution_ready": execution_ready,
        "preview_only": effective_preview_only,
        "total_duration_sec": total_duration_sec,
        "seconds_per_frame": seconds_per_frame,
        "timing_mode": "source_speed" if use_source_timing else "fixed_total",
        "trajectory_time_scale": trajectory_time_scale,
        "cartesian_axis_signs": cartesian_axis_signs.tolist(),
        "joint_axis_signs": joint_axis_signs.tolist(),
        "initial_joint_axis_signs": initial_joint_axis_signs.tolist(),
        "external_axis_vectors": {name: vector.tolist() for name, vector in external_axis_vectors.items()},
        "external_axis_joint_names": external_axis_joint_names,
        "external_axis_root_motion_enabled": external_axis_root_motion_enabled,
        "external_axis_affects_ik": external_axis_affects_ik,
        "initial_settings_applied": bool(initial_settings),
        "initial_settings_source_path": initial_settings.get("source_path") if isinstance(initial_settings, dict) else None,
        "initial_joint_seed_applied": bool(initial_joints_deg),
        "initial_pose_frame_applied": bool(initial_pose_frame),
        "initial_pose_duration_sec": initial_pose_duration_sec if initial_pose_frame else 0.0,
        "initial_pose_hold_sec": initial_pose_hold_sec if initial_pose_frame else 0.0,
        "return_to_initial_pose_on_complete": return_to_initial_pose,
        "initial_pose_return_sec": initial_pose_return_sec if initial_pose_frame and return_to_initial_pose else 0.0,
        "initial_external_axes_mm": initial_external_axes_mm,
        "trailing_external_axis_return": trailing_external_axis_return,
        "frame_time_sanitization": frame_time_sanitization,
        "calibrated_uframes": sorted(uframe_transforms.keys()),
        "calibrated_utools": sorted(utool_transforms.keys()),
    }
    return {
        "ok": True,
        "action": action,
        "preview": joint_trajectory,
        "summary": summary,
    }


def _float_vector(value: Any, size: int, default: List[float]) -> List[float]:
    if isinstance(value, str):
        value = [part for part in re.split(r"[\s,;]+", value.strip()) if part]
    if not isinstance(value, (list, tuple)):
        return list(default)
    result: List[float] = []
    for item in value[:size]:
        try:
            result.append(float(item))
        except (TypeError, ValueError):
            result.append(0.0)
    return (result + list(default))[:size]


def _rotation(axis: np.ndarray, angle: float) -> np.ndarray:
    axis = axis / max(float(np.linalg.norm(axis)), 1e-12)
    x, y, z = axis
    skew = np.asarray([[0, -z, y], [z, 0, -x], [-y, x, 0]], dtype=float)
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)


def _wpr_rotation(wpr: List[float]) -> np.ndarray:
    w, p, r = wpr
    return _rotation(np.asarray([0, 0, 1]), r) @ _rotation(
        np.asarray([0, 1, 0]), p
    ) @ _rotation(np.asarray([1, 0, 0]), w)


def _json_object(value: Any) -> Dict[str, Any]:
    if isinstance(value, str):
        text = value.strip()
        if not text:
            return {}
        try:
            value = json.loads(text)
        except json.JSONDecodeError:
            return {}
    return value if isinstance(value, dict) else {}


def _vector_from_keys(value: Dict[str, Any], keys: List[str], size: int) -> List[float] | None:
    for key in keys:
        candidate = value.get(key)
        if isinstance(candidate, str):
            candidate = [part for part in re.split(r"[\s,;]+", candidate.strip()) if part]
        if isinstance(candidate, (list, tuple)) and len(candidate) >= size:
            try:
                result = [float(item) for item in candidate[:size]]
            except (TypeError, ValueError):
                continue
            if all(math.isfinite(item) for item in result):
                return result
    return None


def _pose_transform(value: Any) -> tuple[np.ndarray, np.ndarray] | None:
    data = _json_object(value)
    if not data:
        return None
    matrix = data.get("matrix") or data.get("transform")
    if isinstance(matrix, (list, tuple)) and len(matrix) == 4:
        transform = np.asarray(matrix, dtype=float)
        if transform.shape == (4, 4):
            return transform[:3, 3], transform[:3, :3]
    position_mm = _vector_from_keys(data, ["xyzMm", "xyz_mm", "positionMm", "position_mm"], 3)
    position_m = _vector_from_keys(data, ["xyzM", "xyz_m", "positionM", "position_m"], 3)
    wpr_deg = _vector_from_keys(data, ["wprDeg", "wpr_deg", "rotationWprDeg", "rotation_wpr_deg"], 3)
    wpr_rad = _vector_from_keys(data, ["wprRad", "wpr_rad", "rotationWprRad", "rotation_wpr_rad"], 3)
    if position_m is None and position_mm is not None:
        position_m = [item * 0.001 for item in position_mm]
    if position_m is None:
        position_m = [0.0, 0.0, 0.0]
    if wpr_rad is None:
        wpr_rad = [math.radians(item) for item in (wpr_deg or [0.0, 0.0, 0.0])]
    return np.asarray(position_m, dtype=float), _wpr_rotation(wpr_rad)


def _indexed_pose_transforms(value: Any) -> Dict[int, tuple[np.ndarray, np.ndarray]]:
    raw = _json_object(value)
    result: Dict[int, tuple[np.ndarray, np.ndarray]] = {}
    for key, item in raw.items():
        try:
            index = int(str(key))
        except ValueError:
            continue
        transform = _pose_transform(item)
        if transform is not None:
            result[index] = transform
    return result


def _apply_transform(
    position: np.ndarray,
    rotation: np.ndarray,
    transform: tuple[np.ndarray, np.ndarray],
) -> tuple[np.ndarray, np.ndarray]:
    origin, orientation = transform
    return origin + orientation @ position, orientation @ rotation


def _cnt_percent(termination: Any) -> int:
    match = re.match(r"CNT(\d+)$", str(termination or "").strip(), re.I)
    if not match:
        return 0
    return max(0, min(100, int(match.group(1))))



def _external_axis_vectors(value: Any) -> Dict[str, np.ndarray]:
    raw = _json_object(value)
    if not raw:
        raw = {"E1": [1.0, 0.0, 0.0]}
    vectors: Dict[str, np.ndarray] = {}
    for raw_name, raw_vector in raw.items():
        name = str(raw_name or "").strip().upper()
        if not re.match(r"^E\d+$", name):
            continue
        vector = _float_vector(raw_vector, 3, [0.0, 0.0, 0.0])
        vectors[name] = np.asarray(vector, dtype=float)
    return vectors


def _external_axis_offset_m(external_axes_mm: Any, vectors: Dict[str, np.ndarray]) -> np.ndarray:
    if not isinstance(external_axes_mm, dict) or not vectors:
        return np.zeros(3)
    offset = np.zeros(3)
    for raw_name, raw_value in external_axes_mm.items():
        name = str(raw_name or "").strip().upper()
        vector = vectors.get(name)
        if vector is None:
            continue
        try:
            value_m = float(raw_value) * 0.001
        except (TypeError, ValueError):
            continue
        if math.isfinite(value_m):
            offset += vector * value_m
    return offset


def _trim_trailing_external_axis_return(
    joint_trajectory: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Drop the final E-axis return-home segment from executable FANUC frames."""
    if len(joint_trajectory) < 3:
        return {"trimmed_frames": 0}

    offsets: List[np.ndarray] = []
    valid_indices: List[int] = []
    for index, frame in enumerate(joint_trajectory):
        root_translation = frame.get("root_translation")
        if not isinstance(root_translation, (list, tuple)):
            continue
        values = _float_vector(root_translation, 3, [0.0, 0.0, 0.0])
        vector = np.asarray(values, dtype=float)
        if np.all(np.isfinite(vector)):
            valid_indices.append(index)
            offsets.append(vector)

    if len(offsets) < 3:
        return {"trimmed_frames": 0}

    norms = [float(np.linalg.norm(offset)) for offset in offsets]
    max_norm = max(norms)
    if max_norm <= 1.0e-6:
        return {"trimmed_frames": 0}
    final_norm = norms[-1]
    if final_norm > max_norm * 0.25:
        return {"trimmed_frames": 0}

    hold_valid_index = None
    for index in range(len(offsets) - 2, -1, -1):
        if norms[index] >= max_norm * 0.8:
            hold_valid_index = index
            break
    if hold_valid_index is None:
        return {"trimmed_frames": 0}

    hold_frame_index = valid_indices[hold_valid_index]
    trimmed_frames = len(joint_trajectory) - hold_frame_index - 1
    if trimmed_frames <= 0:
        return {"trimmed_frames": 0}

    del joint_trajectory[hold_frame_index + 1:]
    return {
        "trimmed_frames": trimmed_frames,
        "hold_frame_index": hold_frame_index,
        "hold_time_sec": joint_trajectory[-1].get("time_sec"),
        "hold_root_translation": joint_trajectory[-1].get("root_translation"),
        "max_root_offset_m": max_norm,
        "final_root_offset_m": final_norm,
    }


def _ensure_strictly_increasing_frame_times(
    joint_trajectory: List[Dict[str, Any]],
    *,
    min_step_sec: float = 0.001,
) -> Dict[str, Any]:
    adjusted_frames = 0
    previous_time: float | None = None
    for frame in joint_trajectory:
        try:
            time_sec = float(frame.get("time_sec", 0.0))
        except (TypeError, ValueError):
            time_sec = 0.0
        if not math.isfinite(time_sec):
            time_sec = 0.0
        if previous_time is not None and time_sec <= previous_time:
            time_sec = previous_time + min_step_sec
            adjusted_frames += 1
        frame["time_sec"] = round(time_sec, 6)
        previous_time = frame["time_sec"]
    return {
        "adjusted_frames": adjusted_frames,
        "min_step_sec": min_step_sec,
    }


def _external_axis_targets_m(external_axes_mm: Any) -> Dict[str, float]:
    if not isinstance(external_axes_mm, dict):
        return {}
    targets: Dict[str, float] = {}
    for raw_name, raw_value in external_axes_mm.items():
        name = str(raw_name or "").strip().upper()
        if not re.match(r"^E\d+$", name):
            continue
        try:
            value_m = float(raw_value) * 0.001
        except (TypeError, ValueError):
            continue
        if math.isfinite(value_m):
            targets[name] = value_m
    return targets

def _external_axis_joint_names(value: Any) -> Dict[str, str]:
    raw = _json_object(value)
    result: Dict[str, str] = {}
    for raw_name, raw_target in raw.items():
        name = str(raw_name or "").strip().upper()
        if not re.match(r"^E\d+$", name):
            continue
        target = str(raw_target or "").strip()
        if target:
            result[name] = target
    return result


def _external_axis_joint_target_map(
    external_axis_targets: Dict[str, float],
    joint_names_by_axis: Dict[str, str],
) -> Dict[str, float]:
    # `external_axis_targets` is stored as meters for root_translation metadata,
    # but joint_target_map uses command units. FANUC E axes are commanded in mm.
    targets: Dict[str, float] = {
        axis_name: value_m * 1000.0
        for axis_name, value_m in external_axis_targets.items()
    }
    for axis_name, value in external_axis_targets.items():
        joint_name = joint_names_by_axis.get(axis_name)
        if joint_name and joint_name != axis_name:
            targets[joint_name] = value * 1000.0
    return targets

def _motion_duration_seconds(
    frame: Dict[str, Any],
    previous_joints: np.ndarray | None,
    next_joints: np.ndarray,
    previous_tcp: np.ndarray | None,
    next_tcp: np.ndarray | None,
    default_seconds: float,
    joint_percent_seconds: float,
) -> float:
    speed = float(frame.get("speed") or 0.0)
    speed_unit = str(frame.get("speed_unit") or "").lower()
    if speed <= 0.0:
        return default_seconds
    if speed_unit == "mm/sec" and previous_tcp is not None and next_tcp is not None:
        distance_m = float(np.linalg.norm(next_tcp - previous_tcp))
        return max(distance_m / (speed * 0.001), 0.02)
    if speed_unit == "%" and previous_joints is not None:
        joint_delta = float(np.max(np.abs(next_joints - previous_joints)))
        speed_scale = max(speed / 100.0, 0.01)
        return max((joint_delta / math.radians(60.0)) * joint_percent_seconds / speed_scale, 0.02)
    return default_seconds


def _omy3m_fk(joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rotation = np.eye(3)
    position = np.zeros(3)
    for origin, axis, angle in zip(OMY3M_JOINT_ORIGINS, OMY3M_JOINT_AXES, joints):
        position = position + rotation @ origin
        rotation = rotation @ _rotation(axis, float(angle))
    return position, rotation


def _orientation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    # World-frame small-angle error, stable near the solution.
    delta = target @ current.T
    return 0.5 * np.asarray(
        [
            delta[2, 1] - delta[1, 2],
            delta[0, 2] - delta[2, 0],
            delta[1, 0] - delta[0, 1],
        ]
    )


def _scale_rotation(rotation: np.ndarray, scale: float) -> np.ndarray:
    """Scale a relative rotation through its axis-angle representation."""
    cosine = float(np.clip((np.trace(rotation) - 1.0) * 0.5, -1.0, 1.0))
    angle = math.acos(cosine)
    if angle < 1e-9:
        return np.eye(3)
    if abs(math.sin(angle)) < 1e-7:
        values, vectors = np.linalg.eigh(rotation)
        axis = np.real(vectors[:, int(np.argmin(np.abs(values - 1.0)))])
    else:
        axis = np.asarray(
            [rotation[2, 1] - rotation[1, 2], rotation[0, 2] - rotation[2, 0], rotation[1, 0] - rotation[0, 1]]
        ) / (2.0 * math.sin(angle))
    return _rotation(axis, angle * scale)


def _m710_fk(joints: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    rotation = np.eye(3)
    position = np.zeros(3)
    for origin, axis, angle in zip(M710_JOINT_ORIGINS, M710_JOINT_AXES, joints):
        position = position + rotation @ origin
        rotation = rotation @ _rotation(axis, float(angle))
    return position + rotation @ M710_TOOL_OFFSET, rotation


def _nearest_limited_equivalent(angle: float, seed: float, lower: float, upper: float) -> float:
    """Prefer the joint equivalent closest to the previous controller branch."""
    candidates = [angle + 2.0 * math.pi * offset for offset in range(-2, 3)]
    valid = [value for value in candidates if lower <= value <= upper]
    if not valid:
        return float(np.clip(angle, lower, upper))
    return min(valid, key=lambda value: abs(value - seed))


def _nearest_m710_branch(joints: np.ndarray, seed: np.ndarray) -> np.ndarray:
    return np.asarray(
        [
            _nearest_limited_equivalent(
                float(angle),
                float(seed[index]),
                float(M710_JOINT_LIMITS[index, 0]),
                float(M710_JOINT_LIMITS[index, 1]),
            )
            for index, angle in enumerate(joints)
        ],
        dtype=float,
    )


def _solve_m710_ik(
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    seed: np.ndarray,
    *,
    max_iterations: int = 140,
    position_tolerance: float = 0.001,
    orientation_tolerance: float = math.radians(1.0),
    orientation_weight: float = 0.75,
    max_joint_delta: float | None = math.radians(140.0),
) -> Dict[str, Any]:
    seed_np = np.asarray(seed, dtype=float)
    joints = np.clip(seed_np, M710_JOINT_LIMITS[:, 0], M710_JOINT_LIMITS[:, 1])
    damping = 0.025
    epsilon = 1e-5
    position_error = float("inf")
    orientation_error = float("inf")
    converged = False
    iteration = 0
    for iteration in range(max(1, max_iterations)):
        current_position, current_rotation = _m710_fk(joints)
        pos_error = np.asarray(target_position) - current_position
        rot_error = _orientation_error(current_rotation, target_rotation)
        position_error = float(np.linalg.norm(pos_error))
        orientation_error = float(np.linalg.norm(rot_error))
        if position_error <= position_tolerance and orientation_error <= orientation_tolerance:
            converged = True
            break
        error = np.concatenate((pos_error, rot_error * orientation_weight))
        jacobian = np.zeros((6, 6), dtype=float)
        for index in range(6):
            perturbed = joints.copy()
            perturbed[index] += epsilon
            next_position, next_rotation = _m710_fk(perturbed)
            jacobian[:3, index] = (next_position - current_position) / epsilon
            jacobian[3:, index] = (
                _orientation_error(current_rotation, next_rotation) / epsilon * orientation_weight
            )
        lhs = jacobian @ jacobian.T + (damping * damping) * np.eye(6)
        delta = jacobian.T @ np.linalg.solve(lhs, error)
        joints = np.clip(joints + np.clip(delta, -0.12, 0.12), M710_JOINT_LIMITS[:, 0], M710_JOINT_LIMITS[:, 1])
        joints = _nearest_m710_branch(joints, seed_np)
    joint_delta = np.abs(joints - seed_np)
    branch_preserved = bool(max_joint_delta is None or np.max(joint_delta) <= max_joint_delta)
    return {
        "ok": converged and branch_preserved,
        "joint_positions": joints.tolist(),
        "position_error_m": position_error,
        "orientation_error_rad": orientation_error,
        "iterations": iteration + 1,
        "configuration_branch_preserved": branch_preserved,
        "max_joint_delta_rad": float(np.max(joint_delta)),
    }


def _solve_omy3m_ik(
    target_position: List[float],
    target_wpr: List[float],
    seed: List[float],
    *,
    max_iterations: int,
    position_tolerance: float,
    orientation_tolerance: float,
) -> Dict[str, Any]:
    joints = np.asarray(seed[:6], dtype=float)
    target_position_np = np.asarray(target_position, dtype=float)
    target_rotation = _wpr_rotation(target_wpr)
    damping = 0.04
    epsilon = 1e-5
    converged = False
    position_error = float("inf")
    orientation_error = float("inf")

    for iteration in range(max(1, max_iterations)):
        current_position, current_rotation = _omy3m_fk(joints)
        pos_error = target_position_np - current_position
        rot_error = _orientation_error(current_rotation, target_rotation)
        position_error = float(np.linalg.norm(pos_error))
        orientation_error = float(np.linalg.norm(rot_error))
        if position_error <= position_tolerance and orientation_error <= orientation_tolerance:
            converged = True
            break

        error = np.concatenate((pos_error, rot_error * 0.45))
        jacobian = np.zeros((6, 6), dtype=float)
        for index in range(6):
            perturbed = joints.copy()
            perturbed[index] += epsilon
            next_position, next_rotation = _omy3m_fk(perturbed)
            jacobian[:3, index] = (next_position - current_position) / epsilon
            jacobian[3:, index] = (
                _orientation_error(current_rotation, next_rotation) / epsilon * 0.45
            )
        lhs = jacobian @ jacobian.T + (damping * damping) * np.eye(6)
        delta = jacobian.T @ np.linalg.solve(lhs, error)
        delta = np.clip(delta, -0.12, 0.12)
        joints = np.clip(joints + delta, -2.0 * math.pi, 2.0 * math.pi)

    return {
        "ok": converged,
        "joint_positions": joints.tolist(),
        "position_error_m": position_error,
        "orientation_error_rad": orientation_error,
        "iterations": iteration + 1,
    }


def exec_fanuc_omy3m_retarget(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    inputs = _incoming_values(node["id"], edges, context)
    data = _props(node)
    program = inputs.get("program")
    if not isinstance(program, dict):
        raise ValueError("FANUC program input is required")
    frames = program.get("frames")
    if not isinstance(frames, list) or not frames:
        raise ValueError("FANUC program has no motion frames")

    scale = float(data.get("positionScale", 0.25))
    target_origin = _float_vector(data.get("targetOrigin"), 3, [0.35, 0.0, 0.25])
    target_axis_signs = _float_vector(data.get("targetAxisSigns"), 3, [1.0, -1.0, 1.0])
    orientation_offset = _float_vector(data.get("orientationOffsetDeg"), 3, [0.0, 0.0, 0.0])
    preview_only = bool(data.get("previewOnly", True))
    solve_ik = bool(data.get("solveIk", True))
    enable_execution = bool(data.get("enableExecution", False))
    require_all_ik = bool(data.get("requireAllIk", True))
    max_iterations = int(data.get("ikMaxIterations", 180))
    position_tolerance = float(data.get("ikPositionToleranceM", 0.008))
    orientation_tolerance = math.radians(float(data.get("ikOrientationToleranceDeg", 8.0)))
    first_cartesian = next(
        (
            frame["position"]["xyz_mm"]
            for frame in frames
            if isinstance(frame.get("position"), dict)
            and frame["position"].get("kind") == "cartesian"
        ),
        None,
    )

    retargeted: List[Dict[str, Any]] = []
    joint_frames: List[Dict[str, Any]] = []
    direct_joint_count = 0
    seed = _float_vector(data.get("ikSeed"), 6, [0.0, -0.5, 0.8, 0.0, 0.4, 0.0])
    ik_failures = 0
    for frame in frames:
        position = frame.get("position") if isinstance(frame.get("position"), dict) else {}
        output = {
            "sequence": frame.get("sequence"),
            "source_line": frame.get("line"),
            "source_position_id": frame.get("position_id"),
            "motion_type": frame.get("motion_type"),
            "speed": frame.get("speed"),
            "speed_unit": frame.get("speed_unit"),
            "termination": frame.get("termination"),
        }
        if position.get("kind") == "joint":
            joints_rad = [math.radians(float(value)) for value in position.get("joints_deg", [])]
            output.update({"kind": "joint", "joint_positions": joints_rad, "ik_required": False})
            joint_frames.append(
                {
                    "time_sec": float(len(joint_frames)),
                    "joint_names": [f"joint{index}" for index in range(1, 7)],
                    "joint_positions": joints_rad,
                    "joint_target_map": {
                        f"joint{index}": value
                        for index, value in enumerate(joints_rad, start=1)
                    },
                }
            )
            direct_joint_count += 1
            seed = list(joints_rad)
        elif position.get("kind") == "cartesian" and first_cartesian:
            xyz = position.get("xyz_mm", [])
            target_position = [
                target_origin[index]
                + (float(xyz[index]) - float(first_cartesian[index])) * 0.001 * scale * target_axis_signs[index]
                for index in range(3)
            ]
            target_wpr = [
                math.radians(float(position.get("wpr_deg", [0, 0, 0])[index]) + orientation_offset[index])
                for index in range(3)
            ]
            output.update(
                {
                    "kind": "cartesian",
                    "target_position": target_position,
                    "target_wpr_rad": target_wpr,
                    "ik_required": True,
                    "uf": position.get("uf"),
                    "ut": position.get("ut"),
                }
            )
            if solve_ik:
                solution = _solve_omy3m_ik(
                    target_position,
                    target_wpr,
                    seed,
                    max_iterations=max_iterations,
                    position_tolerance=position_tolerance,
                    orientation_tolerance=orientation_tolerance,
                )
                output["ik_solution"] = solution
                output["ik_required"] = not bool(solution["ok"])
                if solution["ok"]:
                    seed = list(solution["joint_positions"])
                    joint_frames.append(
                        {
                            "time_sec": float(len(joint_frames)),
                            "joint_names": [f"joint{index}" for index in range(1, 7)],
                            "joint_positions": list(solution["joint_positions"]),
                            "joint_target_map": {
                                f"joint{index}": value
                                for index, value in enumerate(solution["joint_positions"], start=1)
                            },
                            "source_position_id": frame.get("position_id"),
                            "ik_solved": True,
                        }
                    )
                else:
                    ik_failures += 1
        else:
            output.update({"kind": "unresolved", "ik_required": True})
        retargeted.append(output)

    execution_ready = bool(joint_frames) and (not require_all_ik or ik_failures == 0)
    effective_preview_only = preview_only or not enable_execution or not execution_ready
    action = {
        "type": "fanuc_omy3m_trajectory",
        "command": "apply_omy3m_trajectory",
        "target_embodiment": "OMY3M_6DOF",
        "preview_only": effective_preview_only,
        "execution_enabled": enable_execution,
        "execution_ready": execution_ready,
        "trajectory": retargeted,
        "joint_trajectory": joint_frames,
        "requires_ik": any(frame.get("ik_required") for frame in retargeted),
        "requires_uf_ut_calibration": bool(
            program.get("summary", {}).get("requires_uf_ut_calibration")
        ),
        "target_axis_signs": target_axis_signs,
    }
    return {
        "ok": True,
        "action": action,
        "preview": retargeted,
        "summary": {
            "frame_count": len(retargeted),
            "direct_joint_frames": direct_joint_count,
            "ik_frames": sum(1 for frame in retargeted if frame.get("kind") == "cartesian"),
            "ik_failures": ik_failures,
            "executable_joint_frames": len(joint_frames),
            "execution_ready": execution_ready,
            "preview_only": effective_preview_only,
        },
    }


def exec_fanuc_omy3m_isaaclab_export(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    """Export a retargeted command trajectory as an IsaacLab demo episode.

    The observations are planned joint states derived from IK, not measured
    camera/physics rollouts. This provenance is explicit in every episode.
    """
    inputs = _incoming_values(node["id"], edges, context)
    data = _props(node)
    action = inputs.get("action")
    result = inputs.get("result")
    if not isinstance(action, dict):
        raise ValueError("Retargeted OMY3M action input is required")
    if not isinstance(result, dict) or not bool(result.get("ok")) or bool(result.get("skipped")):
        return {
            "ok": False,
            "saved": False,
            "message": "OMY3M trajectory was not accepted; dataset episode was not saved.",
            "execution_result": _json_safe(result),
        }

    trajectory = action.get("joint_trajectory")
    if not isinstance(trajectory, list) or len(trajectory) < 2:
        raise ValueError("At least two executable OMY3M joint frames are required")

    dataset_root = _dataset_root(data.get("datasetPath") or "isaaclab_datasets/omy3m_fanuc")
    episode_name = _safe_name(data.get("episodeName"), "fanuc_omy3m_demo_001")
    episode_dir = dataset_root / episode_name
    episode_dir.mkdir(parents=True, exist_ok=True)
    steps_path = episode_dir / "steps.jsonl"
    instruction = str(
        inputs.get("prompt")
        or data.get("instruction")
        or "Replay FANUC trajectory on OMY3M"
    ).strip()
    joint_names = [f"joint{index}" for index in range(1, 7)]

    records: List[Dict[str, Any]] = []
    for index in range(len(trajectory) - 1):
        current = trajectory[index]
        following = trajectory[index + 1]
        current_positions = _float_vector(current.get("joint_positions"), 6, [0.0] * 6)
        target_positions = _float_vector(following.get("joint_positions"), 6, current_positions)
        dt = max(
            1e-3,
            float(following.get("time_sec", index + 1))
            - float(current.get("time_sec", index)),
        )
        velocity = [
            (target_positions[joint] - current_positions[joint]) / dt
            for joint in range(6)
        ]
        records.append(
            {
                "schema": "vlagraph.isaaclab.omy3m.trajectory.v1",
                "episode": episode_name,
                "step": index,
                "time_sec": float(current.get("time_sec", index)),
                "dt": dt,
                "instruction": instruction,
                "observation": {
                    "joint_positions": current_positions,
                    "joint_velocities": velocity,
                },
                "action": {
                    "joint_positions": target_positions,
                    "joint_target_map": dict(zip(joint_names, target_positions)),
                },
                "reward": 1.0,
                "terminated": index == len(trajectory) - 2,
                "source": {
                    "kind": "fanuc_ls_ik_retarget",
                    "measured_rollout": False,
                    "source_position_id": following.get("source_position_id"),
                    "ik_solved": bool(following.get("ik_solved")),
                },
            }
        )

    with steps_path.open("w", encoding="utf-8") as stream:
        for record in records:
            stream.write(json.dumps(record, ensure_ascii=False) + "\n")

    metadata = {
        "schema": "vlagraph.isaaclab.omy3m.trajectory.v1",
        "episode": episode_name,
        "embodiment": "OMY3M_6DOF",
        "joint_names": joint_names,
        "step_count": len(records),
        "source_frame_count": len(trajectory),
        "provenance": "planned_fanuc_ls_ik_retarget",
        "measured_rollout": False,
        "observation_keys": ["joint_positions", "joint_velocities"],
        "action_keys": ["joint_positions"],
        "execution_result": _json_safe(result),
        "steps_file": str(steps_path),
    }
    metadata_path = episode_dir / "metadata.json"
    metadata_path.write_text(
        json.dumps(metadata, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    host_steps_path = _host_path(steps_path)
    bridge_root = str(data.get("isaacLabBridgeRoot") or r"C:\vlagraph_isaaclab_bridge")
    task_name = str(data.get("isaacLabTask") or "Isaac-OMY3M-FANUC-Imitation-v0")
    train_command = (
        'powershell -ExecutionPolicy Bypass -File '
        f'"{bridge_root}\\scripts\\train_isaaclab.ps1" '
        f'-Trace "{host_steps_path}" '
        f'-Task "{task_name}" -NumEnvs 1 -MaxIterations 200'
    )
    return {
        "ok": True,
        "saved": True,
        "dataset": {
            "path": str(steps_path),
            "host_path": host_steps_path,
            "episode_dir": str(episode_dir),
            "episode_dir_host": _host_path(episode_dir),
            "metadata_path": str(metadata_path),
            "format": "isaaclab_omy3m_trajectory_jsonl_v1",
        },
        "summary": {
            "episode": episode_name,
            "steps": len(records),
            "source_frames": len(trajectory),
            "measured_rollout": False,
        },
        "trace_path_host": host_steps_path,
        "frame_count": len(records),
        "train_command": train_command,
        "message": f"IsaacLab OMY3M trajectory episode saved: {steps_path}",
    }
