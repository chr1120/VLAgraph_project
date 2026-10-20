from __future__ import annotations

import json
import hashlib
import os
import re
import socket
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List
from urllib.parse import urlsplit, urlunsplit

import requests
from websocket import create_connection

from backend.config import (
    GR00T_FALLBACK_TO_STUB,
    ROUTE_FINETUNE,
    ROUTE_GR00T,
    ROUTE_GR00T_REAL,
    ROUTE_GR00T_STUB,
    ROUTE_VLA,
    ROUTE_VLA_STUB,
    ROUTE_VLA_REAL,
    REQUEST_TIMEOUT_SEC,
    UPLOAD_DIR,
)
from backend import state_store
from backend.executors.g1_action_schema import (
    G1_NAVIGATION_KEYS,
    G1_POLICY_GROUPS,
    g1_looks_like_action,
    g1_normalize_action_payload,
)
from backend.executors.g1_skill_nodes import decode_g1_skill_command

_AUTO_UNITY_OBJECT_VALUES = {"", "auto", "default", "robotarm"}
_LEGACY_WEBSOCKET_PORT_REWRITES = {
    8081: 9081,
    8899: 9900,
}
_OMNIVERSE_PAYLOAD_LAST_LOG_AT: Dict[str, float] = {}
_OMNIVERSE_LIVE_LAST_SENT: Dict[str, Dict[str, Any]] = {}


def _incoming_values(
    node_id: str,
    edges: List[Dict[str, Any]],
    context: Dict[str, Any],
) -> Dict[str, Any]:
    result: Dict[str, Any] = {}

    for edge in edges:
        if edge["target"] != node_id:
            continue

        source_id = edge["source"]
        source_handle = edge.get("sourceHandle")
        target_handle = edge.get("targetHandle")

        source_payload = context.get(source_id)
        if source_payload is None:
            continue

        value = source_payload
        if source_handle:
            if not isinstance(source_payload, dict):
                continue
            if source_handle not in source_payload:
                continue
            value = source_payload[source_handle]

        key = target_handle or source_handle or source_id
        result[key] = value

    return result


def _json_safe(value: Any) -> Any:
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(k): _json_safe(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_json_safe(v) for v in value]
    if isinstance(value, tuple):
        return [_json_safe(v) for v in value]
    return str(value)


def _compact_omniverse_action_payload(action: Any) -> Any:
    if not isinstance(action, dict):
        return action

    compact = dict(action)
    retarget_mode = str(compact.get("retarget_mode") or compact.get("retargetMode") or "").strip().lower()

    if isinstance(compact.get("g1_joint_trajectory"), list) and isinstance(compact.get("joint_trajectory"), list):
        compact.pop("joint_trajectory", None)
    if isinstance(compact.get("root_translation_sequence"), list) and isinstance(compact.get("rootTranslationSequence"), list):
        compact.pop("rootTranslationSequence", None)
    if "root_translation" in compact and "rootTranslation" in compact:
        compact.pop("rootTranslation", None)
    if "root_translation_mode" in compact and "rootTranslationMode" in compact:
        compact.pop("rootTranslationMode", None)
    if isinstance(compact.get("joint_targets"), list) and isinstance(compact.get("joint_positions"), list):
        compact.pop("joint_positions", None)
    if isinstance(compact.get("joint_target_map"), dict) and isinstance(compact.get("named_joint_targets"), dict):
        compact.pop("named_joint_targets", None)

    if retarget_mode in {"joint_targets", "streamed_joint_targets"}:
        named_targets = None
        for key in ("g1_named_targets", "g1NamedTargets", "joint_target_map", "jointTargetMap", "named_joint_targets", "namedJointTargets"):
            value = compact.get(key)
            if isinstance(value, dict) and value:
                named_targets = {str(name): value[name] for name in value}
                break
        if named_targets:
            compact["g1_named_targets"] = dict(named_targets)
            compact["joint_target_map"] = dict(named_targets)
            for key in (
                "joint_targets",
                "jointTargets",
                "joint_positions",
                "jointPositions",
                "joint_names",
                "jointNames",
                "joint_order",
                "jointOrder",
                "joint_order_source",
                "jointOrderSource",
                "joint_schema",
                "jointSchema",
                "named_joint_targets",
                "namedJointTargets",
                "joint_groups",
                "jointGroups",
            ):
                compact.pop(key, None)
            for group_name in (
                "waist",
                "left_leg",
                "right_leg",
                "left_arm",
                "right_arm",
                "left_hand",
                "right_hand",
            ):
                compact.pop(group_name, None)

    return compact


def _maybe_log_omniverse_payload(request_id: str, command: str, action: Any, request_payload: Dict[str, Any]) -> None:
    if not isinstance(action, dict):
        return
    retarget_mode = str(action.get("retarget_mode") or action.get("retargetMode") or "").strip().lower()
    if retarget_mode not in {"joint_targets", "streamed_joint_targets"}:
        return

    interval_sec = max(0.0, float(os.getenv("VLAGRAPH_OMNI_PAYLOAD_LOG_INTERVAL_SEC", "1.0") or 1.0))
    if interval_sec <= 0.0:
        return
    now = time.monotonic()
    key = f"{command}:{retarget_mode}"
    last = _OMNIVERSE_PAYLOAD_LAST_LOG_AT.get(key, 0.0)
    if now - last < interval_sec:
        return
    _OMNIVERSE_PAYLOAD_LAST_LOG_AT[key] = now

    try:
        payload_bytes = len(json.dumps(_json_safe(request_payload), ensure_ascii=False).encode("utf-8"))
    except Exception:
        payload_bytes = -1
    named_targets = action.get("g1_named_targets") or action.get("joint_target_map") or {}
    named_count = len(named_targets) if isinstance(named_targets, dict) else 0
    keys = sorted(str(key) for key in action.keys())
    print(
        "[OMNI_PAYLOAD] "
        f"request_id={request_id} command={command} mode={retarget_mode} "
        f"bytes={payload_bytes} action_keys={keys[:18]} "
        f"key_count={len(keys)} named_targets={named_count} "
        f"has_joint_targets={isinstance(action.get('joint_targets'), list)} "
        f"has_root={bool(action.get('root_translation') or action.get('rootTranslation'))}"
    )


def _live_joint_target_map(action: Any) -> Dict[str, float]:
    if not isinstance(action, dict):
        return {}
    for key in ("g1_named_targets", "g1NamedTargets", "joint_target_map", "jointTargetMap", "named_joint_targets", "namedJointTargets"):
        value = action.get(key)
        if isinstance(value, dict) and value:
            result: Dict[str, float] = {}
            for name, raw in value.items():
                try:
                    result[str(name)] = float(raw)
                except (TypeError, ValueError):
                    continue
            return result
    names = action.get("joint_names") or action.get("jointNames")
    targets = action.get("joint_targets") or action.get("jointTargets")
    if isinstance(names, list) and isinstance(targets, list):
        result = {}
        for name, raw in zip(names, targets):
            try:
                result[str(name)] = float(raw)
            except (TypeError, ValueError):
                continue
        return result
    return {}


def _maybe_skip_live_omniverse_payload(
    endpoint: str,
    prim_path: str,
    command: str,
    action: Any,
) -> Dict[str, Any] | None:
    if not isinstance(action, dict):
        return None
    retarget_mode = str(action.get("retarget_mode") or action.get("retargetMode") or "").strip().lower()
    if retarget_mode not in {"joint_targets", "streamed_joint_targets"}:
        return None

    enabled = _coerce_bool(os.getenv("VLAGRAPH_SKIP_UNCHANGED_LIVE_JOINTS"), False)
    if not enabled:
        return None
    threshold = max(0.0, float(os.getenv("VLAGRAPH_LIVE_JOINT_SEND_EPS_RAD", "0.004") or 0.004))
    keepalive_sec = max(0.02, float(os.getenv("VLAGRAPH_LIVE_JOINT_KEEPALIVE_SEC", "0.10") or 0.10))
    current = _live_joint_target_map(action)
    if not current:
        return None

    now = time.monotonic()
    cache_key = f"{endpoint}|{prim_path}|{command}|{retarget_mode}"
    previous = _OMNIVERSE_LIVE_LAST_SENT.get(cache_key)
    if not previous:
        _OMNIVERSE_LIVE_LAST_SENT[cache_key] = {"targets": current, "sent_at": now, "skipped": 0}
        return None

    previous_targets = previous.get("targets") if isinstance(previous.get("targets"), dict) else {}
    all_names = set(current) | set(previous_targets)
    max_delta = 0.0
    changed_count = 0
    for name in all_names:
        delta = abs(float(current.get(name, 0.0)) - float(previous_targets.get(name, 0.0)))
        max_delta = max(max_delta, delta)
        if delta >= threshold:
            changed_count += 1
    age = now - float(previous.get("sent_at", 0.0) or 0.0)
    if changed_count <= 0 and age < keepalive_sec:
        previous["skipped"] = int(previous.get("skipped", 0) or 0) + 1
        return {
            "ok": True,
            "mode": "omniverse",
            "executor": "robot-executor",
            "endpoint": endpoint,
            "command": command,
            "sent": False,
            "skipped": True,
            "skip_reason": "live_joint_targets_below_delta_threshold",
            "max_delta_rad": round(max_delta, 6),
            "threshold_rad": round(threshold, 6),
            "skipped_since_last_send": int(previous.get("skipped", 0) or 0),
            "message": "Skipped live joint target frame because it changed below the send threshold.",
        }

    _OMNIVERSE_LIVE_LAST_SENT[cache_key] = {"targets": current, "sent_at": now, "skipped": 0}
    if int(previous.get("skipped", 0) or 0) > 0:
        print(
            "[OMNI_PAYLOAD] live send resumed "
            f"mode={retarget_mode} skipped={int(previous.get('skipped', 0) or 0)} "
            f"changed={changed_count} max_delta={round(max_delta, 6)}"
        )
    return None


def _coerce_bool(value: Any, default: bool = False) -> bool:
    if value is None:
        return default
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return bool(value)


def _compact_debug_json(value: Any, limit: int = 2000) -> str:
    try:
        text = json.dumps(_json_safe(value), ensure_ascii=False)
    except Exception:
        text = str(value)

    compact = " ".join(str(text).split())
    if len(compact) > limit:
        return compact[: limit - 3] + "..."
    return compact


def _normalize_text(value: Any) -> str:
    return str(value or "").strip()


def _state_payload(record: Any) -> Dict[str, Any]:
    if not isinstance(record, dict):
        return {}

    state = record.get("state")
    if isinstance(state, dict):
        return state

    return record


def _state_object_name(record: Any) -> str:
    state = _state_payload(record)
    return _normalize_text(state.get("object_name") or state.get("target"))


def _resolve_unity_target_object(
    configured_object: Any,
    state_record: Any,
) -> tuple[str, str, bool]:
    configured = _normalize_text(configured_object)
    state_object = _state_object_name(state_record)

    if state_object and configured.lower() in _AUTO_UNITY_OBJECT_VALUES:
        return state_object, state_object, configured != state_object

    return configured or "RobotKyle", state_object, False


def _state_vec3(record: Any, key: str) -> List[float] | None:
    state = _state_payload(record)
    raw = state.get(key)

    if raw is None and key == "position":
        eef_pose = state.get("eef_pose")
        if isinstance(eef_pose, (list, tuple)) and len(eef_pose) >= 3:
            raw = list(eef_pose[:3])
    elif raw is None and key == "rotation":
        eef_pose = state.get("eef_pose")
        if isinstance(eef_pose, (list, tuple)) and len(eef_pose) >= 6:
            raw = list(eef_pose[3:6])

    if raw is None:
        return None

    return _normalize_vec3(raw)


def _sum_vec3(a: List[float], b: List[float]) -> List[float]:
    return [
        float(a[0]) + float(b[0]),
        float(a[1]) + float(b[1]),
        float(a[2]) + float(b[2]),
    ]


def _extract_action_payload(action: Any) -> Any:
    if isinstance(action, dict):
        if "safe_action" in action:
            return _sanitize_g1_skill_payload(action["safe_action"])
        if "decoded" in action and isinstance(action["decoded"], dict):
            inner = action["decoded"]
            if "action" in inner:
                return _sanitize_g1_skill_payload(inner["action"])
            return _sanitize_g1_skill_payload(inner)
        if "action" in action:
            return _sanitize_g1_skill_payload(action["action"])
        if "raw_action" in action and not _is_g1_skill_command(action):
            return action["raw_action"]
        return _sanitize_g1_skill_payload(action)
    return action


def _is_g1_skill_command(action: Any) -> bool:
    if not isinstance(action, dict):
        return False
    action_type = str(action.get("type", "")).strip().lower()
    command = str(action.get("command", "")).strip().lower()
    return action_type == "g1_skill_command" or command in {
        "apply_g1_skill",
        "apply_skill",
        "apply_skill_command",
    }


def _sanitize_g1_skill_payload(action: Any) -> Any:
    if not _is_g1_skill_command(action):
        return action

    sanitized = dict(action)
    for key in ("raw_action", "rawAction"):
        sanitized.pop(key, None)

    for key in G1_POLICY_GROUPS:
        sanitized.pop(key, None)
        sanitized.pop(f"action.{key}", None)

    for key in G1_NAVIGATION_KEYS:
        sanitized.pop(key, None)
        sanitized.pop(f"action.{key}", None)

    return sanitized


def _resolve_vla_route(model_name: str) -> str:
    """
    우선순위:
    1) model_name 이 명시적으로 openvla-real 이면 REAL
    2) model_name 이 명시적으로 openvla-stub 이면 STUB
    3) 그 외에는 config 의 ROUTE_VLA 사용
    """
    name = str(model_name or "").strip().lower()

    if name == "openvla-real":
        return ROUTE_VLA_REAL
    if name == "openvla-stub":
        return ROUTE_VLA_STUB

    return ROUTE_VLA


def _resolve_gr00t_route(model_name: str, endpoint: Any = "") -> str:
    endpoint_text = str(endpoint or "").strip()
    if endpoint_text:
        return endpoint_text.rstrip("/")

    name = str(model_name or "").strip().lower()
    if name == "gr00t-real":
        return ROUTE_GR00T_REAL
    if name == "gr00t-stub":
        return ROUTE_GR00T_STUB

    return ROUTE_GR00T


def _should_fallback_gr00t_to_stub(route: str) -> bool:
    if not GR00T_FALLBACK_TO_STUB:
        return False

    normalized_route = str(route or "").strip().rstrip("/")
    stub_route = str(ROUTE_GR00T_STUB or "").strip().rstrip("/")
    return bool(normalized_route and stub_route and normalized_route != stub_route)


def _route_looks_like_docker_service(route: str) -> bool:
    hostname = (urlsplit(str(route or "")).hostname or "").strip().lower()
    if not hostname:
        return False
    if hostname in {"localhost", "127.0.0.1", "0.0.0.0", "host.docker.internal"}:
        return False
    return True


def _rewrite_uploaded_image_for_service(image_value: Any, route: str) -> Any:
    if not isinstance(image_value, str):
        return image_value

    image_ref = image_value.strip()
    if not image_ref:
        return image_value

    service_base = "http://backend:9000" if _route_looks_like_docker_service(route) else "http://localhost:9000"
    upload_dir = str(UPLOAD_DIR or "").replace("\\", "/").rstrip("/")
    normalized_ref = image_ref.replace("\\", "/")

    if upload_dir and normalized_ref.startswith(f"{upload_dir}/"):
        return f"{service_base}/uploads/{Path(normalized_ref).name}"

    if normalized_ref.startswith("/uploads/"):
        return f"{service_base}{normalized_ref}"

    parsed = urlsplit(image_ref)
    hostname = parsed.hostname or ""
    if parsed.scheme in {"http", "https"} and parsed.path.startswith("/uploads/"):
        if _route_looks_like_docker_service(route) and hostname in {"localhost", "127.0.0.1", "host.docker.internal"}:
            return urlunsplit(("http", "backend:9000", parsed.path, parsed.query, parsed.fragment))

    return image_value


def _rewrite_image_for_real_service(image_value: Any, route: str) -> Any:
    if route != ROUTE_VLA_REAL or not isinstance(image_value, str):
        return image_value

    image_ref = image_value.strip()
    if not image_ref:
        return image_value

    upload_dir = str(UPLOAD_DIR or "").replace("\\", "/").rstrip("/")
    if upload_dir and image_ref.startswith(f"{upload_dir}/"):
        return f"http://backend:9000/uploads/{Path(image_ref).name}"

    if image_ref.startswith("/uploads/"):
        return f"http://backend:9000{image_ref}"

    parsed = urlsplit(image_ref)
    hostname = parsed.hostname or ""
    if parsed.scheme in {"http", "https"} and parsed.path.startswith("/uploads/"):
        if hostname in {"localhost", "127.0.0.1", "host.docker.internal"}:
            port = parsed.port or 9000
            return urlunsplit((parsed.scheme, f"backend:{port}", parsed.path, parsed.query, parsed.fragment))

    return image_value


def _real_image_input_error(image_value: Any, route: str) -> str:
    if route != ROUTE_VLA_REAL:
        return ""

    if not isinstance(image_value, str):
        return "openvla-real image must be a string URL, upload path, data URI, base64 payload, or RGB array."

    image_ref = image_value.strip()
    if not image_ref:
        return "openvla-real requires an image. Upload an image or connect a camera/remote image sensor."

    parsed = urlsplit(image_ref)
    if parsed.scheme in {"http", "https"}:
        return ""
    if image_ref.startswith(("data:", "base64:")):
        return ""

    normalized_upload_dir = str(UPLOAD_DIR or "").replace("\\", "/").rstrip("/")
    normalized_ref = image_ref.replace("\\", "/")
    if normalized_upload_dir and normalized_ref.startswith(f"{normalized_upload_dir}/"):
        return ""
    if normalized_ref.startswith("/uploads/"):
        return ""

    if normalized_ref.startswith("/"):
        return (
            f"Image path is not reachable from openvla-real: {image_ref}. "
            "Use the Image Input upload button so the value becomes http://localhost:9000/uploads/<file>."
        )

    return ""


def _response_error_detail(response: requests.Response | None) -> str:
    if response is None:
        return ""

    detail: Any = ""
    try:
        body = response.json()
        if isinstance(body, dict):
            detail = body.get("detail", body)
        else:
            detail = body
    except Exception:
        detail = response.text

    if isinstance(detail, (dict, list)):
        detail = json.dumps(detail, ensure_ascii=False)

    return str(detail or "").strip()[:1000]


LOCOMOTION_UNITY_COMMANDS = {
    "apply_humanoid_step",
    "apply_base_waypoint",
    "apply_base_velocity",
    "apply_patrol_control",
}
MANIPULATION_PROMPT_MARKERS = (
    "pick up",
    "pickup",
    "pick-up",
    "grab",
    "grasp",
    "lift",
    "hold",
    "take",
    "place",
    "put",
    "stack",
)

_NUMBER_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"


def _extract_unity_world_coordinate(prompt: Any) -> List[float] | None:
    text = str(prompt or "")
    if not text.strip():
        return None

    xyz_pattern = re.compile(
        rf"\bx\s*[:=]\s*({_NUMBER_PATTERN})\D+"
        rf"\by\s*[:=]\s*({_NUMBER_PATTERN})\D+"
        rf"\bz\s*[:=]\s*({_NUMBER_PATTERN})",
        re.IGNORECASE,
    )
    xyz_match = xyz_pattern.search(text)
    if xyz_match:
        return [float(xyz_match.group(1)), float(xyz_match.group(2)), float(xyz_match.group(3))]

    bracket_pattern = re.compile(
        rf"[\[\(]\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})\s*[\]\)]"
    )
    bracket_match = bracket_pattern.search(text)
    if bracket_match:
        return [
            float(bracket_match.group(1)),
            float(bracket_match.group(2)),
            float(bracket_match.group(3)),
        ]

    return None


def _apply_prompt_waypoint_override(
    action: Any,
    *,
    prompt: Any,
    unity_command: str,
    waypoint_speed: Any,
) -> Any:
    if str(unity_command or "").strip().lower() != "apply_base_waypoint":
        return action

    target_position = _extract_unity_world_coordinate(prompt)
    if target_position is None:
        return action

    payload = dict(action) if isinstance(action, dict) else {}
    payload["type"] = "unity_waypoint_action"
    payload["command"] = "apply_base_waypoint"
    payload["target_position"] = target_position
    payload.setdefault("target_rotation", [0.0, 0.0, 0.0])
    payload.setdefault("translation", [0.0, 0.0, 0.0])
    payload.setdefault("rotation", [0.0, 0.0, 0.0])
    payload.setdefault("gripper", "noop")
    payload.setdefault("terminate", False)
    payload.setdefault("stop", False)
    if waypoint_speed is not None and "speed" not in payload:
        payload["speed"] = waypoint_speed
    payload["waypoint_source"] = "prompt_coordinate"
    return payload


def _apply_prompt_patrol_override(
    action: Any,
    *,
    prompt: Any,
    unity_command: str,
    waypoint_speed: Any,
) -> Any:
    if str(unity_command or "").strip().lower() != "apply_patrol_control":
        return action

    prompt_text = str(prompt or "").strip().lower()
    payload = dict(action) if isinstance(action, dict) else {}
    payload["type"] = "unity_patrol_action"
    payload["command"] = "apply_patrol_control"

    if any(marker in prompt_text for marker in ("disable patrol", "stop patrol", "patrol off", "순찰 끄기", "순찰 중지")):
        payload["patrol_mode"] = "disable"
        return payload

    if any(marker in prompt_text for marker in ("internal patrol", "auto patrol", "자동 순찰", "내부 순찰")):
        payload["patrol_mode"] = "internal"
        return payload

    target_position = _extract_unity_world_coordinate(prompt)
    if target_position is not None:
        payload["patrol_mode"] = "target"
        payload["patrol_target_position"] = target_position
        if waypoint_speed is not None and "patrol_speed" not in payload:
            payload["patrol_speed"] = waypoint_speed
        payload["patrol_source"] = "prompt_coordinate"
        return payload

    has_direction_hint = any(
        key in payload
        for key in ("patrol_direction", "patrolDirection", "direction", "linear", "translation", "delta_position", "deltaPosition")
    )
    if not has_direction_hint:
        payload.setdefault("patrol_mode", "enable")
    if waypoint_speed is not None and "patrol_speed" not in payload:
        payload["patrol_speed"] = waypoint_speed
    return payload


def _looks_like_manipulation_prompt(text: Any) -> bool:
    normalized = str(text or "").strip().lower()
    if not normalized:
        return False
    return any(marker in normalized for marker in MANIPULATION_PROMPT_MARKERS)


def _analyze_task_context(
    prompt: Any,
    unity_command: str,
    robot_state: Any,
) -> Dict[str, Any]:
    robot_type = ""
    if isinstance(robot_state, dict):
        robot_type = str(robot_state.get("robot_type") or "").strip()

    normalized_command = str(unity_command or "").strip().lower()
    prompt_text = str(prompt or "").strip()
    task_type = "manipulation" if _looks_like_manipulation_prompt(prompt_text) else "other"
    command_family = "locomotion" if normalized_command in LOCOMOTION_UNITY_COMMANDS else "other"
    warning = ""

    if task_type == "manipulation" and command_family == "locomotion":
        warning = (
            f"Prompt suggests manipulation, but unity_command={normalized_command or '<empty>'} "
            "is locomotion-oriented. This graph can move the robot but cannot complete grasp/pickup "
            "without a manipulation controller."
        )

    return {
        "task_type": task_type,
        "command_family": command_family,
        "unity_command": normalized_command,
        "robot_type": robot_type,
        "warning": warning,
    }


def _round_float(value: Any, digits: int = 3) -> float | None:
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


def _has_image_value(value: Any) -> bool:
    return isinstance(value, str) and bool(value.strip())


def _extract_image_value(value: Any) -> Any:
    if _has_image_value(value):
        return value

    if not isinstance(value, dict):
        return ""

    for key in ("image", "image_url", "data_url", "frame", "snapshot"):
        candidate = value.get(key)
        if _has_image_value(candidate):
            return candidate

    sensor = value.get("sensor")
    if isinstance(sensor, dict):
        for key in ("image", "image_url", "data_url", "frame", "snapshot"):
            candidate = sensor.get(key)
            if _has_image_value(candidate):
                return candidate

    return ""


def _resolve_openvla_image_input(
    *,
    node_id: str,
    edges: List[Dict[str, Any]],
    context: Dict[str, Any],
    inputs: Dict[str, Any],
) -> Any:
    image = _extract_image_value(inputs.get("image"))
    if _has_image_value(image):
        return image

    for key in ("sensor", "remote_image", "camera", "fused"):
        image = _extract_image_value(inputs.get(key))
        if _has_image_value(image):
            return image

    # If the user connected a Sensor node to the new sensor/lidar pins, the
    # handle value may only contain metadata. Look back at the full upstream
    # node payload and recover its top-level image output when available.
    for edge in edges:
        if edge.get("target") != node_id:
            continue
        source_id = edge.get("source")
        if not source_id:
            continue
        image = _extract_image_value(context.get(source_id))
        if _has_image_value(image):
            return image

    return inputs.get("image", "")


def _compact_lidar_context(lidar: Any, sensor: Any = None) -> Dict[str, Any]:
    """Build a small LiDAR summary safe to pass into VLA prompts/state."""
    if not isinstance(lidar, dict) and isinstance(sensor, dict):
        lidar = sensor.get("lidar") if isinstance(sensor.get("lidar"), dict) else {}

    if not isinstance(lidar, dict):
        lidar = {}

    if lidar.get("format") == "error" or lidar.get("ok") is False:
        return {}
    if lidar.get("error") and int(lidar.get("point_count", 0) or 0) <= 0:
        return {}

    sensor_summary = sensor.get("summary", {}) if isinstance(sensor, dict) else {}
    if isinstance(sensor_summary, dict) and sensor_summary.get("lidar_ready") is False:
        return {}
    lidar_summary = lidar.get("summary", {}) if isinstance(lidar.get("summary"), dict) else {}
    scan = lidar.get("scan", {}) if isinstance(lidar.get("scan"), dict) else {}

    nearest_range = (
        lidar_summary.get("nearest_range")
        if lidar_summary.get("nearest_range") is not None
        else lidar_summary.get("nearest_distance")
    )
    if nearest_range is None and isinstance(sensor_summary, dict):
        nearest_range = sensor_summary.get("lidar_nearest_range")

    sectors = lidar_summary.get("sectors") if isinstance(lidar_summary.get("sectors"), dict) else {}
    compact_sectors: Dict[str, Any] = {}
    for name in ("front", "left", "right", "rear"):
        sector = sectors.get(name)
        if not isinstance(sector, dict):
            continue
        compact_sectors[name] = {
            "min_range": _round_float(sector.get("min_range")),
            "count": int(sector.get("count", 0) or 0),
        }

    point_count = lidar.get("point_count")
    if point_count is None and isinstance(sensor_summary, dict):
        point_count = sensor_summary.get("lidar_point_count")

    context = {
        "format": str(lidar.get("format") or ""),
        "frame_id": str(lidar.get("frame_id") or ""),
        "point_count": int(point_count or 0),
        "nearest_range": _round_float(nearest_range),
        "nearest_angle": _round_float(lidar_summary.get("nearest_angle")),
        "obstacle_count": int(lidar_summary.get("obstacle_count", 0) or 0),
        "obstacle_distance": _round_float(lidar_summary.get("obstacle_distance")),
        "sectors": compact_sectors,
        "range_max": _round_float(scan.get("range_max")),
        "timestamp": lidar.get("timestamp"),
    }

    return {key: value for key, value in context.items() if value not in (None, "", {}, [])}


def _format_lidar_context_text(lidar_context: Dict[str, Any]) -> str:
    if not isinstance(lidar_context, dict) or not lidar_context:
        return ""

    parts: List[str] = []
    nearest = lidar_context.get("nearest_range")
    if isinstance(nearest, (int, float)):
        parts.append(f"nearest_obstacle={float(nearest):.2f}m")
    obstacle_count = lidar_context.get("obstacle_count")
    if isinstance(obstacle_count, int):
        parts.append(f"near_obstacle_rays={obstacle_count}")
    point_count = lidar_context.get("point_count")
    if isinstance(point_count, int):
        parts.append(f"points={point_count}")

    sectors = lidar_context.get("sectors")
    if isinstance(sectors, dict):
        sector_parts: List[str] = []
        for name in ("front", "left", "right", "rear"):
            sector = sectors.get(name)
            if not isinstance(sector, dict):
                continue
            min_range = sector.get("min_range")
            if isinstance(min_range, (int, float)):
                sector_parts.append(f"{name}={float(min_range):.2f}m")
        if sector_parts:
            parts.append("sector_min_ranges(" + ", ".join(sector_parts) + ")")

    if not parts:
        return ""
    return "LiDAR context: " + "; ".join(parts) + "."


def _merge_lidar_into_state(state: Any, lidar_context: Dict[str, Any]) -> Any:
    if not lidar_context:
        return state

    merged = dict(state) if isinstance(state, dict) else {"raw_state": state}
    merged["lidar"] = lidar_context
    merged["lidar_summary"] = _format_lidar_context_text(lidar_context)
    return merged


def _suppress_gripper_for_locomotion(
    action: Any,
    task_analysis: Dict[str, Any],
) -> Any:
    if not isinstance(action, dict):
        return action

    if str(task_analysis.get("command_family", "")).strip().lower() != "locomotion":
        return action

    if str(action.get("gripper", "noop")).strip().lower() == "noop":
        return action

    suppressed = dict(action)
    suppressed["raw_gripper"] = action.get("gripper", "noop")
    if "gripper_value" in action:
        suppressed["raw_gripper_value"] = action.get("gripper_value")
    suppressed["gripper"] = "noop"
    suppressed["gripper_suppressed"] = True
    return suppressed


def exec_openvla_infer(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    prompt = inputs.get("prompt", "")
    image = _resolve_openvla_image_input(
        node_id=node["id"],
        edges=edges,
        context=context,
        inputs=inputs,
    )
    state = inputs.get("state", {})
    lidar = inputs.get("lidar", {})
    sensor = inputs.get("sensor", {})
    lidar_context = _compact_lidar_context(lidar, sensor)
    state_for_vla = _merge_lidar_into_state(state, lidar_context)
    lidar_prompt_context = _format_lidar_context_text(lidar_context)
    prompt_for_vla = (
        f"{prompt}\n\n{lidar_prompt_context}"
        if lidar_prompt_context and lidar_prompt_context not in str(prompt)
        else prompt
    )
    model_name = data.get("modelName", "openvla-stub")
    route = _resolve_vla_route(model_name)
    image = _rewrite_image_for_real_service(image, route)
    image_input_error = _real_image_input_error(image, route)

    if image_input_error:
        return {
            "ok": False,
            "error": image_input_error,
            "error_detail": image_input_error,
            "request": {
                "prompt": prompt_for_vla,
                "image": image,
                "state": state_for_vla,
                "lidar": lidar_context,
                "model_name": model_name,
            },
            "action": {
                "type": "error",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "gripper": "noop",
                "terminate": False,
                "error": image_input_error,
            },
            "model": model_name,
            "route": route,
        }

    payload = {
        "prompt": prompt_for_vla,
        "image": image,
        "state": state_for_vla,
        "model_name": model_name,
    }
    if lidar_context:
        payload["lidar"] = lidar_context
        payload["sensor_context"] = {
            "lidar": lidar_context,
        }
    action_format = str(data.get("actionFormat", "")).strip()
    if action_format:
        payload["action_format"] = action_format

    unity_command = str(data.get("unityCommand", "")).strip()
    if unity_command:
        payload["unity_command"] = unity_command

    waypoint_speed = data.get("waypointSpeed")
    if waypoint_speed is not None:
        payload["waypoint_speed"] = waypoint_speed

    task_analysis = _analyze_task_context(prompt, unity_command, state_for_vla)
    if lidar_context:
        task_analysis["lidar_used"] = True
        task_analysis["lidar_context"] = lidar_context
    if task_analysis.get("warning"):
        print(f"[openvla_infer] warning={task_analysis['warning']}")

    print(f"[openvla_infer] request={_compact_debug_json(payload)}")

    try:
        resp = requests.post(
            f"{route}/infer",
            json=payload,
            timeout=REQUEST_TIMEOUT_SEC,
        )
        resp.raise_for_status()
        body = resp.json()
    except requests.RequestException as e:
        response = getattr(e, "response", None)
        detail = _response_error_detail(response)
        error_message = f"{model_name} request failed: {str(e)}"
        if detail:
            error_message = f"{error_message} - detail: {detail}"
        return {
            "ok": False,
            "error": error_message,
            "error_detail": detail,
            "request": payload,
            "action": {
                "type": "error",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "gripper": "noop",
                "terminate": False,
                "error": error_message,
            },
            "model": model_name,
            "route": route,
        }

    action = body.get("action", {})
    action = _suppress_gripper_for_locomotion(action, task_analysis)
    action = _apply_prompt_waypoint_override(
        action,
        prompt=prompt,
        unity_command=unity_command,
        waypoint_speed=waypoint_speed,
    )
    action = _apply_prompt_patrol_override(
        action,
        prompt=prompt,
        unity_command=unity_command,
        waypoint_speed=waypoint_speed,
    )
    if isinstance(action, dict) and waypoint_speed is not None and "waypoint_speed" not in action:
        action = dict(action)
        action["waypoint_speed"] = waypoint_speed
    action = _attach_prompt_metadata(
        action,
        prompt=prompt,
        model_prompt=prompt_for_vla,
    )
    return {
        "action": action,
        "model": body.get("model", model_name),
        "route": route,
        "warning": task_analysis.get("warning", ""),
        "task_analysis": task_analysis,
        "lidar_context": lidar_context,
        "request": payload,
        "raw": body,
    }


def _coerce_positive_int(value: Any, default: int) -> int:
    try:
        parsed = int(value)
    except Exception:
        return default
    return parsed if parsed > 0 else default


def _float_vector(values: Any) -> List[float]:
    if not isinstance(values, (list, tuple)):
        return []

    out: List[float] = []
    for value in list(values):
        try:
            out.append(float(value))
        except Exception:
            out.append(0.0)
    return out


def _first_action_item(value: Any) -> Any:
    if isinstance(value, list):
        if not value:
            return []
        if all(isinstance(item, (int, float)) for item in value):
            return value
        return value[0]
    return value


GR00T_G1_ACTION_KEYS = {
    *G1_POLICY_GROUPS,
    *G1_NAVIGATION_KEYS,
    "joint_targets",
    "jointTargets",
    "joint_trajectory",
    "jointTrajectory",
    "g1_joint_trajectory",
    "g1JointTrajectory",
    "joint_positions",
    "jointPositions",
    "joint_names",
    "jointNames",
    "joint_groups",
    "jointGroups",
    "joint_schema",
    "jointSchema",
    "joint_target_map",
    "jointTargetMap",
    "named_joint_targets",
    "namedJointTargets",
    "g1_named_targets",
    "g1NamedTargets",
    "joint_order_source",
    "jointOrderSource",
    "retarget_source",
    "retargetSource",
    "retarget_mode",
    "retargetMode",
    "left_wrist_eef_9d",
    "right_wrist_eef_9d",
    "action.left_wrist_eef_9d",
    "action.right_wrist_eef_9d",
}
GR00T_G1_ACTION_KEYS.update(f"action.{key}" for key in G1_POLICY_GROUPS)
GR00T_G1_ACTION_KEYS.update(f"action.{key}" for key in G1_NAVIGATION_KEYS)


def _extract_gr00t_response_action(body: Dict[str, Any]) -> Any:
    if not isinstance(body, dict):
        return body

    action_like_keys = {
        "translation",
        "rotation",
        "delta_position",
        "delta_rotation",
        "joint_deltas",
        "move_x",
        "move_z",
        "turn_yaw",
        "linear_x",
        "linear_z",
        "angular_yaw",
    } | GR00T_G1_ACTION_KEYS
    if any(key in body for key in action_like_keys):
        return body

    for key in (
        "action",
        "action_payload",
        "predicted_action",
        "raw_action",
        "actions",
        "action_chunk",
        "action_chunks",
        "prediction",
    ):
        if key not in body:
            continue
        value = _first_action_item(body.get(key))
        if isinstance(value, dict) and not any(action_key in value for action_key in action_like_keys):
            nested = _extract_gr00t_response_action(value)
            if nested is not value:
                return nested
        return value

    return {}


def _gr00t_vector_to_action_payload(
    values: Any,
    *,
    unity_command: str,
    embodiment_tag: str,
) -> Dict[str, Any]:
    vector = _float_vector(values)
    padded = list(vector[:7])
    while len(padded) < 7:
        padded.append(0.0)

    command = str(unity_command or "").strip().lower()
    if command == "apply_humanoid_step" or "humanoid" in str(embodiment_tag or "").lower():
        return {
            "type": "unity_humanoid_action",
            "command": "apply_humanoid_step" if command == "apply_humanoid_step" else unity_command or "apply_humanoid_step",
            "move_x": float(padded[0]),
            "move_z": float(padded[1] if len(vector) < 3 else padded[2]),
            "turn_yaw": float(padded[3] if len(vector) > 3 else 0.0),
            "speed": 1.0,
            "stop": False,
            "raw_vector": vector,
        }

    gripper_value = float(padded[6])
    gripper = "open" if gripper_value >= 0.1 else "close" if gripper_value <= -0.1 else "noop"
    return {
        "type": "delta_action",
        "translation": [float(v) for v in padded[:3]],
        "rotation": [float(v) for v in padded[3:6]],
        "gripper": gripper,
        "gripper_value": gripper_value,
        "terminate": False,
        "raw_vector": vector,
    }


def _normalize_gr00t_action(
    action: Any,
    *,
    unity_command: str,
    embodiment_tag: str,
) -> Dict[str, Any]:
    if isinstance(action, dict):
        normalized = _json_safe(dict(action))
        if "translation" not in normalized and "delta_position" in normalized:
            normalized["translation"] = normalized.get("delta_position")
        if "rotation" not in normalized and "delta_rotation" in normalized:
            normalized["rotation"] = normalized.get("delta_rotation")
        if "joint_targets" not in normalized:
            joint_targets = (
                normalized.get("joint_positions")
                or normalized.get("jointPositions")
                or normalized.get("positions")
            )
            if joint_targets is not None:
                normalized["joint_targets"] = joint_targets
        if unity_command and "command" not in normalized:
            normalized["command"] = unity_command
        normalized.setdefault("type", "gr00t_action")
        return g1_normalize_action_payload(normalized)

    if isinstance(action, (list, tuple)):
        return g1_normalize_action_payload(_gr00t_vector_to_action_payload(
            action,
            unity_command=unity_command,
            embodiment_tag=embodiment_tag,
        ))

    return g1_normalize_action_payload({
        "type": "gr00t_action",
        "command": unity_command or "apply_action",
        "raw": _json_safe(action),
        "translation": [0.0, 0.0, 0.0],
        "rotation": [0.0, 0.0, 0.0],
        "gripper": "noop",
        "terminate": False,
    })


def exec_gr00t_infer(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    prompt = inputs.get("prompt", "")
    image = _resolve_openvla_image_input(
        node_id=node["id"],
        edges=edges,
        context=context,
        inputs=inputs,
    )
    state = inputs.get("state", {})
    lidar = inputs.get("lidar", {})
    sensor = inputs.get("sensor", {})
    lidar_context = _compact_lidar_context(lidar, sensor)
    state_for_gr00t = _merge_lidar_into_state(state, lidar_context)
    lidar_prompt_context = _format_lidar_context_text(lidar_context)
    prompt_for_gr00t = (
        f"{prompt}\n\n{lidar_prompt_context}"
        if lidar_prompt_context and lidar_prompt_context not in str(prompt)
        else prompt
    )

    model_name = str(data.get("modelName", "nvidia/GR00T-N1.7-3B")).strip() or "nvidia/GR00T-N1.7-3B"
    route = _resolve_gr00t_route(model_name, data.get("endpoint", "")).rstrip("/")
    image = _rewrite_uploaded_image_for_service(image, route)
    embodiment_tag = str(data.get("embodimentTag", "REAL_G1")).strip() or "REAL_G1"
    inference_mode = str(data.get("inferenceMode", "pytorch")).strip() or "pytorch"
    action_horizon = _coerce_positive_int(data.get("actionHorizon", 40), 40)

    require_image = bool(data.get("requireImage", False))
    if require_image and not _has_image_value(image):
        error_message = "GR00T inference requires an image input. Connect Image Input or a camera/remote image sensor."
        return {
            "ok": False,
            "error": error_message,
            "error_detail": error_message,
            "request": {
                "prompt": prompt_for_gr00t,
                "image": image,
                "state": state_for_gr00t,
                "model_name": model_name,
                "embodiment_tag": embodiment_tag,
            },
            "action": {
                "type": "error",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "gripper": "noop",
                "terminate": False,
                "error": error_message,
            },
            "model": model_name,
            "route": route,
        }

    payload = {
        "instruction": prompt_for_gr00t,
        "prompt": prompt_for_gr00t,
        "image": image,
        "image_url": image if isinstance(image, str) else "",
        "state": state_for_gr00t,
        "robot_state": state_for_gr00t,
        "model_name": model_name,
        "embodiment_tag": embodiment_tag,
        "inference_mode": inference_mode,
        "action_horizon": action_horizon,
    }

    if lidar_context:
        payload["lidar"] = lidar_context
        payload["sensor_context"] = {
            "lidar": lidar_context,
        }

    action_format = str(data.get("actionFormat", "")).strip()
    if action_format:
        payload["action_format"] = action_format

    unity_command = str(data.get("unityCommand", "")).strip()
    if unity_command:
        payload["unity_command"] = unity_command

    waypoint_speed = data.get("waypointSpeed")
    if waypoint_speed is not None:
        payload["waypoint_speed"] = waypoint_speed

    task_analysis = _analyze_task_context(prompt, unity_command, state_for_gr00t)
    task_analysis["model_family"] = "gr00t"
    task_analysis["embodiment_tag"] = embodiment_tag
    task_analysis["action_horizon"] = action_horizon
    if lidar_context:
        task_analysis["lidar_used"] = True
        task_analysis["lidar_context"] = lidar_context
    if task_analysis.get("warning"):
        print(f"[gr00t_infer] warning={task_analysis['warning']}")

    print(f"[gr00t_infer] request={_compact_debug_json(payload)}")

    try:
        resp = requests.post(
            f"{route}/infer",
            json=payload,
            timeout=REQUEST_TIMEOUT_SEC,
        )
        resp.raise_for_status()
        body = resp.json()
    except requests.RequestException as e:
        response = getattr(e, "response", None)
        detail = _response_error_detail(response)
        error_message = f"{model_name} GR00T request failed: {str(e)}"
        if detail:
            error_message = f"{error_message} - detail: {detail}"
        if _should_fallback_gr00t_to_stub(route):
            fallback_route = str(ROUTE_GR00T_STUB).rstrip("/")
            print(
                "[gr00t_infer] real route failed; falling back to stub. "
                f"route={route} fallback={fallback_route} error={error_message}"
            )
            try:
                fallback_resp = requests.post(
                    f"{fallback_route}/infer",
                    json=payload,
                    timeout=REQUEST_TIMEOUT_SEC,
                )
                fallback_resp.raise_for_status()
                body = fallback_resp.json()
                body.setdefault("warning", error_message)
                body["fallback_to_stub"] = True
                body["failed_route"] = route
                body["route"] = fallback_route
                route = fallback_route
            except requests.RequestException as fallback_error:
                fallback_detail = _response_error_detail(
                    getattr(fallback_error, "response", None)
                )
                if fallback_detail:
                    error_message = (
                        f"{error_message}; fallback to {fallback_route} failed: "
                        f"{fallback_error} - detail: {fallback_detail}"
                    )
                else:
                    error_message = (
                        f"{error_message}; fallback to {fallback_route} failed: "
                        f"{fallback_error}"
                    )
                return {
                    "ok": False,
                    "error": error_message,
                    "error_detail": detail,
                    "request": payload,
                    "action": {
                        "type": "error",
                        "translation": [0.0, 0.0, 0.0],
                        "rotation": [0.0, 0.0, 0.0],
                        "gripper": "noop",
                        "terminate": False,
                        "error": error_message,
                    },
                    "model": model_name,
                    "route": route,
                    "fallback_route": fallback_route,
                }
        else:
            return {
                "ok": False,
                "error": error_message,
                "error_detail": detail,
                "request": payload,
                "action": {
                    "type": "error",
                    "translation": [0.0, 0.0, 0.0],
                    "rotation": [0.0, 0.0, 0.0],
                    "gripper": "noop",
                    "terminate": False,
                    "error": error_message,
                },
                "model": model_name,
                "route": route,
            }

    raw_action = _extract_gr00t_response_action(body)
    if raw_action in ({}, None):
        error_message = "GR00T response did not include an action/action_chunk payload."
        return {
            "ok": False,
            "error": error_message,
            "error_detail": error_message,
            "request": payload,
            "action": {
                "type": "error",
                "translation": [0.0, 0.0, 0.0],
                "rotation": [0.0, 0.0, 0.0],
                "gripper": "noop",
                "terminate": False,
                "error": error_message,
            },
            "model": model_name,
            "route": route,
            "raw": body,
        }

    action = _normalize_gr00t_action(
        raw_action,
        unity_command=unity_command,
        embodiment_tag=embodiment_tag,
    )
    action = _suppress_gripper_for_locomotion(action, task_analysis)
    action = _apply_prompt_waypoint_override(
        action,
        prompt=prompt,
        unity_command=unity_command,
        waypoint_speed=waypoint_speed,
    )
    action = _apply_prompt_patrol_override(
        action,
        prompt=prompt,
        unity_command=unity_command,
        waypoint_speed=waypoint_speed,
    )
    if isinstance(action, dict) and waypoint_speed is not None and "waypoint_speed" not in action:
        action = dict(action)
        action["waypoint_speed"] = waypoint_speed
    action = _attach_prompt_metadata(
        action,
        prompt=prompt,
        model_prompt=prompt_for_gr00t,
    )
    if isinstance(action, dict):
        action = g1_normalize_action_payload(action)

    return {
        "action": action,
        "model": body.get("model", body.get("model_name", model_name)),
        "route": route,
        "embodiment_tag": body.get("embodiment_tag", embodiment_tag),
        "inference_mode": body.get("inference_mode", inference_mode),
        "action_horizon": body.get("action_horizon", action_horizon),
        "warning": task_analysis.get("warning", ""),
        "task_analysis": task_analysis,
        "lidar_context": lidar_context,
        "request": payload,
        "raw": body,
    }


def exec_action_decode(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    action = inputs.get("action", {})
    decode_mode = data.get("decodeMode", "default")

    decode_key = str(decode_mode or "default").strip().lower()

    if not isinstance(action, dict):
        decoded = {
            "type": "decoded_action",
            "raw": action,
            "decodeMode": decode_mode,
        }
    else:
        decoded = _json_safe(dict(action))
        decoded.setdefault("type", action.get("type", "decoded_action"))
        decoded["decodeMode"] = decode_mode
        prompt = (
            _first_text(decoded, "prompt", "instruction", "task", "goal", "text")
            or _first_text(inputs, "prompt", "instruction", "task", "goal", "text")
            or _prompt_text_from_context(context)
        )
        if prompt:
            decoded.setdefault("prompt", prompt)
            decoded.setdefault("instruction", prompt)
        if _should_decode_g1_skill(decoded, decode_key, prompt_text=prompt):
            decoded = decode_g1_skill_command(prompt=prompt, action=decoded, props=data)
            decoded["decodeMode"] = decode_mode
        elif g1_looks_like_action(decoded):
            decoded = g1_normalize_action_payload(decoded)

    return {
        "decoded": decoded,
    }


def _should_decode_g1_skill(action: Dict[str, Any], decode_mode: str, *, prompt_text: str = "") -> bool:
    if decode_mode in {"raw", "policy", "joint", "joint_policy", "gr00t_raw"}:
        return False
    if decode_mode in {"g1_skill", "skill", "intent", "intent_skill", "planner"}:
        return True
    command = str(action.get("command") or action.get("omniverse_command") or action.get("omniverseCommand") or "").strip().lower()
    action_type = str(action.get("type") or "").strip().lower()
    if (
        decode_mode in {"", "default", "auto"}
        and command in {"apply_humanoid_step", "apply_humanoid_motion"}
        and action_type in {"gr00t_action", "unity_humanoid_action", "humanoid_motion_action", "decoded_action"}
        and _prompt_requests_g1_locomotion(prompt_text)
    ):
        return True
    return False


def _prompt_requests_g1_locomotion(prompt_text: str) -> bool:
    text = re.sub(r"\s+", " ", str(prompt_text or "").strip().lower())
    if not text:
        return False
    locomotion_words = (
        "walk",
        "go forward",
        "go straight",
        "step forward",
        "move forward",
        "forward",
        "turn",
        "rotate",
        "sidestep",
        "strafe",
        "run",
        "checkpoint",
        "walk ",
    )
    return any(word in text for word in locomotion_words)


def _prompt_text_from_context(context: Dict[str, Any]) -> str:
    for value in context.values():
        if not isinstance(value, dict):
            continue
        node_type = str(value.get("type") or value.get("node_type") or "").strip().lower()
        prompt = _first_text(value, "text", "prompt", "instruction", "task", "goal")
        if prompt and (node_type == "prompt" or "label" in value):
            return prompt
    for value in context.values():
        if isinstance(value, dict):
            prompt = _first_text(value, "prompt", "instruction", "task", "goal", "text")
            if prompt:
                return prompt
    return ""


def exec_safety_clamp(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    action = inputs.get("action", {})
    enabled = bool(data.get("enabled", True))
    max_delta = float(data.get("maxDelta", 0.05))

    if not isinstance(action, dict):
        return {
            "safe_action": {
                "type": "invalid_action",
                "error": "Expected dict action payload",
            }
        }

    def clamp_list(values: Any, limit: float) -> List[float]:
        if not isinstance(values, (list, tuple)):
            return []

        out: List[float] = []
        for v in list(values):
            try:
                fv = float(v)
            except Exception:
                fv = 0.0
            out.append(max(-limit, min(limit, fv)))

        return out

    def clamp_scalar(value: Any, limit: float) -> float:
        try:
            fv = float(value)
        except Exception:
            fv = 0.0
        return max(-limit, min(limit, fv))

    safe_limit = max_delta if enabled else 999999.0
    safe_action = _sanitize_g1_skill_payload(_json_safe(dict(action)))

    for key in ("translation", "rotation", "delta_position", "delta_rotation", "joint_deltas"):
        if key not in action:
            continue
        clamped = clamp_list(action.get(key), safe_limit)
        if clamped:
            safe_action[key] = clamped

    for key in (
        "move_x",
        "move_z",
        "turn_yaw",
        "linear_x",
        "linear_z",
        "angular_yaw",
        "forward_velocity",
        "lateral_velocity",
        "turn_velocity",
    ):
        if key not in action:
            continue
        safe_action[key] = clamp_scalar(action.get(key), safe_limit)

    velocity = action.get("velocity")
    if isinstance(velocity, dict):
        safe_velocity = dict(velocity)
        for key in ("forward", "lateral", "yaw"):
            if key in velocity:
                safe_velocity[key] = clamp_scalar(velocity.get(key), safe_limit)
        safe_action["velocity"] = _json_safe(safe_velocity)

    safe_action.setdefault("type", action.get("type", "safe_action"))
    safe_action["safety"] = {
        "enabled": enabled,
        "maxDelta": max_delta,
    }
    if g1_looks_like_action(safe_action):
        safe_action = g1_normalize_action_payload(safe_action)
    safe_action = _sanitize_g1_skill_payload(safe_action)

    return {
        "safe_action": safe_action,
    }


def _simulate_execute(data: Dict[str, Any], action: Any) -> Dict[str, Any]:
    payload = _sanitize_g1_skill_payload(_extract_action_payload(action))
    return {
        "ok": True,
        "mode": "simulate",
        "executor": data.get("executor", "robot-executor"),
        "simulate_name": data.get("simulateName", "default-sim"),
        "simulate_speed": data.get("simulateSpeed", 1.0),
        "dry_run": data.get("dryRun", True),
        "executed_action": _json_safe(payload),
        "message": "Simulated robot execution completed.",
    }


def _normalize_vec3(values: Any) -> List[float]:
    if isinstance(values, dict):
        return [
            float(values.get("x", 0.0)),
            float(values.get("y", 0.0)),
            float(values.get("z", 0.0)),
        ]

    if isinstance(values, (list, tuple)):
        out: List[float] = []
        for v in list(values)[:3]:
            try:
                out.append(float(v))
            except Exception:
                out.append(0.0)
        while len(out) < 3:
            out.append(0.0)
        return out

    return [0.0, 0.0, 0.0]


def _normalize_float_list(values: Any) -> List[float]:
    if isinstance(values, (list, tuple)):
        out: List[float] = []
        for value in list(values):
            try:
                out.append(float(value))
            except Exception:
                out.append(0.0)
        return out
    return []


def _coerce_float(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _first_text(mapping: Dict[str, Any], *keys: str) -> str:
    for key in keys:
        value = mapping.get(key)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return ""


def _attach_prompt_metadata(
    action: Any,
    *,
    prompt: Any,
    model_prompt: Any,
) -> Any:
    if not isinstance(action, dict):
        return action

    prompt_text = str(prompt or "").strip()
    model_prompt_text = str(model_prompt or "").strip()
    if not prompt_text and not model_prompt_text:
        return action

    enriched = dict(action)
    if prompt_text:
        enriched.setdefault("prompt", prompt_text)
        enriched.setdefault("instruction", prompt_text)
    if model_prompt_text and model_prompt_text != prompt_text:
        enriched.setdefault("model_prompt", model_prompt_text)
    return enriched


def _extract_prompt_context(action: Any) -> Dict[str, str]:
    payload = _sanitize_g1_skill_payload(_extract_action_payload(action))
    if not isinstance(payload, dict):
        return {}

    prompt = _first_text(
        payload,
        "prompt",
        "instruction",
        "model_prompt",
        "task",
        "goal",
        "text",
    )
    model_prompt = _first_text(payload, "model_prompt", "prompt_with_context")
    if not prompt and not model_prompt:
        return {}

    context: Dict[str, str] = {}
    if prompt:
        context["prompt"] = prompt
        context["instruction"] = _first_text(payload, "instruction") or prompt
    if model_prompt:
        context["model_prompt"] = model_prompt
    return context


def _first_value(mapping: Dict[str, Any], *keys: str) -> Any:
    for key in keys:
        if key in mapping and mapping.get(key) is not None:
            return mapping.get(key)
    return None


def _resolve_unity_command(action: Any, data: Dict[str, Any]) -> str:
    payload = _extract_action_payload(action)
    if isinstance(payload, dict):
        resolved = _first_text(payload, "command", "unity_command", "unityCommand")
        if resolved:
            return resolved
    return str(data.get("unityCommand", "apply_action")).strip() or "apply_action"


def _resolve_unity_robot_type(action: Any, data: Dict[str, Any]) -> str:
    payload = _extract_action_payload(action)
    if isinstance(payload, dict):
        resolved = _first_text(
            payload,
            "robot_type",
            "robotType",
            "unity_robot_type",
            "unityRobotType",
        )
        if resolved:
            return resolved
    return str(data.get("unityRobotType", "generic")).strip() or "generic"


def _resolve_unity_controller_mode(action: Any, data: Dict[str, Any]) -> str:
    payload = _extract_action_payload(action)
    if isinstance(payload, dict):
        resolved = _first_text(
            payload,
            "controller_mode",
            "controllerMode",
            "unity_controller_mode",
            "unityControllerMode",
        )
        if resolved:
            return resolved
    return str(data.get("unityControllerMode", "task")).strip() or "task"


def _normalize_unity_action(
    action: Any,
    *,
    command: str,
    robot_type: str,
    controller_mode: str,
    reference_state: Any = None,
    default_waypoint_speed: float | None = None,
) -> Dict[str, Any]:
    payload = _sanitize_g1_skill_payload(_extract_action_payload(action))
    command_name = str(command or "").strip() or "apply_action"
    command_key = command_name.lower()

    if not isinstance(payload, dict):
        return {
            "type": "raw_action",
            "command": command_name,
            "robot_type": robot_type or "generic",
            "controller_mode": controller_mode or "task",
            "raw": _json_safe(payload),
        }

    normalized = {
        "command": command_name,
        "robot_type": robot_type or "generic",
        "controller_mode": controller_mode or "task",
    }

    translation = (
        payload.get("translation")
        or payload.get("position")
        or payload.get("delta_position")
        or [0.0, 0.0, 0.0]
    )
    rotation = (
        payload.get("rotation")
        or payload.get("euler")
        or payload.get("delta_rotation")
        or [0.0, 0.0, 0.0]
    )

    if command_key in {
        "apply_fanuc_joint_target",
        "apply_fanuc_trajectory",
    }:
        for source_key, target_key in (
            ("unit", "unit"),
            ("joint_names", "joint_names"),
            ("jointNames", "joint_names"),
            ("joint_positions", "joint_positions"),
            ("jointPositions", "joint_positions"),
            ("joint_targets", "joint_targets"),
            ("jointTargets", "joint_targets"),
            ("joint_target_map", "joint_target_map"),
            ("jointTargetMap", "joint_target_map"),
            ("named_joint_targets", "named_joint_targets"),
            ("namedJointTargets", "named_joint_targets"),
            ("joint_trajectory", "joint_trajectory"),
            ("jointTrajectory", "joint_trajectory"),
            ("duration_sec", "duration_sec"),
            ("durationSec", "duration_sec"),
            ("seconds_per_frame", "seconds_per_frame"),
            ("secondsPerFrame", "seconds_per_frame"),
            ("preview_only", "preview_only"),
            ("execution_ready", "execution_ready"),
            ("target_embodiment", "target_embodiment"),
        ):
            if source_key in payload:
                normalized[target_key] = _json_safe(payload[source_key])
        normalized["stop"] = bool(
            payload.get("stop", payload.get("terminate", False))
        )
    elif command_key == "apply_joint_targets":
        joint_targets = (
            payload.get("joint_targets")
            or payload.get("jointTargets")
            or payload.get("joint_positions")
            or payload.get("jointPositions")
            or []
        )
        normalized["joint_targets"] = _normalize_float_list(joint_targets)
        normalized["gripper"] = payload.get("gripper", "noop")
        normalized["terminate"] = bool(payload.get("terminate", False))
    elif command_key == "apply_joint_deltas":
        joint_deltas = payload.get("joint_deltas") or payload.get("jointDeltas") or []
        normalized["joint_deltas"] = _normalize_float_list(joint_deltas)
        normalized["gripper"] = payload.get("gripper", "noop")
        normalized["terminate"] = bool(payload.get("terminate", False))
    elif command_key == "apply_joint_velocities":
        joint_velocities = payload.get("joint_velocities") or payload.get("jointVelocities") or []
        normalized["joint_velocities"] = _normalize_float_list(joint_velocities)
        normalized["gripper"] = payload.get("gripper", "noop")
        normalized["terminate"] = bool(payload.get("terminate", False))
    elif command_key == "apply_base_velocity":
        linear = payload.get("linear") or translation
        angular = payload.get("angular") or rotation
        linear_vec = _normalize_vec3(linear)
        angular_vec = _normalize_vec3(angular)
        normalized["linear_x"] = _coerce_float(
            payload.get("linear_x", payload.get("linearX", linear_vec[0]))
        )
        normalized["linear_z"] = _coerce_float(
            payload.get("linear_z", payload.get("linearZ", linear_vec[2]))
        )
        normalized["angular_yaw"] = _coerce_float(
            payload.get("angular_yaw", payload.get("angularYaw", angular_vec[1]))
        )
        normalized["stop"] = bool(payload.get("stop", payload.get("terminate", False)))
    elif command_key == "apply_base_waypoint":
        explicit_target_position = payload.get("target_position")
        if explicit_target_position is None:
            explicit_target_position = payload.get("targetPosition")

        explicit_target_rotation = payload.get("target_rotation")
        if explicit_target_rotation is None:
            explicit_target_rotation = payload.get("targetRotation")

        translation_vec = _normalize_vec3(translation)
        rotation_vec = _normalize_vec3(rotation)
        reference_position = _state_vec3(reference_state, "position")
        reference_rotation = _state_vec3(reference_state, "rotation")
        action_type = str(payload.get("type", "")).strip().lower()
        delta_like = action_type == "delta_action" or "delta_position" in payload or "translation" in payload

        if explicit_target_position is not None:
            target_position_vec = _normalize_vec3(explicit_target_position)
        elif delta_like and reference_position is not None:
            target_position_vec = _sum_vec3(reference_position, translation_vec)
        else:
            target_position_vec = translation_vec

        if explicit_target_rotation is not None:
            target_rotation_vec = _normalize_vec3(explicit_target_rotation)
        elif delta_like and reference_rotation is not None:
            target_rotation_vec = _sum_vec3(reference_rotation, rotation_vec)
        else:
            target_rotation_vec = rotation_vec

        speed_value = payload.get("speed")
        if speed_value is None:
            speed_value = payload.get("waypoint_speed")
        if speed_value is None:
            speed_value = default_waypoint_speed

        normalized["target_position"] = target_position_vec
        normalized["target_rotation"] = target_rotation_vec
        normalized["speed"] = _coerce_float(speed_value, 0.0)
        normalized["gripper"] = payload.get("gripper", "noop")
        normalized["stop"] = bool(payload.get("stop", payload.get("terminate", False)))
    elif command_key == "apply_humanoid_step":
        translation_vec = _normalize_vec3(translation)
        rotation_vec = _normalize_vec3(rotation)
        normalized["move_x"] = _coerce_float(
            payload.get("move_x", payload.get("moveX", translation_vec[0]))
        )
        normalized["move_z"] = _coerce_float(
            payload.get("move_z", payload.get("moveZ", translation_vec[2]))
        )
        normalized["turn_yaw"] = _coerce_float(
            payload.get("turn_yaw", payload.get("turnYaw", rotation_vec[1]))
        )
        normalized["speed"] = _coerce_float(payload.get("speed", 1.0), 1.0)
        if "duration_sec" in payload or "durationSec" in payload:
            normalized["duration_sec"] = _coerce_float(
                payload.get("duration_sec", payload.get("durationSec", 1.0)),
                1.0,
            )
        normalized["local_frame"] = bool(payload.get("local_frame", payload.get("localFrame", False)))
        normalized["stop"] = bool(payload.get("stop", payload.get("terminate", False)))
    elif command_key in {"apply_g1_skill", "apply_skill", "apply_skill_command"}:
        normalized["command"] = "apply_g1_skill"
        normalized["skill"] = _first_text(payload, "skill", "motion_skill", "motionSkill")
        normalized["duration_sec"] = _coerce_float(
            payload.get("duration_sec", payload.get("durationSec", 1.0)),
            1.0,
        )
        normalized["speed"] = _coerce_float(payload.get("speed", 1.0), 1.0)
        normalized["move_x"] = _coerce_float(payload.get("move_x", payload.get("moveX", 0.0)), 0.0)
        normalized["move_z"] = _coerce_float(payload.get("move_z", payload.get("moveZ", 0.0)), 0.0)
        normalized["turn_yaw"] = _coerce_float(payload.get("turn_yaw", payload.get("turnYaw", 0.0)), 0.0)
        normalized["forward_velocity"] = _coerce_float(payload.get("forward_velocity", 0.0), 0.0)
        normalized["lateral_velocity"] = _coerce_float(payload.get("lateral_velocity", 0.0), 0.0)
        normalized["turn_velocity"] = _coerce_float(payload.get("turn_velocity", 0.0), 0.0)
        velocity = payload.get("velocity")
        if isinstance(velocity, dict):
            normalized["velocity"] = _json_safe(velocity)
        target_pose = _first_value(payload, "target_pose", "targetPose")
        if target_pose is not None:
            normalized["target_pose"] = _json_safe(target_pose)
        normalized["stop"] = bool(payload.get("stop", payload.get("terminate", False)))
    elif command_key == "apply_humanoid_motion":
        translation_vec = _normalize_vec3(translation)
        rotation_vec = _normalize_vec3(rotation)
        normalized["move_x"] = _coerce_float(
            payload.get("move_x", payload.get("moveX", translation_vec[0]))
        )
        normalized["move_z"] = _coerce_float(
            payload.get("move_z", payload.get("moveZ", translation_vec[2]))
        )
        normalized["turn_yaw"] = _coerce_float(
            payload.get("turn_yaw", payload.get("turnYaw", rotation_vec[1]))
        )
        normalized["speed"] = _coerce_float(payload.get("speed", 1.0), 1.0)
        normalized["duration_sec"] = _coerce_float(
            payload.get("duration_sec", payload.get("durationSec", 2.0)),
            2.0,
        )
        normalized["skill"] = _first_text(payload, "skill", "motion_skill", "motionSkill")
        normalized["gesture"] = _first_text(payload, "gesture")
        normalized["animation"] = bool(payload.get("animation", True))
        normalized["trace_path"] = _first_text(payload, "trace_path", "tracePath")
        normalized["motion_summary"] = _json_safe(payload.get("motion_summary", payload.get("motionSummary", {})))
        normalized["pose_sequence"] = _json_safe(payload.get("pose_sequence", payload.get("poseSequence", [])))
        for source_key, target_key in (
            ("joint_targets", "joint_targets"),
            ("jointTargets", "joint_targets"),
            ("joint_trajectory", "joint_trajectory"),
            ("jointTrajectory", "joint_trajectory"),
            ("g1_joint_trajectory", "g1_joint_trajectory"),
            ("g1JointTrajectory", "g1_joint_trajectory"),
            ("joint_trajectory_count", "joint_trajectory_count"),
            ("jointTrajectoryCount", "joint_trajectory_count"),
            ("joint_trajectory_duration_sec", "joint_trajectory_duration_sec"),
            ("jointTrajectoryDurationSec", "joint_trajectory_duration_sec"),
            ("joint_positions", "joint_positions"),
            ("jointPositions", "joint_positions"),
            ("joint_names", "joint_names"),
            ("jointNames", "joint_names"),
            ("joint_groups", "joint_groups"),
            ("jointGroups", "joint_groups"),
            ("joint_schema", "joint_schema"),
            ("jointSchema", "joint_schema"),
            ("joint_target_map", "joint_target_map"),
            ("jointTargetMap", "joint_target_map"),
            ("named_joint_targets", "named_joint_targets"),
            ("namedJointTargets", "named_joint_targets"),
            ("g1_named_targets", "g1_named_targets"),
            ("g1NamedTargets", "g1_named_targets"),
            ("joint_order_source", "joint_order_source"),
            ("jointOrderSource", "joint_order_source"),
            ("embodiment_tag", "embodiment_tag"),
            ("embodimentTag", "embodiment_tag"),
            ("retarget_source", "retarget_source"),
            ("retargetSource", "retarget_source"),
            ("retarget_mode", "retarget_mode"),
            ("retargetMode", "retarget_mode"),
            ("retarget_window", "retarget_window"),
            ("retargetWindow", "retarget_window"),
            ("retarget_frame_indices", "retarget_frame_indices"),
            ("retargetFrameIndices", "retarget_frame_indices"),
            ("root_translation", "root_translation"),
            ("rootTranslation", "root_translation"),
            ("root_delta_position", "root_delta_position"),
            ("rootDeltaPosition", "root_delta_position"),
            ("root_translation_mode", "root_translation_mode"),
            ("rootTranslationMode", "root_translation_mode"),
            ("root_rotation", "root_rotation"),
            ("rootRotation", "root_rotation"),
            ("root_delta_rotation", "root_delta_rotation"),
            ("rootDeltaRotation", "root_delta_rotation"),
            ("root_motion_source", "root_motion_source"),
            ("rootMotionSource", "root_motion_source"),
            ("root_motion_delta", "root_motion_delta"),
            ("rootMotionDelta", "root_motion_delta"),
            ("root_motion_mode", "root_motion_mode"),
            ("rootMotionMode", "root_motion_mode"),
            ("foot_lock", "foot_lock"),
            ("footLock", "foot_lock"),
            ("pelvis_offset", "pelvis_offset"),
            ("pelvisOffset", "pelvis_offset"),
            ("pelvis_offset_sequence", "pelvis_offset_sequence"),
            ("pelvisOffsetSequence", "pelvis_offset_sequence"),
        ):
            if source_key in payload:
                normalized[target_key] = _json_safe(payload.get(source_key))
        for group_key in (
            "waist",
            "left_leg",
            "right_leg",
            "left_arm",
            "right_arm",
            "left_hand",
            "right_hand",
        ):
            if group_key in payload:
                normalized[group_key] = _json_safe(payload.get(group_key))
        has_joint_trajectory = any(
            key in normalized
            for key in ("joint_trajectory", "g1_joint_trajectory")
        )
        if any(
            key in normalized
            for key in (
                "joint_targets",
                "joint_trajectory",
                "g1_joint_trajectory",
                "joint_positions",
                "joint_target_map",
                "named_joint_targets",
                "g1_named_targets",
            )
        ):
            normalized.setdefault("retarget_mode", "joint_trajectory" if has_joint_trajectory else "joint_targets")
            normalized.setdefault("embodiment_tag", "REAL_G1")
        normalized["stop"] = bool(payload.get("stop", payload.get("terminate", False)))
    elif command_key == "apply_patrol_control":
        explicit_mode = _first_text(payload, "patrol_mode", "patrolMode", "mode")
        target_position = _first_value(
            payload,
            "patrol_target_position",
            "patrolTargetPosition",
            "target_position",
            "targetPosition",
        )
        patrol_direction = _first_value(
            payload,
            "patrol_direction",
            "patrolDirection",
            "direction",
            "linear",
            "translation",
            "delta_position",
            "deltaPosition",
        )
        stop_requested = bool(payload.get("stop", payload.get("terminate", False)))

        if stop_requested:
            patrol_mode = "disable"
        elif explicit_mode:
            patrol_mode = explicit_mode.lower()
        elif target_position is not None:
            patrol_mode = "target"
        elif patrol_direction is not None:
            patrol_mode = "direction"
        else:
            patrol_mode = "enable"

        speed_value = _first_value(payload, "patrol_speed", "patrolSpeed", "speed", "waypoint_speed")
        if speed_value is None:
            speed_value = default_waypoint_speed

        normalized["patrol_mode"] = patrol_mode
        normalized["patrol_speed"] = _coerce_float(speed_value, 0.0)
        normalized["stop"] = stop_requested

        if target_position is not None:
            normalized["patrol_target_position"] = _normalize_vec3(target_position)

        if patrol_direction is not None:
            normalized["patrol_direction"] = _normalize_vec3(patrol_direction)
    else:
        normalized["translation"] = _normalize_vec3(translation)
        normalized["rotation"] = _normalize_vec3(rotation)
        normalized["gripper"] = payload.get("gripper", "noop")
        normalized["terminate"] = bool(payload.get("terminate", False))

    action_type = payload.get("type")
    if action_type is not None:
        normalized["type"] = action_type

    safety = payload.get("safety")
    if isinstance(safety, dict):
        normalized["safety"] = _json_safe(safety)

    meta_keys = {
        "command",
        "unity_command",
        "unityCommand",
        "robot_type",
        "robotType",
        "unity_robot_type",
        "unityRobotType",
        "controller_mode",
        "controllerMode",
        "unity_controller_mode",
        "unityControllerMode",
        "patrolMode",
        "patrolTargetPosition",
        "patrolDirection",
        "patrolSpeed",
        "deltaPosition",
        "raw_action",
        "rawAction",
        "raw_action_summary",
        "rawActionSummary",
    }
    for key, value in payload.items():
        if key in normalized or key in meta_keys:
            continue
        normalized[key] = _json_safe(value)

    if str(robot_type or "").strip().lower() == "humanoid" or str(controller_mode or "").strip().lower() == "locomotion":
        normalized = g1_normalize_action_payload(normalized)

    return normalized


def _replace_endpoint_host(endpoint: str, host: str) -> str:
    parsed = urlsplit(endpoint)
    if not parsed.netloc:
        return endpoint

    original_host = parsed.hostname or ""
    host_port = parsed.netloc
    if "@" in host_port:
        auth_part, host_port = host_port.rsplit("@", 1)
        auth_prefix = f"{auth_part}@"
    else:
        auth_prefix = ""

    port = parsed.port
    if port is None:
        new_netloc = f"{auth_prefix}{host}"
    else:
        new_netloc = f"{auth_prefix}{host}:{port}"

    return urlunsplit((parsed.scheme, new_netloc, parsed.path, parsed.query, parsed.fragment))


def _toggle_robot_path_suffix(endpoint: str) -> str:
    parsed = urlsplit(endpoint)
    path = parsed.path or ""

    if path.endswith("/"):
        toggled_path = path[:-1]
    else:
        toggled_path = f"{path}/"

    return urlunsplit((parsed.scheme, parsed.netloc, toggled_path, parsed.query, parsed.fragment))


def _websocket_endpoint_candidates(endpoint: str, default_endpoint: str) -> List[str]:
    raw = _rewrite_legacy_websocket_port(str(endpoint or "").strip() or default_endpoint)
    candidates: List[str] = []

    def add(value: str) -> None:
        normalized = str(value or "").strip()
        if normalized and normalized not in candidates:
            candidates.append(normalized)

    add(raw)
    add(_toggle_robot_path_suffix(raw))

    hostname = urlsplit(raw).hostname or ""
    if hostname in {"localhost", "127.0.0.1", "0.0.0.0"}:
        docker_host_variant = _replace_endpoint_host(raw, "host.docker.internal")
        add(docker_host_variant)
        add(_toggle_robot_path_suffix(docker_host_variant))
    elif hostname == "host.docker.internal":
        for host in re.split(r"[\s,;]+", os.getenv("VLAGRAPH_OMNIVERSE_HOSTS", "").strip()):
            if host:
                host_variant = _replace_endpoint_host(raw, host)
                add(host_variant)
                add(_toggle_robot_path_suffix(host_variant))
        host_variant = _replace_endpoint_host(raw, "localhost")
        add(host_variant)
        add(_toggle_robot_path_suffix(host_variant))

    return candidates


def _rewrite_legacy_websocket_port(endpoint: str) -> str:
    try:
        parsed = urlsplit(str(endpoint or "").strip())
    except Exception:
        return str(endpoint or "").strip()

    port = parsed.port
    if port not in _LEGACY_WEBSOCKET_PORT_REWRITES:
        return str(endpoint or "").strip()

    replacement = _LEGACY_WEBSOCKET_PORT_REWRITES[port]
    netloc = parsed.netloc.rsplit(f":{port}", 1)[0] + f":{replacement}"
    return urlunsplit((parsed.scheme, netloc, parsed.path, parsed.query, parsed.fragment))


def _unity_endpoint_candidates(endpoint: str) -> List[str]:
    return _websocket_endpoint_candidates(
        endpoint,
        "ws://host.docker.internal:9081/robot",
    )


def _omniverse_endpoint_candidates(endpoint: str) -> List[str]:
    return _websocket_endpoint_candidates(
        endpoint,
        "ws://host.docker.internal:9900/omniverse",
    )


def _unity_execute(data: Dict[str, Any], action: Any) -> Dict[str, Any]:
    endpoint = data.get("unityEndpoint", "ws://host.docker.internal:9081/robot")
    scene = data.get("unityScene", "MainScene")
    command = _resolve_unity_command(action, data)
    robot_type = _resolve_unity_robot_type(action, data)
    controller_mode = _resolve_unity_controller_mode(action, data)
    timeout_sec = float(data.get("unityTimeoutSec", data.get("timeoutSec", 5)))
    response_timeout_sec = max(timeout_sec, 15.0)

    request_id = str(uuid.uuid4())
    state_client_id = str(
        data.get("unityStateClientId", data.get("clientId", "default"))
    ).strip() or "default"
    state_record = state_store.get_state(state_client_id)
    configured_object = data.get("unityObject", "RobotKyle")
    obj, state_object_name, object_overridden = _resolve_unity_target_object(
        configured_object,
        state_record,
    )

    normalized_action = _normalize_unity_action(
        action,
        command=command,
        robot_type=robot_type,
        controller_mode=controller_mode,
        reference_state=state_record,
        default_waypoint_speed=_coerce_float(data.get("waypointSpeed"), 0.0) or None,
    )
    prompt_context = _extract_prompt_context(action)

    if object_overridden:
        print(
            "[unity_execute] target override="
            + _compact_debug_json(
                {
                    "configured_object": configured_object,
                    "resolved_object": obj,
                    "unity_state_client_id": state_client_id,
                }
            )
        )

    request_payload = {
        "type": "robot_command",
        "request_id": request_id,
        "scene": scene,
        "object": obj,
        "command": command,
        "robot_type": robot_type,
        "controller_mode": controller_mode,
        "action": _json_safe(normalized_action),
    }
    if prompt_context:
        request_payload.update(prompt_context)

    print(f"[unity_execute] request={_compact_debug_json(request_payload)}")

    attempted_errors: List[str] = []

    for candidate_endpoint in _unity_endpoint_candidates(endpoint):
        ws = None
        request_sent = False
        try:
            ws = create_connection(candidate_endpoint, timeout=timeout_sec)
            ws.settimeout(response_timeout_sec)
            ws.send(json.dumps(request_payload, ensure_ascii=False))
            request_sent = True
            primary_response: Dict[str, Any] | None = None
            latest_state: Dict[str, Any] | None = None
            received_messages: List[Dict[str, Any]] = []

            raw_response = ws.recv()
            response_candidates = [raw_response]

            followup_timeout = max(0.1, min(response_timeout_sec, 0.25))
            ws.settimeout(followup_timeout)
            while True:
                try:
                    response_candidates.append(ws.recv())
                except (socket.timeout, TimeoutError):
                    break
                except Exception:
                    break

            for raw_message in response_candidates:
                try:
                    parsed_message = json.loads(raw_message)
                except Exception:
                    parsed_message = {
                        "ok": False,
                        "request_id": request_id,
                        "applied": False,
                        "error": f"Invalid JSON response from Unity: {raw_message}",
                    }

                if not isinstance(parsed_message, dict):
                    parsed_message = {
                        "ok": False,
                        "request_id": request_id,
                        "applied": False,
                        "error": f"Unexpected Unity response type: {type(parsed_message).__name__}",
                    }

                received_messages.append(parsed_message)

                message_type = str(parsed_message.get("type", "")).strip().lower()
                if message_type == "robot_state":
                    latest_state = parsed_message
                    continue

                if primary_response is None:
                    primary_response = parsed_message

            response = primary_response or latest_state or {
                "ok": False,
                "request_id": request_id,
                "applied": False,
                "error": "No response received from Unity.",
            }

            if response.get("request_id") not in (None, request_id):
                return {
                    "ok": False,
                    "mode": "unity",
                    "executor": data.get("executor", "robot-executor"),
                    "endpoint": candidate_endpoint,
                    "scene": scene,
                    "object": obj,
                    "robot_type": robot_type,
                    "controller_mode": controller_mode,
                    "command": command,
                    "request": request_payload,
                    "response": response,
                    "received_messages": received_messages,
                    "message": "Unity response request_id mismatch.",
                }

            stored_state = None
            if latest_state is not None:
                stored_state = state_store.update_state(state_client_id, latest_state)

            response_ok = response.get("ok")
            if response_ok is None:
                response_ok = response.get("applied")

            return {
                "ok": bool(response_ok),
                "mode": "unity",
                "executor": data.get("executor", "robot-executor"),
                "endpoint": candidate_endpoint,
                "scene": scene,
                "object": obj,
                "robot_type": robot_type,
                "controller_mode": controller_mode,
                "command": command,
                "request": request_payload,
                "response": response,
                "received_messages": received_messages,
                "unity_state_client_id": state_client_id,
                "resolved_object": obj,
                "configured_object": configured_object,
                "state_object_name": state_object_name,
                "state_update": stored_state,
                "message": response.get("message", "Unity command executed."),
            }

        except Exception as e:
            attempted_errors.append(f"{candidate_endpoint} -> {str(e)}")
            if request_sent:
                break
        finally:
            if ws is not None:
                try:
                    ws.close()
                except Exception:
                    pass

    attempted_text = "; ".join(attempted_errors) if attempted_errors else "unknown websocket error"
    return {
        "ok": False,
        "mode": "unity",
        "executor": data.get("executor", "robot-executor"),
        "endpoint": endpoint,
        "scene": scene,
        "object": obj,
        "robot_type": robot_type,
        "controller_mode": controller_mode,
        "command": command,
        "request": request_payload,
        "tried_endpoints": _unity_endpoint_candidates(endpoint),
        "resolved_object": obj,
        "configured_object": configured_object,
        "state_object_name": state_object_name,
        "message": f"Unity websocket execution failed: {attempted_text}",
    }


def _omniverse_execute(data: Dict[str, Any], action: Any) -> Dict[str, Any]:
    stage_url = data.get("omniverseUrl", "omniverse://localhost/Projects/Robot")
    stage = data.get("omniverseStage", "/World/RobotStage")
    prim_path = data.get("omniversePrimPath", "/World/Robot")
    payload = _extract_action_payload(action)
    action_command = _first_text(payload, "command", "omniverse_command", "omniverseCommand") if isinstance(payload, dict) else ""
    command = action_command or data.get("omniverseCommand", "execute_policy")
    timeout_sec = float(data.get("omniverseTimeoutSec", data.get("timeoutSec", 15)))
    wait_for_response = _coerce_bool(
        data.get("omniverseWaitForResponse", data.get("waitForResponse")),
        True,
    )
    response_timeout_sec = max(timeout_sec, 15.0)
    normalized_action = _normalize_unity_action(
        action,
        command=command,
        robot_type=str(
            data.get(
                "omniverseRobotType",
                data.get("robotType", data.get("unityRobotType", "generic")),
            )
        ),
        controller_mode=str(
            data.get(
                "omniverseControllerMode",
                data.get("controllerMode", data.get("unityControllerMode", "task")),
            )
        ),
    )
    request_id = str(uuid.uuid4())

    endpoint = str(data.get("omniverseEndpoint", "")).strip()
    if not endpoint:
        legacy_url = str(stage_url or "").strip()
        if legacy_url.startswith("ws://") or legacy_url.startswith("wss://"):
            endpoint = legacy_url
    prompt_context = _extract_prompt_context(action)

    normalized_action = _compact_omniverse_action_payload(normalized_action)

    request_payload = {
        "type": "omniverse_command",
        "request_id": request_id,
        "url": stage_url,
        "stage": stage,
        "prim_path": prim_path,
        "command": command,
        "action": _json_safe(normalized_action),
    }
    if prompt_context:
        request_payload.update(prompt_context)
    command_id = data.get("commandId") or (payload.get("command_id") if isinstance(payload, dict) else None)
    if command_id:
        request_payload["command_id"] = str(command_id)
    _maybe_log_omniverse_payload(request_id, command, normalized_action, request_payload)

    attempted_errors: List[str] = []

    for candidate_endpoint in _omniverse_endpoint_candidates(endpoint):
        ws = None
        request_sent = False
        try:
            skipped_live = _maybe_skip_live_omniverse_payload(
                candidate_endpoint,
                prim_path,
                command,
                normalized_action,
            )
            if skipped_live is not None:
                skipped_live.update(
                    {
                        "url": stage_url,
                        "stage": stage,
                        "prim_path": prim_path,
                        "request": request_payload,
                    }
                )
                return skipped_live
            ws = create_connection(candidate_endpoint, timeout=timeout_sec)
            ws.settimeout(response_timeout_sec)
            ws.send(json.dumps(request_payload, ensure_ascii=False))
            request_sent = True

            if not wait_for_response:
                return {
                    "ok": True,
                    "mode": "omniverse",
                    "executor": data.get("executor", "robot-executor"),
                    "endpoint": candidate_endpoint,
                    "url": stage_url,
                    "stage": stage,
                    "prim_path": prim_path,
                    "command": command,
                    "request": request_payload,
                    "sent": True,
                    "wait_for_response": False,
                    "message": "Omniverse command sent without waiting for a response.",
                }

            raw_response = ws.recv()
            response_candidates = [raw_response]
            ws.settimeout(max(0.1, min(response_timeout_sec, 0.25)))
            while True:
                try:
                    response_candidates.append(ws.recv())
                except (socket.timeout, TimeoutError):
                    break
                except Exception:
                    break

            primary_response: Dict[str, Any] | None = None
            latest_state: Dict[str, Any] | None = None
            received_messages: List[Dict[str, Any]] = []
            for raw_message in response_candidates:
                try:
                    parsed_message = json.loads(raw_message)
                except Exception:
                    parsed_message = {
                        "ok": False,
                        "request_id": request_id,
                        "applied": False,
                        "error": f"Invalid JSON response from Omniverse: {raw_message}",
                    }
                if not isinstance(parsed_message, dict):
                    parsed_message = {
                        "ok": False,
                        "request_id": request_id,
                        "applied": False,
                        "error": f"Unexpected Omniverse response type: {type(parsed_message).__name__}",
                    }
                received_messages.append(parsed_message)
                if str(parsed_message.get("type") or "").strip().lower() == "robot_state":
                    latest_state = parsed_message
                    continue
                if primary_response is None:
                    primary_response = parsed_message

            response = primary_response or latest_state or {
                "ok": False,
                "request_id": request_id,
                "applied": False,
                "error": "No response received from Omniverse.",
            }

            if response.get("request_id") not in (None, request_id):
                return {
                    "ok": False,
                    "mode": "omniverse",
                    "executor": data.get("executor", "robot-executor"),
                    "endpoint": candidate_endpoint,
                    "url": stage_url,
                    "stage": stage,
                    "prim_path": prim_path,
                    "command": command,
                    "request": request_payload,
                    "response": response,
                    "received_messages": received_messages,
                    "message": "Omniverse response request_id mismatch.",
                }

            response_ok = response.get("ok")
            if response_ok is None:
                response_ok = response.get("applied")
            stored_state = None
            if latest_state is None and isinstance(response.get("state"), dict):
                latest_state = response.get("state")
            if latest_state is not None:
                state_client_id = str(
                    latest_state.get("client_id")
                    or data.get("unityStateClientId")
                    or data.get("clientId")
                    or "default"
                ).strip() or "default"
                stored_state = state_store.update_state(state_client_id, latest_state)

            return {
                "ok": bool(response_ok),
                "mode": "omniverse",
                "executor": data.get("executor", "robot-executor"),
                "endpoint": candidate_endpoint,
                "url": stage_url,
                "stage": stage,
                "prim_path": prim_path,
                "command": command,
                "request": request_payload,
                "response": response,
                "received_messages": received_messages,
                "state_update": stored_state,
                "message": response.get("message", "Omniverse command executed."),
            }
        except Exception as e:
            attempted_errors.append(f"{candidate_endpoint} -> {str(e)}")
            if request_sent:
                break
        finally:
            if ws is not None:
                try:
                    ws.close()
                except Exception:
                    pass

    attempted_text = "; ".join(attempted_errors) if attempted_errors else "unknown websocket error"
    return {
        "ok": False,
        "mode": "omniverse",
        "executor": data.get("executor", "robot-executor"),
        "endpoint": endpoint or "ws://host.docker.internal:9900/omniverse",
        "url": stage_url,
        "stage": stage,
        "prim_path": prim_path,
        "command": command,
        "request": request_payload,
        "tried_endpoints": _omniverse_endpoint_candidates(endpoint),
        "message": f"Omniverse websocket execution failed: {attempted_text}",
    }


def _humanoid_pose_sequence_signature(data: Dict[str, Any], action_payload: Any) -> str:
    if not isinstance(action_payload, dict):
        return ""

    command = str(
        action_payload.get("command")
        or action_payload.get("omniverse_command")
        or action_payload.get("omniverseCommand")
        or data.get("omniverseCommand")
        or ""
    ).strip().lower()
    action_type = str(action_payload.get("type", "")).strip().lower()
    pose_sequence = action_payload.get("pose_sequence") or action_payload.get("poseSequence")
    joint_trajectory = (
        action_payload.get("g1_joint_trajectory")
        or action_payload.get("g1JointTrajectory")
        or action_payload.get("joint_trajectory")
        or action_payload.get("jointTrajectory")
    )
    if command != "apply_humanoid_motion" and action_type != "humanoid_motion_action":
        return ""
    has_pose_sequence = isinstance(pose_sequence, list) and bool(pose_sequence)
    has_joint_trajectory = isinstance(joint_trajectory, list) and bool(joint_trajectory)
    if not has_pose_sequence and not has_joint_trajectory:
        return ""

    root_sequence = (
        action_payload.get("root_translation_sequence")
        or action_payload.get("rootTranslationSequence")
        or []
    )
    first_frame = pose_sequence[0] if has_pose_sequence else joint_trajectory[0]
    last_frame = pose_sequence[-1] if has_pose_sequence else joint_trajectory[-1]
    sample = {
        "endpoint": str(data.get("omniverseEndpoint", "")).strip(),
        "prim_path": str(data.get("omniversePrimPath", "")).strip(),
        "command": command,
        "type": action_type,
        "retarget_mode": action_payload.get("retarget_mode"),
        "skill": action_payload.get("skill"),
        "trace_path": action_payload.get("trace_path") or action_payload.get("tracePath"),
        "pose_frame_count": len(pose_sequence) if has_pose_sequence else 0,
        "joint_trajectory_count": len(joint_trajectory) if has_joint_trajectory else 0,
        "joint_trajectory_duration_sec": action_payload.get("joint_trajectory_duration_sec"),
        "root_frame_count": len(root_sequence) if isinstance(root_sequence, list) else 0,
        "first_frame": first_frame,
        "last_frame": last_frame,
    }
    encoded = json.dumps(_json_safe(sample), sort_keys=True, separators=(",", ":"))
    return hashlib.sha256(encoded.encode("utf-8")).hexdigest()


def _ros_execute(data: Dict[str, Any], action: Any) -> Dict[str, Any]:
    payload = _extract_action_payload(action)
    return {
        "ok": True,
        "mode": "ros",
        "executor": data.get("executor", "robot-executor"),
        "bridge_url": data.get("rosBridgeUrl", "ws://localhost:9090"),
        "topic": data.get("rosTopic", "/robot/action_cmd"),
        "robot_name": data.get("rosRobotName", "robot1"),
        "frame_id": data.get("rosFrameId", "base_link"),
        "msg_type": data.get("rosMsgType", "geometry_msgs/Twist"),
        "payload": {
            "op": "publish",
            "topic": data.get("rosTopic", "/robot/action_cmd"),
            "msg": {
                "robot_name": data.get("rosRobotName", "robot1"),
                "frame_id": data.get("rosFrameId", "base_link"),
                "action": _json_safe(payload),
            },
            "type": data.get("rosMsgType", "geometry_msgs/Twist"),
        },
        "message": "ROS execution payload prepared.",
    }


def exec_robot_execute(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    action = inputs.get("action")
    mode = str(data.get("mode", "simulate")).lower().strip()

    if _coerce_bool(data.get("requireExecutionGuard"), False):
        from backend.world_model.fanuc_preflight import action_fingerprint
        sources = [context.get(edge.get("source"), {}) for edge in edges
                   if edge.get("target") == node["id"] and edge.get("targetHandle") == "action"]
        guard = sources[0] if len(sources) == 1 and isinstance(sources[0], dict) else {}
        decision = guard.get("decision") if isinstance(guard.get("decision"), dict) else {}
        valid = False
        try:
            valid = (guard.get("type") == "execution_guard_result" and guard.get("approved") is True
                     and guard.get("action_digest") == action_fingerprint(action))
        except (TypeError, ValueError):
            pass
        if decision.get("validation_policy") == "static_simulation" and mode != "omniverse":
            valid = False
        if not valid:
            result = {"ok": False, "skipped": True, "mode": mode,
                      "message": "Execution guard blocked the action or authorization is missing.",
                      "guard_decision": guard.get("decision")}
            return {"ok": False, "skipped": True, "result": result, "message": result["message"]}

    if action is None:
        return {
            "result": {
                "ok": False,
                "mode": mode,
                "message": "No action input connected.",
            }
        }

    action_payload = _extract_action_payload(action)
    if isinstance(action_payload, dict):
        if action_payload.get("type") == "execution_blocked":
            result = {"ok": False, "skipped": True, "mode": mode,
                      "message": "Execution blocked by trajectory validation.", "reasons": action_payload.get("reasons", [])}
            return {"ok": False, "skipped": True, "result": result, "message": result["message"]}
        action_error = action_payload.get("error")
        if str(action_payload.get("type", "")).strip().lower() == "error" or action_error:
            return {
                "result": {
                    "ok": False,
                    "mode": mode,
                    "message": "Action input contains an error; robot execution skipped.",
                    "error": str(action_error or "Action type is error."),
                    "skipped": True,
                    "action": _json_safe(action_payload),
                }
            }
        if _coerce_bool(action_payload.get("preview_only"), False):
            return {
                "result": {
                    "ok": True,
                    "mode": mode,
                    "preview_only": True,
                    "skipped": True,
                    "execution_ready": bool(action_payload.get("execution_ready")),
                    "message": (
                        "Preview-only action; robot execution skipped. "
                        "Enable execution only after calibration and IK validation."
                    ),
                    "action": _json_safe(action_payload),
                },
                "ok": True,
                "skipped": True,
                "message": "Preview-only action; robot execution skipped.",
            }

    if mode != "simulate" and _coerce_bool(data.get("dryRun"), False):
        result = _simulate_execute(data, action)
        result.update(
            {
                "mode": mode,
                "dry_run": True,
                "skipped": True,
                "message": f"Dry-run enabled; skipped {mode} robot execution.",
            }
        )
        return {
            "result": result,
        }

    if mode == "simulate":
        result = _simulate_execute(data, action)
    elif mode == "unity":
        result = _unity_execute(data, action)
    elif mode == "omniverse":
        result = _omniverse_execute(data, action)
    elif mode == "ros":
        result = _ros_execute(data, action)
    else:
        result = {
            "ok": False,
            "mode": mode,
            "message": f"Unsupported robot execute mode: {mode}",
        }

    output: Dict[str, Any] = {"result": result}
    if isinstance(result, dict):
        output["ok"] = bool(result.get("ok"))
        output["message"] = result.get("message", "")
        if result.get("skipped") is True:
            output["skipped"] = True
        if result.get("duplicate_pose_sequence") is True:
            output["duplicate_pose_sequence"] = True

    return output


def exec_openvla_finetune(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {})
    inputs = _incoming_values(node["id"], edges, context)

    dataset = inputs.get("dataset", {})
    dataset_path = ""
    if isinstance(dataset, dict):
        dataset_path = str(
            dataset.get("path")
            or dataset.get("dataset_path")
            or dataset.get("file_path")
            or dataset.get("datasetPath")
            or ""
        ).strip()
    elif isinstance(dataset, str):
        dataset_path = dataset.strip()

    if not dataset_path:
        dataset_payload = next(
            (
                value
                for value in inputs.values()
                if isinstance(value, dict)
                and (
                    value.get("dataset_path")
                    or value.get("path")
                    or value.get("file_path")
                    or value.get("datasetPath")
                )
            ),
            None,
        )
        if isinstance(dataset_payload, dict):
            dataset_path = str(
                dataset_payload.get("dataset_path")
                or dataset_payload.get("path")
                or dataset_payload.get("file_path")
                or dataset_payload.get("datasetPath")
                or ""
            ).strip()

    if not dataset_path:
        raise ValueError("openvla_finetune node requires a dataset input with a valid path")

    payload = {
        "dataset_path": dataset_path,
        "base_model": str(data.get("baseModel", "openvla/openvla-7b")),
        "output_name": str(data.get("outputName", "openvla-exp")),
        "epochs": int(data.get("epochs", 1)),
        "batch_size": int(data.get("batchSize", 2)),
        "learning_rate": float(data.get("learningRate", 1e-4)),
        "lora_enabled": bool(data.get("loraEnabled", True)),
    }

    response = requests.post(
        f"{ROUTE_FINETUNE}/finetune/start",
        json=payload,
        timeout=REQUEST_TIMEOUT_SEC,
    )
    response.raise_for_status()
    body = response.json()

    training_args = body.get("training_args", {})
    return {
        "model": {
            "ok": bool(body.get("ok", True)),
            "status": body.get("status", "queued"),
            "jobId": body.get("job_id", ""),
            "baseModel": body.get("base_model", payload["base_model"]),
            "outputName": body.get("output_name", payload["output_name"]),
            "checkpointPath": body.get("checkpoint_path", ""),
            "dataset": {
                "path": payload["dataset_path"],
            },
            "trainingArgs": {
                "epochs": training_args.get("epochs", payload["epochs"]),
                "batchSize": training_args.get("batch_size", payload["batch_size"]),
                "learningRate": training_args.get(
                    "learning_rate",
                    payload["learning_rate"],
                ),
                "loraEnabled": training_args.get(
                    "lora_enabled",
                    payload["lora_enabled"],
                ),
            },
            "service": ROUTE_FINETUNE,
            "raw": body,
        }
    }
