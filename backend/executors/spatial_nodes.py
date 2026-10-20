from __future__ import annotations

import hashlib
import json
import math
import re
import time
import uuid
from pathlib import Path
from typing import Any, Dict, List, Tuple

try:
    from backend.config import UPLOAD_DIR
except Exception:
    UPLOAD_DIR = str(Path.cwd() / "uploads")


DEFAULT_MEMORY_PATH = Path(UPLOAD_DIR) / "spatial_memory" / "spatial_memory.jsonl"


def _incoming_values(
    node_id: str,
    edges: List[Dict[str, Any]],
    context: Dict[str, Any],
) -> Dict[str, Any]:
    result: Dict[str, Any] = {}
    for edge in edges:
        if edge.get("target") != node_id:
            continue
        source_id = edge.get("source")
        source_handle = edge.get("sourceHandle")
        target_handle = edge.get("targetHandle")
        source_payload = context.get(source_id)
        if source_payload is None:
            continue

        value = source_payload
        if source_handle:
            if not isinstance(source_payload, dict) or source_handle not in source_payload:
                continue
            value = source_payload[source_handle]
        result[str(target_handle or source_handle or source_id)] = value
    return result


def _props(node: Dict[str, Any]) -> Dict[str, Any]:
    data = node.get("data") or {}
    return data if isinstance(data, dict) else {}


def _text(value: Any) -> str:
    return str(value or "").strip()


def _to_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _to_bool(value: Any, default: bool = False) -> bool:
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


def _as_dict(value: Any) -> Dict[str, Any]:
    return value if isinstance(value, dict) else {}


def _memory_path(value: Any = None) -> Path:
    text = _text(value)
    if not text:
        return DEFAULT_MEMORY_PATH
    path = Path(text)
    if not path.is_absolute():
        path = Path(UPLOAD_DIR) / path
    return path


def _json_safe(value: Any) -> Any:
    try:
        json.dumps(value)
        return value
    except Exception:
        if isinstance(value, dict):
            return {str(key): _json_safe(item) for key, item in value.items()}
        if isinstance(value, (list, tuple)):
            return [_json_safe(item) for item in value]
        return str(value)


def _position(value: Any) -> List[float]:
    if isinstance(value, dict):
        raw = [value.get("x"), value.get("y"), value.get("z")]
    elif isinstance(value, (list, tuple)):
        raw = list(value)
    else:
        raw = []
    out: List[float] = []
    for item in raw[:3]:
        try:
            out.append(round(float(item), 6))
        except Exception:
            out.append(0.0)
    while len(out) < 3:
        out.append(0.0)
    return out


def _scene_id(scan: Dict[str, Any], fallback: str = "default_scene") -> str:
    value = (
        scan.get("scene_id")
        or scan.get("sceneId")
        or scan.get("memory_namespace")
        or scan.get("stage_identifier")
        or fallback
    )
    text = _text(value)
    if not text:
        text = fallback
    digest = hashlib.sha1(text.encode("utf-8", errors="ignore")).hexdigest()[:10]
    if len(text) > 80 or any(char in text for char in "\\/:*?\"<>|"):
        return f"scene_{digest}"
    return text


def _object_signature(scan: Dict[str, Any]) -> str:
    objects = scan.get("objects")
    if not isinstance(objects, list):
        objects = []
    parts = []
    for item in objects:
        if not isinstance(item, dict):
            continue
        parts.append(
            "|".join(
                [
                    _text(item.get("prim_path") or item.get("primPath")),
                    _text(item.get("name")),
                    _text(item.get("type")),
                ]
            )
        )
    raw = json.dumps(
        {
            "scene_id": scan.get("scene_id") or scan.get("sceneId"),
            "stage_identifier": scan.get("stage_identifier") or scan.get("stageIdentifier"),
            "robot": _as_dict(scan.get("robot")).get("prim_path"),
            "objects": sorted(parts),
        },
        sort_keys=True,
        ensure_ascii=False,
    )
    return hashlib.sha1(raw.encode("utf-8")).hexdigest()


def _tokens(value: Any) -> set[str]:
    text = _text(value)
    text = re.sub(r"(?<=[a-z0-9])(?=[A-Z])", " ", text)
    text = text.replace("_", " ").replace("-", " ").lower()
    return {
        token
        for token in re.findall(r"[\w가-힣]+", text, flags=re.UNICODE)
        if len(token) >= 2
    }


def _target_object_candidate(item: Dict[str, Any]) -> bool:
    type_name = _text(item.get("type")).lower()
    name_text = _object_text(item).lower()
    if type_name in {"distantlight", "sphere_light", "dome_light", "physicsscene", "scope", "material", "shader"}:
        return False
    if any(token in name_text for token in ("physics", "light", "camera", "ground", "looks", "environment")):
        return False
    size = item.get("size")
    if isinstance(size, list) and size:
        try:
            if any(abs(float(value)) > 1.0e6 for value in size):
                return False
        except Exception:
            pass
    return True


def _omniverse_endpoint_candidates(endpoint: str) -> List[str]:
    text = _text(endpoint) or "ws://host.docker.internal:9900/omniverse"
    candidates = [text]
    if "host.docker.internal" in text:
        candidates.extend(
            [
                text.replace("host.docker.internal", "127.0.0.1"),
                text.replace("host.docker.internal", "localhost"),
            ]
        )
    elif "127.0.0.1" in text:
        candidates.append(text.replace("127.0.0.1", "host.docker.internal"))
    elif "localhost" in text:
        candidates.append(text.replace("localhost", "host.docker.internal"))

    unique: List[str] = []
    for candidate in candidates:
        if candidate and candidate not in unique:
            unique.append(candidate)
    return unique


def _object_text(item: Dict[str, Any]) -> str:
    return " ".join(
        _text(item.get(key))
        for key in ("name", "label", "prim_path", "primPath", "type")
        if _text(item.get(key))
    )


def _choose_target_object(scan: Dict[str, Any], prompt: str) -> Dict[str, Any]:
    objects = scan.get("objects")
    if not isinstance(objects, list):
        return {}
    prompt_tokens = _tokens(prompt)
    best: Tuple[float, Dict[str, Any]] = (0.0, {})
    fallback: Dict[str, Any] = {}
    for item in objects:
        if not isinstance(item, dict):
            continue
        if not _target_object_candidate(item):
            continue
        if not fallback:
            fallback = item
        object_tokens = _tokens(_object_text(item))
        overlap = len(prompt_tokens.intersection(object_tokens))
        score = float(overlap)
        if prompt_tokens and _text(item.get("label") or item.get("name")).lower().replace("_", "").replace("-", "") in {
            "".join(token for token in prompt_tokens if token.isascii()),
            "".join(sorted(token for token in prompt_tokens if token.isascii())),
        }:
            score += 2.0
        if score > best[0]:
            best = (score, item)
    if best[1]:
        return best[1]
    return fallback


def _approach_waypoint(robot: Dict[str, Any], obj: Dict[str, Any]) -> List[float]:
    robot_pos = _position(robot.get("position"))
    object_pos = _position(obj.get("position") or obj.get("bbox_center"))
    dx = object_pos[0] - robot_pos[0]
    dy = object_pos[1] - robot_pos[1]
    distance = math.sqrt(dx * dx + dy * dy)
    if distance <= 1.0e-6:
        return [object_pos[0], object_pos[1], 0.0]
    stand_off = min(0.8, max(0.25, distance * 0.35))
    scale = max(0.0, (distance - stand_off) / distance)
    yaw = math.atan2(dy, dx)
    return [
        round(robot_pos[0] + dx * scale, 6),
        round(robot_pos[1] + dy * scale, 6),
        round(yaw, 6),
    ]


def _record_text(scene_id: str, obj: Dict[str, Any], waypoint: List[float], prompt: str) -> str:
    name = _text(obj.get("label") or obj.get("name") or obj.get("prim_path") or "object")
    pos = _position(obj.get("position") or obj.get("bbox_center"))
    task = _text(prompt)
    prefix = f"In scene {scene_id}, {name} is at x={pos[0]} y={pos[1]} z={pos[2]}."
    waypoint_text = f" Stored approach waypoint: [{waypoint[0]}, {waypoint[1]}, {waypoint[2]}]."
    return f"{prefix}{waypoint_text}" + (f" Task: {task}" if task else "")


def _context_record_text(record: Dict[str, Any]) -> str:
    record = normalize_memory_record(record)
    metadata = _as_dict(record.get("metadata"))
    obj = _text(metadata.get("object") or metadata.get("target_object"))
    pos = metadata.get("object_position")
    waypoint = metadata.get("approach_waypoint")
    if obj or pos or waypoint:
        parts = []
        if obj:
            parts.append(f"object={obj}")
        if pos is not None:
            parts.append(f"object_position={pos}")
        if waypoint is not None:
            parts.append(f"stored_approach_waypoint={waypoint}")
            if isinstance(waypoint, list) and len(waypoint) >= 3:
                parts.append("waypoint_validation=unverified; revalidate before execution")
        parts.append(f"record_kind={record['record_kind']}; outcome={record['outcome']}")
        source = _text(metadata.get("source"))
        if source:
            parts.append(f"source={source}")
        parts.append(f"coordinate_frame={metadata.get('coordinate_frame', 'stage_unknown')}")
        parts.append(f"meters_per_unit={metadata.get('meters_per_unit')}; up_axis={metadata.get('up_axis')}")
        return "; ".join(parts)

    text = _text(record.get("text"))
    text = re.sub(
        r"\bApproach\s+waypoint\s+is\b",
        "Stored approach waypoint:",
        text,
        flags=re.IGNORECASE,
    )
    return f"{text}; record_kind={record['record_kind']}; outcome={record['outcome']}"


def _scan_context_record(scan: Dict[str, Any], prompt: str) -> Dict[str, Any]:
    if not scan:
        return {}
    obj = _choose_target_object(scan, prompt)
    if not obj:
        return {}
    scene_id = _scene_id(scan, "unknown_scene")
    robot = _as_dict(scan.get("robot"))
    waypoint = _approach_waypoint(robot, obj)
    text = _record_text(scene_id, obj, waypoint, prompt)
    return {
        "id": f"{scene_id}_live_scan",
        "text": text,
        "metadata": {
            "scene_id": scene_id,
            "object": _text(obj.get("label") or obj.get("name") or obj.get("prim_path")),
            "object_position": _position(obj.get("position") or obj.get("bbox_center")),
            "approach_waypoint": waypoint,
            "source": "live_scene_scan",
            "coordinate_frame": "isaac_stage",
            "meters_per_unit": scan.get("meters_per_unit"),
            "up_axis": scan.get("up_axis"),
            "target_object": _text(obj.get("label") or obj.get("name")),
        },
        "scan": _json_safe(scan),
        "score": 999.0,
        "score_debug": {"score": 999.0, "reasons": ["live_scene_scan"]},
    }


def normalize_memory_record(record: Dict[str, Any]) -> Dict[str, Any]:
    """Interpret old records without treating an acknowledgement as task success.

    Experience evidence is supplied by the completion/evaluation producer:
    result.status=completed, result.actual_state and result.evaluation containing
    verified=true and outcome=success|failure. A legacy success flag is ignored.
    This is a read adapter; the original JSONL file is never migrated in place.
    """
    result = _as_dict(record.get("result"))
    evaluation = _as_dict(result.get("evaluation"))
    verified = bool(
        _as_dict(record.get("scan"))
        and _as_dict(record.get("scan")).get("ok") is not False
        and _as_dict(record.get("action"))
        and result.get("status") == "completed"
        and _as_dict(result.get("actual_state"))
        and _as_dict(result.get("actual_state")).get("ok") is not False
        and result.get("ok") is not False
        and evaluation.get("verified") is True
        and evaluation.get("outcome") in ("success", "failure")
    )
    outcome = evaluation["outcome"] if verified else "unknown"
    return {
        **record,
        "schema_version": 2,
        "record_kind": "experience" if verified else "observation",
        "outcome": outcome,
        "metadata": {
            **_as_dict(record.get("metadata")),
            "success": outcome == "success" if verified else None,
            "outcome": outcome,
            "outcome_verified": verified,
        },
    }


def _read_records(path: Path, *, normalize: bool = True) -> List[Dict[str, Any]]:
    if not path.exists():
        return []
    records: List[Dict[str, Any]] = []
    for line in path.read_text(encoding="utf-8-sig").splitlines():
        line = line.lstrip("\ufeff")
        if not line.strip():
            continue
        try:
            item = json.loads(line)
        except Exception:
            continue
        if isinstance(item, dict):
            records.append(normalize_memory_record(item) if normalize else item)
    return records


def _write_records(path: Path, records: List[Dict[str, Any]]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    payload = "\n".join(json.dumps(item, ensure_ascii=False) for item in records)
    path.write_text(payload + ("\n" if payload else ""), encoding="utf-8")


def _extract_scan(inputs: Dict[str, Any], props: Dict[str, Any]) -> Dict[str, Any]:
    for key in ("scan", "scene", "state"):
        value = inputs.get(key)
        if isinstance(value, dict):
            if isinstance(value.get("scan"), dict):
                return {**value["scan"], **({"ok": False} if value.get("ok") is False else {})}
            return value
    value = props.get("scan")
    return value if isinstance(value, dict) else {}


def normalize_scene_scan_response(response: Dict[str, Any], fallback_scene_id: str = "unknown_scene") -> Dict[str, Any]:
    scan = _as_dict(response.get("scan"))
    if not scan and isinstance(response.get("response"), dict):
        scan = _as_dict(response["response"].get("scan"))
    if not scan:
        scan = {
            **response,
            "scene_id": _text(response.get("scene_id") or response.get("sceneId") or fallback_scene_id),
            "robot": _as_dict(response.get("robot")),
            "objects": response.get("objects") if isinstance(response.get("objects"), list) else [],
            "obstacles": response.get("obstacles") if isinstance(response.get("obstacles"), list) else [],
        }

    objects = scan.get("objects") if isinstance(scan.get("objects"), list) else []
    robot = _as_dict(scan.get("robot"))
    obstacles = scan.get("obstacles") if isinstance(scan.get("obstacles"), list) else []
    normalized = {
        **scan,
        "scene_id": _scene_id(scan, fallback_scene_id),
        "robot": robot,
        "objects": objects,
        "obstacles": obstacles,
    }
    if response.get("ok") is False:
        normalized["ok"] = False
    normalized.setdefault("object_count", len(objects))
    normalized["observation_quality"] = inspect_scene_scan(normalized)
    return normalized


def inspect_scene_scan(scan: Dict[str, Any]) -> Dict[str, Any]:
    """Report missing or ambiguous data; do not invent units or free space."""
    objects = scan.get("objects") if isinstance(scan.get("objects"), list) else []
    seen: set[str] = set()
    duplicate_paths: List[str] = []
    invalid_positions: List[str] = []
    for index, item in enumerate(objects):
        item = _as_dict(item)
        path = _text(item.get("prim_path"))
        if path and path in seen:
            duplicate_paths.append(path)
        if path:
            seen.add(path)
        pos = item.get("position")
        if not isinstance(pos, (list, tuple)) or len(pos) != 3 or any(
            isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value)
            for value in pos
        ):
            invalid_positions.append(path or str(index))
    units = scan.get("meters_per_unit")
    units_known = (
        isinstance(units, (int, float)) and not isinstance(units, bool)
        and math.isfinite(units) and units > 0
    )
    return {
        "units_known": units_known,
        "up_axis_known": scan.get("up_axis") in ("Y", "Z"),
        "sim_time_known": (
            isinstance(scan.get("sim_time"), (int, float))
            and not isinstance(scan.get("sim_time"), bool)
            and math.isfinite(scan["sim_time"]) and scan["sim_time"] >= 0
        ),
        "coverage": "partial" if scan.get("truncated") is True else "unknown",
        "duplicate_prim_paths": duplicate_paths,
        "invalid_object_positions": invalid_positions,
    }


def exec_scene_scan(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    from websocket import create_connection

    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    state = _as_dict(inputs.get("state") or props.get("state"))
    endpoint = _text(
        props.get("omniverseEndpoint")
        or state.get("omniverseEndpoint")
        or "ws://host.docker.internal:9900/omniverse"
    )
    timeout_sec = _to_float(props.get("timeoutSec") or props.get("omniverseTimeoutSec"), 15.0)
    state_prim_path = _text(state.get("prim_path") or state.get("primPath"))
    configured_prim_path = _text(props.get("omniversePrimPath") or props.get("robotPrimPath"))
    if configured_prim_path == "/World/KyleRobot" and state_prim_path:
        configured_prim_path = ""
    prim_path = _text(configured_prim_path or state_prim_path or "/World/GR00T_Humanoid")
    request_id = f"scene-scan-{uuid.uuid4().hex}"
    request = {
        "type": "omniverse_command",
        "request_id": request_id,
        "command": "scan_scene",
        "prim_path": prim_path,
        "root_path": _text(props.get("rootPath") or props.get("sceneRootPath") or "/World"),
        "scene_id": _text(props.get("sceneId") or props.get("scene_id")),
        "max_objects": _to_int(props.get("maxObjects"), 256),
    }
    attempted_errors: List[str] = []
    for candidate in _omniverse_endpoint_candidates(endpoint):
        ws = None
        try:
            ws = create_connection(candidate, timeout=timeout_sec)
            ws.settimeout(max(timeout_sec, 15.0))
            ws.send(json.dumps(request, ensure_ascii=False))
            response = json.loads(ws.recv())
            if not isinstance(response, dict):
                response = {"ok": False, "message": f"Unexpected response: {type(response).__name__}"}
            if (
                "scan" not in response
                and not isinstance(response.get("objects"), list)
                and _text(response.get("command")).lower() == "scan_scene"
            ):
                return {
                    "ok": False,
                    "type": "scene_scan_result",
                    "endpoint": candidate,
                    "request": request,
                    "response": response,
                    "scan": {"ok": False, "scene_id": "unknown_scene", "robot": {}, "objects": [], "obstacles": []},
                    "objects": [],
                    "robot": {},
                    "scene_id": "unknown_scene",
                    "message": (
                        "Isaac Sim bridge did not return a scene scan. "
                        "Reload examples/isaacsim/VlaGraphIsaacSimBridge.py inside Isaac Sim."
                    ),
                }
            scan = normalize_scene_scan_response(
                response,
                fallback_scene_id=_text(props.get("sceneId") or "unknown_scene"),
            )
            return {
                "ok": bool(response.get("ok", response.get("applied", True))),
                "type": "scene_scan_result",
                "endpoint": candidate,
                "request": request,
                "response": response,
                "scan": scan,
                "objects": scan.get("objects", []),
                "robot": scan.get("robot", {}),
                "scene_id": scan.get("scene_id"),
                "message": response.get("message", f"Scene scan returned {len(scan.get('objects', []))} objects."),
            }
        except Exception as exc:
            attempted_errors.append(f"{candidate} -> {exc}")
        finally:
            if ws is not None:
                try:
                    ws.close()
                except Exception:
                    pass
    return {
        "ok": False,
        "type": "scene_scan_result",
        "endpoint": endpoint,
        "request": request,
        "scan": {"ok": False, "scene_id": "unknown_scene", "robot": {}, "objects": [], "obstacles": []},
        "objects": [],
        "robot": {},
        "scene_id": "unknown_scene",
        "message": "Scene scan failed: " + "; ".join(attempted_errors),
    }


def exec_scene_context_reset(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    scan = _extract_scan(inputs, props)
    if scan.get("ok") is False:
        return {"ok": False, "type": "scene_context_reset_result", "scene": scan,
                "message": "Scene scan failed; scene context was not updated."}
    current_scene_id = _scene_id(scan, _text(props.get("sceneId") or "default_scene"))
    signature = _object_signature(scan)
    previous = _as_dict(context.get("_spatial_scene_context"))
    previous_signature = _text(previous.get("signature"))
    previous_scene_id = _text(previous.get("scene_id"))
    changed = bool(previous_signature and previous_signature != signature)
    namespace = current_scene_id
    reset = {
        "scene_changed": changed,
        "previous_scene_id": previous_scene_id,
        "current_scene_id": current_scene_id,
        "reset_applied": changed,
        "memory_namespace": namespace,
        "signature": signature,
        "message": "Scene changed; runtime spatial cache reset." if changed else "Scene context unchanged.",
    }
    context["_spatial_scene_context"] = {
        "scene_id": current_scene_id,
        "signature": signature,
        "memory_namespace": namespace,
        "updated_at": time.time(),
    }
    if changed:
        context["_spatial_retrieval_cache"] = {}

    if _to_bool(props.get("clearSceneMemory"), False):
        path = _memory_path(props.get("memoryPath"))
        kept = [
            record
            for record in _read_records(path, normalize=False)
            if _text(_as_dict(record.get("metadata")).get("scene_id")) != current_scene_id
        ]
        _write_records(path, kept)
        reset["cleared_scene_memory"] = True

    scene = {
        **scan,
        "scene_id": current_scene_id,
        "memory_namespace": namespace,
        "signature": signature,
    }
    return {
        "ok": True,
        "type": "scene_context_reset_result",
        "scene": scene,
        "scene_id": current_scene_id,
        "reset": reset,
        "message": reset["message"],
    }


def exec_spatial_memory_save(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    scan = _extract_scan(inputs, props)
    if scan.get("ok") is False:
        return {"ok": False, "type": "spatial_memory_record", "skipped": True,
                "record": {}, "message": "Scene scan failed; memory was not saved."}
    prompt = _text(inputs.get("prompt") or props.get("prompt"))
    action = inputs.get("action") or props.get("action") or {}
    result = inputs.get("result") or props.get("result") or {}
    robot = _as_dict(scan.get("robot"))
    obj = _choose_target_object(scan, prompt)
    scene_id = _scene_id(scan, _text(props.get("sceneId") or "default_scene"))
    waypoint = _approach_waypoint(robot, obj) if obj else [0.0, 0.0, 0.0]
    if not obj and not _to_bool(props.get("allowEmptySceneMemory"), False):
        return {
            "ok": True,
            "type": "spatial_memory_record",
            "skipped": True,
            "record": {},
            "dataset": str(_memory_path(props.get("memoryPath"))),
            "message": (
                f"Skipped spatial memory save for scene={scene_id}: "
                "scene scan returned no target objects."
            ),
        }
    text = _text(props.get("text")) or _record_text(scene_id, obj, waypoint, prompt)
    metadata = {
        "scene_id": scene_id,
        "object": _text(obj.get("label") or obj.get("name") or obj.get("prim_path")),
        "object_position": _position(obj.get("position") or obj.get("bbox_center")),
        "approach_waypoint": waypoint,
        "source": "isaac_sim",
        "coordinate_frame": "isaac_stage",
        "stage_identifier": scan.get("stage_identifier"),
        "meters_per_unit": scan.get("meters_per_unit"),
        "up_axis": scan.get("up_axis"),
        "error": _text(_as_dict(result).get("error")),
        "action_type": _text(_as_dict(action).get("command") or _as_dict(action).get("type")),
        "target_object": _text(obj.get("label") or obj.get("name")),
        "observed_at": time.time(),
    }
    record = normalize_memory_record({
        "id": _text(props.get("recordId")) or f"{scene_id}_{uuid.uuid4().hex[:12]}",
        "text": text,
        "metadata": metadata,
        "scan": _json_safe(scan),
        "action": _json_safe(action),
        "result": _json_safe(result),
    })
    path = _memory_path(props.get("memoryPath"))
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as handle:
        handle.write(json.dumps(record, ensure_ascii=False) + "\n")
    return {
        "ok": True,
        "type": "spatial_memory_record",
        "record": record,
        "dataset": str(path),
        "message": f"Saved spatial memory record for scene={scene_id}.",
    }


def _record_score(record: Dict[str, Any], prompt: str, scene_id: str, robot_position: List[float]) -> Tuple[float, Dict[str, Any]]:
    record = normalize_memory_record(record)
    metadata = _as_dict(record.get("metadata"))
    score = 0.0
    reasons: List[str] = []
    if scene_id and _text(metadata.get("scene_id")) == scene_id:
        score += 3.0
        reasons.append("scene_id")
    prompt_tokens = _tokens(prompt)
    object_tokens = _tokens(metadata.get("object")) | _tokens(metadata.get("target_object"))
    text_tokens = _tokens(record.get("text"))
    object_overlap = len(prompt_tokens.intersection(object_tokens))
    task_overlap = len(prompt_tokens.intersection(text_tokens))
    if object_overlap:
        score += 2.0 * object_overlap
        reasons.append("object_keyword")
    if task_overlap:
        score += 1.0 * task_overlap
        reasons.append("task_keyword")
    if record["outcome"] == "success" and metadata["outcome_verified"]:
        score += 1.0
        reasons.append("success")
    waypoint = metadata.get("approach_waypoint")
    if isinstance(waypoint, list) and len(waypoint) >= 2:
        dx = float(waypoint[0]) - robot_position[0]
        dy = float(waypoint[1]) - robot_position[1]
        distance = math.sqrt(dx * dx + dy * dy)
        score -= min(2.0, distance * 0.05)
        reasons.append("distance")
    return score, {"score": round(score, 6), "reasons": reasons}


def exec_spatial_rag_retrieve(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    prompt = _text(inputs.get("prompt") or props.get("prompt"))
    scan = _extract_scan(inputs, props)
    scene_context = _as_dict(context.get("_spatial_scene_context"))
    scene_id = _text(inputs.get("scene_id") or props.get("sceneId") or scene_context.get("scene_id") or _scene_id(scan, ""))
    robot_position = _position(_as_dict(scan.get("robot")).get("position"))
    top_k = max(1, _to_int(props.get("topK") or props.get("top_k"), 5))
    records = _read_records(_memory_path(props.get("memoryPath")))
    scored: List[Tuple[float, Dict[str, Any], Dict[str, Any]]] = []
    for record in records:
        score, debug = _record_score(record, prompt, scene_id, robot_position)
        if score <= 0.0 and scene_id and _text(_as_dict(record.get("metadata")).get("scene_id")) != scene_id:
            continue
        scored.append((score, record, debug))
    scored.sort(key=lambda item: item[0], reverse=True)
    memories = []
    for score, record, debug in scored[:top_k]:
        item = dict(record)
        item["score"] = round(score, 6)
        item["score_debug"] = debug
        memories.append(item)
    context["_spatial_retrieval_cache"] = {"scene_id": scene_id, "memories": memories, "prompt": prompt}
    return {
        "ok": True,
        "type": "spatial_rag_retrieve_result",
        "memories": memories,
        "records": memories,
        "scene_id": scene_id,
        "query": prompt,
        "top_k": top_k,
        "message": f"Retrieved {len(memories)} spatial memories for scene={scene_id or '<any>'}.",
    }


def exec_spatial_context_builder(node: Dict[str, Any], context: Dict[str, Any], edges: List[Dict[str, Any]]) -> Dict[str, Any]:
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    prompt = _text(inputs.get("prompt") or props.get("prompt"))
    scan = _extract_scan(inputs, props)
    memories_value = inputs.get("memories") or inputs.get("records") or props.get("memories")
    if isinstance(memories_value, dict):
        memories = memories_value.get("memories") or memories_value.get("records") or []
    else:
        memories = memories_value
    if not isinstance(memories, list):
        memories = []
    max_lines = max(1, min(8, _to_int(props.get("maxLines"), 5)))
    lines: List[str] = []
    context_records: List[Dict[str, Any]] = []
    live_record = _scan_context_record(scan, prompt)
    if live_record:
        context_records.append(normalize_memory_record(live_record))
    for record in memories:
        if not isinstance(record, dict):
            continue
        if live_record and _text(record.get("id")) == _text(live_record.get("id")):
            continue
        context_records.append(normalize_memory_record(record))
        if len(context_records) >= max_lines:
            break

    for record in context_records[:max_lines]:
        if not isinstance(record, dict):
            continue
        text = _context_record_text(record)
        lines.append(f"- {text}")
    if lines:
        built = "Known spatial context:\n" + "\n".join(lines)
        if prompt:
            built = f"{built}\n\nUser task:\n{prompt}"
    else:
        built = prompt
    return {
        "ok": True,
        "type": "spatial_context",
        "text": built,
        "context": built,
        "memories": context_records[:max_lines],
        "prompt": prompt,
        "message": f"Built spatial context with {len(lines)} memory lines.",
    }
