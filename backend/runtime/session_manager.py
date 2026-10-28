from __future__ import annotations

import asyncio
import copy
import os
from pathlib import Path
import subprocess
import threading
import time
from typing import Any, Dict, List
from uuid import uuid4

import requests

from backend import state_store
from backend.runtime.runner import GraphRunCancelled, run_graph


FINAL_SESSION_STATES = {"completed", "cancelled", "error"}
AMR_TERMINAL_STATES = {"completed", "cancelled", "failed", "blocked"}
DEFAULT_UNITY_WAYPOINT_SPEED = 1.5
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
LEGACY_URL_REWRITES = {
    ":8000": ":9000",
    ":8040": ":9040",
    ":8060": ":9060",
    ":8061": ":9061",
    ":8081": ":9081",
    ":8090": ":9090",
    ":8091": ":9091",
    ":8092": ":9092",
    ":8899": ":9900",
    ":5173": ":9173",
}
URL_LIKE_NODE_DATA_KEYS = {
    "unityEndpoint",
    "omniverseEndpoint",
    "rosBridgeUrl",
    "remoteImageUrl",
    "remoteLidarUrl",
    "remoteRgbdUrl",
    "remoteRGBDUrl",
    "imageUrl",
}
REALSENSE_BRIDGE_SERVICE = "realsense_d435i_bridge"
REALSENSE_BRIDGE_DEFAULT_URL = "http://host.docker.internal:9180/frame"


def _now() -> float:
    return time.time()


def _coerce_bool(value: Any) -> bool | None:
    if value is None:
        return None
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "y", "on"}:
            return True
        if normalized in {"0", "false", "no", "n", "off"}:
            return False
    return bool(value)


def _coerce_int(value: Any, default: int) -> int:
    try:
        return int(value)
    except Exception:
        return default


def _coerce_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _trim_logs(lines: List[str], limit: int) -> List[str]:
    if limit <= 0:
        return lines
    if len(lines) <= limit:
        return lines
    return lines[-limit:]


def _text_value(value: Any) -> str:
    return str(value or "").strip()


def _looks_like_manipulation_prompt(text: str) -> bool:
    normalized = _text_value(text).lower()
    if not normalized:
        return False
    return any(marker in normalized for marker in MANIPULATION_PROMPT_MARKERS)


def _prompt_texts(graph: Dict[str, Any]) -> List[str]:
    prompts: List[str] = []
    for node in graph.get("nodes", []):
        if not isinstance(node, dict) or node.get("type") != "prompt":
            continue
        data = node.get("data", {}) or {}
        text = _text_value(data.get("text"))
        if text:
            prompts.append(text)
    return prompts


def _graph_has_g1_gr00t_infer(nodes: List[Dict[str, Any]]) -> bool:
    for node in nodes:
        if not isinstance(node, dict) or node.get("type") != "gr00t_infer":
            continue
        data = node.get("data", {}) or {}
        if not isinstance(data, dict):
            continue
        model_name = _text_value(data.get("modelName")).lower()
        embodiment_tag = _text_value(data.get("embodimentTag")).upper()
        if embodiment_tag == "REAL_G1" or "gr00t" in model_name:
            return True
    return False


def _looks_like_isaacsim_g1_execute(data: Dict[str, Any]) -> bool:
    prim_path = _text_value(data.get("omniversePrimPath"))
    unity_object = _text_value(data.get("unityObject"))
    prim_name = prim_path.rsplit("/", 1)[-1]
    return (
        prim_path.endswith("/GR00T_Humanoid")
        or prim_path.endswith("/GROOT_Humanoid")
        or prim_path.endswith("/G1")
        or unity_object == "GR00T_Humanoid"
        or unity_object == "GROOT_Humanoid"
        or unity_object == "G1"
        or prim_name in {"GR00T_Humanoid", "GROOT_Humanoid", "G1"}
    )


def _isaacsim_g1_omniverse_execute_nodes(graph: Dict[str, Any]) -> List[Dict[str, Any]]:
    execute_nodes: List[Dict[str, Any]] = []
    for node in graph.get("nodes", []):
        if not isinstance(node, dict) or node.get("type") != "robot_execute":
            continue
        data = node.get("data", {}) or {}
        if not isinstance(data, dict):
            continue
        if _text_value(data.get("mode")).lower() != "omniverse":
            continue
        if not _looks_like_isaacsim_g1_execute(data):
            continue
        execute_nodes.append(node)
    return execute_nodes


def _g1_stop_action(reason: str) -> Dict[str, Any]:
    return {
        "type": "humanoid_motion_action",
        "command": "stop",
        "robot_type": "humanoid",
        "controller_mode": "locomotion",
        "skill": "stop",
        "gesture": "none",
        "move_x": 0.0,
        "move_z": 0.0,
        "turn_yaw": 0.0,
        "speed": 0.0,
        "duration_sec": 0.0,
        "animation": False,
        "stop": True,
        "terminate": True,
        "freeze": True,
        "retarget_mode": "stop",
        "stop_reason": reason,
        "pose_sequence": [],
    }


def _looks_like_default_realsense_bridge_url(value: Any) -> bool:
    text = _text_value(value).lower()
    return (
        "host.docker.internal:9180" in text
        and (text.endswith("/frame") or "/frame?" in text)
    )


def _realsense_bridge_health_url(frame_url: str) -> str:
    text = _text_value(frame_url) or REALSENSE_BRIDGE_DEFAULT_URL
    if text.endswith("/frame"):
        return text[: -len("/frame")] + "/health"
    if "/frame?" in text:
        return text.split("/frame?", 1)[0] + "/health"
    return text.rstrip("/") + "/health"


def _realsense_bridge_host_candidates(url: str) -> List[str]:
    text = _text_value(url) or REALSENSE_BRIDGE_DEFAULT_URL
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


def _realsense_bridge_is_ready(frame_url: str, timeout_sec: float = 0.7) -> bool:
    for candidate in _realsense_bridge_host_candidates(frame_url):
        try:
            response = requests.get(_realsense_bridge_health_url(candidate), timeout=timeout_sec)
            if not response.ok:
                continue
            try:
                payload = response.json()
            except Exception:
                payload = {}
            source = _text_value(payload.get("source")).lower() if isinstance(payload, dict) else ""
            if source == "mock" and _coerce_bool(os.getenv("REALSENSE_BRIDGE_ACCEPT_MOCK", "false")) is not True:
                continue
            return True
        except Exception:
            continue
    return False


def _realsense_bridge_urls(graph: Dict[str, Any]) -> List[str]:
    urls: List[str] = []
    for node in graph.get("nodes", []):
        if not isinstance(node, dict) or node.get("type") != "realsense_d435i_sensor":
            continue
        data = node.get("data", {}) or {}
        if not isinstance(data, dict):
            data = {}
        url = _text_value(data.get("remoteRgbdUrl") or data.get("remoteRGBDUrl"))
        if not url:
            url = REALSENSE_BRIDGE_DEFAULT_URL
        if url not in urls:
            urls.append(url)
    return urls


def _running_in_container() -> bool:
    return Path("/.dockerenv").exists() or Path("/run/.containerenv").exists()


def _ensure_realsense_bridge_for_graph(graph: Dict[str, Any]) -> List[str]:
    urls = _realsense_bridge_urls(graph)
    if not urls:
        return []

    logs: List[str] = []
    if any(_realsense_bridge_is_ready(url) for url in urls):
        logs.append("[GRAPH] RealSense D435i bridge is already reachable on port 9180.")
        return logs

    if _coerce_bool(os.getenv("REALSENSE_BRIDGE_AUTO_START", "true")) is not True:
        logs.append("[GRAPH WARNING] RealSense D435i bridge is not reachable and auto-start is disabled.")
        return logs

    if _running_in_container():
        logs.append(
            "[GRAPH WARNING] RealSense D435i bridge is not reachable. "
            "Because backend is inside Docker it cannot start the Windows USB camera directly. "
            "Run this on the Windows host, then run the graph again: "
            r"powershell -ExecutionPolicy Bypass -File scripts\start_realsense_host_bridge.ps1 -NoMockFallback"
        )
        return logs

    compose_file = Path(
        os.getenv(
            "REALSENSE_BRIDGE_COMPOSE_FILE",
            str(Path(__file__).resolve().parents[2] / "docker-compose.yml"),
        )
    )
    service_name = os.getenv("REALSENSE_BRIDGE_SERVICE", REALSENSE_BRIDGE_SERVICE)
    command = ["docker", "compose", "-f", str(compose_file), "up", "-d", service_name]
    try:
        result = subprocess.run(
            command,
            check=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            timeout=max(5.0, _coerce_float(os.getenv("REALSENSE_BRIDGE_START_TIMEOUT_SEC"), 20.0)),
        )
        output = " ".join((result.stdout or result.stderr or "").split())
        logs.append(
            f"[GRAPH] Started RealSense D435i bridge via docker compose service '{service_name}'."
            + (f" {output}" if output else "")
        )
        time.sleep(1.0)
        if any(_realsense_bridge_is_ready(url, timeout_sec=1.5) for url in urls):
            logs.append("[GRAPH] RealSense D435i bridge is reachable after auto-start.")
        else:
            logs.append("[GRAPH WARNING] RealSense D435i bridge auto-started, but /health is not reachable yet.")
    except Exception as exc:
        logs.append(f"[GRAPH WARNING] Failed to auto-start RealSense D435i bridge: {exc}")
    return logs


def _rewrite_legacy_url_text(value: Any) -> str:
    rewritten = str(value or "").strip()
    for old, new in LEGACY_URL_REWRITES.items():
        rewritten = rewritten.replace(old, new)
    return rewritten


def prepare_graph_for_execution(graph: Dict[str, Any]) -> tuple[Dict[str, Any], List[str]]:
    normalized_graph = copy.deepcopy(graph or {})
    nodes = normalized_graph.get("nodes", [])
    edges = normalized_graph.get("edges", [])
    prompt_texts = _prompt_texts(normalized_graph)
    has_manipulation_prompt = any(
        _looks_like_manipulation_prompt(text) for text in prompt_texts
    )
    has_g1_gr00t_infer = _graph_has_g1_gr00t_infer(nodes)
    has_rgbd_follow_nodes = any(
        isinstance(node, dict)
        and node.get("type") in {"realsense_d435i_sensor", "mediapipe_pose", "rgbd_pose_fusion"}
        for node in nodes
    )
    has_motion_learning_nodes = any(
        isinstance(node, dict)
        and node.get("type") in {"video_motion_input", "motion_extract"}
        for node in nodes
    )
    has_isaacsim_g1_execute = any(
        isinstance(node, dict)
        and node.get("type") == "robot_execute"
        and _text_value((node.get("data", {}) or {}).get("mode")).lower() == "omniverse"
        and _looks_like_isaacsim_g1_execute(node.get("data", {}) or {})
        for node in nodes
    )
    has_humanoid_pose_pipeline = has_rgbd_follow_nodes or has_motion_learning_nodes
    humanoid_pose_pipeline_label = (
        "Isaac Sim RGB-D follow"
        if has_rgbd_follow_nodes
        else "Isaac Sim motion learning"
    )
    node_map = {
        str(node.get("id")): node
        for node in nodes
        if isinstance(node, dict) and str(node.get("id") or "").strip()
    }
    adjacency: Dict[str, List[str]] = {}
    for edge in edges:
        if not isinstance(edge, dict):
            continue
        source = str(edge.get("source") or "").strip()
        target = str(edge.get("target") or "").strip()
        if not source or not target:
            continue
        adjacency.setdefault(source, []).append(target)

    def find_downstream_unity_execute(node_id: str) -> Dict[str, Any] | None:
        queue = list(adjacency.get(node_id, []))
        visited = {node_id}

        while queue:
            current_id = queue.pop(0)
            if current_id in visited:
                continue
            visited.add(current_id)

            current_node = node_map.get(current_id)
            if not isinstance(current_node, dict):
                continue

            if current_node.get("type") == "robot_execute":
                data = current_node.get("data", {}) or {}
                mode = _text_value(data.get("mode")).lower() or "simulate"
                if mode == "unity":
                    return current_node

            queue.extend(adjacency.get(current_id, []))

        return None

    prepare_logs: List[str] = []

    for node in nodes:
        if not isinstance(node, dict):
            continue
        data = node.get("data", {})
        if not isinstance(data, dict):
            continue

        for key in URL_LIKE_NODE_DATA_KEYS:
            current = data.get(key)
            if not isinstance(current, str) or not current.strip():
                continue
            rewritten = _rewrite_legacy_url_text(current)
            if rewritten == current:
                continue
            data[key] = rewritten
            prepare_logs.append(
                f"[GRAPH] Rewrote legacy endpoint on {_text_value(node.get('id'))} "
                f"({node.get('type')}): {key}={rewritten}"
            )

        if node.get("type") != "robot_execute":
            if (
                node.get("type") == "realsense_d435i_sensor"
                and _looks_like_default_realsense_bridge_url(data.get("remoteRgbdUrl"))
                and _coerce_bool(data.get("mockWhenUnavailable")) is not False
            ):
                data["mockWhenUnavailable"] = False
                prepare_logs.append(
                    f"[GRAPH] Disabled mock fallback on {_text_value(node.get('id'))} "
                    "(realsense_d435i_sensor) so RealSense camera failures are visible."
                )
            if (
                node.get("type") == "rgbd_pose_fusion"
                and _text_value(data.get("motionLabel")).lower() == "rgbd_follow"
            ):
                data["motionLabel"] = ""
                prepare_logs.append(
                    f"[GRAPH] Cleared stale rgbd_follow motionLabel on {_text_value(node.get('id'))} "
                    "so motion can be inferred from pose history."
                )
            if node.get("type") == "video_motion_input" and has_motion_learning_nodes:
                stale_label = _text_value(data.get("motionLabel")).lower()
                if stale_label in {"walk_forward_turn_left", "walk_like", "rgbd_follow"}:
                    data["motionLabel"] = ""
                    prepare_logs.append(
                        f"[GRAPH] Cleared stale motionLabel on {_text_value(node.get('id'))} "
                        "so the uploaded video pose can drive retargeting."
                    )
            if node.get("type") == "motion_extract" and has_motion_learning_nodes:
                extract_changed = False
                if _coerce_int(data.get("everyNFrames"), 2) != 1:
                    data["everyNFrames"] = 1
                    extract_changed = True
                if _coerce_float(data.get("minVisibility"), 0.35) > 0.16:
                    data["minVisibility"] = 0.15
                    extract_changed = True
                if extract_changed:
                    prepare_logs.append(
                        f"[GRAPH] Relaxed Motion Extract sampling on {_text_value(node.get('id'))} "
                        "(motion_extract) so Motion Learning produces enough trajectory frames."
                    )
            if node.get("type") == "rgbd_pose_fusion" and has_rgbd_follow_nodes:
                rgbd_changed = False
                if _coerce_float(data.get("minPoseVisibility"), 0.35) < 0.55:
                    data["minPoseVisibility"] = 0.55
                    rgbd_changed = True
                landmark_smoothing = _coerce_float(data.get("landmarkSmoothing"), 0.0)
                if landmark_smoothing < 0.30 or landmark_smoothing > 0.65:
                    data["landmarkSmoothing"] = 0.45
                    rgbd_changed = True
                landmark_max_delta = _coerce_float(data.get("landmarkMaxDelta"), 0.0)
                if landmark_max_delta <= 0.0 or landmark_max_delta > 0.24:
                    data["landmarkMaxDelta"] = 0.18
                    rgbd_changed = True
                if _coerce_int(data.get("maxMotionFrames"), 40) != 1:
                    data["maxMotionFrames"] = 1
                    rgbd_changed = True
                if _coerce_bool(data.get("holdLargeJumps")) is not True:
                    data["holdLargeJumps"] = True
                    rgbd_changed = True
                if _coerce_bool(data.get("requireStableTorso")) is not False:
                    data["requireStableTorso"] = False
                    rgbd_changed = True
                if rgbd_changed:
                    prepare_logs.append(
                        f"[GRAPH] Applied low-latency RGB-D live filtering on {_text_value(node.get('id'))} "
                        "(rgbd_pose_fusion) to reduce stale/noisy retargeting."
                    )
            if node.get("type") == "realsense_d435i_sensor" and has_rgbd_follow_nodes:
                sensor_changed = False
                fps = _coerce_int(data.get("fps"), 30)
                if fps <= 0 or fps > 15:
                    data["fps"] = 15
                    sensor_changed = True
                if sensor_changed:
                    prepare_logs.append(
                        f"[GRAPH] Limited RealSense capture on {_text_value(node.get('id'))} "
                        "(realsense_d435i_sensor) to 15 FPS for lower live command backlog."
                    )
            if node.get("type") == "mediapipe_pose" and has_rgbd_follow_nodes:
                mediapipe_changed = False
                if _coerce_int(data.get("modelComplexity"), 1) < 1:
                    data["modelComplexity"] = 1
                    mediapipe_changed = True
                if _coerce_float(data.get("minPoseVisibility"), 0.35) < 0.55:
                    data["minPoseVisibility"] = 0.55
                    mediapipe_changed = True
                if mediapipe_changed:
                    prepare_logs.append(
                        f"[GRAPH] Raised MediaPipe pose model complexity on {_text_value(node.get('id'))} "
                        "(mediapipe_pose) for more accurate live RGB-D retargeting."
                    )
            if (
                node.get("type") == "motion_retarget"
                and has_humanoid_pose_pipeline
                and has_isaacsim_g1_execute
            ):
                changed = False
                if _text_value(data.get("targetEmbodiment")).upper() != "REAL_G1":
                    data["targetEmbodiment"] = "REAL_G1"
                    changed = True
                if has_motion_learning_nodes:
                    if _coerce_bool(data.get("streamVideoFrames")) is not False:
                        data["streamVideoFrames"] = False
                        changed = True
                    stream_joint_scale = _coerce_float(data.get("streamJointScale"), 0.0)
                    if data.get("streamJointScale") in (None, "") or stream_joint_scale <= 0.0 or stream_joint_scale > 0.8:
                        data["streamJointScale"] = 0.62
                        changed = True
                    if data.get("streamRootLateralScale") in (None, ""):
                        data["streamRootLateralScale"] = 0.9
                        changed = True
                    if data.get("streamRootVerticalScale") in (None, ""):
                        data["streamRootVerticalScale"] = 1.8
                        changed = True
                    if data.get("streamRootDepthScale") in (None, ""):
                        data["streamRootDepthScale"] = 0.25
                        changed = True
                    if data.get("streamRootMaxLateral") in (None, ""):
                        data["streamRootMaxLateral"] = 0.12
                        changed = True
                    if data.get("streamRootMaxVertical") in (None, ""):
                        data["streamRootMaxVertical"] = 0.18
                        changed = True
                    if data.get("streamRootMaxDepth") in (None, ""):
                        data["streamRootMaxDepth"] = 0.08
                        changed = True
                    # Keep Motion Learning moving even when IsaacSim has not reloaded the
                    # latest extension code yet: send a compact G1 trajectory by default.
                    # Raw MediaPipe pose_sequence can be re-enabled after the Isaac extension
                    # is reloaded and verified to accept retarget_mode=isaac_usd_ik.
                    if _coerce_bool(data.get("emitG1Trajectory")) is not True:
                        data["emitG1Trajectory"] = True
                        changed = True
                    if _coerce_bool(data.get("emitG1JointTargets")) is not False:
                        data["emitG1JointTargets"] = False
                        changed = True
                    if _coerce_bool(data.get("useIsaacUsdIk")) is not False:
                        data["useIsaacUsdIk"] = False
                        changed = True
                    speed = _coerce_float(data.get("speed"), 1.0)
                    if data.get("speed") in (None, "") or speed < 0.20 or speed > 0.60:
                        data["speed"] = 0.45
                        changed = True
                    step_scale = _coerce_float(data.get("stepScale"), 0.45)
                    if data.get("stepScale") in (None, "") or step_scale < 0.15 or step_scale > 0.35:
                        data["stepScale"] = 0.25
                        changed = True
                    duration_sec = _coerce_float(data.get("durationSec"), 2.0)
                    if data.get("durationSec") in (None, "") or abs(duration_sec - 2.0) < 1.0e-6:
                        data["durationSec"] = 0.0
                        changed = True
                    if data.get("maxPoseFrames") in (None, "") or _coerce_int(data.get("maxPoseFrames"), 40) < 96:
                        data["maxPoseFrames"] = 96
                        changed = True
                    if _text_value(data.get("jointSelection")).lower() not in {"recent", "latest", "average", "mean"}:
                        data["jointSelection"] = "recent"
                        changed = True
                    if data.get("jointAverageWindow") in (None, "") or _coerce_int(data.get("jointAverageWindow"), 1) != 1:
                        data["jointAverageWindow"] = 1
                        changed = True
                    joint_smoothing = _coerce_float(data.get("jointSmoothing"), 0.0)
                    if data.get("jointSmoothing") in (None, "") or joint_smoothing < 0.18 or joint_smoothing > 0.45:
                        data["jointSmoothing"] = 0.25
                        changed = True
                    joint_max_delta = _coerce_float(data.get("jointMaxDeltaRad"), 0.0)
                    if data.get("jointMaxDeltaRad") in (None, "") or joint_max_delta < 0.06 or joint_max_delta > 0.18:
                        data["jointMaxDeltaRad"] = 0.10
                        changed = True
                    pose_scale = _coerce_float(data.get("poseScale"), 1.0)
                    if data.get("poseScale") in (None, "") or pose_scale < 0.90 or pose_scale > 1.30:
                        data["poseScale"] = 1.18
                        changed = True
                    lower_body_scale = _coerce_float(data.get("lowerBodyScale"), 0.55)
                    if data.get("lowerBodyScale") in (None, "") or lower_body_scale < 0.25 or lower_body_scale > 0.80:
                        data["lowerBodyScale"] = 0.55
                        changed = True
                    if data.get("inactiveArmDeadzone") in (None, "") or _coerce_float(data.get("inactiveArmDeadzone"), 0.08) > 0.02:
                        data["inactiveArmDeadzone"] = 0.0
                        changed = True
                    if data.get("limbMinVisibility") in (None, "") or _coerce_float(data.get("limbMinVisibility"), 0.42) > 0.35:
                        data["limbMinVisibility"] = 0.3
                        changed = True
                    if _text_value(data.get("lowerBodyMode")).lower() in {"", "auto", "locked", "lock", "freeze"}:
                        data["lowerBodyMode"] = "full_body"
                        changed = True
                    # Motion Learning defaults to foot-locked in-place retargeting.
                    # Set rootMotionMode=locomotion for walking clips, or auto for
                    # best-effort detection.
                    root_motion_mode = _text_value(data.get("rootMotionMode")).lower()
                    if root_motion_mode in {"", "auto"}:
                        data["rootMotionMode"] = "in_place"
                        changed = True
                    elif root_motion_mode not in {"in_place", "inplace", "fixed_feet", "foot_lock", "locomotion"}:
                        data["rootMotionMode"] = "in_place"
                        changed = True
                    if _coerce_bool(data.get("rootTranslationEnabled")) is not True:
                        data["rootTranslationEnabled"] = True
                        changed = True
                    root_lateral_scale = _coerce_float(data.get("rootLateralScale"), 0.0)
                    if data.get("rootLateralScale") in (None, "") or abs(root_lateral_scale) > 1.0e-6:
                        data["rootLateralScale"] = 0.0
                        changed = True
                    root_vertical_scale = _coerce_float(data.get("rootVerticalScale"), 0.0)
                    if data.get("rootVerticalScale") in (None, "") or root_vertical_scale < 1.0 or root_vertical_scale > 1.6:
                        data["rootVerticalScale"] = 1.15
                        changed = True
                    root_depth_scale = _coerce_float(data.get("rootDepthScale"), 0.0)
                    if data.get("rootDepthScale") in (None, "") or root_depth_scale < 0.15 or root_depth_scale > 0.55:
                        data["rootDepthScale"] = 0.35
                        changed = True
                    root_image_x_depth_scale = _coerce_float(data.get("rootImageXDepthScale"), 0.0)
                    if data.get("rootImageXDepthScale") in (None, "") or root_image_x_depth_scale < 0.7 or root_image_x_depth_scale > 1.8:
                        data["rootImageXDepthScale"] = 1.2
                        changed = True
                    root_max_lateral = _coerce_float(data.get("rootMaxLateral"), 0.0)
                    if data.get("rootMaxLateral") in (None, "") or abs(root_max_lateral) > 1.0e-6:
                        data["rootMaxLateral"] = 0.0
                        changed = True
                    root_max_vertical = _coerce_float(data.get("rootMaxVertical"), 0.0)
                    if data.get("rootMaxVertical") in (None, "") or root_max_vertical < 0.16 or root_max_vertical > 0.22:
                        data["rootMaxVertical"] = 0.18
                        changed = True
                    root_max_depth = _coerce_float(data.get("rootMaxDepth"), 0.0)
                    if data.get("rootMaxDepth") in (None, "") or root_max_depth < 0.08 or root_max_depth > 0.18:
                        data["rootMaxDepth"] = 0.12
                        changed = True
                    if _coerce_bool(data.get("rootPitchEnabled", data.get("rootRotationEnabled"))) is not True:
                        data["rootPitchEnabled"] = True
                        changed = True
                    root_pitch_scale = _coerce_float(data.get("rootPitchScale"), 0.0)
                    if data.get("rootPitchScale") in (None, "") or root_pitch_scale < 0.8 or root_pitch_scale > 1.5:
                        data["rootPitchScale"] = 1.15
                        changed = True
                    root_max_pitch = _coerce_float(data.get("rootMaxPitchDeg"), 0.0)
                    if data.get("rootMaxPitchDeg") in (None, "") or root_max_pitch < 6.0 or root_max_pitch > 14.0:
                        data["rootMaxPitchDeg"] = 10.0
                        changed = True
                elif _coerce_bool(data.get("useIsaacUsdIk")) is not False:
                    data["useIsaacUsdIk"] = False
                    changed = True
                if has_rgbd_follow_nodes and not has_motion_learning_nodes:
                    if _coerce_bool(data.get("useIsaacUsdIk")) is not False:
                        data["useIsaacUsdIk"] = False
                        changed = True
                    if _coerce_bool(data.get("emitG1JointTargets")) is not True:
                        data["emitG1JointTargets"] = True
                        changed = True
                    speed = _coerce_float(data.get("speed"), 1.0)
                    if data.get("speed") in (None, "") or speed < 0.20 or speed > 0.60:
                        data["speed"] = 0.45
                        changed = True
                    step_scale = _coerce_float(data.get("stepScale"), 0.35)
                    if data.get("stepScale") in (None, "") or step_scale < 0.12 or step_scale > 0.30:
                        data["stepScale"] = 0.22
                        changed = True
                    if data.get("durationSec") in (None, "") or abs(_coerce_float(data.get("durationSec"), 1.0)) > 1.0e-6:
                        data["durationSec"] = 0.0
                        changed = True
                    if _coerce_int(data.get("maxPoseFrames"), 40) != 1:
                        data["maxPoseFrames"] = 1
                        changed = True
                    joint_smoothing = _coerce_float(data.get("jointSmoothing"), 0.0)
                    if data.get("jointSmoothing") in (None, "") or joint_smoothing < 0.18 or joint_smoothing > 0.45:
                        data["jointSmoothing"] = 0.25
                        changed = True
                    joint_max_delta = _coerce_float(data.get("jointMaxDeltaRad"), 0.0)
                    if data.get("jointMaxDeltaRad") in (None, "") or joint_max_delta < 0.06 or joint_max_delta > 0.18:
                        data["jointMaxDeltaRad"] = 0.10
                        changed = True
                    joint_deadband = _coerce_float(data.get("jointDeadbandRad"), 0.0)
                    if data.get("jointDeadbandRad") in (None, "") or joint_deadband < 0.0 or joint_deadband > 0.01:
                        data["jointDeadbandRad"] = 0.003
                        changed = True
                    if _coerce_int(data.get("jointAverageWindow"), 1) != 1:
                        data["jointAverageWindow"] = 1
                        changed = True
                    if _text_value(data.get("jointSelection")).lower() not in {"latest", "recent"}:
                        data["jointSelection"] = "latest"
                        changed = True
                    pose_scale = _coerce_float(data.get("poseScale"), 1.0)
                    if pose_scale < 1.0 or pose_scale > 1.2:
                        data["poseScale"] = 1.05
                        changed = True
                    if _text_value(data.get("lowerBodyMode")).lower() in {"", "locked", "lock", "freeze", "upper", "upper_body", "seated"}:
                        data["lowerBodyMode"] = "full_body"
                        changed = True
                    lower_body_scale = _coerce_float(data.get("lowerBodyScale"), 0.0)
                    if data.get("lowerBodyScale") in (None, "") or lower_body_scale <= 0.0 or lower_body_scale > 0.9:
                        data["lowerBodyScale"] = 0.75
                        changed = True
                if _coerce_bool(data.get("rootMotionEnabled")) is not False:
                    data["rootMotionEnabled"] = False
                    changed = True
                if _coerce_float(data.get("jointSmoothing"), -1.0) < 0.0:
                    data["jointSmoothing"] = 0.25
                    changed = True
                if changed:
                    if has_motion_learning_nodes:
                        prepare_logs.append(
                            f"[GRAPH] Enabled compact G1 trajectory retarget on {_text_value(node.get('id'))} "
                            f"(motion_retarget) for {humanoid_pose_pipeline_label}."
                        )
                    else:
                        prepare_logs.append(
                            f"[GRAPH] Enabled REAL_G1 articulation joint targets on {_text_value(node.get('id'))} "
                            f"(motion_retarget) for {humanoid_pose_pipeline_label}."
                        )
            continue
        mode = _text_value(data.get("mode")).lower() or "simulate"
        if has_humanoid_pose_pipeline and mode == "omniverse" and _looks_like_isaacsim_g1_execute(data):
            if _coerce_bool(data.get("dryRun")) is True:
                data["dryRun"] = False
                prepare_logs.append(
                    f"[GRAPH] Disabled stale dryRun on {_text_value(node.get('id'))} "
                    f"(robot_execute) for {humanoid_pose_pipeline_label}."
                )
            desired_wait = False if has_rgbd_follow_nodes or has_motion_learning_nodes else True
            if _coerce_bool(data.get("omniverseWaitForResponse")) is not desired_wait:
                data["omniverseWaitForResponse"] = desired_wait
                if desired_wait:
                    prepare_logs.append(
                        f"[GRAPH] Enabled Omniverse response wait on {_text_value(node.get('id'))} "
                        "(robot_execute) so Isaac Sim errors are visible."
                    )
                else:
                    prepare_logs.append(
                        f"[GRAPH] Disabled Omniverse response wait on {_text_value(node.get('id'))} "
                        "(robot_execute) for lower-latency RGB-D live follow."
                    )
            if _coerce_float(data.get("omniverseTimeoutSec"), 0.0) < 10.0:
                data["omniverseTimeoutSec"] = 10
                prepare_logs.append(
                    f"[GRAPH] Raised Omniverse timeout on {_text_value(node.get('id'))} "
                    f"(robot_execute) for {humanoid_pose_pipeline_label}."
                )
        if (
            has_g1_gr00t_infer
            and mode == "omniverse"
            and _coerce_bool(data.get("dryRun")) is True
            and _looks_like_isaacsim_g1_execute(data)
        ):
            data["dryRun"] = False
            prepare_logs.append(
                f"[GRAPH] Disabled stale dryRun on {_text_value(node.get('id'))} "
                "(robot_execute) for Isaac Sim GR00T execution."
            )

    prepare_logs.extend(_ensure_realsense_bridge_for_graph(normalized_graph))

    for node in nodes:
        if not isinstance(node, dict) or node.get("type") != "openvla_infer":
            continue

        node_id = _text_value(node.get("id"))
        if not node_id:
            continue

        downstream_unity_node = find_downstream_unity_execute(node_id)
        if downstream_unity_node is None:
            continue

        data = node.get("data", {})
        if not isinstance(data, dict):
            data = {}
            node["data"] = data

        execute_data = downstream_unity_node.get("data", {}) or {}
        unity_command = _text_value(execute_data.get("unityCommand"))
        unity_command_lower = unity_command.lower()
        current_unity_command = _text_value(data.get("unityCommand"))
        current_unity_command_lower = current_unity_command.lower()
        current_action_format = _text_value(data.get("actionFormat"))
        current_action_format_lower = current_action_format.lower()
        current_waypoint_speed = data.get("waypointSpeed")
        changed_fields: List[str] = []

        if unity_command and current_unity_command_lower in {"", "apply_action"}:
            if current_unity_command_lower != unity_command_lower:
                data["unityCommand"] = unity_command
                current_unity_command = unity_command
                current_unity_command_lower = unity_command_lower
                changed_fields.append(f"unityCommand={unity_command}")

        if unity_command_lower == "apply_base_waypoint":

            if current_action_format_lower in {"", "delta_action"}:
                data["actionFormat"] = "unity_waypoint"
                current_action_format = "unity_waypoint"
                current_action_format_lower = "unity_waypoint"
                changed_fields.append("actionFormat=unity_waypoint")

            if current_waypoint_speed in (None, ""):
                data["waypointSpeed"] = DEFAULT_UNITY_WAYPOINT_SPEED
                current_waypoint_speed = DEFAULT_UNITY_WAYPOINT_SPEED
                changed_fields.append(
                    f"waypointSpeed={DEFAULT_UNITY_WAYPOINT_SPEED:0.1f}"
                )
        if current_unity_command_lower == "apply_base_waypoint":
            if not current_action_format:
                data["actionFormat"] = "unity_waypoint"
                current_action_format = "unity_waypoint"
                current_action_format_lower = "unity_waypoint"
                changed_fields.append("actionFormat=unity_waypoint")

            if current_waypoint_speed in (None, ""):
                data["waypointSpeed"] = DEFAULT_UNITY_WAYPOINT_SPEED
                current_waypoint_speed = DEFAULT_UNITY_WAYPOINT_SPEED
                changed_fields.append(
                    f"waypointSpeed={DEFAULT_UNITY_WAYPOINT_SPEED:0.1f}"
                )

        if unity_command_lower == "apply_patrol_control":
            if current_action_format_lower in {"", "delta_action"}:
                data["actionFormat"] = "patrol_control"
                current_action_format = "patrol_control"
                current_action_format_lower = "patrol_control"
                changed_fields.append("actionFormat=patrol_control")

            if current_waypoint_speed in (None, ""):
                data["waypointSpeed"] = DEFAULT_UNITY_WAYPOINT_SPEED
                current_waypoint_speed = DEFAULT_UNITY_WAYPOINT_SPEED
                changed_fields.append(
                    f"waypointSpeed={DEFAULT_UNITY_WAYPOINT_SPEED:0.1f}"
                )

        if current_unity_command_lower == "apply_patrol_control":
            if not current_action_format:
                data["actionFormat"] = "patrol_control"
                current_action_format = "patrol_control"
                current_action_format_lower = "patrol_control"
                changed_fields.append("actionFormat=patrol_control")

            if current_waypoint_speed in (None, ""):
                data["waypointSpeed"] = DEFAULT_UNITY_WAYPOINT_SPEED
                current_waypoint_speed = DEFAULT_UNITY_WAYPOINT_SPEED
                changed_fields.append(
                    f"waypointSpeed={DEFAULT_UNITY_WAYPOINT_SPEED:0.1f}"
                )

        if changed_fields:
            prepare_logs.append(
                f"[GRAPH] Auto-configured {node_id} (openvla_infer) from "
                f"{_text_value(downstream_unity_node.get('id'))}: "
                + ", ".join(changed_fields)
            )
        else:
            prepare_logs.append(
                f"[GRAPH] Prepared {node_id} (openvla_infer): "
                f"downstreamUnityCommand={unity_command or '<empty>'}, "
                f"unityCommand={current_unity_command or '<empty>'}, "
                f"actionFormat={current_action_format or '<empty>'}, "
                f"waypointSpeed={current_waypoint_speed if current_waypoint_speed not in (None, '') else '<empty>'}"
            )

        effective_command = current_unity_command_lower or unity_command_lower
        if has_manipulation_prompt and effective_command in LOCOMOTION_UNITY_COMMANDS:
            prompt_preview = _text_value(prompt_texts[0]) if prompt_texts else ""
            if len(prompt_preview) > 72:
                prompt_preview = prompt_preview[:69] + "..."
            prepare_logs.append(
                f"[GRAPH WARNING] Prompt looks like manipulation "
                f"('{prompt_preview or 'task'}'), but the Unity command is "
                f"{effective_command}. This graph is configured for locomotion, "
                "so it can move toward the object but cannot perform grasping/pickup."
            )

    return normalized_graph, prepare_logs


def _graph_summary(graph: Dict[str, Any]) -> Dict[str, Any]:
    nodes = graph.get("nodes", [])
    edges = graph.get("edges", [])
    robot_nodes = [node for node in nodes if node.get("type") == "robot_execute"]
    unity_state_nodes = [node for node in nodes if node.get("type") == "unity_state"]
    robot_modes = sorted(
        {
            str((node.get("data", {}) or {}).get("mode", "simulate")).strip().lower() or "simulate"
            for node in robot_nodes
        }
    )
    return {
        "node_count": len(nodes),
        "edge_count": len(edges),
        "has_robot_execute": bool(robot_nodes),
        "has_unity_state": bool(unity_state_nodes),
        "robot_modes": robot_modes,
    }


def _unity_state_client_ids(graph: Dict[str, Any]) -> List[str]:
    client_ids: List[str] = []
    for node in graph.get("nodes", []):
        if node.get("type") != "unity_state":
            continue
        data = node.get("data", {}) or {}
        client_id = str(data.get("clientId", "default")).strip() or "default"
        if client_id not in client_ids:
            client_ids.append(client_id)
    return client_ids


def _unity_command_client_ids(graph: Dict[str, Any]) -> List[str]:
    client_ids: List[str] = []
    for node in graph.get("nodes", []):
        if node.get("type") != "robot_execute":
            continue
        data = node.get("data", {}) or {}
        mode = str(data.get("mode", "simulate")).strip().lower() or "simulate"
        if mode != "unity":
            continue
        client_id = str(
            data.get("unityStateClientId", data.get("clientId", "default"))
        ).strip() or "default"
        if client_id not in client_ids:
            client_ids.append(client_id)
    return client_ids


def _graph_has_live_rgbd_mocap(graph: Dict[str, Any]) -> bool:
    nodes = graph.get("nodes", [])
    if not isinstance(nodes, list):
        return False
    return any(
        isinstance(node, dict)
        and node.get("type") in {"realsense_d435i_sensor", "mediapipe_pose", "rgbd_pose_fusion"}
        for node in nodes
    )


def _graph_has_motion_learning_input(graph: Dict[str, Any]) -> bool:
    nodes = graph.get("nodes", [])
    if not isinstance(nodes, list):
        return False
    return any(
        isinstance(node, dict)
        and node.get("type") in {"video_motion_input", "motion_extract"}
        for node in nodes
    )


def _graph_has_isaacsim_g1_execute(graph: Dict[str, Any]) -> bool:
    nodes = graph.get("nodes", [])
    if not isinstance(nodes, list):
        return False
    return any(
        isinstance(node, dict)
        and node.get("type") == "robot_execute"
        and _text_value((node.get("data", {}) or {}).get("mode")).lower() == "omniverse"
        and _looks_like_isaacsim_g1_execute(node.get("data", {}) or {})
        for node in nodes
    )


def _runtime_options(graph: Dict[str, Any], options: Dict[str, Any] | None) -> Dict[str, Any]:
    raw = options or {}
    unity_state_clients = _unity_state_client_ids(graph)
    unity_command_clients = _unity_command_client_ids(graph)
    graph_info = _graph_summary(graph)
    has_live_rgbd_mocap = _graph_has_live_rgbd_mocap(graph)
    has_motion_learning_input = _graph_has_motion_learning_input(graph)
    live_isaac_mocap = has_live_rgbd_mocap and _graph_has_isaacsim_g1_execute(graph)
    motion_learning_oneshot = has_motion_learning_input and not has_live_rgbd_mocap

    persistent_requested = _coerce_bool(raw.get("persistent"))
    require_feedback_requested = _coerce_bool(
        raw.get("requireFeedback", raw.get("require_feedback"))
    )
    continue_on_error = bool(
        _coerce_bool(raw.get("continueOnError", raw.get("continue_on_error"))) is True
    )
    live_single_context_requested = _coerce_bool(
        raw.get("liveSingleContext", raw.get("live_single_context"))
    )

    auto_persistent = bool(unity_state_clients and unity_command_clients) or live_isaac_mocap
    persistent = auto_persistent if persistent_requested is None else persistent_requested
    live_single_context = (
        bool(live_isaac_mocap)
        if live_single_context_requested is None
        else bool(live_single_context_requested)
    )
    require_feedback = (
        persistent and bool(unity_command_clients)
        if require_feedback_requested is None
        else require_feedback_requested
    )

    max_iterations = max(
        0,
        _coerce_int(raw.get("maxIterations", raw.get("max_iterations", 0)), 0),
    )
    max_iterations_requested = (
        "maxIterations" in raw
        or "max_iterations" in raw
    )
    default_cycle_interval_sec = 0.12 if live_isaac_mocap else 0.75
    cycle_interval_sec = max(
        0.1,
        _coerce_float(
            raw.get("cycleIntervalSec", raw.get("cycle_interval_sec", default_cycle_interval_sec)),
            default_cycle_interval_sec,
        ),
    )
    feedback_timeout_sec = max(
        1.0,
        _coerce_float(
            raw.get("feedbackTimeoutSec", raw.get("feedback_timeout_sec", 30.0)),
            30.0,
        ),
    )
    log_limit = max(50, _coerce_int(raw.get("logLimit", raw.get("log_limit", 400)), 400))

    if not persistent and max_iterations == 0:
        max_iterations = 1
    elif (
        persistent
        and persistent_requested is None
        and not live_single_context
        and not max_iterations_requested
        and max_iterations == 0
    ):
        # Auto-persistent robot graphs are usually one-shot command graphs.  Leaving
        # them at 0 repeats the same locomotion command indefinitely.
        max_iterations = 1

    if motion_learning_oneshot:
        persistent = False
        live_single_context = False
        require_feedback = False
        max_iterations = 1

    return {
        "persistent": persistent,
        "require_feedback": require_feedback,
        "continue_on_error": continue_on_error,
        "live_single_context": live_single_context,
        "max_iterations": max_iterations,
        "cycle_interval_sec": cycle_interval_sec,
        "feedback_timeout_sec": feedback_timeout_sec,
        "log_limit": log_limit,
        "unity_state_client_ids": unity_state_clients,
        "unity_command_client_ids": unity_command_clients,
        "graph_summary": graph_info,
        "motion_learning_oneshot": motion_learning_oneshot,
    }


def _result_has_errors(result: Dict[str, Any]) -> bool:
    if not isinstance(result, dict):
        return True

    if int(result.get("error_count", 0) or 0) > 0:
        return True

    context = result.get("context", {})
    if not isinstance(context, dict):
        return False

    for payload in context.values():
        if isinstance(payload, dict) and (
            payload.get("ok") is False or bool(payload.get("error"))
        ):
            return True
        if isinstance(payload, dict):
            nested_result = payload.get("result")
            if isinstance(nested_result, dict) and (
                nested_result.get("ok") is False or bool(nested_result.get("error"))
            ):
                return True

    return False


def _node_stops_runtime_on_terminate(node: Dict[str, Any]) -> bool:
    data = node.get("data", {}) or {}
    value = _coerce_bool(
        data.get(
            "stopRuntimeOnTerminate",
            data.get("terminateRuntime", data.get("respectTerminate")),
        )
    )
    return value is True


def _result_requests_terminate(graph: Dict[str, Any], result: Dict[str, Any]) -> bool:
    context = result.get("context", {}) if isinstance(result, dict) else {}
    if not isinstance(context, dict):
        return False

    for node in graph.get("nodes", []):
        if node.get("type") != "robot_execute":
            continue

        payload = context.get(node.get("id"))
        if not isinstance(payload, dict):
            continue

        result_payload = payload.get("result")
        if not isinstance(result_payload, dict):
            continue

        request_payload = result_payload.get("request")
        if not isinstance(request_payload, dict):
            continue

        action = request_payload.get("action")
        if not isinstance(action, dict):
            continue

        if any(
            bool(action.get(key))
            for key in ("runtime_terminate", "terminate_runtime", "stop_runtime")
        ):
            return True

        if bool(action.get("terminate")) and _node_stops_runtime_on_terminate(node):
            return True

    return False


def _feedback_checkpoint(
    graph: Dict[str, Any],
    result: Dict[str, Any],
    fallback_client_ids: List[str],
) -> Dict[str, float]:
    checkpoints: Dict[str, float] = {}
    context = result.get("context", {}) if isinstance(result, dict) else {}
    if not isinstance(context, dict):
        context = {}

    for node in graph.get("nodes", []):
        if node.get("type") != "unity_state":
            continue

        payload = context.get(node.get("id"))
        if not isinstance(payload, dict):
            continue

        client_id = str(
            payload.get(
                "client_id",
                (node.get("data", {}) or {}).get("clientId", "default"),
            )
        ).strip() or "default"
        timestamp = payload.get("timestamp")
        if isinstance(timestamp, (int, float)):
            checkpoints[client_id] = max(float(timestamp), checkpoints.get(client_id, 0.0))

    for node in graph.get("nodes", []):
        if node.get("type") != "robot_execute":
            continue

        payload = context.get(node.get("id"))
        if not isinstance(payload, dict):
            continue

        result_payload = payload.get("result")
        if not isinstance(result_payload, dict):
            continue

        client_id = str(
            result_payload.get(
                "unity_state_client_id",
                (node.get("data", {}) or {}).get(
                    "unityStateClientId",
                    (node.get("data", {}) or {}).get("clientId", "default"),
                ),
            )
        ).strip() or "default"
        state_update = result_payload.get("state_update")
        if not isinstance(state_update, dict):
            continue

        timestamp = state_update.get("timestamp")
        if isinstance(timestamp, (int, float)):
            checkpoints[client_id] = max(float(timestamp), checkpoints.get(client_id, 0.0))

    for client_id in fallback_client_ids:
        if client_id in checkpoints:
            continue
        record = state_store.get_state(client_id)
        if record is None:
            checkpoints[client_id] = 0.0
            continue
        checkpoints[client_id] = float(record.get("timestamp", 0.0) or 0.0)

    return checkpoints


def _current_state_timestamps(client_ids: List[str]) -> Dict[str, float]:
    checkpoints: Dict[str, float] = {}
    for client_id in client_ids:
        normalized_client_id = str(client_id or "").strip() or "default"
        record = state_store.get_state(normalized_client_id)
        if record is None:
            checkpoints[normalized_client_id] = 0.0
            continue
        checkpoints[normalized_client_id] = float(record.get("timestamp", 0.0) or 0.0)
    return checkpoints


def _reset_graph_runtime_state(graph: Dict[str, Any]) -> List[str]:
    nodes = graph.get("nodes") if isinstance(graph.get("nodes"), list) else []
    fusion_node_ids = [
        _text_value(node.get("id"))
        for node in nodes
        if isinstance(node, dict) and node.get("type") == "rgbd_pose_fusion"
    ]
    retarget_node_ids = [
        _text_value(node.get("id"))
        for node in nodes
        if isinstance(node, dict) and node.get("type") == "motion_retarget"
    ]

    logs: List[str] = []
    if fusion_node_ids:
        from backend.executors.rgbd_pose_nodes import reset_rgbd_pose_runtime_state

        reset_rgbd_pose_runtime_state([node_id for node_id in fusion_node_ids if node_id])
        logs.append("[GRAPH] Reset RGB-D pose fusion history/smoothing state for this runtime session.")

    if retarget_node_ids:
        from backend.executors.motion_nodes import reset_motion_retarget_runtime_state

        reset_motion_retarget_runtime_state([node_id for node_id in retarget_node_ids if node_id])
        logs.append("[GRAPH] Reset motion retarget joint smoothing state for this runtime session.")

    return logs


class GraphRuntimeManager:
    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._sessions: Dict[str, Dict[str, Any]] = {}

    def _append_logs(self, session: Dict[str, Any], *lines: str) -> None:
        next_logs = list(session.get("logs", []))
        for line in lines:
            text = str(line or "").strip()
            if not text:
                continue
            next_logs.append(text)
        session["logs"] = _trim_logs(next_logs, int(session["options"]["log_limit"]))

    def _snapshot(self, session: Dict[str, Any]) -> Dict[str, Any]:
        return {
            "session_id": session["session_id"],
            "status": session["status"],
            "message": session.get("message", ""),
            "error": session.get("error"),
            "created_at": session.get("created_at"),
            "started_at": session.get("started_at"),
            "updated_at": session.get("updated_at"),
            "finished_at": session.get("finished_at"),
            "iteration_count": session.get("iteration_count", 0),
            "frame_count": session.get("frame_count", 0),
            "active_node_id": session.get("active_node_id"),
            "active_node_type": session.get("active_node_type"),
            "cancel_requested": bool(session.get("cancel_requested")),
            "graph_loaded": bool(session.get("graph_loaded")),
            "graph_summary": dict(session.get("graph_summary", {})),
            "options": {
                "persistent": bool(session["options"]["persistent"]),
                "require_feedback": bool(session["options"]["require_feedback"]),
                "continue_on_error": bool(session["options"].get("continue_on_error", False)),
                "live_single_context": bool(session["options"].get("live_single_context", False)),
                "max_iterations": int(session["options"]["max_iterations"]),
                "cycle_interval_sec": float(session["options"]["cycle_interval_sec"]),
                "feedback_timeout_sec": float(session["options"]["feedback_timeout_sec"]),
                "unity_state_client_ids": list(session["options"]["unity_state_client_ids"]),
                "unity_command_client_ids": list(session["options"]["unity_command_client_ids"]),
            },
            "last_result": session.get("last_result"),
            "last_result_at": session.get("last_result_at"),
            "amr_commands": [{key: copy.deepcopy(value) for key, value in command.items()
                              if key not in {"props", "deadline"}}
                             for command in session.get("amr_commands", {}).values()],
            "fanuc_commands": list(copy.deepcopy(session.get("fanuc_commands", {})).values()),
            "logs": list(session.get("logs", [])),
        }

    def _get_session(self, session_id: str) -> Dict[str, Any]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                raise KeyError(session_id)
            return session

    def _is_cancel_requested(self, session_id: str) -> bool:
        with self._lock:
            session = self._sessions.get(session_id)
            return bool(session and session.get("cancel_requested"))

    def _set_status(
        self,
        session_id: str,
        status: str,
        message: str,
        *,
        error: str | None = None,
    ) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["status"] = status
            session["message"] = message
            session["error"] = error
            session["updated_at"] = _now()
            if status == "running" and session.get("started_at") is None:
                session["started_at"] = session["updated_at"]
            if status in FINAL_SESSION_STATES:
                session["finished_at"] = session["updated_at"]
                session["graph_loaded"] = False
                session["graph"] = None
            self._append_logs(session, f"[SESSION] {message}")

    def _start_iteration(self, session_id: str, iteration: int) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["status"] = "running"
            session["message"] = f"{iteration}회차 그래프를 실행 중입니다."
            session["updated_at"] = _now()
            if session.get("started_at") is None:
                session["started_at"] = session["updated_at"]
            self._append_logs(session, f"[ITER {iteration}] Cycle started")

    def _start_live_frame(self, session_id: str, frame: int) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["status"] = "running"
            session["iteration_count"] = 1
            session["frame_count"] = frame
            session["message"] = f"라이브 RGB-D 세션 1회차에서 {frame}번째 프레임을 처리 중입니다."
            session["updated_at"] = _now()
            if session.get("started_at") is None:
                session["started_at"] = session["updated_at"]
            self._append_logs(session, f"[LIVE frame {frame}] Cycle started")

    def _store_node_event(self, session_id: str, iteration: int, event: Dict[str, Any]) -> None:
        self._record_amr_result(session_id, event)
        self._record_fanuc_result(session_id, event)
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["active_node_id"] = event.get("node_id")
            session["active_node_type"] = event.get("node_type")
            session["message"] = (
                f"{iteration}회차 {event.get('node_type', 'node')} 노드를 처리했습니다."
            )
            session["updated_at"] = _now()
            self._append_logs(
                session,
                f"[ITER {iteration}] {event.get('log_line', '')}",
            )

    def _store_live_node_event(self, session_id: str, frame: int, event: Dict[str, Any]) -> None:
        self._record_amr_result(session_id, event)
        self._record_fanuc_result(session_id, event)
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["active_node_id"] = event.get("node_id")
            session["active_node_type"] = event.get("node_type")
            session["iteration_count"] = 1
            session["frame_count"] = frame
            session["message"] = (
                f"라이브 RGB-D {frame}번째 프레임에서 {event.get('node_type', 'node')} 노드를 처리했습니다."
            )
            session["updated_at"] = _now()
            self._append_logs(
                session,
                f"[LIVE frame {frame}] {event.get('log_line', '')}",
            )

    def _store_result(self, session_id: str, iteration: int, result: Dict[str, Any]) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["iteration_count"] = iteration
            session["last_result"] = result
            session["last_result_at"] = _now()
            session["updated_at"] = session["last_result_at"]
            session["active_node_id"] = None
            session["active_node_type"] = None

    def _store_live_result(self, session_id: str, frame: int, result: Dict[str, Any]) -> None:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                return
            session["iteration_count"] = 1
            session["frame_count"] = frame
            session["last_result"] = result
            session["last_result_at"] = _now()
            session["updated_at"] = session["last_result_at"]
            session["active_node_id"] = None
            session["active_node_type"] = None

    def _begin_amr_node(self, session_id: str, event: Dict[str, Any]) -> None:
        node = event["node"]
        with self._lock:
            session = self._sessions[session_id]
            if session.get("cancel_requested"):
                raise GraphRunCancelled("Runtime cancelled before node execution.")
            session.update(active_node_id=node["id"], active_node_type=node.get("type"), updated_at=_now())
            if node.get("type") != "amr_navigate" or node.get("data", {}).get("enableExecution") is not True:
                return
            props = node.setdefault("data", {})
            if node["id"] in session["amr_auto_id_nodes"]:
                props["commandId"] = f"amr-{session_id}-{uuid4().hex}"
            command_id = props["commandId"]
            session["amr_commands"][node["id"]] = {
                "node_id": node["id"], "command_id": command_id, "props": copy.deepcopy(props),
                "status": "submitting", "completion_verified": False,
                "deadline": time.monotonic() + min(600, max(1, _coerce_float(props.get("navigationTimeoutSec"), 120))) + 10,
            }

    def _record_amr_result(self, session_id: str, event: Dict[str, Any]) -> None:
        if event.get("node_type") != "amr_navigate":
            return
        with self._lock:
            session = self._sessions.get(session_id)
            command = session.get("amr_commands", {}).get(event.get("node_id")) if session else None
            if command is None:
                return
            result = event.get("result") or {}
            execution = result.get("execution")
            if isinstance(execution, dict) and execution.get("command_id") == command["command_id"]:
                if command["status"] not in AMR_TERMINAL_STATES:
                    command.update(status=execution.get("status", "unknown"),
                                   completion_verified=execution.get("completion_verified") is True,
                                   execution=copy.deepcopy(execution))
            elif command["status"] not in AMR_TERMINAL_STATES:
                command["status"] = "unknown"
            cancelled = bool(session.get("cancel_requested"))
        # A cancellation can arrive while navigate is in flight, before the
        # bridge knows its ID. Retry stop after submission returns as well.
        if cancelled:
            logs = self._stop_amr_commands(session_id)
            with self._lock:
                self._append_logs(self._sessions[session_id], *logs)

    def _update_amr_execution(self, session_id, node_id, command_id, execution):
        if not isinstance(execution, dict) or execution.get("command_id") != command_id:
            raise ValueError("AMR status/stop response command identity mismatch")
        if execution.get("status") not in AMR_TERMINAL_STATES | {"running", "stopping"}:
            raise ValueError("Unknown AMR command status")
        with self._lock:
            command = self._sessions[session_id]["amr_commands"][node_id]
            if command["command_id"] == command_id and command["status"] not in AMR_TERMINAL_STATES:
                command.update(status=execution["status"], completion_verified=execution.get("completion_verified") is True,
                               execution=copy.deepcopy(execution))

    def _stop_amr_commands(self, session_id):
        from backend.executors.amr_nodes import bridge_request
        with self._lock:
            commands = copy.deepcopy(list(self._sessions.get(session_id, {}).get("amr_commands", {}).values()))
        logs = []
        for command in commands:
            if command["status"] in AMR_TERMINAL_STATES:
                continue
            try:
                props = {**command["props"], "timeoutSec": 3}
                response = bridge_request(props, "amr_stop", command_id=command["command_id"])
                if response.get("ok") is not True:
                    raise ValueError(response.get("message", "AMR stop rejected"))
                self._update_amr_execution(session_id, command["node_id"], command["command_id"], response.get("execution"))
                logs.append(f"[SESSION] AMR stop {command['command_id']}: {response['execution']['status']}")
            except Exception as exc:
                logs.append(f"[SESSION WARNING] AMR stop unconfirmed for {command['command_id']}: {exc}")
        return logs

    async def _wait_for_amr_commands(self, session_id):
        from backend.executors.amr_nodes import bridge_request
        while True:
            if self._is_cancel_requested(session_id):
                raise GraphRunCancelled("Runtime cancelled while waiting for AMR completion.")
            with self._lock:
                commands = copy.deepcopy(list(self._sessions[session_id]["amr_commands"].values()))
            pending = False
            for command in commands:
                if command["status"] in AMR_TERMINAL_STATES:
                    if command["status"] != "completed" or not command["completion_verified"]:
                        raise RuntimeError(f"AMR {command['command_id']} ended without verified arrival: {command['status']}")
                    continue
                pending = True
                if time.monotonic() >= command["deadline"]:
                    raise RuntimeError(f"AMR completion timeout: {command['command_id']}")
                response = await asyncio.to_thread(bridge_request, command["props"], "amr_status", command_id=command["command_id"])
                if response.get("ok") is not True:
                    raise RuntimeError(response.get("message", "AMR status rejected"))
                self._update_amr_execution(session_id, command["node_id"], command["command_id"], response.get("execution"))
            if not pending:
                return
            await asyncio.sleep(.1)

    def _record_fanuc_result(self, session_id, event):
        payload = event.get("result") or {}
        with self._lock:
            session = self._sessions[session_id]
            commands = session.setdefault("fanuc_commands", {})
            if event.get("node_type") == "robot_execute":
                result = payload.get("result") or {}
                request = result.get("request") or {}
                if request.get("command") == "apply_fanuc_trajectory" and request.get("command_id") and result.get("endpoint"):
                    commands[request["command_id"]] = {
                        "command_id": request["command_id"], "node_id": event["node_id"],
                        "prim_path": request.get("prim_path"), "endpoint": result["endpoint"], "status": "submitted",
                    }
            if event.get("node_type") == "execution_observe":
                execution = payload.get("execution") or {}
                command = commands.get(execution.get("command_id"))
                if command and execution.get("prim_path") == command["prim_path"]:
                    command["status"] = execution.get("status", command["status"])
            cancelled = session.get("cancel_requested")
        if cancelled:
            logs = self._stop_fanuc_commands(session_id)
            with self._lock:
                self._append_logs(self._sessions[session_id], *logs)

    def _stop_fanuc_commands(self, session_id):
        from backend.executors.world_model_nodes import _fanuc_observation_request
        with self._lock:
            commands = copy.deepcopy(list(self._sessions.get(session_id, {}).get("fanuc_commands", {}).values()))
        logs = []
        for command in commands:
            if command["status"] in {"completed", "failed", "cancelled"}:
                continue
            try:
                response = _fanuc_observation_request({"omniverseEndpoint": command["endpoint"], "timeoutSec": 3},
                    "stop_command", command_id=command["command_id"], prim_path=command["prim_path"])
                record = response.get("execution") or {}
                if record.get("command_id") != command["command_id"] or record.get("prim_path") != command["prim_path"] or record.get("status") not in {"completed", "failed", "cancelled"}:
                    raise ValueError("Targeted FANUC stop was not confirmed")
                with self._lock:
                    self._sessions[session_id]["fanuc_commands"][command["command_id"]]["status"] = record["status"]
                logs.append(f"[SESSION] FANUC stop {command['command_id']}: {record['status']}")
            except Exception as exc:
                logs.append(f"[SESSION WARNING] FANUC stop unconfirmed for {command['command_id']}: {exc}")
        return logs

    def _send_cancel_stop_commands(self, session_id: str, graph: Dict[str, Any]) -> List[str]:
        stop_logs: List[str] = self._stop_amr_commands(session_id) + self._stop_fanuc_commands(session_id)
        try:
            from backend.executors.vla_nodes import _omniverse_execute
        except Exception as exc:
            return [f"[SESSION WARNING] Could not load Omniverse stop executor: {exc}"]

        execute_nodes = _isaacsim_g1_omniverse_execute_nodes(graph)
        if not execute_nodes:
            return stop_logs

        for node in execute_nodes:
            node_id = _text_value(node.get("id")) or "robot_execute"
            data = copy.deepcopy(node.get("data", {}) or {})
            if not isinstance(data, dict):
                data = {}
            data["mode"] = "omniverse"
            data["omniverseCommand"] = "stop"
            data["omniverseWaitForResponse"] = True
            data["omniverseTimeoutSec"] = min(
                max(_coerce_float(data.get("omniverseTimeoutSec"), 3.0), 1.0),
                3.0,
            )
            try:
                result = _omniverse_execute(data, _g1_stop_action("runtime_cancel"))
                ok = bool(result.get("ok"))
                message = _text_value(result.get("message")) or ("ok" if ok else "failed")
                stop_logs.append(
                    f"[SESSION] Sent cancel stop to {node_id} (robot_execute): ok={ok} message={message}"
                )
            except Exception as exc:
                stop_logs.append(
                    f"[SESSION WARNING] Cancel stop failed for {node_id} (robot_execute): {exc}"
                )
        return stop_logs

    async def start_session(
        self,
        graph: Dict[str, Any],
        input_data: Dict[str, Any] | None = None,
        options: Dict[str, Any] | None = None,
    ) -> Dict[str, Any]:
        session_id = uuid4().hex
        prepared_graph, prepare_logs = prepare_graph_for_execution(graph)
        prepare_logs = prepare_logs + _reset_graph_runtime_state(prepared_graph)
        normalized_options = _runtime_options(prepared_graph, options)
        now = _now()
        session = {
            "session_id": session_id,
            "status": "starting",
            "message": "그래프 런타임을 메모리에 올리는 중입니다.",
            "error": None,
            "created_at": now,
            "started_at": None,
            "updated_at": now,
            "finished_at": None,
            "iteration_count": 0,
            "frame_count": 0,
            "active_node_id": None,
            "active_node_type": None,
            "cancel_requested": False,
            "graph_loaded": True,
            "graph": prepared_graph,
            "graph_summary": normalized_options["graph_summary"],
            "input_data": input_data or {},
            "options": normalized_options,
            "last_result": None,
            "last_result_at": None,
            "amr_commands": {},
            "fanuc_commands": {},
            "amr_auto_id_nodes": {node["id"] for node in prepared_graph.get("nodes", [])
                                  if node.get("type") == "amr_navigate" and not node.get("data", {}).get("commandId")},
            "logs": [
                "[SESSION] 그래프 런타임을 메모리에 적재했습니다.",
            ] + prepare_logs,
            "task": None,
        }

        with self._lock:
            self._sessions[session_id] = session

        task = asyncio.create_task(self._run_session(session_id))
        with self._lock:
            if session_id in self._sessions:
                self._sessions[session_id]["task"] = task

        return self.get_session(session_id)

    def get_session(self, session_id: str) -> Dict[str, Any]:
        session = self._get_session(session_id)
        with self._lock:
            return self._snapshot(session)

    async def cancel_session(self, session_id: str) -> Dict[str, Any]:
        with self._lock:
            session = self._sessions.get(session_id)
            if session is None:
                raise KeyError(session_id)

            if session["status"] in FINAL_SESSION_STATES:
                return self._snapshot(session)

            session["cancel_requested"] = True
            session["status"] = "cancelling"
            session["message"] = "취소 요청을 받아 현재 사이클을 정리 중입니다."
            session["updated_at"] = _now()
            self._append_logs(session, "[SESSION] Cancel requested")
            task = session.get("task")
            graph = copy.deepcopy(session.get("graph", {}) or {})

        stop_logs = await asyncio.to_thread(self._send_cancel_stop_commands, session_id, graph)
        if stop_logs:
            with self._lock:
                session = self._sessions.get(session_id)
                if session is not None:
                    self._append_logs(session, *stop_logs)

        if task is not None and task.done():
            try:
                await task
            except Exception:
                pass

        return self.get_session(session_id)

    async def _wait_for_initial_feedback(
        self,
        session_id: str,
        client_ids: List[str],
        timeout_sec: float,
    ) -> bool:
        if not client_ids:
            return True

        deadline = _now() + timeout_sec
        while _now() < deadline:
            if self._is_cancel_requested(session_id):
                return False

            ready = all(state_store.get_state(client_id) is not None for client_id in client_ids)
            if ready:
                return True

            await asyncio.sleep(0.1)

        return False

    async def _wait_for_feedback_update(
        self,
        session_id: str,
        client_ids: List[str],
        checkpoints: Dict[str, float],
        timeout_sec: float,
    ) -> bool:
        if not client_ids:
            return True

        deadline = _now() + timeout_sec
        while _now() < deadline:
            if self._is_cancel_requested(session_id):
                return False

            for client_id in client_ids:
                record = state_store.get_state(client_id)
                if record is None:
                    continue
                timestamp = float(record.get("timestamp", 0.0) or 0.0)
                if timestamp > float(checkpoints.get(client_id, 0.0) or 0.0):
                    return True

            await asyncio.sleep(0.1)

        return False

    async def _sleep_with_cancel(self, session_id: str, duration_sec: float) -> bool:
        deadline = _now() + max(0.0, duration_sec)
        while _now() < deadline:
            if self._is_cancel_requested(session_id):
                return False
            await asyncio.sleep(min(0.1, max(0.01, deadline - _now())))
        return not self._is_cancel_requested(session_id)

    async def _run_session(self, session_id: str) -> None:
        graph: Dict[str, Any] = {}
        try:
            session = self._get_session(session_id)
            graph = dict(session["graph"])
            options = dict(session["options"])

            if options["persistent"] and options["require_feedback"]:
                self._set_status(
                    session_id,
                    "waiting_feedback",
                    "그래프는 메모리에 올라갔고 첫 Unity 상태를 기다리는 중입니다.",
                )
                ready = await self._wait_for_initial_feedback(
                    session_id,
                    list(options["unity_state_client_ids"]),
                    float(options["feedback_timeout_sec"]),
                )
                if not ready:
                    if self._is_cancel_requested(session_id):
                        raise GraphRunCancelled("Runtime cancelled while waiting for initial feedback.")
                    raise RuntimeError(
                        "Unity 상태를 기다리다 timeout 되었습니다. "
                        "Unity state websocket 전송과 clientId 연결을 확인해 주세요."
                    )

            if bool(options.get("live_single_context")):
                await self._run_live_single_context_session(session_id, graph, options)
                return

            iteration = 0
            while True:
                if self._is_cancel_requested(session_id):
                    raise GraphRunCancelled("Runtime cancelled before next iteration.")

                iteration += 1
                self._start_iteration(session_id, iteration)
                feedback_client_ids = list(options["unity_command_client_ids"])
                pre_feedback_checkpoints = (
                    _current_state_timestamps(feedback_client_ids)
                    if options["require_feedback"]
                    else {}
                )

                result = await asyncio.to_thread(
                    run_graph,
                    graph,
                    session.get("input_data", {}),
                    on_node_result=lambda event, sid=session_id, current_iter=iteration: self._store_node_event(
                        sid,
                        current_iter,
                        event,
                    ),
                    on_node_start=lambda event: self._begin_amr_node(session_id, event),
                    stop_requested=lambda sid=session_id: self._is_cancel_requested(sid),
                )

                self._store_result(session_id, iteration, result)

                if self._is_cancel_requested(session_id):
                    raise GraphRunCancelled("Runtime cancelled after node execution.")

                if _result_has_errors(result) and not bool(options.get("continue_on_error", False)):
                    raise RuntimeError("그래프 실행 중 오류가 발생해 런타임을 중지했습니다.")

                await self._wait_for_amr_commands(session_id)

                if _result_requests_terminate(graph, result):
                    self._set_status(
                        session_id,
                        "completed",
                        f"{iteration}회차 실행에서 terminate 신호를 받아 런타임을 종료했습니다.",
                    )
                    return

                if not options["persistent"]:
                    self._set_status(
                        session_id,
                        "completed",
                        f"{iteration}회 그래프 실행을 완료했습니다.",
                    )
                    return

                max_iterations = int(options["max_iterations"])
                if max_iterations > 0 and iteration >= max_iterations:
                    self._set_status(
                        session_id,
                        "completed",
                        f"설정된 최대 반복 {max_iterations}회에 도달해 런타임을 종료했습니다.",
                    )
                    return

                if self._is_cancel_requested(session_id):
                    raise GraphRunCancelled("Runtime cancelled after iteration.")

                if options["require_feedback"]:
                    self._set_status(
                        session_id,
                        "waiting_feedback",
                        f"{iteration}회차 명령 전송을 마쳤고 Unity 처리 결과를 기다리는 중입니다.",
                    )
                    ready = await self._wait_for_feedback_update(
                        session_id,
                        feedback_client_ids,
                        pre_feedback_checkpoints,
                        float(options["feedback_timeout_sec"]),
                    )
                    if not ready:
                        if self._is_cancel_requested(session_id):
                            raise GraphRunCancelled("Runtime cancelled while waiting for feedback.")
                        raise RuntimeError(
                            "Unity 처리 결과(state update)를 기다리다 timeout 되었습니다."
                        )
                    continue

                self._set_status(
                    session_id,
                    "running",
                    f"{iteration}회차를 마쳤고 다음 사이클을 준비 중입니다.",
                )
                if not await self._sleep_with_cancel(
                    session_id,
                    float(options["cycle_interval_sec"]),
                ):
                    raise GraphRunCancelled("Runtime cancelled during cycle interval.")

        except GraphRunCancelled:
            self._set_status(
                session_id,
                "cancelled",
                "취소 요청을 반영해 그래프 런타임을 메모리에서 내렸습니다.",
            )
        except Exception as exc:
            self._set_status(
                session_id,
                "error",
                "그래프 런타임이 오류로 중지되었습니다.",
                error=str(exc),
            )
        finally:
            stop_logs = await asyncio.to_thread(self._stop_amr_commands, session_id)
            stop_logs += await asyncio.to_thread(self._stop_fanuc_commands, session_id)
            with self._lock:
                session = self._sessions.get(session_id, {})
                unresolved = [command["command_id"] for command in session.get("amr_commands", {}).values()
                              if command["status"] not in AMR_TERMINAL_STATES]
                original_error = session.get("error")
                unresolved_fanuc = [command["command_id"] for command in session.get("fanuc_commands", {}).values()
                                    if command["status"] not in {"completed", "failed", "cancelled"}]
            if unresolved:
                detail = "AMR stop unconfirmed: " + ", ".join(unresolved)
                self._set_status(session_id, "error", "AMR 정지 여부를 확인하지 못해 런타임을 오류 상태로 종료했습니다.",
                                 error=f"{original_error}; {detail}" if original_error else detail)
            if unresolved_fanuc:
                detail = "FANUC stop unconfirmed: " + ", ".join(unresolved_fanuc)
                self._set_status(session_id, "error", "FANUC 정지 여부를 확인하지 못했습니다.", error=detail)
            cleanup_logs = _reset_graph_runtime_state(graph) if isinstance(graph, dict) else []
            with self._lock:
                session = self._sessions.get(session_id)
                if session is not None:
                    self._append_logs(session, *stop_logs)
                    if cleanup_logs:
                        self._append_logs(session, *cleanup_logs)
                    session["task"] = None

    async def _run_live_single_context_session(
        self,
        session_id: str,
        graph: Dict[str, Any],
        options: Dict[str, Any],
    ) -> None:
        frame = 0
        live_context: Dict[str, Any] = {}
        while True:
            if self._is_cancel_requested(session_id):
                raise GraphRunCancelled("Runtime cancelled before next live frame.")

            frame += 1
            self._start_live_frame(session_id, frame)
            live_context["_live_frame"] = frame

            session = self._get_session(session_id)
            result = await asyncio.to_thread(
                run_graph,
                graph,
                session.get("input_data", {}),
                on_node_result=lambda event, sid=session_id, current_frame=frame: self._store_live_node_event(
                    sid,
                    current_frame,
                    event,
                ),
                on_node_start=lambda event: self._begin_amr_node(session_id, event),
                stop_requested=lambda sid=session_id: self._is_cancel_requested(sid),
                initial_context=live_context,
            )
            result["frame_count"] = frame
            result["iteration_count"] = 1
            self._store_live_result(session_id, frame, result)

            if self._is_cancel_requested(session_id):
                raise GraphRunCancelled("Runtime cancelled after live node execution.")

            if _result_has_errors(result) and not bool(options.get("continue_on_error", False)):
                raise RuntimeError("라이브 RGB-D 프레임 처리 중 오류가 발생해 런타임을 중지했습니다.")

            await self._wait_for_amr_commands(session_id)

            if _result_requests_terminate(graph, result):
                self._set_status(
                    session_id,
                    "completed",
                    f"라이브 RGB-D 세션에서 terminate 신호를 받아 {frame}프레임 후 종료했습니다.",
                )
                return

            max_iterations = int(options["max_iterations"])
            if max_iterations > 0 and frame >= max_iterations:
                self._set_status(
                    session_id,
                    "completed",
                    f"설정된 최대 라이브 프레임 {max_iterations}개에 도달해 런타임을 종료했습니다.",
                )
                return

            if not await self._sleep_with_cancel(
                session_id,
                float(options["cycle_interval_sec"]),
            ):
                raise GraphRunCancelled("Runtime cancelled during live frame interval.")


graph_runtime_manager = GraphRuntimeManager()














