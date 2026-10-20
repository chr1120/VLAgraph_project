"""
Isaac Sim bridge for VLA Graph.

이 파일은 Isaac Sim 안에서 직접 실행되는 "Omniverse 쪽 브리지"입니다.
VLA Graph의 Robot Execute(mode=omniverse) 노드가 WebSocket으로 명령을 보내면,
이 스크립트가 Isaac Sim update-thread에서 USD prim / 카메라 / Skeleton을 조작합니다.

Run inside Isaac Sim, for example from Window > Script Editor:

    exec(open(r"C:/Users/0/Desktop/vlagraph/examples/isaacsim/VlaGraphIsaacSimBridge.py").read())

The bridge listens for Robot Execute(mode=omniverse) commands at:

    ws://0.0.0.0:9900/omniverse

It also publishes robot state, and optionally camera images, to the existing
VLA Graph backend compatibility WebSocket endpoints.

큰 흐름:
1. Isaac Sim 내부에서 WebSocket 서버를 연다.
2. backend/graph에서 들어온 로봇 명령을 큐에 넣는다.
3. Isaac Sim update callback에서 큐를 비우며 실제 USD stage를 수정한다.
4. 현재 로봇 상태와 카메라 프레임을 backend WebSocket으로 다시 보낸다.
"""

from __future__ import annotations

import asyncio
import base64
import builtins
import contextlib
import importlib
import io
import json
import math
import os
import queue
import threading
import time
import traceback
from dataclasses import dataclass, field
from typing import Any, Dict, Optional
from urllib.parse import urlencode, urlparse, urlunparse

BRIDGE_BUILD = "2026-07-30-omy3m-direct-articulation-v42"


def _safe_articulation_name(articulation_path: str) -> str:
    name = "".join(char if char.isalnum() or char == "_" else "_" for char in articulation_path.strip("/"))
    return name or "vlagraph_articulation"


# ---------------------------------------------------------------------------
# 환경 변수와 payload를 안전하게 읽기 위한 작은 유틸리티들
# Isaac Sim Script Editor에서 실행할 때는 argparse보다 환경 변수가 다루기 쉽습니다.
# ---------------------------------------------------------------------------
def _env_bool(name: str, default: bool = False) -> bool:
    # VLAGRAPH_SEND_IMAGE=true 같은 값을 Python bool로 변환합니다.
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _env_float(name: str, default: float) -> float:
    # 잘못된 숫자가 들어와도 브리지가 죽지 않도록 기본값으로 되돌립니다.
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return float(value)
    except ValueError:
        return default


def _env_int(name: str, default: int) -> int:
    # 포트/해상도처럼 정수가 필요한 설정을 읽습니다.
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _default_omniverse_host() -> str:
    # Windows can reject 0.0.0.0 binds from embedded/plugin Python with WSAEACCES
    # depending on firewall or excluded-port policy. Loopback is enough for the
    # local Isaac Sim + Docker Desktop flow because the backend also tries
    # localhost/127.0.0.1 variants.
    if os.name == "nt":
        return "127.0.0.1"
    return "0.0.0.0"


def _command_bind_hosts(configured_host: str, explicit: bool) -> list[str]:
    if explicit:
        return [configured_host]

    hosts: list[str] = []
    for host in (configured_host, "127.0.0.1", "localhost", "0.0.0.0"):
        if host and host not in hosts:
            hosts.append(host)
    return hosts


def _env_float_list(name: str, default: list[float], length: int = 3) -> list[float]:
    # "0,0,1.65" 또는 "0 0 1.65" 같은 벡터 환경 변수를 [x, y, z]로 변환합니다.
    value = os.getenv(name)
    if value is None or not value.strip():
        return list(default[:length])

    normalized = value.replace(";", ",").replace(" ", ",")
    raw = [item for item in normalized.split(",") if item.strip()]
    out: list[float] = []
    for item in raw[:length]:
        try:
            out.append(float(item))
        except ValueError:
            out.append(0.0)
    while len(out) < length:
        out.append(default[len(out)] if len(default) > len(out) else 0.0)
    return out


def _child_prim_path(parent_path: str, child_name: str) -> str:
    # USD prim 이름에는 특수문자를 피하는 편이 안전해서 카메라 이름을 정리합니다.
    safe_name = "".join(
        char if char.isalnum() or char == "_" else "_"
        for char in (child_name.strip() or "VlaGraphCamera")
    )
    return f"{parent_path.rstrip('/')}/{safe_name}"


def _as_float_list(value: Any, length: int = 3) -> list[float]:
    # action payload는 list 또는 {"x":..,"y":..,"z":..} 형태가 섞여 들어올 수 있습니다.
    if isinstance(value, dict):
        raw = [value.get("x", 0.0), value.get("y", 0.0), value.get("z", 0.0)]
    elif isinstance(value, (list, tuple)):
        raw = list(value)
    else:
        raw = []

    out: list[float] = []
    for item in raw[:length]:
        try:
            out.append(float(item))
        except Exception:
            out.append(0.0)
    while len(out) < length:
        out.append(0.0)
    return out


def _as_float_value(value: Any, default: float = 0.0) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _as_float_sequence(value: Any) -> list[float]:
    if isinstance(value, dict):
        return []
    if isinstance(value, (str, bytes)) or value is None:
        return []
    if not isinstance(value, (list, tuple)):
        try:
            return [float(value)]
        except Exception:
            return []
    out: list[float] = []
    for item in value:
        try:
            out.append(float(item))
        except Exception:
            out.append(0.0)
    return out


def _as_nested_float_sequence(value: Any) -> list[float]:
    if isinstance(value, dict):
        return []
    if isinstance(value, (str, bytes)) or value is None:
        return []
    if not isinstance(value, (list, tuple)):
        try:
            return [float(value)]
        except Exception:
            return []
    out: list[float] = []

    def add(item: Any) -> None:
        if isinstance(item, (list, tuple)):
            for nested in item:
                add(nested)
            return
        try:
            out.append(float(item))
        except Exception:
            out.append(0.0)

    add(value)
    return out


def _json_safe(value: Any) -> Any:
    # WebSocket 응답은 JSON 직렬화 가능해야 하므로 USD/Gf 객체 등은 문자열로 낮춥니다.
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def _first_text_value(mapping: Dict[str, Any], *keys: str) -> str:
    # prompt/instruction/task처럼 같은 의미의 키가 여러 이름으로 올 때 첫 유효 값을 고릅니다.
    for key in keys:
        value = mapping.get(key)
        if value is None:
            continue
        text = str(value).strip()
        if text:
            return text
    return ""


def _preview_text(value: str, limit: int = 120) -> str:
    # Isaac Sim console 로그가 너무 길어지지 않도록 prompt를 짧게 표시합니다.
    text = " ".join(str(value or "").split())
    if len(text) <= limit:
        return text
    return text[: limit - 3] + "..."


G1_SCHEMA_NAME = "unitree_g1_gr00t_v1"
G1_WAIST_3DOF_JOINTS: tuple[str, ...] = (
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
)

G1_JOINT_GROUPS: dict[str, tuple[str, ...]] = {
    "waist": ("torso_joint",),
    "left_leg": (
        "left_hip_pitch_joint",
        "left_hip_roll_joint",
        "left_hip_yaw_joint",
        "left_knee_joint",
        "left_ankle_pitch_joint",
        "left_ankle_roll_joint",
    ),
    "right_leg": (
        "right_hip_pitch_joint",
        "right_hip_roll_joint",
        "right_hip_yaw_joint",
        "right_knee_joint",
        "right_ankle_pitch_joint",
        "right_ankle_roll_joint",
    ),
    "left_arm": (
        "left_shoulder_pitch_joint",
        "left_shoulder_roll_joint",
        "left_shoulder_yaw_joint",
        "left_elbow_pitch_joint",
        "left_elbow_roll_joint",
    ),
    "right_arm": (
        "right_shoulder_pitch_joint",
        "right_shoulder_roll_joint",
        "right_shoulder_yaw_joint",
        "right_elbow_pitch_joint",
        "right_elbow_roll_joint",
    ),
    "left_hand": (
        "left_zero_joint",
        "left_one_joint",
        "left_two_joint",
        "left_three_joint",
        "left_four_joint",
        "left_five_joint",
        "left_six_joint",
    ),
    "right_hand": (
        "right_zero_joint",
        "right_one_joint",
        "right_two_joint",
        "right_three_joint",
        "right_four_joint",
        "right_five_joint",
        "right_six_joint",
    ),
}
G1_GROUP_ORDER: tuple[str, ...] = (
    "waist",
    "left_leg",
    "right_leg",
    "left_arm",
    "right_arm",
    "left_hand",
    "right_hand",
)
G1_FULL_JOINT_ORDER: tuple[str, ...] = tuple(
    joint
    for group_name in G1_GROUP_ORDER
    for joint in G1_JOINT_GROUPS[group_name]
)
G1_ACTIVE_23DOF_JOINTS: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "torso_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_pitch_joint",
    "left_elbow_roll_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_pitch_joint",
    "right_elbow_roll_joint",
)
G1_UNITREE_29DOF_JOINTS: tuple[str, ...] = (
    "left_hip_pitch_joint",
    "left_hip_roll_joint",
    "left_hip_yaw_joint",
    "left_knee_joint",
    "left_ankle_pitch_joint",
    "left_ankle_roll_joint",
    "right_hip_pitch_joint",
    "right_hip_roll_joint",
    "right_hip_yaw_joint",
    "right_knee_joint",
    "right_ankle_pitch_joint",
    "right_ankle_roll_joint",
    "waist_yaw_joint",
    "waist_roll_joint",
    "waist_pitch_joint",
    "left_shoulder_pitch_joint",
    "left_shoulder_roll_joint",
    "left_shoulder_yaw_joint",
    "left_elbow_joint",
    "left_wrist_roll_joint",
    "left_wrist_pitch_joint",
    "left_wrist_yaw_joint",
    "right_shoulder_pitch_joint",
    "right_shoulder_roll_joint",
    "right_shoulder_yaw_joint",
    "right_elbow_joint",
    "right_wrist_roll_joint",
    "right_wrist_pitch_joint",
    "right_wrist_yaw_joint",
)
G1_NEUTRAL_JOINTS: dict[str, float] = {
    "torso_joint": 0.0,
    "waist_yaw_joint": 0.0,
    "waist_roll_joint": 0.0,
    "waist_pitch_joint": 0.0,
    "left_hip_pitch_joint": -0.08,
    "left_hip_roll_joint": 0.0,
    "left_hip_yaw_joint": 0.0,
    "left_knee_joint": 0.22,
    "left_ankle_pitch_joint": -0.12,
    "left_ankle_roll_joint": 0.0,
    "right_hip_pitch_joint": -0.08,
    "right_hip_roll_joint": 0.0,
    "right_hip_yaw_joint": 0.0,
    "right_knee_joint": 0.22,
    "right_ankle_pitch_joint": -0.12,
    "right_ankle_roll_joint": 0.0,
    "left_shoulder_pitch_joint": 0.0,
    "left_shoulder_roll_joint": 0.18,
    "left_shoulder_yaw_joint": 0.0,
    "left_elbow_pitch_joint": 0.35,
    "left_elbow_roll_joint": 0.0,
    "right_shoulder_pitch_joint": 0.0,
    "right_shoulder_roll_joint": -0.18,
    "right_shoulder_yaw_joint": 0.0,
    "right_elbow_pitch_joint": 0.35,
    "right_elbow_roll_joint": 0.0,
    "left_zero_joint": 0.0,
    "left_one_joint": 0.0,
    "left_two_joint": 0.0,
    "left_three_joint": 0.0,
    "left_four_joint": 0.0,
    "left_five_joint": 0.0,
    "left_six_joint": 0.0,
    "right_zero_joint": 0.0,
    "right_one_joint": 0.0,
    "right_two_joint": 0.0,
    "right_three_joint": 0.0,
    "right_four_joint": 0.0,
    "right_five_joint": 0.0,
    "right_six_joint": 0.0,
}
G1_JOINT_LIMITS_RAD: dict[str, tuple[float, float]] = {
    "torso_joint": (-0.45, 0.45),
    "waist_yaw_joint": (-0.45, 0.45),
    "waist_roll_joint": (-0.35, 0.35),
    "waist_pitch_joint": (-0.55, 0.55),
    "left_hip_pitch_joint": (-1.25, 1.0),
    "right_hip_pitch_joint": (-1.25, 1.0),
    "left_hip_roll_joint": (-0.65, 0.65),
    "right_hip_roll_joint": (-0.65, 0.65),
    "left_hip_yaw_joint": (-0.55, 0.55),
    "right_hip_yaw_joint": (-0.55, 0.55),
    "left_knee_joint": (0.0, 1.55),
    "right_knee_joint": (0.0, 1.55),
    "left_ankle_pitch_joint": (-0.65, 0.45),
    "right_ankle_pitch_joint": (-0.65, 0.45),
    "left_ankle_roll_joint": (-0.35, 0.35),
    "right_ankle_roll_joint": (-0.35, 0.35),
    "left_shoulder_pitch_joint": (-1.65, 1.05),
    "right_shoulder_pitch_joint": (-1.65, 1.05),
    "left_shoulder_roll_joint": (-0.55, 1.35),
    "right_shoulder_roll_joint": (-1.35, 0.55),
    "left_shoulder_yaw_joint": (-1.15, 1.15),
    "right_shoulder_yaw_joint": (-1.15, 1.15),
    "left_elbow_pitch_joint": (0.0, 1.55),
    "right_elbow_pitch_joint": (0.0, 1.55),
    "left_elbow_roll_joint": (-0.55, 0.55),
    "right_elbow_roll_joint": (-0.55, 0.55),
    "left_zero_joint": (-0.9, 0.9),
    "left_one_joint": (-0.9, 0.9),
    "left_two_joint": (-0.9, 0.9),
    "left_three_joint": (-0.9, 0.9),
    "left_four_joint": (-0.9, 0.9),
    "left_five_joint": (-0.9, 0.9),
    "left_six_joint": (-0.9, 0.9),
    "right_zero_joint": (-0.9, 0.9),
    "right_one_joint": (-0.9, 0.9),
    "right_two_joint": (-0.9, 0.9),
    "right_three_joint": (-0.9, 0.9),
    "right_four_joint": (-0.9, 0.9),
    "right_five_joint": (-0.9, 0.9),
    "right_six_joint": (-0.9, 0.9),
}
G1_JOINT_ALIASES: dict[str, tuple[str, ...]] = {
    "torso_joint": ("waist_joint", "waist_yaw_joint", "torso"),
    "left_elbow_pitch_joint": ("left_elbow_joint",),
    "right_elbow_pitch_joint": ("right_elbow_joint",),
    "left_elbow_roll_joint": ("left_wrist_roll_joint", "left_palm_joint"),
    "right_elbow_roll_joint": ("right_wrist_roll_joint", "right_palm_joint"),
}


def _g1_group_dimensions() -> dict[str, int]:
    return {name: len(joints) for name, joints in G1_JOINT_GROUPS.items()}


def _g1_schema_payload() -> dict[str, Any]:
    return {
        "name": G1_SCHEMA_NAME,
        "groups": {name: list(joints) for name, joints in G1_JOINT_GROUPS.items()},
        "group_order": list(G1_GROUP_ORDER),
        "group_dimensions": _g1_group_dimensions(),
        "full_joint_order": list(G1_FULL_JOINT_ORDER),
        "active_23dof_joint_order": list(G1_ACTIVE_23DOF_JOINTS),
        "unitree_29dof_joint_order": list(G1_UNITREE_29DOF_JOINTS),
        "waist_3dof_joint_order": list(G1_WAIST_3DOF_JOINTS),
    }


def _g1_joint_order_from_payload(action: Dict[str, Any], vector_length: int = 0) -> tuple[str, ...]:
    if not isinstance(action, dict):
        return ()

    for key in ("joint_names", "jointNames", "joints", "joint_order", "jointOrder"):
        value = action.get(key)
        if isinstance(value, (list, tuple)) and value:
            return tuple(str(name) for name in value)

    schema = action.get("joint_schema") or action.get("jointSchema") or action.get("schema")
    if isinstance(schema, dict):
        for key in ("joint_order", "jointOrder", "full_joint_order", "fullJointOrder"):
            value = schema.get(key)
            if isinstance(value, (list, tuple)) and value:
                return tuple(str(name) for name in value)
        for key in ("unitree_29dof_joint_order", "unitree29DofJointOrder", "active_23dof_joint_order"):
            value = schema.get(key)
            if isinstance(value, (list, tuple)) and value and (vector_length <= 0 or len(value) == vector_length):
                return tuple(str(name) for name in value)

    source = str(action.get("joint_order_source") or action.get("jointOrderSource") or "").lower()
    if vector_length == len(G1_UNITREE_29DOF_JOINTS) or "29" in source or "unitree" in source:
        return G1_UNITREE_29DOF_JOINTS
    if vector_length == len(G1_ACTIVE_23DOF_JOINTS) or "23" in source:
        return G1_ACTIVE_23DOF_JOINTS
    if vector_length == len(G1_FULL_JOINT_ORDER) and ("g1_schema" in source or "full" in source):
        return G1_FULL_JOINT_ORDER
    return ()


def _annotate_g1_action_schema(action: Dict[str, Any]) -> None:
    if not isinstance(action, dict):
        return
    action.setdefault("embodiment_tag", "REAL_G1")
    action.setdefault("joint_schema", _g1_schema_payload())
    raw_values = _as_nested_float_sequence(
        action.get("joint_targets")
        or action.get("jointTargets")
        or action.get("joint_positions")
        or action.get("jointPositions")
    )
    ordered_names = _g1_joint_order_from_payload(action, len(raw_values))
    if ordered_names:
        action.setdefault("joint_order", list(ordered_names))
        action.setdefault("joint_order_source", "g1_schema_named")


KYLE_USDSKEL_NEUTRAL_JOINTS_DEG: dict[str, list[float]] = {
    "Hips": [0.0, 0.0, 0.0],
    "Spine1": [0.0, 0.0, 0.0],
    "Spine2": [0.0, 0.0, 0.0],
    "Neck": [0.0, 0.0, 0.0],
    "Head": [0.0, 0.0, 0.0],
    "LeftShoulder": [0.0, 0.0, 0.0],
    "RightShoulder": [0.0, 0.0, 0.0],
    "LeftArm": [25.0, 0.0, -55.0],
    "RightArm": [25.0, 0.0, 55.0],
    "LeftForeArm": [8.0, 0.0, 0.0],
    "RightForeArm": [8.0, 0.0, 0.0],
    "LeftForearm": [8.0, 0.0, 0.0],
    "RightForearm": [8.0, 0.0, 0.0],
    "LeftLeg": [0.0, 0.0, 0.0],
    "RightLeg": [0.0, 0.0, 0.0],
    "LeftUpLeg": [0.0, 0.0, 0.0],
    "RightUpLeg": [0.0, 0.0, 0.0],
    "LeftKnee": [0.0, 0.0, 0.0],
    "RightKnee": [0.0, 0.0, 0.0],
}


def _joint_name_suffix_variants(name: str) -> list[str]:
    key = str(name or "").strip()
    if not key:
        return []
    names = [key]
    base = key.rsplit("/", 1)[-1]
    if base and base != key:
        names.append(base)
    for candidate in list(names):
        if candidate.endswith("_joint"):
            names.append(candidate[:-6])
        else:
            names.append(f"{candidate}_joint")
    return list(dict.fromkeys(item for item in names if item))


def _joint_name_candidates(name: str) -> tuple[str, ...]:
    candidates = _joint_name_suffix_variants(name)
    probe = set(candidates)
    for canonical, aliases in G1_JOINT_ALIASES.items():
        alias_family: list[str] = []
        alias_family.extend(_joint_name_suffix_variants(canonical))
        for alias in aliases:
            alias_family.extend(_joint_name_suffix_variants(alias))
        if probe.intersection(alias_family):
            candidates.extend(alias_family)
    return tuple(dict.fromkeys(item for item in candidates if item))


def _lookup_named_joint_target(named_targets: dict[str, float], joint_name: str) -> tuple[bool, float, str]:
    for candidate in _joint_name_candidates(joint_name):
        if candidate in named_targets:
            return True, float(named_targets[candidate]), candidate
    return False, 0.0, ""


def _canonical_g1_joint_name(joint_name: str) -> str:
    for candidate in _joint_name_candidates(joint_name):
        if candidate in G1_NEUTRAL_JOINTS:
            return candidate
    return ""


def _clamp_float(value: float, low: float, high: float) -> float:
    return max(low, min(high, value))


def _scaled_joint_limit(joint_name: str, limit_scale: float) -> tuple[float, float] | None:
    canonical = _canonical_g1_joint_name(joint_name)
    if not canonical:
        return None
    bounds = G1_JOINT_LIMITS_RAD.get(canonical)
    neutral = float(G1_NEUTRAL_JOINTS.get(canonical, 0.0))
    if not bounds:
        return neutral - 0.9, neutral + 0.9

    limit_scale = _clamp_float(float(limit_scale), 0.1, 1.0)
    low, high = bounds
    return (
        neutral + (float(low) - neutral) * limit_scale,
        neutral + (float(high) - neutral) * limit_scale,
    )


def _extract_prompt_context(payload: Dict[str, Any], action: Dict[str, Any]) -> Dict[str, str]:
    # prompt는 최상위 payload에도, action 내부에도 들어올 수 있어 양쪽을 모두 확인합니다.
    prompt = _first_text_value(
        payload,
        "prompt",
        "instruction",
        "model_prompt",
        "task",
        "goal",
        "text",
    )
    instruction = _first_text_value(payload, "instruction")
    model_prompt = _first_text_value(payload, "model_prompt", "prompt_with_context")

    if isinstance(action, dict):
        if not prompt:
            prompt = _first_text_value(
                action,
                "prompt",
                "instruction",
                "model_prompt",
                "task",
                "goal",
                "text",
            )
        if not instruction:
            instruction = _first_text_value(action, "instruction")
        if not model_prompt:
            model_prompt = _first_text_value(action, "model_prompt", "prompt_with_context")

    if not prompt and not instruction and not model_prompt:
        return {}

    context: Dict[str, str] = {}
    if prompt:
        context["prompt"] = prompt
    if instruction or prompt:
        context["instruction"] = instruction or prompt
    if model_prompt:
        context["model_prompt"] = model_prompt
    return context


def _http_base_to_ws(base_url: str, path: str, query: Dict[str, str]) -> str:
    # backend 기본 주소(http://localhost:9000)를 WebSocket 주소(ws://...)로 바꿉니다.
    parsed = urlparse(base_url.strip().rstrip("/"))
    scheme = "wss" if parsed.scheme == "https" else "ws"
    return urlunparse(
        (
            scheme,
            parsed.netloc,
            path,
            "",
            urlencode(query),
            "",
        )
    )


@dataclass
class PendingCommand:
    # 네트워크 thread에서 받은 명령을 Isaac Sim update-thread로 넘기기 위한 큐 아이템입니다.
    # done 이벤트가 set되면 네트워크 thread가 응답을 클라이언트로 돌려보냅니다.
    payload: Dict[str, Any]
    done: threading.Event = field(default_factory=threading.Event)
    response: Dict[str, Any] = field(default_factory=dict)


class VlaGraphIsaacSimBridge:
    def __init__(self) -> None:
        # ------------------------------------------------------------------
        # 1) Omniverse 명령 수신 서버 설정
        # backend의 Robot Execute(mode=omniverse) 노드가 이 주소로 접속합니다.
        # ------------------------------------------------------------------
        self._host_explicit = "VLAGRAPH_OMNIVERSE_HOST" in os.environ
        self.host = os.getenv("VLAGRAPH_OMNIVERSE_HOST", _default_omniverse_host())
        self.port = _env_int("VLAGRAPH_OMNIVERSE_PORT", 9900)
        self.path = os.getenv("VLAGRAPH_OMNIVERSE_PATH", "/omniverse")
        self.backend_base_url = os.getenv("VLAGRAPH_BACKEND_BASE", "http://localhost:9000")

        # ------------------------------------------------------------------
        # 2) backend에 상태/이미지를 보낼 때 사용할 클라이언트 식별자와 로봇 기본값
        # 그래프의 unity_state 노드에서 client_id=isaacsim01로 읽는 값과 맞춰야 합니다.
        # ------------------------------------------------------------------
        self.client_id = os.getenv("VLAGRAPH_CLIENT_ID", "isaacsim01")
        self.robot_prim_path = os.getenv("VLAGRAPH_ROBOT_PRIM", "/World/GR00T_Humanoid")
        self.articulation_prim_path = os.getenv("VLAGRAPH_ARTICULATION_PRIM", "").strip()
        self.root_motion_prim_path = os.getenv("VLAGRAPH_ROOT_MOTION_PRIM", "").strip()
        self.home_root_position = _env_float_list("VLAGRAPH_HOME_ROOT_POSITION", [], 3)
        self.home_root_rotation = _env_float_list("VLAGRAPH_HOME_ROOT_ROTATION_DEG", [0.0, 0.0, 0.0], 3)
        self.robot_type = os.getenv("VLAGRAPH_ROBOT_TYPE", "humanoid")
        self.controller_mode = os.getenv("VLAGRAPH_CONTROLLER_MODE", "locomotion")

        # ------------------------------------------------------------------
        # 3) 상태/카메라 publish 주기
        # 상태는 짧은 JSON이라 자주 보내도 괜찮지만, 이미지는 부하가 커서 더 느리게 보냅니다.
        # ------------------------------------------------------------------
        self.send_state = _env_bool("VLAGRAPH_SEND_STATE", True)
        self.send_image = _env_bool("VLAGRAPH_SEND_IMAGE", False)
        self.state_interval_sec = _env_float("VLAGRAPH_STATE_INTERVAL_SEC", 0.1)
        self.image_interval_sec = _env_float("VLAGRAPH_IMAGE_INTERVAL_SEC", 0.5)

        # ------------------------------------------------------------------
        # 4) KyleRobot에 자동으로 붙이는 카메라 설정
        # VLAGRAPH_SEND_IMAGE=true일 때 backend의 sensor_remote_image 노드가 이 이미지를 받습니다.
        # ------------------------------------------------------------------
        self.camera_width = _env_int("VLAGRAPH_CAMERA_WIDTH", 640)
        self.camera_height = _env_int("VLAGRAPH_CAMERA_HEIGHT", 360)
        self.auto_attach_camera = _env_bool("VLAGRAPH_AUTO_ATTACH_CAMERA", True)
        self.camera_name = os.getenv("VLAGRAPH_CAMERA_NAME", "VlaGraphCamera")
        configured_camera_prim = os.getenv("VLAGRAPH_CAMERA_PRIM", "").strip()
        self.camera_prim_path = configured_camera_prim
        self.camera_is_auto_path = False
        if not self.camera_prim_path and self.send_image and self.auto_attach_camera:
            self.camera_prim_path = _child_prim_path(self.robot_prim_path, self.camera_name)
            self.camera_is_auto_path = True
        self.camera_local_position = _env_float_list(
            "VLAGRAPH_CAMERA_LOCAL_POSITION",
            [0.0, 0.0, 1.65],
        )
        self.camera_local_rotation_deg = _env_float_list(
            "VLAGRAPH_CAMERA_LOCAL_ROTATION_DEG",
            [90.0, 0.0, 0.0],
        )
        self.camera_local_scale = _env_float_list(
            "VLAGRAPH_CAMERA_LOCAL_SCALE",
            [1.0, 1.0, 1.0],
        )
        self.camera_focal_length = _env_float("VLAGRAPH_CAMERA_FOCAL_LENGTH", 24.0)
        self.camera_horizontal_aperture = _env_float(
            "VLAGRAPH_CAMERA_HORIZONTAL_APERTURE",
            20.955,
        )
        self.camera_update_pose = _env_bool(
            "VLAGRAPH_CAMERA_UPDATE_POSE",
            self.camera_is_auto_path,
        )

        # ------------------------------------------------------------------
        # 5) action payload를 실제 USD root 이동/회전으로 바꿀 때 쓰는 스케일
        # move_z=0.45가 그대로 0.45m가 되지 않도록 안전한 축소 계수를 둡니다.
        # ------------------------------------------------------------------
        self.translation_scale = _env_float("VLAGRAPH_TRANSLATION_SCALE", 1.0)
        self.step_scale = _env_float("VLAGRAPH_STEP_SCALE", 0.25)
        self.turn_scale_deg = _env_float("VLAGRAPH_TURN_SCALE_DEG", 15.0)
        self.turn_sign = _env_float("VLAGRAPH_TURN_SIGN", 1.0)
        self.rotation_scale_deg = _env_float("VLAGRAPH_ROTATION_SCALE_DEG", 1.0)
        self.rotation_sign = _env_float_list("VLAGRAPH_ROTATION_SIGN", [1.0, 1.0, 1.0])
        self.pose_rotation_sign = _env_float_list("VLAGRAPH_POSE_ROTATION_SIGN", [1.0, 1.0, 1.0])
        self.command_timeout_sec = _env_float("VLAGRAPH_COMMAND_TIMEOUT_SEC", 10.0)
        self.print_prompts = _env_bool("VLAGRAPH_PRINT_PROMPTS", True)
        self.print_g1_leg_targets = _env_bool("VLAGRAPH_PRINT_G1_LEG_TARGETS", False)
        self.enforce_humanoid_joint_limits = _env_bool("VLAGRAPH_ENFORCE_HUMANOID_JOINT_LIMITS", True)
        self.humanoid_joint_limit_scale = _env_float("VLAGRAPH_HUMANOID_JOINT_LIMIT_SCALE", 0.90)
        self.humanoid_joint_max_delta_rad = _env_float("VLAGRAPH_HUMANOID_JOINT_MAX_DELTA_RAD", 0.14)
        self.reset_humanoid_on_first_command = _env_bool("VLAGRAPH_RESET_HUMANOID_ON_FIRST_COMMAND", True)
        self.humanoid_drive_stiffness = _env_float("VLAGRAPH_HUMANOID_DRIVE_STIFFNESS", 90.0)
        self.humanoid_drive_damping = _env_float("VLAGRAPH_HUMANOID_DRIVE_DAMPING", 36.0)
        self.reset_humanoid_each_trajectory = _env_bool("VLAGRAPH_RESET_HUMANOID_EACH_TRAJECTORY", True)
        self.reset_humanoid_after_trajectory = _env_bool("VLAGRAPH_RESET_HUMANOID_AFTER_TRAJECTORY", True)
        self.zero_joint_velocities = _env_bool("VLAGRAPH_ZERO_JOINT_VELOCITIES", True)
        # Joint trajectories own the humanoid root unless explicitly disabled.
        # Keeping this off pins the pelvis at the home height during squats and
        # kneeling motions, leaving both feet visibly above the ground.
        self.apply_trajectory_root_translation = _env_bool("VLAGRAPH_APPLY_TRAJECTORY_ROOT_TRANSLATION", True)
        self.lock_trajectory_root_rotation = _env_bool("VLAGRAPH_LOCK_TRAJECTORY_ROOT_ROTATION", False)
        self.trajectory_root_max_vertical_m = _env_float("VLAGRAPH_TRAJECTORY_ROOT_MAX_VERTICAL_M", 0.45)
        self.freeze_humanoid_when_idle = _env_bool("VLAGRAPH_FREEZE_HUMANOID_WHEN_IDLE", True)
        self.humanoid_idle_timeout_sec = _env_float("VLAGRAPH_HUMANOID_IDLE_TIMEOUT_SEC", 1.25)
        self.neutralize_humanoid_when_idle = _env_bool("VLAGRAPH_NEUTRALIZE_HUMANOID_WHEN_IDLE", True)
        self.restore_root_on_stop = _env_bool("VLAGRAPH_RESTORE_ROOT_ON_STOP", True)
        self.hold_root_motion_pose = _env_bool("VLAGRAPH_HOLD_ROOT_MOTION_POSE", False)
        self.root_pose_use_articulation_api = _env_bool("VLAGRAPH_ROOT_POSE_USE_ARTICULATION_API", False)
        self.trajectory_max_step_sec = _env_float("VLAGRAPH_TRAJECTORY_MAX_STEP_SEC", 0.05)

        # ------------------------------------------------------------------
        # 6) Skeleton / SkelAnimation 설정
        # 기존 Take_001 재생 또는 pose_sequence 기반 관절 샘플 적용에 사용합니다.
        # ------------------------------------------------------------------
        self.enable_animation = _env_bool("VLAGRAPH_ENABLE_ANIMATION", True)
        self.play_timeline = _env_bool("VLAGRAPH_PLAY_TIMELINE", True)
        self.play_timeline_on_start = _env_bool("VLAGRAPH_PLAY_TIMELINE_ON_START", False)
        self.disable_animation_for_articulation = _env_bool(
            "VLAGRAPH_DISABLE_ANIMATION_FOR_ARTICULATION",
            True,
        )
        self.animation_loop = _env_bool("VLAGRAPH_ANIMATION_LOOP", True)
        self.animation_prim_path = os.getenv("VLAGRAPH_ANIMATION_PRIM", "").strip()
        self.skeleton_prim_path = os.getenv("VLAGRAPH_SKELETON_PRIM", "").strip()
        self.pose_animation_prim_path = os.getenv("VLAGRAPH_POSE_ANIMATION_PRIM", "").strip()
        self.pose_animation_is_auto_path = False
        if not self.pose_animation_prim_path:
            self.pose_animation_prim_path = _child_prim_path(
                self.robot_prim_path,
                "VlaGraphPoseAnimation",
            )
            self.pose_animation_is_auto_path = True

        # ------------------------------------------------------------------
        # 7) thread 간 통신과 런타임 캐시
        # Isaac Sim의 USD 조작은 update callback에서 처리하고, 네트워크 thread는 큐만 넣습니다.
        # ------------------------------------------------------------------
        self._command_queue: queue.Queue[PendingCommand] = queue.Queue()
        self._publish_queue: queue.Queue[tuple[str, Dict[str, Any]]] = queue.Queue(maxsize=8)
        self._update_subscription: Any = None
        self._network_thread: Optional[threading.Thread] = None
        self._network_loop: Optional[asyncio.AbstractEventLoop] = None
        self._network_stop: Optional[asyncio.Event] = None
        self._network_started = threading.Event()
        self._network_failed = threading.Event()
        self._network_error: str = ""
        self._backend_ws_by_kind: Dict[str, Any] = {}
        self._backend_ws_next_connect_at_by_kind: Dict[str, float] = {}
        self._backend_ws_last_error_log_at_by_kind: Dict[str, float] = {}
        self._backend_ws_retry_interval_sec = _env_float(
            "VLAGRAPH_BACKEND_WS_RETRY_INTERVAL_SEC",
            2.0,
        )
        self._backend_ws_error_log_interval_sec = _env_float(
            "VLAGRAPH_BACKEND_WS_ERROR_LOG_INTERVAL_SEC",
            10.0,
        )
        self._articulations: Dict[str, Any] = {}
        self._articulation_reset_done: set[str] = set()
        self._last_joint_targets_by_articulation: Dict[str, list[float]] = {}
        self._active_joint_trajectory: Optional[Dict[str, Any]] = None
        self._last_fanuc_external_axis_log_at = 0.0
        # OMY3M trajectory state is deliberately isolated from G1/humanoid
        # trajectory state. Neither controller may consume the other's frames.
        self._active_omy3m_trajectory: Optional[Dict[str, Any]] = None
        self._rotation_by_prim: Dict[str, list[float]] = {}
        self._root_pose_position_by_prim: Dict[str, list[float]] = {}
        self._root_base_position_by_prim: Dict[str, list[float]] = {}
        self._initial_root_pose_by_prim: Dict[str, tuple[list[float], list[float]]] = {}
        self._next_state_publish = 0.0
        self._next_image_publish = 0.0
        self._rgb_annotator: Any = None
        self._render_product: Any = None
        self._camera_ready = False
        self._camera_notice_printed = False
        self._last_prompt = ""
        self._last_instruction = ""
        self._last_model_prompt = ""
        self._last_prompt_timestamp = 0.0
        self._prompt_history: list[Dict[str, Any]] = []
        self._last_motion_command_time = 0.0
        self._humanoid_frozen = False
        self._pose_animation_active_until = 0.0
        self._last_joint_safety_notice_at = 0.0
        self._last_live_apply_notice_at = 0.0
        self._live_apply_notice_interval_sec = _env_float(
            "VLAGRAPH_LIVE_APPLY_LOG_INTERVAL_SEC",
            1.0,
        )
        self._last_command_diagnostics: Dict[str, Any] = {}
        self._last_root_motion_debug: Dict[str, Any] = {}
        self._held_root_motion_pose: Dict[str, Any] = {}
        self._last_stage_unavailable_notice_at = 0.0
        self._last_robot_unresolved_notice_at = 0.0
        self._next_robot_resolve_attempt_at = 0.0
        self._robot_unresolved_notice_interval_sec = _env_float(
            "VLAGRAPH_ROBOT_UNRESOLVED_NOTICE_INTERVAL_SEC",
            10.0,
        )

        # 애니메이션/관절 prim 탐색은 비싸므로 최초 성공 후 경로를 캐시합니다.
        self._animation_setup_done = False
        self._animation_notice_printed = False
        self._joint_prim_paths: Dict[str, str] = {}

    def start(self) -> None:
        # Isaac Sim update loop에 콜백을 등록한 뒤 별도 thread에서 WebSocket 서버를 띄웁니다.
        self._install_update_callback()
        self._resolve_robot_prim_path()
        self._capture_initial_root_pose(self._root_motion_target_path(self.robot_prim_path))
        self._network_thread = threading.Thread(
            target=self._network_main,
            name="vlagraph-isaacsim-bridge",
            daemon=True,
        )
        self._network_thread.start()
        self._network_started.wait(timeout=1.0)
        if self._network_failed.is_set():
            print(
                "[VlaGraphIsaacSimBridge] start failed "
                f"command_ws=ws://{self.host}:{self.port}{self.path} "
                f"error={self._network_error or 'unknown'}"
            )
            return
        print(
            "[VlaGraphIsaacSimBridge] started "
            f"command_ws=ws://{self.host}:{self.port}{self.path} "
            f"backend={self.backend_base_url} client_id={self.client_id} "
            f"robot={self.robot_prim_path} build={BRIDGE_BUILD}"
        )

    def stop(self) -> None:
        # Script Editor에서 여러 번 실행해도 이전 콜백/네트워크 루프를 정리할 수 있게 합니다.
        if self._update_subscription is not None:
            try:
                self._update_subscription.unsubscribe()
            except Exception:
                pass
            self._update_subscription = None

        if self._network_loop is not None and self._network_stop is not None:
            try:
                self._network_loop.call_soon_threadsafe(self._network_stop.set)
            except Exception:
                pass
        if (
            self._network_thread is not None
            and self._network_thread.is_alive()
            and threading.current_thread() is not self._network_thread
        ):
            self._network_thread.join(timeout=2.0)
        self._network_thread = None
        self._network_loop = None
        self._network_stop = None

    def _is_invalidated_simulation_view_error(self, exc: Exception) -> bool:
        text = str(exc).lower()
        return (
            "simulation view object is invalidated" in text
            or "basearticulationview" in text
            or "getsharedmetatype" in text
        )

    def _discard_articulation_cache(self, articulation_path: str, reason: str = "") -> None:
        if not articulation_path:
            return
        self._articulations.pop(articulation_path, None)
        self._articulation_reset_done.discard(articulation_path)
        self._root_base_position_by_prim.pop(articulation_path, None)
        if self.articulation_prim_path == articulation_path:
            self.articulation_prim_path = ""
        if reason:
            print(
                "[VlaGraphIsaacSimBridge] articulation cache discarded "
                f"articulation={articulation_path} reason={reason}"
            )

    def _get_or_create_articulation(self, Articulation: Any, articulation_path: str) -> Any:
        articulation = self._articulations.get(articulation_path)
        if articulation is not None:
            return articulation

        name = _safe_articulation_name(articulation_path)
        constructor_attempts = (
            ((), {"prim_path": articulation_path, "name": name}),
            ((), {"prim_path": articulation_path}),
            ((), {"prim_paths_expr": articulation_path, "name": name}),
            ((), {"paths": articulation_path}),
            ((articulation_path,), {}),
        )
        errors: list[str] = []
        for args, kwargs in constructor_attempts:
            try:
                articulation = Articulation(*args, **kwargs)
                break
            except TypeError as exc:
                errors.append(str(exc))
                articulation = None
        if articulation is None:
            raise TypeError("; ".join(errors) or f"Could not construct Articulation for {articulation_path}")

        initialize = getattr(articulation, "initialize", None)
        if callable(initialize):
            initialize()
        self._articulations[articulation_path] = articulation
        return articulation

    def _root_motion_target_path(self, prim_path: str) -> str:
        # Articulation root는 관절 제어용으로 두고, 전체 이동/회전은 모델 래퍼에 적용합니다.
        # G1 같은 articulation root에 root xform을 계속 쓰면 physics reset/cancel 후
        # 기준 pose가 꼬여 아래로 처지는 경우가 있습니다.
        configured = str(self.root_motion_prim_path or "").strip()
        articulation_path = str(self.articulation_prim_path or "").strip()
        if configured and configured == articulation_path and self._valid_prim_path(prim_path):
            return prim_path
        if configured and self._valid_prim_path(configured):
            return configured
        if configured:
            resolved = self._find_prim_by_suffix(configured)
            if resolved:
                if resolved != configured:
                    print(
                        "[VlaGraphIsaacSimBridge] root motion prim resolved "
                        f"configured={configured} resolved={resolved}"
                    )
                return resolved
        if self._valid_prim_path(prim_path):
            return prim_path
        return self.articulation_prim_path or prim_path

    def _find_prim_by_suffix(self, prim_path: str) -> str:
        suffix = "/" + str(prim_path or "").strip().strip("/")
        if not suffix or suffix == "/":
            return ""
        stage = self._get_stage()
        if stage is None:
            return ""
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path.endswith(suffix):
                return path
        leaf = suffix.rsplit("/", 1)[-1]
        if not leaf:
            return ""
        matches = [str(prim.GetPath()) for prim in stage.Traverse() if str(prim.GetPath()).endswith(f"/{leaf}")]
        if len(matches) == 1:
            return matches[0]
        return ""

    def _capture_initial_root_pose(self, prim_path: str) -> None:
        if not prim_path or prim_path in self._initial_root_pose_by_prim:
            return
        try:
            position = self._prim_position(prim_path)
            rotation = list(self._rotation_by_prim.get(prim_path, [0.0, 0.0, 0.0]))
            self._initial_root_pose_by_prim[prim_path] = (position, rotation)
            self._root_base_position_by_prim.setdefault(prim_path, list(position))
            self._rotation_by_prim.setdefault(prim_path, list(rotation))
            print(
                "[VlaGraphIsaacSimBridge] initial root pose captured "
                f"prim={prim_path} position={position} rotation={rotation}"
            )
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] initial root pose capture skipped {prim_path}: {exc}")

    def _restore_initial_root_pose(self, prim_path: str) -> bool:
        if not prim_path:
            return False
        self._capture_initial_root_pose(prim_path)
        pose = self._initial_root_pose_by_prim.get(prim_path)
        if not pose:
            return False
        position, rotation = pose
        try:
            if prim_path in self._articulations:
                self._set_articulation_root_pose(prim_path, list(position), list(rotation))
                self._zero_articulation_root_velocity_by_path(prim_path)
            else:
                self._set_prim_pose(prim_path, list(position), list(rotation))
            self._root_base_position_by_prim[prim_path] = list(position)
            self._root_pose_position_by_prim[prim_path] = list(position)
            self._rotation_by_prim[prim_path] = list(rotation)
            print(
                "[VlaGraphIsaacSimBridge] root pose restored "
                f"prim={prim_path} position={position} rotation={rotation}"
            )
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] root pose restore skipped {prim_path}: {exc}")
            return False

    def _initial_root_position(self, prim_path: str) -> Optional[list[float]]:
        if not prim_path:
            return None
        self._capture_initial_root_pose(prim_path)
        pose = self._initial_root_pose_by_prim.get(prim_path)
        if not pose:
            return None
        return list(pose[0])

    def _reset_root_motion_baseline(self, prim_path: str) -> None:
        root_target_path = self._root_motion_target_path(prim_path)
        self._restore_initial_root_pose(root_target_path)
        initial_position = self._initial_root_position(root_target_path)
        if initial_position is not None:
            self._root_base_position_by_prim[root_target_path] = list(initial_position)
        if self.articulation_prim_path:
            self._restore_initial_root_pose(self.articulation_prim_path)
            articulation_initial = self._initial_root_position(self.articulation_prim_path)
            if articulation_initial is not None:
                self._root_base_position_by_prim[self.articulation_prim_path] = list(articulation_initial)

    def _set_root_motion_baseline_to_current(self, prim_path: str) -> None:
        root_target_path = self._root_motion_target_path(prim_path)
        try:
            position = self._prim_position(root_target_path)
            rotation = list(self._rotation_by_prim.get(root_target_path, [0.0, 0.0, 0.0]))
            self._root_base_position_by_prim[root_target_path] = list(position)
            self._root_pose_position_by_prim[root_target_path] = list(position)
            self._initial_root_pose_by_prim[root_target_path] = (list(position), list(rotation))
            self._rotation_by_prim[root_target_path] = list(rotation)
            if self.articulation_prim_path and self._valid_prim_path(self.articulation_prim_path):
                articulation_position = self._prim_position(self.articulation_prim_path)
                articulation_rotation = list(
                    self._rotation_by_prim.get(self.articulation_prim_path, [0.0, 0.0, 0.0])
                )
                self._root_base_position_by_prim[self.articulation_prim_path] = list(articulation_position)
                self._root_pose_position_by_prim[self.articulation_prim_path] = list(articulation_position)
                self._initial_root_pose_by_prim[self.articulation_prim_path] = (
                    list(articulation_position),
                    list(articulation_rotation),
                )
                self._rotation_by_prim[self.articulation_prim_path] = list(articulation_rotation)
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] root motion current baseline skipped: {exc}")

    def _explicit_home_root_pose(self, prim_path: str) -> tuple[list[float], list[float]]:
        root_target_path = self._root_motion_target_path(prim_path)
        try:
            current_position = self._prim_position(root_target_path)
        except Exception:
            current_position = [0.0, 0.0, 0.0]
        initial_pose = self._initial_root_pose_by_prim.get(root_target_path)
        initial_position = list(initial_pose[0]) if initial_pose else list(current_position)
        if len(self.home_root_position) >= 3:
            position = list(self.home_root_position[:3])
        else:
            position = [0.0, 0.0, float(initial_position[2])]
        rotation = list(self.home_root_rotation[:3])
        return position, rotation

    def _apply_explicit_home_root_pose(self, prim_path: str) -> bool:
        root_target_path = self._root_motion_target_path(prim_path)
        if not root_target_path:
            return False
        position, rotation = self._explicit_home_root_pose(prim_path)
        try:
            self._set_articulation_root_pose(root_target_path, position, rotation)
            self._initial_root_pose_by_prim[root_target_path] = (list(position), list(rotation))
            self._root_base_position_by_prim[root_target_path] = list(position)
            self._root_pose_position_by_prim[root_target_path] = list(position)
            self._rotation_by_prim[root_target_path] = list(rotation)
            print(
                "[VlaGraphIsaacSimBridge] explicit home root pose applied "
                f"prim={root_target_path} position={position} rotation={rotation}"
            )
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] explicit home root pose skipped {root_target_path}: {exc}")
            return False

    def _clear_humanoid_motion_state(self, reason: str) -> None:
        self._active_joint_trajectory = None
        self._last_motion_command_time = 0.0
        print(f"[VlaGraphIsaacSimBridge] humanoid motion state cleared reason={reason}")

    def _install_update_callback(self) -> None:
        # Isaac Sim stage 조작은 이 update callback에서 처리됩니다.
        # 네트워크 thread에서 직접 USD prim을 만지지 않는 것이 핵심입니다.
        import omni.kit.app

        app = omni.kit.app.get_app()
        self._update_subscription = (
            app.get_update_event_stream().create_subscription_to_pop(
                self._on_update,
                name="VLA Graph Isaac Sim Bridge",
            )
        )

    def _network_main(self) -> None:
        # websockets 서버는 asyncio 루프가 필요하므로 daemon thread에 전용 event loop를 둡니다.
        self._network_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._network_loop)
        self._network_stop = asyncio.Event()

        try:
            self._network_loop.run_until_complete(self._network_run())
        except OSError as exc:
            self._network_failed.set()
            self._network_error = str(exc)
            if getattr(exc, "errno", None) == 10013:
                print(
                    "[VlaGraphIsaacSimBridge] Windows denied the command server bind: "
                    f"ws://{self.host}:{self.port}{self.path}. "
                    "If VLAGRAPH_OMNIVERSE_HOST is set to 0.0.0.0, unset it or set it "
                    "to 127.0.0.1. Also check Windows excluded port ranges with "
                    "'netsh interface ipv4 show excludedportrange protocol=tcp', "
                    "or set VLAGRAPH_OMNIVERSE_PORT to another port."
                )
            elif getattr(exc, "errno", None) == 10048:
                print(
                    "[VlaGraphIsaacSimBridge] Port is already in use: "
                    f"ws://{self.host}:{self.port}{self.path}. "
                    "A previous bridge may still be running. Restart Isaac Sim, "
                    "run shutdown(), or set VLAGRAPH_OMNIVERSE_PORT to another port."
                )
            else:
                print(f"[VlaGraphIsaacSimBridge] network error: {exc}")
        except Exception:
            self._network_failed.set()
            self._network_error = traceback.format_exc(limit=3)
            print("[VlaGraphIsaacSimBridge] network thread failed:")
            print(traceback.format_exc())
        finally:
            self._network_loop.close()

    async def _network_run(self) -> None:
        # 이 서버는 backend의 Robot Execute 노드가 연결하는 명령 수신 포트입니다.
        try:
            import websockets
        except Exception as exc:
            print(
                "[VlaGraphIsaacSimBridge] Missing dependency 'websockets'. "
                "Install it with Isaac Sim python: ./python.sh -m pip install websockets"
            )
            print(f"[VlaGraphIsaacSimBridge] dependency error: {exc}")
            return

        last_bind_error: Optional[OSError] = None
        command_server = None
        for bind_host in _command_bind_hosts(self.host, self._host_explicit):
            try:
                command_server = await websockets.serve(
                    self._handle_command_ws,
                    bind_host,
                    self.port,
                    max_size=_env_int("VLAGRAPH_WS_MAX_SIZE", 16 * 1024 * 1024),
                    ping_interval=20,
                    ping_timeout=20,
                )
                self.host = bind_host
                if last_bind_error is not None:
                    print(
                        "[VlaGraphIsaacSimBridge] command server bind recovered on "
                        f"ws://{self.host}:{self.port}{self.path}"
                    )
                break
            except OSError as exc:
                last_bind_error = exc
                if self._host_explicit or getattr(exc, "errno", None) not in {10013, 10048}:
                    raise
                print(
                    "[VlaGraphIsaacSimBridge] command server bind failed on "
                    f"{bind_host}:{self.port}: {exc}; trying another local address"
                )
        if command_server is None:
            assert last_bind_error is not None
            raise last_bind_error

        self._network_started.set()
        # 상태/이미지 publish는 같은 네트워크 루프의 별도 task로 계속 처리합니다.
        publisher_task = asyncio.create_task(self._publisher_loop())

        try:
            assert self._network_stop is not None
            await self._network_stop.wait()
        finally:
            publisher_task.cancel()
            with contextlib.suppress(asyncio.CancelledError):
                await publisher_task
            await self._close_backend_websockets()
            command_server.close()
            await command_server.wait_closed()

    async def _handle_command_ws(self, websocket: Any, path: Optional[str] = None) -> None:
        # 하나의 WebSocket 연결 안에서 여러 명령이 들어올 수 있습니다.
        # path가 다르면 다른 서비스로 착각한 요청이므로 명확히 거절합니다.
        request_path = path or getattr(getattr(websocket, "request", None), "path", "") or ""
        request_path = request_path.split("?", 1)[0] or self.path
        if request_path != self.path:
            await websocket.send(
                json.dumps(
                    {
                        "ok": False,
                        "applied": False,
                        "message": f"Unsupported path: {request_path}",
                    }
                )
            )
            return

        async for raw_message in websocket:
            try:
                # backend에서 보내는 payload는 항상 JSON object여야 합니다.
                payload = json.loads(raw_message)
                if not isinstance(payload, dict):
                    raise ValueError("message must be a JSON object")
            except Exception as exc:
                await websocket.send(
                    json.dumps(
                        {
                            "ok": False,
                            "applied": False,
                            "message": f"Invalid JSON command: {exc}",
                        }
                    )
                )
                continue

            # 실제 USD 조작은 update-thread에서 하도록 pending command로 넘깁니다.
            pending = PendingCommand(payload=payload)
            self._command_queue.put(pending)

            # update-thread가 처리 완료 신호를 줄 때까지 기다린 뒤 ack를 응답합니다.
            loop = asyncio.get_running_loop()
            await loop.run_in_executor(None, pending.done.wait, self.command_timeout_sec)
            response = pending.response or {
                "ok": False,
                "type": "omniverse_ack",
                "request_id": payload.get("request_id"),
                "applied": False,
                "message": "Timed out waiting for Isaac Sim update thread.",
            }
            try:
                await websocket.send(json.dumps(_json_safe(response), ensure_ascii=False))
            except Exception as exc:
                # The browser/backend may intentionally close the command socket
                # after submitting a long-running motion.  A normal WebSocket
                # close (code 1000) is not a bridge or trajectory failure.
                received_close = getattr(exc, "rcvd", None)
                close_code = getattr(received_close, "code", None)
                connection_closed = any(
                    name in {"ConnectionClosed", "ConnectionClosedOK"}
                    for name in (base.__name__ for base in type(exc).__mro__)
                )
                if connection_closed or close_code == 1000:
                    print(
                        "[VlaGraphIsaacSimBridge] command ack skipped; "
                        f"client already closed request_id={payload.get('request_id')}"
                    )
                    return
                raise

    async def _publisher_loop(self) -> None:
        # update-thread가 생성한 state/image payload를 backend WebSocket으로 보내는 루프입니다.
        # 큐를 써서 렌더/update 흐름이 네트워크 지연에 막히지 않게 합니다.
        while True:
            try:
                kind, payload = self._publish_queue.get_nowait()
            except queue.Empty:
                await asyncio.sleep(0.03)
                continue

            try:
                await self._send_backend_payload(kind, payload)
            except Exception as exc:
                print(f"[VlaGraphIsaacSimBridge] publish {kind} failed: {exc}")

    def _backend_ws_url(self, kind: str) -> str:
        # 기존 Unity 호환 WebSocket endpoint를 그대로 재사용합니다.
        # Isaac Sim도 backend 입장에서는 unity_state/image client처럼 보입니다.
        if kind == "image":
            path = "/ws/unity/image"
        elif kind == "lidar":
            path = "/ws/unity/lidar"
        else:
            path = "/ws/unity/state"

        return _http_base_to_ws(
            self.backend_base_url,
            path,
            {"client_id": self.client_id},
        )

    def _websocket_is_closed(self, ws: Any) -> bool:
        # websockets 버전에 따라 closed/state 속성 모양이 달라서 둘 다 확인합니다.
        closed = getattr(ws, "closed", None)
        if isinstance(closed, bool):
            return closed

        state = getattr(ws, "state", None)
        if state is not None and str(state).upper().endswith("CLOSED"):
            return True

        return False

    async def _get_backend_websocket(self, kind: str) -> Any:
        # state/image/lidar 종류별로 backend 연결을 재사용합니다.
        # 닫힌 연결이면 버리고 새로 연결합니다.
        try:
            import websockets
        except Exception:
            return None

        ws = self._backend_ws_by_kind.get(kind)
        if ws is not None and not self._websocket_is_closed(ws):
            return ws

        if ws is not None:
            with contextlib.suppress(Exception):
                await ws.close()
            self._backend_ws_by_kind.pop(kind, None)

        now = time.time()
        next_connect_at = self._backend_ws_next_connect_at_by_kind.get(kind, 0.0)
        if now < next_connect_at:
            return None

        url = self._backend_ws_url(kind)
        try:
            ws = await websockets.connect(
                url,
                open_timeout=1.0,
                close_timeout=0.2,
                ping_interval=20,
                ping_timeout=20,
            )
        except Exception as exc:
            retry_interval = max(0.2, self._backend_ws_retry_interval_sec)
            self._backend_ws_next_connect_at_by_kind[kind] = now + retry_interval
            last_log_at = self._backend_ws_last_error_log_at_by_kind.get(kind, 0.0)
            if now >= last_log_at + max(1.0, self._backend_ws_error_log_interval_sec):
                self._backend_ws_last_error_log_at_by_kind[kind] = now
                print(
                    "[VlaGraphIsaacSimBridge] backend websocket unavailable "
                    f"kind={kind} url={url} retry_in={retry_interval:.1f}s error={exc}"
                )
            return None

        self._backend_ws_next_connect_at_by_kind.pop(kind, None)
        if kind in self._backend_ws_last_error_log_at_by_kind:
            print(f"[VlaGraphIsaacSimBridge] backend websocket recovered kind={kind} url={url}")
            self._backend_ws_last_error_log_at_by_kind.pop(kind, None)
        self._backend_ws_by_kind[kind] = ws
        return ws

    async def _close_backend_websockets(self) -> None:
        # bridge 종료 시 backend로 나가는 연결을 모두 닫습니다.
        for kind, ws in list(self._backend_ws_by_kind.items()):
            with contextlib.suppress(Exception):
                await ws.close()
            self._backend_ws_by_kind.pop(kind, None)

    async def _send_backend_payload_once(self, kind: str, payload: Dict[str, Any]) -> None:
        # backend는 일부 WebSocket에서 ack를 보내므로 짧게 recv를 기다립니다.
        # timeout은 정상 흐름으로 보고 무시합니다.
        ws = await self._get_backend_websocket(kind)
        if ws is None:
            return

        await ws.send(json.dumps(_json_safe(payload), ensure_ascii=False))
        try:
            await asyncio.wait_for(ws.recv(), timeout=1.0)
        except asyncio.TimeoutError:
            pass

    async def _send_backend_payload(self, kind: str, payload: Dict[str, Any]) -> None:
        # 전송 중 끊기면 연결을 버리고 한 번 재시도합니다.
        try:
            await self._send_backend_payload_once(kind, payload)
        except Exception:
            ws = self._backend_ws_by_kind.pop(kind, None)
            if ws is not None:
                with contextlib.suppress(Exception):
                    await ws.close()
            await self._send_backend_payload_once(kind, payload)

    def _on_update(self, _event: Any) -> None:
        # Isaac Sim의 매 프레임/update마다 호출되는 중심 루프입니다.
        # 명령 처리, 상태 publish, 이미지 publish가 여기서 스케줄됩니다.
        now = time.time()
        if self._get_stage() is None:
            if now >= self._last_stage_unavailable_notice_at + 5.0:
                self._last_stage_unavailable_notice_at = now
                print("[VlaGraphIsaacSimBridge] update skipped; USD stage is not available")
            return

        if now >= self._next_robot_resolve_attempt_at:
            self._next_robot_resolve_attempt_at = now + 5.0
            resolved_path = self._resolve_robot_prim_path()
            if self._valid_prim_path(resolved_path):
                self._capture_initial_root_pose(self._root_motion_target_path(resolved_path))

        if (
            self.enable_animation
            and not self._animation_setup_done
            and self._last_motion_command_time > 0.0
        ):
            self._setup_skeletal_animation()

        # 네트워크 thread가 넣어둔 명령을 Isaac Sim thread에서 처리합니다.
        self._drain_commands()
        if self._advance_omy3m_trajectory(now):
            self._last_motion_command_time = now
        if self._advance_joint_trajectory(now):
            self._last_motion_command_time = now
            self._humanoid_frozen = False
        self._freeze_humanoid_if_idle(now)
        self._hold_root_motion_pose_if_needed()

        # 로봇 위치/회전/prompt 상태를 주기적으로 backend에 보냅니다.
        if self.send_state and now >= self._next_state_publish:
            self._next_state_publish = now + max(0.02, self.state_interval_sec)
            payload = self._build_state_payload(now)
            self._offer_publish("state", payload)

        # 카메라 전송은 렌더 비용이 있으므로 별도 interval로 제한합니다.
        if self.send_image and self.camera_prim_path and now >= self._next_image_publish:
            self._next_image_publish = now + max(0.1, self.image_interval_sec)
            payload = self._build_image_payload(now)
            if payload:
                self._offer_publish("image", payload)

    def _offer_publish(self, kind: str, payload: Dict[str, Any]) -> None:
        # publish 큐가 꽉 차면 오래된 항목을 하나 버리고 최신 값을 우선합니다.
        # 이미지/상태는 과거 데이터보다 최신 데이터가 더 중요합니다.
        try:
            self._publish_queue.put_nowait((kind, payload))
        except queue.Full:
            try:
                self._publish_queue.get_nowait()
                self._publish_queue.put_nowait((kind, payload))
            except Exception:
                pass

    def _is_streamed_joint_command(self, payload: Dict[str, Any]) -> bool:
        action = payload.get("action") if isinstance(payload.get("action"), dict) else {}
        retarget_mode = str(action.get("retarget_mode") or action.get("retargetMode") or "").strip().lower()
        return retarget_mode in {"joint_targets", "streamed_joint_targets"} and not bool(
            action.get("stop")
            or action.get("terminate")
            or action.get("cancel")
            or action.get("freeze")
        )

    def _drain_commands(self) -> None:
        # update 한 번에 가능한 명령을 처리합니다. 실시간 RGB-D follow는 오래된 frame보다 최신 frame이 중요합니다.
        # 각 PendingCommand는 성공/실패 응답을 담고 done 이벤트를 켭니다.
        pending_items: list[PendingCommand] = []
        while True:
            try:
                pending_items.append(self._command_queue.get_nowait())
            except queue.Empty:
                break

        latest_stream_index = -1
        for index, pending in enumerate(pending_items):
            if self._is_streamed_joint_command(pending.payload):
                latest_stream_index = index

        for index, pending in enumerate(pending_items):
            if self._is_streamed_joint_command(pending.payload) and index != latest_stream_index:
                pending.response = {
                    "ok": True,
                    "type": "omniverse_ack",
                    "request_id": pending.payload.get("request_id"),
                    "applied": False,
                    "skipped": True,
                    "message": "Skipped stale streamed joint target; newer live frame is queued.",
                }
                pending.done.set()
                continue

            try:
                pending.response = self._apply_command(pending.payload)
                if pending.response.get("applied"):
                    command_key = str(pending.payload.get("command") or "").strip().lower()
                    action = pending.payload.get("action") if isinstance(pending.payload.get("action"), dict) else {}
                    stop_like = command_key in {"stop", "cancel", "pause", "freeze"} or bool(
                        action.get("stop", False)
                        or action.get("terminate", False)
                        or action.get("cancel", False)
                        or action.get("freeze", False)
                    )
                    if not stop_like:
                        self._last_motion_command_time = time.time()
                        self._humanoid_frozen = False
            except Exception as exc:
                pending.response = {
                    "ok": False,
                    "type": "omniverse_ack",
                    "request_id": pending.payload.get("request_id"),
                    "applied": False,
                    "message": f"Isaac Sim command failed: {exc}",
                    "trace": traceback.format_exc(limit=4),
                }
            finally:
                pending.done.set()

    def _get_stage(self) -> Any:
        # Isaac Sim 버전에 따라 stage 접근 API 위치가 다르므로 신/구 경로를 모두 지원합니다.
        try:
            from isaacsim.core.utils.stage import get_current_stage

            stage = get_current_stage()
            if stage is not None:
                return stage
        except Exception:
            pass

        try:
            import omni.usd

            return omni.usd.get_context().get_stage()
        except Exception:
            return None

    def _resolve_articulation_class(self) -> Any:
        # Isaac Sim 4.x/5.x/6.x가 Articulation 클래스를 서로 다른 모듈에 둡니다.
        # 6.x 문서는 experimental.prims.Articulation, 5.x 문서는 core.prims.SingleArticulation,
        # 구 버전은 omni.isaac.core.articulations.Articulation을 사용합니다.
        candidates = (
            ("isaacsim.core.api.articulations", "Articulation"),
            ("isaacsim.core.prims", "SingleArticulation"),
            ("isaacsim.core.prims", "Articulation"),
            ("isaacsim.core.experimental.prims", "Articulation"),
            ("omni.isaac.core.articulations", "Articulation"),
            ("omni.isaac.core.prims", "SingleArticulation"),
        )
        errors: list[str] = []
        for module_name, attr_name in candidates:
            try:
                module = importlib.import_module(module_name)
                articulation_class = getattr(module, attr_name)
                return articulation_class
            except Exception as exc:
                errors.append(f"{module_name}.{attr_name}: {exc}")
        raise ModuleNotFoundError("No Isaac Sim Articulation API found. Tried: " + " | ".join(errors))

    def _resolve_omy3m_articulation_class(self) -> Any:
        """Prefer the single-prim wrapper used by the OMY3M extension.

        Isaac Sim 5/6 exposes a collection-style Articulation class first, but
        that class expects an externally supplied simulation view in some Kit
        builds. SingleArticulation initializes its own view for one robot.
        """
        candidates = (
            ("isaacsim.core.prims", "SingleArticulation"),
            ("omni.isaac.core.prims", "SingleArticulation"),
            ("omni.isaac.core.articulations", "Articulation"),
            ("isaacsim.core.api.articulations", "Articulation"),
        )
        errors: list[str] = []
        for module_name, attr_name in candidates:
            try:
                module = importlib.import_module(module_name)
                return getattr(module, attr_name)
            except Exception as exc:
                errors.append(f"{module_name}.{attr_name}: {exc}")
        raise ModuleNotFoundError(
            "No OMY3M-compatible Articulation API found. Tried: " + " | ".join(errors)
        )

    def _call_first_articulation_method(
        self,
        articulation: Any,
        method_names: tuple[str, ...],
        value: Any,
    ) -> bool:
        for method_name in method_names:
            method = getattr(articulation, method_name, None)
            if not callable(method):
                continue
            try:
                method(value)
                return True
            except TypeError:
                with contextlib.suppress(Exception):
                    method(positions=value)
                    return True
            except Exception:
                raise
        return False

    def _set_articulation_joint_positions(self, articulation: Any, values: Any, *, target: bool) -> None:
        target_methods = (
            "set_joint_position_targets",
            "set_dof_position_targets",
            "set_joint_position_target",
        )
        position_methods = (
            "set_joint_positions",
            "set_dof_positions",
            "set_joint_position",
        )
        ordered_methods = target_methods + position_methods if target else position_methods + target_methods
        if not self._call_first_articulation_method(articulation, ordered_methods, values):
            raise AttributeError("Articulation has no supported joint position setter")

    def _get_prim(self, prim_path: str) -> Any:
        # prim이 없으면 후속 계산이 조용히 실패하지 않도록 즉시 명확한 예외를 냅니다.
        stage = self._get_stage()
        if stage is None:
            raise ValueError("USD stage is not available")
        prim = stage.GetPrimAtPath(prim_path)
        if not prim or not prim.IsValid():
            raise ValueError(f"Prim not found: {prim_path}")
        return prim

    def _prim_position(self, prim_path: str) -> list[float]:
        # 현재 local/world transform에서 translation만 뽑아 state payload와 이동 계산에 씁니다.
        from pxr import Usd, UsdGeom

        prim = self._get_prim(prim_path)
        xform = UsdGeom.Xformable(prim)
        matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = matrix.ExtractTranslation()
        return [float(translation[0]), float(translation[1]), float(translation[2])]

    def _cached_or_prim_position(self, prim_path: str) -> list[float]:
        cached = self._root_pose_position_by_prim.get(prim_path)
        if isinstance(cached, list) and len(cached) >= 3:
            return [float(cached[0]), float(cached[1]), float(cached[2])]
        return self._prim_position(prim_path)

    def _set_prim_pose(
        self,
        prim_path: str,
        position: list[float],
        rotation_euler_deg: list[float],
    ) -> None:
        # root prim 자체를 이동/회전합니다.
        # 보행 policy가 아직 물리 articulation을 직접 제어하지 않을 때 쓰는 단순 제어 경로입니다.
        from pxr import Gf, UsdGeom

        prim = self._get_prim(prim_path)
        xformable = UsdGeom.Xformable(prim)

        def find_op(op_type: Any) -> Any:
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == op_type:
                    return op
            return None

        translate_op = find_op(UsdGeom.XformOp.TypeTranslate)
        orient_op = find_op(UsdGeom.XformOp.TypeOrient)
        if translate_op is not None and orient_op is not None:
            translate_op.Set(Gf.Vec3d(*position))
            orient_op.Set(self._euler_deg_to_quatf(rotation_euler_deg))
        else:
            common = UsdGeom.XformCommonAPI(prim)
            common.SetTranslate(Gf.Vec3d(*position))
            common.SetRotate(Gf.Vec3f(*rotation_euler_deg), UsdGeom.XformCommonAPI.RotationOrderXYZ)
        self._root_pose_position_by_prim[prim_path] = list(position)
        self._rotation_by_prim[prim_path] = list(rotation_euler_deg)

    def _set_articulation_root_pose(
        self,
        prim_path: str,
        position: list[float],
        rotation_euler_deg: list[float],
    ) -> str:
        if not prim_path:
            return ""

        applied_with_articulation_api = False
        articulation = self._articulations.get(prim_path)
        if articulation is not None and self.root_pose_use_articulation_api:
            try:
                import numpy as np

                set_world_pose = getattr(articulation, "set_world_pose", None)
                if callable(set_world_pose):
                    orientation = np.asarray(
                        self._euler_deg_to_quat_wxyz(rotation_euler_deg),
                        dtype=np.float32,
                    )
                    try:
                        set_world_pose(
                            position=np.asarray(position, dtype=np.float32),
                            orientation=orientation,
                        )
                    except TypeError:
                        set_world_pose(np.asarray(position, dtype=np.float32), orientation)
                    applied_with_articulation_api = True
            except Exception as exc:
                if self._is_invalidated_simulation_view_error(exc):
                    self._discard_articulation_cache(prim_path, str(exc))
                print(f"[VlaGraphIsaacSimBridge] articulation root pose API skipped: {exc}")

        if not applied_with_articulation_api:
            self._set_prim_pose(prim_path, position, rotation_euler_deg)
        else:
            self._root_pose_position_by_prim[prim_path] = list(position)
            self._rotation_by_prim[prim_path] = list(rotation_euler_deg)
        self._last_root_motion_debug = {
            "prim_path": prim_path,
            "position": [float(position[0]), float(position[1]), float(position[2])],
            "rotation": [
                float(rotation_euler_deg[0]),
                float(rotation_euler_deg[1]),
                float(rotation_euler_deg[2]),
            ],
            "used_articulation_api": applied_with_articulation_api,
            "timestamp": time.time(),
        }
        if self.hold_root_motion_pose and prim_path == self._root_motion_target_path(self.robot_prim_path):
            self._held_root_motion_pose = {
                "prim_path": prim_path,
                "position": list(position),
                "rotation": list(rotation_euler_deg),
                "timestamp": time.time(),
            }
        if applied_with_articulation_api:
            return f"articulation+prim={prim_path}"
        return f"prim={prim_path}"

    def _hold_root_motion_pose_if_needed(self) -> None:
        if not self.hold_root_motion_pose:
            return
        held = self._held_root_motion_pose
        if not isinstance(held, dict) or not held:
            return
        prim_path = str(held.get("prim_path") or "")
        if not prim_path or not self._valid_prim_path(prim_path):
            return
        try:
            target_position = _as_float_list(held.get("position"))
            target_rotation = _as_float_list(held.get("rotation"))
            current_position = self._prim_position(prim_path)
            drift = max(abs(float(a) - float(b)) for a, b in zip(current_position, target_position))
            if drift > 1.0e-4:
                self._set_prim_pose(prim_path, target_position, target_rotation)
                self._last_root_motion_debug = {
                    "prim_path": prim_path,
                    "position": list(target_position),
                    "rotation": list(target_rotation),
                    "held": True,
                    "drift": drift,
                    "timestamp": time.time(),
                }
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] root motion hold skipped {prim_path}: {exc}")

    def _find_child_prim_by_type(self, parent_path: str, type_name: str) -> str:
        # Skeleton, SkelAnimation, Camera처럼 type 이름으로 하위 prim을 찾습니다.
        stage = self._get_stage()
        if stage is None:
            return ""
        prefix = parent_path.rstrip("/") + "/"
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path == parent_path or path.startswith(prefix):
                if prim.GetTypeName() == type_name:
                    return path
        return ""

    def _find_prim_by_type_anywhere(self, type_name: str) -> str:
        # 경로가 예상과 다를 때 마지막 fallback으로 stage 전체에서 타입을 찾습니다.
        stage = self._get_stage()
        if stage is None:
            return ""
        for prim in stage.Traverse():
            if prim.GetTypeName() == type_name:
                return str(prim.GetPath())
        return ""

    def _valid_prim_path(self, prim_path: str) -> bool:
        if not prim_path:
            return False
        stage = self._get_stage()
        if stage is None:
            return False
        prim = stage.GetPrimAtPath(prim_path)
        return bool(prim and prim.IsValid())

    def _prim_world_pose(self, prim: Any) -> Dict[str, Any]:
        from pxr import Usd, UsdGeom

        xform = UsdGeom.Xformable(prim)
        matrix = xform.ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        translation = matrix.ExtractTranslation()
        return {
            "position": [
                round(float(translation[0]), 6),
                round(float(translation[1]), 6),
                round(float(translation[2]), 6),
            ],
        }

    def _prim_world_bbox(self, prim: Any) -> Dict[str, Any]:
        try:
            from pxr import Usd, UsdGeom

            cache = UsdGeom.BBoxCache(
                Usd.TimeCode.Default(),
                [UsdGeom.Tokens.default_, UsdGeom.Tokens.render, UsdGeom.Tokens.proxy],
            )
            bound = cache.ComputeWorldBound(prim).ComputeAlignedBox()
            minimum = bound.GetMin()
            maximum = bound.GetMax()
            size = [
                round(float(maximum[0] - minimum[0]), 6),
                round(float(maximum[1] - minimum[1]), 6),
                round(float(maximum[2] - minimum[2]), 6),
            ]
            center = [
                round(float((minimum[0] + maximum[0]) * 0.5), 6),
                round(float((minimum[1] + maximum[1]) * 0.5), 6),
                round(float((minimum[2] + maximum[2]) * 0.5), 6),
            ]
            return {"center": center, "size": size}
        except Exception:
            return {"center": [], "size": []}

    def _prim_semantic_label(self, prim: Any) -> str:
        for attr_name in (
            "semantic:class",
            "semantic:label",
            "primvars:semantic:class",
            "primvars:semantic:label",
        ):
            try:
                attr = prim.GetAttribute(attr_name)
                if attr and attr.HasValue():
                    value = attr.Get()
                    text = str(value or "").strip()
                    if text:
                        return text
            except Exception:
                continue
        return str(prim.GetName() or "")

    def _scan_scene(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        from pxr import UsdGeom

        stage = self._get_stage()
        if stage is None:
            raise ValueError("USD stage is not available")
        root_path = str(payload.get("root_path") or payload.get("rootPath") or "/World")
        max_objects = int(payload.get("max_objects") or payload.get("maxObjects") or 256)
        include_types = {
            item.strip()
            for item in str(payload.get("include_types") or payload.get("includeTypes") or "").split(",")
            if item.strip()
        }
        exclude_tokens = {
            "camera",
            "light",
            "scope",
            "material",
            "shader",
            "skeleton",
            "skelanimation",
        }
        scene_id = (
            str(payload.get("scene_id") or payload.get("sceneId") or "").strip()
            or str(stage.GetRootLayer().identifier or "").strip()
            or "isaac_stage"
        )
        root_prim = stage.GetPrimAtPath(root_path)
        if not root_prim or not root_prim.IsValid():
            root_path = "/World"

        robot_path = str(payload.get("prim_path") or payload.get("robot_prim_path") or self.robot_prim_path)
        if not self._valid_prim_path(robot_path):
            robot_path = self._resolve_robot_prim_path() or robot_path

        robot: Dict[str, Any] = {"prim_path": robot_path}
        if self._valid_prim_path(robot_path):
            robot_prim = stage.GetPrimAtPath(robot_path)
            robot.update(self._prim_world_pose(robot_prim))
            robot["name"] = str(robot_prim.GetName())
            robot["type"] = str(robot_prim.GetTypeName())

        objects: list[Dict[str, Any]] = []
        truncated = False
        prefix = root_path.rstrip("/") + "/"
        robot_prefix = robot_path.rstrip("/") + "/"
        for prim in stage.Traverse():
            if len(objects) >= max(1, max_objects):
                truncated = True
                break
            if not prim or not prim.IsValid() or not prim.IsActive():
                continue
            path = str(prim.GetPath())
            if path == root_path or not path.startswith(prefix):
                continue
            if robot_path and (path == robot_path or path.startswith(robot_prefix)):
                continue
            type_name = str(prim.GetTypeName() or "")
            if include_types and type_name not in include_types:
                continue
            if type_name.lower() in exclude_tokens:
                continue
            if prim.IsAbstract():
                continue
            bbox = self._prim_world_bbox(prim)
            pose = self._prim_world_pose(prim)
            size = bbox.get("size") if isinstance(bbox.get("size"), list) else []
            if not size and type_name in {"Xform", ""}:
                continue
            label = self._prim_semantic_label(prim)
            objects.append(
                {
                    "name": str(prim.GetName()),
                    "label": label,
                    "prim_path": path,
                    "type": type_name,
                    "position": pose.get("position", []),
                    "bbox_center": bbox.get("center", []),
                    "size": size,
                }
            )

        return {
            "ok": True,
            "type": "scene_scan_result",
            "scene_id": scene_id,
            "stage_identifier": str(stage.GetRootLayer().identifier or ""),
            "root_path": root_path,
            "robot": robot,
            "objects": objects,
            "obstacles": [],
            "object_count": len(objects),
            "timestamp": time.time(),
            "meters_per_unit": float(UsdGeom.GetStageMetersPerUnit(stage)),
            "up_axis": str(UsdGeom.GetStageUpAxis(stage)),
            "truncated": truncated,
            "scan_scope": {
                "root_path": root_path,
                "max_objects": max(1, max_objects),
                "include_types": sorted(include_types),
                "excluded_types": sorted(exclude_tokens),
                "robot_excluded": True,
            },
        }

    def _stage_top_level_model_path(self, descendant_path: str) -> str:
        parts = [part for part in str(descendant_path).split("/") if part]
        if len(parts) >= 2 and parts[0] == "World":
            return f"/World/{parts[1]}"
        if parts:
            return f"/{parts[0]}"
        return ""

    def _find_robot_prim_candidate(self) -> str:
        stage = self._get_stage()
        if stage is None:
            return ""
        preferred_names = [
            str(self.robot_prim_path).rsplit("/", 1)[-1],
            "GROOT_Humanoid",
            "GR00T_Humanoid",
            "KyleRobot",
            "Humanoid",
            "G1",
            "Robot",
        ]
        seen_names: set[str] = set()
        for name in preferred_names:
            if not name or name in seen_names:
                continue
            seen_names.add(name)
            path = f"/World/{name}"
            prim = stage.GetPrimAtPath(path)
            if prim and prim.IsValid():
                return path

        skeleton_path = self._find_prim_by_type_anywhere("Skeleton")
        if skeleton_path:
            skel_root_path = self._find_ancestor_prim_by_type(skeleton_path, "SkelRoot")
            if skel_root_path:
                return skel_root_path
            top_level = self._stage_top_level_model_path(skeleton_path)
            if self._valid_prim_path(top_level):
                return top_level

        for prim in stage.Traverse():
            name = prim.GetName()
            path = str(prim.GetPath())
            lower = name.lower()
            if path == "/World":
                continue
            if any(token in lower for token in ("kyle", "humanoid", "robot", "g1")):
                return path
        return ""

    def _stage_top_level_preview(self, limit: int = 12) -> list[str]:
        try:
            stage = self._get_stage()
            names: list[str] = []
            for prim in stage.GetPseudoRoot().GetChildren():
                names.append(str(prim.GetPath()))
                if len(names) >= max(1, int(limit)):
                    break
            return names
        except Exception as exc:
            return [f"<stage preview failed: {exc}>"]

    def _refresh_robot_dependent_paths(self, old_robot_path: str) -> None:
        if self.camera_is_auto_path or (
            self.camera_prim_path
            and old_robot_path
            and str(self.camera_prim_path).startswith(old_robot_path.rstrip("/") + "/")
        ):
            self.camera_prim_path = _child_prim_path(self.robot_prim_path, self.camera_name)
            self.camera_is_auto_path = True
        if self.pose_animation_is_auto_path or (
            self.pose_animation_prim_path
            and old_robot_path
            and str(self.pose_animation_prim_path).startswith(old_robot_path.rstrip("/") + "/")
        ):
            self.pose_animation_prim_path = _child_prim_path(
                self.robot_prim_path,
                "VlaGraphPoseAnimation",
            )
            self.pose_animation_is_auto_path = True
        self._animation_setup_done = False
        self._animation_notice_printed = False
        self._joint_prim_paths.clear()
        self._camera_ready = False
        self._camera_notice_printed = False

    def _resolve_robot_prim_path(self) -> str:
        if self._valid_prim_path(self.robot_prim_path):
            return self.robot_prim_path

        old_robot_path = self.robot_prim_path
        candidate = self._find_robot_prim_candidate()
        if candidate and self._valid_prim_path(candidate):
            self.robot_prim_path = candidate
            self._refresh_robot_dependent_paths(old_robot_path)
            root_target_path = self._root_motion_target_path(self.robot_prim_path)
            self._capture_initial_root_pose(root_target_path)
            print(
                "[VlaGraphIsaacSimBridge] robot prim auto-resolved "
                f"from={old_robot_path} to={self.robot_prim_path} root={root_target_path}"
            )
            return self.robot_prim_path

        now = time.time()
        if now >= self._last_robot_unresolved_notice_at + self._robot_unresolved_notice_interval_sec:
            self._last_robot_unresolved_notice_at = now
            print(
                "[VlaGraphIsaacSimBridge] robot prim unresolved "
                f"configured={old_robot_path} stage_top={self._stage_top_level_preview()}. "
                "Load/init the humanoid USD stage or set VLAGRAPH_ROBOT_PRIM."
            )
        return self.robot_prim_path

    def _resolve_skeleton_path(self) -> str:
        # 환경 변수 경로가 틀려도 robot 아래 Skeleton을 다시 찾아서 복구합니다.
        self._resolve_robot_prim_path()
        if self._valid_prim_path(self.skeleton_prim_path):
            return self.skeleton_prim_path
        found = self._find_child_prim_by_type(self.robot_prim_path, "Skeleton")
        if found:
            return found
        return self._find_prim_by_type_anywhere("Skeleton")

    def _resolve_animation_path(self) -> str:
        # /World/KyleRobot/Take_001이 무효한 경우 실제 SkelAnimation prim을 자동 탐색합니다.
        self._resolve_robot_prim_path()
        if self._valid_prim_path(self.animation_prim_path):
            return self.animation_prim_path
        found = self._find_child_prim_by_type(self.robot_prim_path, "SkelAnimation")
        if found:
            return found
        return self._find_prim_by_type_anywhere("SkelAnimation")

    def _find_descendant_prim_by_name(self, parent_path: str, names: list[str]) -> str:
        # 모델마다 joint 이름이 조금씩 다를 수 있어 exact match 후 부분 match fallback을 둡니다.
        stage = self._get_stage()
        prefix = parent_path.rstrip("/") + "/"
        wanted = {name.strip().lower() for name in names if name.strip()}
        if not wanted:
            return ""

        fallback = ""
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path != parent_path and not path.startswith(prefix):
                continue
            prim_name = prim.GetName().lower()
            if prim_name in wanted:
                return path
            if not fallback and any(name in prim_name or prim_name in name for name in wanted):
                fallback = path
        return fallback

    def _joint_path_aliases(self, joint_name: str) -> list[str]:
        # Motion Retarget 노드가 보내는 표준 joint 이름을 KyleRobot/다른 rig 이름 후보로 확장합니다.
        # 예: LeftLeg가 실제 stage에서는 LeftUpLeg 또는 L_UpLeg일 수 있습니다.
        aliases: Dict[str, list[str]] = {
            "hips": ["Hips", "Hip", "Pelvis"],
            "spine1": ["Spine1", "Spine", "Chest"],
            "leftleg": ["LeftLeg", "LeftUpLeg", "L_UpLeg", "LeftUpperLeg"],
            "rightleg": ["RightLeg", "RightUpLeg", "R_UpLeg", "RightUpperLeg"],
            "leftupleg": ["LeftUpLeg", "LeftLeg", "L_UpLeg", "LeftUpperLeg"],
            "rightupleg": ["RightUpLeg", "RightLeg", "R_UpLeg", "RightUpperLeg"],
            "leftknee": ["LeftKnee", "LeftLeg", "LeftLowerLeg", "L_Leg"],
            "rightknee": ["RightKnee", "RightLeg", "RightLowerLeg", "R_Leg"],
            "leftarm": ["LeftArm", "LeftShoulder", "L_Arm", "LeftUpperArm"],
            "rightarm": ["RightArm", "RightShoulder", "R_Arm", "RightUpperArm"],
            "leftforearm": ["LeftForeArm", "LeftForearm", "LeftLowerArm", "L_ForeArm"],
            "rightforearm": ["RightForeArm", "RightForearm", "RightLowerArm", "R_ForeArm"],
        }
        normalized = joint_name.strip().lower()
        return aliases.get(normalized, [joint_name])

    def _find_joint_prim_path(self, joint_name: str) -> str:
        # pose_sequence 한 프레임마다 joint를 찾으면 느리므로, 한 번 찾은 경로는 캐시합니다.
        cached = self._joint_prim_paths.get(joint_name)
        if cached:
            return cached

        search_root = self._resolve_skeleton_path()
        if not search_root:
            search_root = self.robot_prim_path

        found = self._find_descendant_prim_by_name(
            search_root,
            self._joint_path_aliases(joint_name),
        )
        if found:
            self._joint_prim_paths[joint_name] = found
        return found

    def _find_ancestor_prim_by_type(self, prim_path: str, type_name: str) -> str:
        # Skeleton에서 위로 올라가며 SkelRoot 같은 상위 prim을 찾습니다.
        stage = self._get_stage()
        prim = stage.GetPrimAtPath(prim_path)
        while prim and prim.IsValid():
            if prim.GetTypeName() == type_name:
                return str(prim.GetPath())
            prim = prim.GetParent()
        return ""

    def _animation_time_range(self, animation_prim: Any) -> tuple[Optional[float], Optional[float]]:
        # SkelAnimation에 실제 time sample이 있으면 그 범위를 timeline 재생 구간으로 사용합니다.
        try:
            from pxr import UsdSkel

            animation = UsdSkel.Animation(animation_prim)
            attrs = [
                animation.GetTranslationsAttr(),
                animation.GetRotationsAttr(),
                animation.GetScalesAttr(),
            ]
            times: list[float] = []
            for attr in attrs:
                if not attr:
                    continue
                for sample in attr.GetTimeSamples():
                    times.append(float(sample))
            if times:
                return min(times), max(times)
        except Exception:
            pass

        try:
            stage = self._get_stage()
            start = float(stage.GetStartTimeCode())
            end = float(stage.GetEndTimeCode())
            if end > start:
                return start, end
        except Exception:
            pass

        return None, None

    def _play_animation_timeline(
        self,
        start_time: Optional[float],
        end_time: Optional[float],
    ) -> None:
        # Skeleton time-sample 또는 기존 Take_001 애니메이션을 Isaac Sim timeline으로 재생합니다.
        if not self.play_timeline:
            return

        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            if start_time is not None and hasattr(timeline, "set_start_time"):
                timeline.set_start_time(float(start_time))
            if end_time is not None and hasattr(timeline, "set_end_time"):
                timeline.set_end_time(float(end_time))
            if hasattr(timeline, "set_looping"):
                timeline.set_looping(bool(self.animation_loop))
            if start_time is not None and hasattr(timeline, "set_current_time"):
                timeline.set_current_time(float(start_time))
            timeline.play()
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] timeline play failed: {exc}")

    def _pause_animation_timeline(self) -> None:
        if self.articulation_prim_path and not self.enable_animation:
            return
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            pause = getattr(timeline, "pause", None)
            stop = getattr(timeline, "stop", None)
            if callable(pause):
                pause()
            elif callable(stop):
                stop()
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] timeline pause failed: {exc}")

    def _ensure_articulation_timeline_playing(self) -> None:
        if not self.articulation_prim_path:
            return
        try:
            import omni.timeline

            timeline = omni.timeline.get_timeline_interface()
            is_playing = getattr(timeline, "is_playing", None)
            if callable(is_playing) and is_playing():
                return
            timeline.play()
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] articulation timeline play skipped: {exc}")

    def _freeze_articulations(self, *, neutralize: bool = False) -> None:
        try:
            import numpy as np
        except Exception:
            np = None

        for articulation_path, articulation in list(self._articulations.items()):
            try:
                joint_names = self._articulation_joint_names(articulation)
                count = len(joint_names)
                if count <= 0:
                    previous = self._last_joint_targets_by_articulation.get(articulation_path, [])
                    count = len(previous)
                if np is not None and count > 0:
                    zeros = np.zeros(count, dtype=np.float32)
                    set_joint_velocities = getattr(articulation, "set_joint_velocities", None)
                    if callable(set_joint_velocities):
                        set_joint_velocities(zeros)

                hold_targets = None
                if neutralize and joint_names:
                    hold_targets = self._neutral_joint_vector(joint_names)
                    hold_targets = self._constrain_humanoid_joint_targets(
                        articulation_path,
                        joint_names,
                        hold_targets,
                        limit_delta=False,
                    )
                    self._last_joint_targets_by_articulation[articulation_path] = list(hold_targets)
                else:
                    hold_targets = self._last_joint_targets_by_articulation.get(articulation_path)
                if hold_targets and np is not None:
                    self._set_articulation_joint_positions(
                        articulation,
                        np.asarray(hold_targets, dtype=np.float32),
                        target=False,
                    )
            except Exception as exc:
                if self._is_invalidated_simulation_view_error(exc):
                    self._discard_articulation_cache(articulation_path, str(exc))
                print(f"[VlaGraphIsaacSimBridge] articulation freeze skipped {articulation_path}: {exc}")

    def _freeze_humanoid_motion(self, reason: str) -> None:
        self._clear_humanoid_motion_state(reason)
        self._pose_animation_active_until = 0.0
        self._pause_animation_timeline()
        neutralize = self.neutralize_humanoid_when_idle and reason in {
            "idle_timeout",
            "stop",
            "cancel",
            "pause",
            "freeze",
            "bridge_stop",
            "trajectory_complete",
        }
        if os.getenv("VLAGRAPH_FREEZE_ARTICULATION_ON_STOP", "").strip().lower() in {"1", "true", "yes", "on"}:
            self._freeze_articulations(neutralize=neutralize)
        if self.restore_root_on_stop and reason in {"stop", "cancel", "pause", "freeze", "bridge_stop"}:
            self._restore_initial_root_pose(self._root_motion_target_path(self.robot_prim_path))
            if self.articulation_prim_path:
                self._restore_initial_root_pose(self.articulation_prim_path)
                self._zero_articulation_velocities_by_path(self.articulation_prim_path)
        self._humanoid_frozen = True
        print(
            "[VlaGraphIsaacSimBridge] humanoid motion frozen "
            f"reason={reason} neutralize={neutralize}"
        )

    def _freeze_humanoid_if_idle(self, now: float) -> None:
        # G1 idle management must never pause/freeze an active OMY3M arm.
        if self._active_omy3m_trajectory is not None:
            return
        if not self.freeze_humanoid_when_idle or self._humanoid_frozen:
            return
        if now < self._pose_animation_active_until:
            return
        if self._last_motion_command_time <= 0.0:
            return
        if now - self._last_motion_command_time < max(0.1, self.humanoid_idle_timeout_sec):
            return
        self._freeze_humanoid_motion("idle_timeout")

    def _mark_pose_animation_active(self, start_time: float, end_time: float) -> None:
        fps = self._stage_time_codes_per_second()
        duration_sec = max(0.5, (float(end_time) - float(start_time)) / max(1.0, fps))
        self._pose_animation_active_until = time.time() + duration_sec + 0.5
        self._humanoid_frozen = False

    def _stage_time_codes_per_second(self) -> float:
        # 현재 stage의 timeCode/sec를 읽어 영상 timestamp를 USD timeline time code로 바꿀 때 씁니다.
        try:
            fps = float(self._get_stage().GetTimeCodesPerSecond())
            if fps > 0:
                return fps
        except Exception:
            pass
        return 24.0

    def _pose_sequence_time_codes(
        self,
        sequence: list[Any],
        action: Dict[str, Any],
    ) -> list[float]:
        # pose_sequence의 timestamp_sec를 우선 사용하고, 없으면 duration_sec 기준으로 균등 배치합니다.
        fps = self._stage_time_codes_per_second()
        duration_sec = 0.0
        try:
            duration_sec = max(0.0, float(action.get("duration_sec", action.get("durationSec", 0.0))))
        except Exception:
            duration_sec = 0.0

        timestamps: list[Optional[float]] = []
        for frame in sequence:
            if not isinstance(frame, dict):
                timestamps.append(None)
                continue
            try:
                timestamps.append(float(frame.get("timestamp_sec")))
            except Exception:
                timestamps.append(None)

        valid_timestamps = [value for value in timestamps if value is not None]
        if len(valid_timestamps) >= 2 and max(valid_timestamps) > min(valid_timestamps):
            first = min(valid_timestamps)
            span = max(valid_timestamps) - first
            total_time = (duration_sec if duration_sec > 0 else span) * fps
            return [
                ((timestamp - first) / span) * total_time if timestamp is not None else float(index)
                for index, timestamp in enumerate(timestamps)
            ]

        if duration_sec > 0 and len(sequence) > 1:
            total_time = duration_sec * fps
            return [
                (index / max(1, len(sequence) - 1)) * total_time
                for index in range(len(sequence))
            ]

        time_codes: list[float] = []
        for index, frame in enumerate(sequence):
            if isinstance(frame, dict):
                try:
                    time_codes.append(float(frame.get("time_code", index)))
                    continue
                except Exception:
                    pass
            time_codes.append(float(index))
        return time_codes

    def _euler_deg_to_quat_tuple(self, rotation_euler_deg: list[float]) -> tuple[float, float, float, float]:
        x_deg, y_deg, z_deg = _as_float_list(rotation_euler_deg)
        hx = math.radians(x_deg) * 0.5
        hy = math.radians(y_deg) * 0.5
        hz = math.radians(z_deg) * 0.5

        qx = (math.cos(hx), math.sin(hx), 0.0, 0.0)
        qy = (math.cos(hy), 0.0, math.sin(hy), 0.0)
        qz = (math.cos(hz), 0.0, 0.0, math.sin(hz))

        def multiply(a: tuple[float, float, float, float], b: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
            aw, ax, ay, az = a
            bw, bx, by, bz = b
            return (
                aw * bw - ax * bx - ay * by - az * bz,
                aw * bx + ax * bw + ay * bz - az * by,
                aw * by - ax * bz + ay * bw + az * bx,
                aw * bz + ax * by - ay * bx + az * bw,
            )

        # rotateXYZ와 비슷하게 X -> Y -> Z 순서로 합성합니다.
        w, x, y, z = multiply(qz, multiply(qy, qx))
        length = math.sqrt(w * w + x * x + y * y + z * z)
        if length <= 1.0e-8:
            return (1.0, 0.0, 0.0, 0.0)
        return (w / length, x / length, y / length, z / length)

    def _quat_dot(
        self,
        left: tuple[float, float, float, float],
        right: tuple[float, float, float, float],
    ) -> float:
        return (
            left[0] * right[0]
            + left[1] * right[1]
            + left[2] * right[2]
            + left[3] * right[3]
        )

    def _quat_tuple_to_quatf(self, quat: tuple[float, float, float, float]) -> Any:
        from pxr import Gf

        w, x, y, z = quat
        return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))

    def _euler_deg_to_quatf(self, rotation_euler_deg: list[float]) -> Any:
        # UsdSkel.Animation/xform orient는 Euler가 아니라 quaternion을 요구합니다.
        return self._quat_tuple_to_quatf(self._euler_deg_to_quat_tuple(rotation_euler_deg))

    def _euler_deg_to_quat_wxyz(self, rotation_euler_deg: list[float]) -> list[float]:
        # Isaac articulation APIs expect a plain numeric quaternion, not pxr.Gf.Quatf.
        w, x, y, z = self._euler_deg_to_quat_tuple(rotation_euler_deg)
        return [float(w), float(x), float(y), float(z)]

    def _bind_animation_source(self, skeleton_path: str, animation_path: str) -> list[str]:
        # Skeleton 또는 SkelRoot의 animation source를 지정한 SkelAnimation으로 갈아끼웁니다.
        from pxr import Sdf, UsdSkel

        stage = self._get_stage()
        target = Sdf.Path(animation_path)
        bind_paths = [skeleton_path]
        skel_root_path = self._find_ancestor_prim_by_type(skeleton_path, "SkelRoot")
        if skel_root_path and skel_root_path not in bind_paths:
            bind_paths.append(skel_root_path)

        for bind_path in bind_paths:
            bind_prim = stage.GetPrimAtPath(bind_path)
            if not bind_prim or not bind_prim.IsValid():
                continue
            binding_api = UsdSkel.BindingAPI.Apply(bind_prim)
            binding_api.CreateAnimationSourceRel().SetTargets([target])
        return bind_paths

    def _clear_animation_source(self) -> None:
        try:
            from pxr import UsdSkel

            stage = self._get_stage()
            skeleton_path = self._resolve_skeleton_path()
            if not skeleton_path:
                return

            bind_paths = [skeleton_path]
            skel_root_path = self._find_ancestor_prim_by_type(skeleton_path, "SkelRoot")
            if skel_root_path and skel_root_path not in bind_paths:
                bind_paths.append(skel_root_path)

            for bind_path in bind_paths:
                bind_prim = stage.GetPrimAtPath(bind_path)
                if not bind_prim or not bind_prim.IsValid():
                    continue
                binding_api = UsdSkel.BindingAPI.Apply(bind_prim)
                binding_api.CreateAnimationSourceRel().ClearTargets(True)
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] animation source clear skipped: {exc}")

    def _skeleton_joint_tokens(self, skeleton_path: str) -> tuple[Any, list[str]]:
        # Skeleton의 공식 joint token 순서를 읽습니다. UsdSkel.Animation rotations 배열은 이 순서를 따라야 합니다.
        from pxr import UsdSkel

        stage = self._get_stage()
        skeleton_prim = stage.GetPrimAtPath(skeleton_path)
        if not skeleton_prim or not skeleton_prim.IsValid():
            return None, []

        skeleton = UsdSkel.Skeleton(skeleton_prim)
        joints_value = skeleton.GetJointsAttr().Get()
        if not joints_value:
            return None, []
        return joints_value, [str(item) for item in joints_value]

    def _rotation_for_skeleton_joint(
        self,
        skeleton_joint_token: str,
        frame_joints: Dict[str, Any],
    ) -> Optional[list[float]]:
        # Skeleton joint token은 "Hips/LeftLeg"처럼 경로형이고,
        # pose_sequence는 "LeftLeg"처럼 짧은 이름이라 alias 후보를 만들어 매칭합니다.
        short_name = skeleton_joint_token.rsplit("/", 1)[-1]
        candidates = [skeleton_joint_token, short_name]
        candidates.extend(self._joint_path_aliases(short_name))

        lower_to_key = {str(key).lower(): key for key in frame_joints.keys()}
        for candidate in candidates:
            if candidate in frame_joints:
                rotation = _as_float_list(frame_joints[candidate])
                return [rotation[index] * self.pose_rotation_sign[index] for index in range(3)]
            key = lower_to_key.get(str(candidate).lower())
            if key is not None:
                rotation = _as_float_list(frame_joints[key])
                return [rotation[index] * self.pose_rotation_sign[index] for index in range(3)]
        return None

    def _neutral_rotation_for_skeleton_joint(self, skeleton_joint_token: str) -> list[float]:
        short_name = skeleton_joint_token.rsplit("/", 1)[-1]
        candidates = [skeleton_joint_token, short_name]
        candidates.extend(self._joint_path_aliases(short_name))
        for candidate in candidates:
            neutral = KYLE_USDSKEL_NEUTRAL_JOINTS_DEG.get(str(candidate))
            if neutral is not None:
                return [float(value) for value in neutral[:3]]
        return [0.0, 0.0, 0.0]

    def _root_translation_from_pose_frame(self, frame: Dict[str, Any]) -> Optional[list[float]]:
        for key in (
            "root_translation",
            "rootTranslation",
            "root_position",
            "rootPosition",
            "hips_position",
            "hipsPosition",
        ):
            value = frame.get(key)
            if value is not None:
                return _as_float_list(value)
        return None

    def _translation_for_skeleton_joint(
        self,
        skeleton_joint_token: str,
        frame: Dict[str, Any],
    ) -> list[float]:
        return [0.0, 0.0, 0.0]

    def _set_root_translation_sample(
        self,
        prim_path: str,
        translation: list[float],
        time_code: float,
        *,
        mode: str = "offset_from_initial",
    ) -> bool:
        try:
            from pxr import Gf, Usd, UsdGeom

            stage = self._get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False

            xformable = UsdGeom.Xformable(prim)
            translate_op = None
            other_ops = []
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate and translate_op is None:
                    translate_op = op
                else:
                    other_ops.append(op)
            if translate_op is None:
                translate_op = xformable.AddTranslateOp()
            xformable.SetXformOpOrder([translate_op, *other_ops])

            current_position = self._prim_position(prim_path)
            mode_key = str(mode or "").strip().lower()
            if mode_key in {"offset_from_initial", "initial_offset", "base_offset"}:
                initial_position = self._initial_root_position(prim_path)
                base_position = self._root_base_position_by_prim.setdefault(
                    prim_path,
                    list(initial_position or current_position),
                )
                target_position = [
                    base_position[0] + translation[0],
                    base_position[1] + translation[1],
                    base_position[2] + translation[2],
                ]
            else:
                target_position = _as_float_list(translation)

            target_vec = Gf.Vec3d(*target_position)
            translate_op.Set(target_vec)
            translate_op.Set(target_vec, Usd.TimeCode(float(time_code)))
            self._root_pose_position_by_prim[prim_path] = list(target_position)
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] root translation sample skipped {prim_path}: {exc}")
            return False

    def _apply_usdskel_pose_sequence(
        self,
        action: Dict[str, Any],
        sequence: list[Any],
    ) -> str:
        # 가장 중요한 경로입니다.
        # Motion Retarget이 보낸 pose_sequence를 새 UsdSkel.Animation prim으로 만들고 Skeleton에 바인딩합니다.
        try:
            from pxr import Gf, Usd, UsdSkel, Vt

            if not self.enable_animation:
                return ""

            stage = self._get_stage()
            skeleton_path = self._resolve_skeleton_path()
            if not skeleton_path:
                return ""

            joints_value, joint_tokens = self._skeleton_joint_tokens(skeleton_path)
            if not joint_tokens:
                return ""

            animation = UsdSkel.Animation.Define(stage, self.pose_animation_prim_path)
            animation_prim = animation.GetPrim()
            animation.CreateJointsAttr().Set(joints_value)
            rotations_attr = animation.CreateRotationsAttr()
            translations_attr = animation.CreateTranslationsAttr()

            time_codes = self._pose_sequence_time_codes(sequence, action)
            previous_quats: list[Optional[tuple[float, float, float, float]]] = [None] * len(joint_tokens)
            applied_frames = 0
            applied_samples = 0
            applied_translation_frames = 0
            root_motion_path = self._resolve_robot_prim_path() or skeleton_path
            root_translation_mode = str(
                action.get("root_translation_mode")
                or action.get("rootTranslationMode")
                or "offset_from_initial"
            )

            for index, frame in enumerate(sequence):
                if not isinstance(frame, dict):
                    continue
                frame_joints = frame.get("joints")
                if not isinstance(frame_joints, dict):
                    continue

                rotations = []
                translations = []
                frame_samples = 0
                root_translation = self._root_translation_from_pose_frame(frame)
                for joint_index, joint_token in enumerate(joint_tokens):
                    rotation = self._rotation_for_skeleton_joint(joint_token, frame_joints)
                    if rotation is None:
                        rotation = self._neutral_rotation_for_skeleton_joint(joint_token)
                    quat = self._euler_deg_to_quat_tuple(rotation)
                    previous = previous_quats[joint_index]
                    if previous is not None and self._quat_dot(previous, quat) < 0.0:
                        quat = (-quat[0], -quat[1], -quat[2], -quat[3])
                    previous_quats[joint_index] = quat
                    rotations.append(self._quat_tuple_to_quatf(quat))
                    translations.append(Gf.Vec3f(*self._translation_for_skeleton_joint(joint_token, frame)))
                    frame_samples += 1

                if frame_samples <= 0:
                    continue

                time_code = time_codes[index] if index < len(time_codes) else float(index)
                rotations_attr.Set(Vt.QuatfArray(rotations), Usd.TimeCode(float(time_code)))
                if root_translation is not None:
                    translations_attr.Set(Vt.Vec3fArray(translations), Usd.TimeCode(float(time_code)))
                    if self._set_root_translation_sample(
                        root_motion_path,
                        root_translation,
                        float(time_code),
                        mode=root_translation_mode,
                    ):
                        applied_translation_frames += 1
                applied_frames += 1
                applied_samples += frame_samples

            if applied_samples <= 0:
                return ""

            bind_paths = self._bind_animation_source(skeleton_path, self.pose_animation_prim_path)
            start_time, end_time = self._animation_time_range(animation_prim)
            if start_time is None:
                start_time = min(time_codes) if time_codes else 0.0
            if end_time is None:
                end_time = max(time_codes) if time_codes else start_time
            if end_time <= start_time:
                end_time = start_time + 1.0

            stage.SetStartTimeCode(float(start_time))
            stage.SetEndTimeCode(float(end_time))
            self.skeleton_prim_path = skeleton_path
            self.animation_prim_path = self.pose_animation_prim_path
            self._animation_setup_done = True
            self._mark_pose_animation_active(start_time, end_time)
            self._play_animation_timeline(start_time, end_time)

            return (
                "humanoid UsdSkel pose animation applied "
                f"frames={applied_frames} samples={applied_samples} "
                f"root_pivot_translation_frames={applied_translation_frames} "
                f"root_motion_path={root_motion_path} "
                f"animation={self.pose_animation_prim_path} bound_to={bind_paths}"
            )
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] UsdSkel pose animation skipped: {exc}")
            return ""

    def _set_joint_rotation_sample(
        self,
        prim_path: str,
        rotation_euler_deg: list[float],
        time_code: float,
    ) -> bool:
        # pose_sequence의 한 joint 회전을 USD xformOp:rotateXYZ time sample로 기록합니다.
        # 이 방식은 엄밀한 UsdSkel joint animation은 아니지만, OmniJoint prim이 노출된 KyleRobot에는
        # 디버깅/초기 retarget 확인용으로 빠르게 효과를 볼 수 있습니다.
        try:
            from pxr import Gf, Usd, UsdGeom

            stage = self._get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False

            xformable = UsdGeom.Xformable(prim)
            rotate_op = None
            # 이미 rotateXYZ op가 있으면 재사용하고, 없으면 새로 추가합니다.
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeRotateXYZ:
                    rotate_op = op
                    break
            if rotate_op is None:
                rotate_op = xformable.AddRotateXYZOp()

            rotate_op.Set(
                Gf.Vec3f(*_as_float_list(rotation_euler_deg)),
                Usd.TimeCode(float(time_code)),
            )
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] joint sample skipped {prim_path}: {exc}")
            return False

    def _set_joint_translation_sample(
        self,
        prim_path: str,
        translation: list[float],
        time_code: float,
    ) -> bool:
        try:
            from pxr import Gf, Usd, UsdGeom

            stage = self._get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            if not prim or not prim.IsValid():
                return False

            xformable = UsdGeom.Xformable(prim)
            translate_op = None
            for op in xformable.GetOrderedXformOps():
                if op.GetOpType() == UsdGeom.XformOp.TypeTranslate:
                    translate_op = op
                    break
            if translate_op is None:
                translate_op = xformable.AddTranslateOp()

            translate_op.Set(
                Gf.Vec3d(*_as_float_list(translation)),
                Usd.TimeCode(float(time_code)),
            )
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] joint translation sample skipped {prim_path}: {exc}")
            return False

    def _apply_pose_sequence(self, action: Dict[str, Any]) -> str:
        # Motion Retarget 노드가 만든 pose_sequence를 KyleRobot joint prim에 적용합니다.
        # sequence가 없으면 빈 문자열을 반환해서 기존 SkelAnimation 재생 경로로 넘어갑니다.
        sequence = action.get("pose_sequence") or action.get("poseSequence") or []
        if not isinstance(sequence, list) or not sequence:
            return ""

        usdskel_message = self._apply_usdskel_pose_sequence(action, sequence)
        if usdskel_message:
            return usdskel_message

        if not self.skeleton_prim_path:
            # 사용자가 VLAGRAPH_SKELETON_PRIM을 지정하지 않아도 robot 아래에서 자동 탐색합니다.
            self.skeleton_prim_path = self._resolve_skeleton_path()

        applied_frames = 0
        applied_samples = 0
        missing_joints: set[str] = set()
        time_codes = self._pose_sequence_time_codes(sequence, action)
        start_time = min(time_codes) if time_codes else 0.0
        end_time = max(time_codes) if time_codes else 0.0

        for index, frame in enumerate(sequence):
            # 각 frame은 {"time_code": n, "joints": {"LeftLeg": [rx,ry,rz]}} 형태입니다.
            if not isinstance(frame, dict):
                continue
            joints = frame.get("joints")
            if not isinstance(joints, dict):
                continue

            time_code = time_codes[index] if index < len(time_codes) else float(index)
            if applied_frames == 0:
                start_time = time_code
            end_time = max(end_time, time_code)

            frame_samples = 0
            for joint_name, rotation in joints.items():
                # 없는 joint는 실패로 중단하지 않고 missing 목록에 남깁니다.
                # rig 이름이 다를 때도 나머지 joint는 계속 적용하기 위함입니다.
                if not isinstance(rotation, (list, tuple, dict)):
                    continue
                joint_path = self._find_joint_prim_path(str(joint_name))
                if not joint_path:
                    missing_joints.add(str(joint_name))
                    continue
                if self._set_joint_rotation_sample(joint_path, _as_float_list(rotation), time_code):
                    frame_samples += 1
                    applied_samples += 1

            root_translation = self._root_translation_from_pose_frame(frame)
            if root_translation is not None:
                root_motion_path = self._resolve_robot_prim_path() or self.skeleton_prim_path
                root_translation_mode = str(
                    action.get("root_translation_mode")
                    or action.get("rootTranslationMode")
                    or "offset_from_initial"
                )
                if root_motion_path and self._set_root_translation_sample(
                    root_motion_path,
                    root_translation,
                    float(time_code),
                    mode=root_translation_mode,
                ):
                    frame_samples += 1
                    applied_samples += 1

            if frame_samples:
                applied_frames += 1

        if not applied_samples:
            return "humanoid pose sequence had no matching Isaac Sim joints"

        try:
            # 새로 기록한 time sample 범위로 timeline을 맞추고 재생합니다.
            stage = self._get_stage()
            if end_time > start_time:
                stage.SetStartTimeCode(float(start_time))
                stage.SetEndTimeCode(float(end_time))
            self._mark_pose_animation_active(start_time, end_time)
            self._play_animation_timeline(start_time, end_time)
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] pose sequence timeline skipped: {exc}")

        missing_text = ""
        if missing_joints:
            missing_text = f" missing={','.join(sorted(missing_joints)[:6])}"
        return (
            "humanoid pose sequence applied "
            f"frames={applied_frames} samples={applied_samples}{missing_text}"
        )

    def _setup_skeletal_animation(self) -> bool:
        # 기존 KyleRobot의 SkelAnimation(Take_001 등)을 Skeleton/SkelRoot에 바인딩합니다.
        # pose_sequence가 없을 때도 기본 애니메이션을 재생할 수 있게 하는 fallback 경로입니다.
        if not self.enable_animation:
            return False
        if self._animation_setup_done:
            return True

        try:
            from pxr import UsdSkel

            stage = self._get_stage()
            robot_prim = stage.GetPrimAtPath(self.robot_prim_path)
            if not robot_prim or not robot_prim.IsValid():
                raise ValueError(f"Robot prim not found: {self.robot_prim_path}")

            # 환경 변수 경로가 틀려도 robot prim 아래에서 자동으로 Skeleton/SkelAnimation을 다시 찾습니다.
            skeleton_path = self._resolve_skeleton_path()
            animation_path = self._resolve_animation_path()
            if not skeleton_path:
                raise ValueError(f"Skeleton prim not found under {self.robot_prim_path}")
            if not animation_path:
                raise ValueError(f"SkelAnimation prim not found under {self.robot_prim_path}")

            skeleton_prim = stage.GetPrimAtPath(skeleton_path)
            animation_prim = stage.GetPrimAtPath(animation_path)
            if not skeleton_prim or not skeleton_prim.IsValid():
                raise ValueError(f"Skeleton prim is invalid: {skeleton_path}")
            if not animation_prim or not animation_prim.IsValid():
                raise ValueError(f"Animation prim is invalid: {animation_path}")

            # UsdSkel BindingAPI의 animation source relationship을 animation prim으로 연결합니다.
            bind_paths = self._bind_animation_source(skeleton_path, animation_path)

            # 시작 시 자동 재생하면 그래프가 멈춰도 휴머노이드가 계속 움직일 수 있습니다.
            # 기본은 바인딩만 하고, 실제 명령이 들어왔을 때만 timeline을 재생합니다.
            start_time, end_time = self._animation_time_range(animation_prim)
            if self.play_timeline_on_start:
                self._play_animation_timeline(start_time, end_time)
            else:
                self._pause_animation_timeline()

            self.skeleton_prim_path = skeleton_path
            self.animation_prim_path = animation_path
            self._joint_prim_paths.clear()
            self._animation_setup_done = True
            if not self._animation_notice_printed:
                print(
                    "[VlaGraphIsaacSimBridge] skeletal animation ready "
                    f"skeleton={skeleton_path} animation={animation_path} "
                    f"bound_to={bind_paths} range={start_time}-{end_time} "
                    f"timeline_play={self.play_timeline} "
                    f"timeline_play_on_start={self.play_timeline_on_start}"
                )
                self._animation_notice_printed = True
            return True
        except Exception as exc:
            if not self._animation_notice_printed:
                print(f"[VlaGraphIsaacSimBridge] skeletal animation setup skipped: {exc}")
                self._animation_notice_printed = True
            return False

    def _extract_motion(self, action: Dict[str, Any]) -> tuple[list[float], list[float], Optional[list[float]], Optional[list[float]], bool]:
        # 다양한 action 형식을 하나의 root translation/rotation 명령으로 정규화합니다.
        # 반환값: delta translation, delta rotation, absolute target position/rotation, stop flag
        stop = bool(action.get("stop", False) or action.get("terminate", False))
        target_position = None
        target_rotation = None

        # target_position/target_rotation이 있으면 상대 이동이 아니라 절대 위치로 이동합니다.
        if action.get("target_position") is not None:
            target_position = _as_float_list(action.get("target_position"))
        elif action.get("patrol_target_position") is not None:
            target_position = _as_float_list(action.get("patrol_target_position"))

        if action.get("target_rotation") is not None:
            target_rotation = _as_float_list(action.get("target_rotation"))

        explicit_root_translation = (
            action.get("root_translation")
            or action.get("rootTranslation")
            or action.get("root_delta_position")
            or action.get("rootDeltaPosition")
        )
        if explicit_root_translation is not None:
            translation = _as_float_list(explicit_root_translation)
            explicit_root_rotation = (
                action.get("root_rotation")
                or action.get("rootRotation")
                or action.get("root_delta_rotation")
                or action.get("rootDeltaRotation")
            )
            rotation = _as_float_list(explicit_root_rotation) if explicit_root_rotation is not None else [0.0, 0.0, 0.0]
            return translation, rotation, target_position, target_rotation, stop

        velocity = action.get("velocity") if isinstance(action.get("velocity"), dict) else {}
        command_name = str(action.get("command", "")).strip().lower()
        has_skill_velocity = command_name == "apply_g1_skill" and (
            action.get("forward_velocity") is not None
            or action.get("lateral_velocity") is not None
            or action.get("turn_velocity") is not None
            or any(key in velocity for key in ("forward", "lateral", "yaw"))
        )
        if has_skill_velocity:
            duration = max(
                0.0,
                _as_float_value(action.get("duration_sec", action.get("durationSec", 1.0)), 1.0),
            )
            forward = _as_float_value(
                action.get("forward_velocity", velocity.get("forward", 0.0))
            )
            lateral = _as_float_value(
                action.get("lateral_velocity", velocity.get("lateral", 0.0))
            )
            yaw_rate = _as_float_value(
                action.get("turn_velocity", velocity.get("yaw", 0.0))
            )
            translation = [
                lateral * duration * self.step_scale,
                0.0,
                forward * duration * self.step_scale,
            ]
            rotation = [
                0.0,
                0.0,
                math.degrees(yaw_rate * duration) * self.turn_sign,
            ]
            return translation, rotation, target_position, target_rotation, stop

        # humanoid step/motion 액션은 move_x/move_z/turn_yaw를 사용합니다.
        if action.get("move_x") is not None or action.get("move_z") is not None:
            translation = [
                float(action.get("move_z", 0.0)) * self.step_scale,
                float(action.get("move_x", 0.0)) * self.step_scale,
                0.0,
            ]
            rotation = [0.0, 0.0, float(action.get("turn_yaw", 0.0)) * self.turn_scale_deg * self.turn_sign]
            return translation, rotation, target_position, target_rotation, stop

        # mobile base 계열 액션은 linear_x/linear_z/angular_yaw를 사용합니다.
        if action.get("linear_x") is not None or action.get("linear_z") is not None:
            translation = [
                float(action.get("linear_z", 0.0)) * self.step_scale,
                float(action.get("linear_x", 0.0)) * self.step_scale,
                0.0,
            ]
            rotation = [0.0, 0.0, float(action.get("angular_yaw", 0.0)) * self.turn_scale_deg * self.turn_sign]
            return translation, rotation, target_position, target_rotation, stop

        # patrol 또는 generic action의 translation/rotation fallback입니다.
        if action.get("patrol_direction") is not None:
            translation = [
                value * self.step_scale
                for value in _as_float_list(action.get("patrol_direction"))
            ]
        else:
            translation = [
                value * self.translation_scale
                for value in _as_float_list(
                    action.get("translation")
                    or action.get("delta_position")
                    or action.get("position")
                )
            ]

        rotation = [
            value * self.rotation_scale_deg * self.rotation_sign[index]
            for index, value in enumerate(_as_float_list(
                action.get("rotation")
                or action.get("delta_rotation")
                or action.get("euler")
            ))
        ]
        return translation, rotation, target_position, target_rotation, stop

    def _rotate_planar_translation_by_yaw(self, translation: list[float], yaw_deg: float) -> list[float]:
        values = _as_float_list(translation)
        if len(values) < 3:
            values = (values + [0.0, 0.0, 0.0])[:3]
        yaw = math.radians(float(yaw_deg))
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        local_x = float(values[0])
        local_y = float(values[1])
        return [
            local_x * cos_yaw - local_y * sin_yaw,
            local_x * sin_yaw + local_y * cos_yaw,
            float(values[2]),
        ]

    def _candidate_articulation_paths(self, prim_path: str) -> list[str]:
        prim_prefix = prim_path.rstrip("/") + "/"
        cached_articulation = str(self.articulation_prim_path or "").strip()
        if cached_articulation != prim_path and not cached_articulation.startswith(prim_prefix):
            # The robot prim may have been auto-remapped (for example G1 ->
            # OMY3M). Never try an articulation cached for the previous robot.
            cached_articulation = ""
        candidates = [
            cached_articulation,
            prim_path,
            _child_prim_path(prim_path, "G1"),
        ]

        try:
            from pxr import UsdPhysics

            stage = self._get_stage()
            prefix = prim_path.rstrip("/") + "/"
            for prim in stage.Traverse():
                path = str(prim.GetPath())
                if path != prim_path and not path.startswith(prefix):
                    continue
                if prim.HasAPI(UsdPhysics.ArticulationRootAPI):
                    candidates.append(path)
        except Exception:
            pass

        unique: list[str] = []
        for path in candidates:
            if not path or path in unique:
                continue
            if self._valid_prim_path(path):
                unique.append(path)
        return unique

    def _initialize_humanoid_articulation_neutral(self) -> bool:
        if self.robot_type.strip().lower() != "humanoid":
            return False

        try:
            import numpy as np

            Articulation = self._resolve_articulation_class()

            initialized = False
            for candidate_path in self._candidate_articulation_paths(self.robot_prim_path):
                try:
                    articulation = self._get_or_create_articulation(Articulation, candidate_path)
                    joint_names = self._articulation_joint_names(articulation)
                    if not joint_names:
                        continue
                    self._configure_humanoid_articulation(candidate_path, articulation, joint_names)
                    neutral = self._neutral_joint_vector(joint_names)
                    neutral = self._constrain_humanoid_joint_targets(
                        candidate_path,
                        joint_names,
                        neutral,
                        limit_delta=False,
                    )
                    self._set_articulation_joint_positions(
                        articulation,
                        np.asarray(neutral, dtype=np.float32),
                        target=False,
                    )
                    self._last_joint_targets_by_articulation[candidate_path] = list(neutral)
                    self.articulation_prim_path = candidate_path
                    initialized = True
                    print(
                        "[VlaGraphIsaacSimBridge] humanoid articulation neutralized "
                        f"articulation={candidate_path} joints={len(joint_names)}"
                    )
                    break
                except Exception as exc:
                    if self._is_invalidated_simulation_view_error(exc):
                        self._discard_articulation_cache(candidate_path, str(exc))
                    print(
                        "[VlaGraphIsaacSimBridge] articulation neutral init skipped "
                        f"{candidate_path}: {exc}"
                    )

            if initialized and self.disable_animation_for_articulation:
                self.enable_animation = False
                self.play_timeline = False
                self.play_timeline_on_start = False
                self._pause_animation_timeline()
                print("[VlaGraphIsaacSimBridge] skeletal animation disabled for articulation control")
            return initialized
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] humanoid neutral init failed: {exc}")
            return False

    def _inspect_humanoid_runtime(self, prim_path: str) -> Dict[str, Any]:
        diagnostics: Dict[str, Any] = {
            "prim_path": prim_path,
            "configured_robot_prim_path": self.robot_prim_path,
            "configured_skeleton_prim_path": self.skeleton_prim_path,
            "configured_animation_prim_path": self.animation_prim_path,
            "configured_pose_animation_prim_path": self.pose_animation_prim_path,
            "skeleton_path": "",
            "animation_path": "",
            "skeleton_joint_count": 0,
            "skeleton_joint_sample": [],
            "articulation_candidates": [],
            "recommended_retarget_mode": "",
        }
        try:
            stage = self._get_stage()
            prim = stage.GetPrimAtPath(prim_path)
            diagnostics["prim_valid"] = bool(prim and prim.IsValid())
            diagnostics["scene"] = str(stage.GetRootLayer().identifier or "")

            skeleton_path = self._resolve_skeleton_path()
            animation_path = self._resolve_animation_path()
            diagnostics["skeleton_path"] = skeleton_path
            diagnostics["animation_path"] = animation_path
            if skeleton_path:
                _, joint_tokens = self._skeleton_joint_tokens(skeleton_path)
                diagnostics["skeleton_joint_count"] = len(joint_tokens)
                diagnostics["skeleton_joint_sample"] = joint_tokens[:12]

            articulation_infos: list[Dict[str, Any]] = []
            try:
                Articulation = self._resolve_articulation_class()
            except Exception:
                Articulation = None

            for candidate in self._candidate_articulation_paths(prim_path):
                info: Dict[str, Any] = {"path": candidate, "valid": False, "joint_count": 0, "joint_sample": []}
                candidate_prim = stage.GetPrimAtPath(candidate)
                info["valid"] = bool(candidate_prim and candidate_prim.IsValid())
                if Articulation is not None and info["valid"]:
                    try:
                        articulation = self._get_or_create_articulation(Articulation, candidate)
                        joint_names = self._articulation_joint_names(articulation)
                        info["joint_count"] = len(joint_names)
                        info["joint_sample"] = joint_names[:12]
                    except Exception as exc:
                        if self._is_invalidated_simulation_view_error(exc):
                            self._discard_articulation_cache(candidate, str(exc))
                        info["error"] = str(exc)
                articulation_infos.append(info)

            diagnostics["articulation_candidates"] = articulation_infos
            has_articulation_joints = any(info.get("joint_count", 0) > 0 for info in articulation_infos)
            if diagnostics["skeleton_joint_count"] and not has_articulation_joints:
                diagnostics["recommended_retarget_mode"] = "isaac_usd_ik_pose_sequence"
            elif has_articulation_joints:
                diagnostics["recommended_retarget_mode"] = "articulation_joint_targets"
            else:
                diagnostics["recommended_retarget_mode"] = "root_transform_only"
        except Exception as exc:
            diagnostics["error"] = str(exc)
            diagnostics["trace"] = traceback.format_exc(limit=4)
        return _json_safe(diagnostics)

    def _action_value_diagnostics(self, action: Dict[str, Any]) -> Dict[str, Any]:
        def range_for(values: list[float]) -> Dict[str, Any]:
            finite = [float(value) for value in values if math.isfinite(float(value))]
            if not finite:
                return {"count": 0, "min": None, "max": None, "abs_max": None}
            return {
                "count": len(finite),
                "min": round(min(finite), 6),
                "max": round(max(finite), 6),
                "abs_max": round(max(abs(value) for value in finite), 6),
            }

        pose_sequence = action.get("pose_sequence") or action.get("poseSequence") or []
        pose_rotations: list[float] = []
        pose_joint_counts: list[int] = []
        pose_root_translation_values: list[float] = []
        pose_root_translation_frames = 0
        if isinstance(pose_sequence, list):
            for frame in pose_sequence:
                if not isinstance(frame, dict):
                    continue
                joints = frame.get("joints")
                if not isinstance(joints, dict):
                    continue
                pose_joint_counts.append(len(joints))
                for rotation in joints.values():
                    pose_rotations.extend(_as_float_list(rotation)[:3])
                root_translation = self._root_translation_from_pose_frame(frame)
                if root_translation is not None:
                    pose_root_translation_frames += 1
                    pose_root_translation_values.extend(_as_float_list(root_translation)[:3])

        root_translation_sequence = (
            action.get("root_translation_sequence")
            or action.get("rootTranslationSequence")
            or []
        )
        joint_trajectory = (
            action.get("g1_joint_trajectory")
            or action.get("g1JointTrajectory")
            or action.get("joint_trajectory")
            or action.get("jointTrajectory")
            or []
        )
        root_translation_sequence_values: list[float] = []
        if isinstance(root_translation_sequence, list):
            for item in root_translation_sequence:
                root_translation_sequence_values.extend(_as_float_list(item)[:3])
        joint_targets = (
            action.get("joint_targets")
            or action.get("jointTargets")
            or action.get("joint_positions")
            or action.get("jointPositions")
        )
        joint_target_values = _as_float_sequence(joint_targets)
        joint_trajectory_root_values: list[float] = []
        joint_trajectory_root_frames = 0
        if isinstance(joint_trajectory, list):
            for frame in joint_trajectory:
                if not isinstance(frame, dict):
                    continue
                named_targets = (
                    frame.get("g1_named_targets")
                    or frame.get("g1NamedTargets")
                    or frame.get("named_joint_targets")
                    or frame.get("namedJointTargets")
                    or frame.get("joint_target_map")
                    or frame.get("jointTargetMap")
                )
                if isinstance(named_targets, dict):
                    for value in named_targets.values():
                        with contextlib.suppress(Exception):
                            joint_target_values.append(float(value))
                root_translation = frame.get("root_translation") or frame.get("rootTranslation")
                if root_translation is not None:
                    joint_trajectory_root_frames += 1
                    joint_trajectory_root_values.extend(_as_float_list(root_translation)[:3])

        joint_order = _g1_joint_order_from_payload(action, len(joint_target_values))
        joint_schema = action.get("joint_schema") or action.get("jointSchema") or {}
        return {
            "retarget_mode": action.get("retarget_mode", action.get("retargetMode", "")),
            "use_isaac_usd_ik": bool(action.get("use_isaac_usd_ik", action.get("useIsaacUsdIk", False))),
            "pose_sequence_count": len(pose_sequence) if isinstance(pose_sequence, list) else 0,
            "pose_joint_count_min": min(pose_joint_counts) if pose_joint_counts else 0,
            "pose_joint_count_max": max(pose_joint_counts) if pose_joint_counts else 0,
            "pose_rotation_deg": range_for(pose_rotations),
            "pose_root_translation_frames": pose_root_translation_frames,
            "pose_root_translation": range_for(pose_root_translation_values),
            "root_translation_sequence_count": len(root_translation_sequence)
            if isinstance(root_translation_sequence, list)
            else 0,
            "root_translation_sequence": range_for(root_translation_sequence_values),
            "root_translation_frame_count": action.get("root_translation_frame_count"),
            "root_translation_nonzero_frames": action.get("root_translation_nonzero_frames"),
            "joint_trajectory_root_frames": joint_trajectory_root_frames,
            "joint_trajectory_root_translation": range_for(joint_trajectory_root_values),
            "joint_target_rad": range_for(joint_target_values),
            "joint_order_count": len(joint_order),
            "joint_schema_name": joint_schema.get("name", "") if isinstance(joint_schema, dict) else "",
            "joint_target_rejected_reason": action.get("_joint_target_rejected_reason", ""),
            "has_joint_targets": self._action_has_joint_targets(action),
        }

    def _articulation_joint_names(self, articulation: Any) -> list[str]:
        for attr_name in ("dof_names", "joint_names"):
            value = getattr(articulation, attr_name, None)
            if callable(value):
                with contextlib.suppress(Exception):
                    value = value()
            if isinstance(value, (list, tuple)) and value:
                return [str(item) for item in value]
        return []

    def _joint_targets_from_groups(self, action: Dict[str, Any]) -> tuple[list[str], list[float]]:
        names: list[str] = []
        values: list[float] = []
        groups = action.get("joint_groups") or action.get("jointGroups")
        groups = groups if isinstance(groups, dict) else {}
        for group_name in G1_GROUP_ORDER:
            group_values = action.get(group_name)
            if group_values is None:
                group_values = groups.get(group_name)
            vector = _as_float_sequence(group_values)
            if not vector:
                continue
            group_joints = G1_JOINT_GROUPS[group_name]
            for index, joint_name in enumerate(group_joints):
                if index >= len(vector):
                    break
                names.append(joint_name)
                values.append(float(vector[index]))
        return names, values

    def _normalize_joint_target_vector(
        self,
        action: Dict[str, Any],
        articulation_joint_names: list[str],
    ) -> Optional[list[float]]:
        raw_targets = (
            action.get("joint_targets")
            or action.get("jointTargets")
            or action.get("joint_positions")
            or action.get("jointPositions")
        )

        named_targets: dict[str, float] = {}
        for named_key in (
            "joint_target_map",
            "jointTargetMap",
            "named_joint_targets",
            "namedJointTargets",
            "g1_named_targets",
            "g1NamedTargets",
        ):
            named_value = action.get(named_key)
            if not isinstance(named_value, dict):
                continue
            for key, value in named_value.items():
                named_targets[str(key)] = _as_float_value(value)

        if isinstance(raw_targets, dict):
            for key, value in raw_targets.items():
                named_targets[str(key)] = _as_float_value(value)
        elif not named_targets:
            raw_values = _as_float_sequence(raw_targets)
            raw_names = (
                action.get("joint_names")
                or action.get("jointNames")
                or action.get("joints")
                or []
            )
            if isinstance(raw_names, (list, tuple)) and raw_values:
                for name, value in zip(raw_names, raw_values):
                    named_targets[str(name)] = float(value)
            elif raw_values:
                if _env_bool("VLAGRAPH_ALLOW_ANONYMOUS_G1_VECTOR", False):
                    ordered_names = _g1_joint_order_from_payload(action, len(raw_values))
                else:
                    ordered_names = ()
                if not ordered_names:
                    action["_joint_target_rejected_reason"] = "anonymous_joint_targets_require_joint_names_or_named_map"
                    raw_values = []
                for name, value in zip(ordered_names, raw_values):
                    named_targets[str(name)] = float(value)

        group_names, group_values = self._joint_targets_from_groups(action)
        for name, value in zip(group_names, group_values):
            named_targets[str(name)] = float(value)

        if not named_targets:
            return None

        if articulation_joint_names:
            resolved_targets: list[float] = []
            matched = 0
            matched_aliases: list[str] = []
            unmatched: list[str] = []
            for name in articulation_joint_names:
                found, value, matched_name = _lookup_named_joint_target(named_targets, name)
                resolved_targets.append(float(value) if found else 0.0)
                if found:
                    matched += 1
                    if matched_name != name:
                        matched_aliases.append(f"{name}->{matched_name}")
                else:
                    unmatched.append(name)
            action["_joint_target_match_count"] = matched
            action["_joint_target_nonzero_count"] = sum(1 for value in resolved_targets if abs(value) > 1.0e-5)
            action["_joint_target_alias_matches"] = matched_aliases[:8]
            action["_joint_target_unmatched_names"] = unmatched[:8]
            return resolved_targets

        resolved_targets = []
        for name in G1_FULL_JOINT_ORDER:
            found, value, _ = _lookup_named_joint_target(named_targets, name)
            if found:
                resolved_targets.append(float(value))
        return resolved_targets

    def _neutral_joint_vector(self, joint_names: list[str]) -> list[float]:
        neutral: list[float] = []
        for name in joint_names:
            canonical = _canonical_g1_joint_name(name)
            neutral.append(float(G1_NEUTRAL_JOINTS.get(canonical, 0.0)))
        return neutral

    def _constrain_humanoid_joint_targets(
        self,
        articulation_path: str,
        joint_names: list[str],
        targets: list[float],
        *,
        limit_delta: bool = True,
    ) -> list[float]:
        if not self.enforce_humanoid_joint_limits:
            return [float(value) for value in targets]

        previous = self._last_joint_targets_by_articulation.get(articulation_path)
        constrained: list[float] = []
        clamp_count = 0
        delta_count = 0
        max_delta = max(0.0, float(self.humanoid_joint_max_delta_rad))

        for index, raw_value in enumerate(targets):
            name = joint_names[index] if index < len(joint_names) else ""
            value = float(raw_value)
            bounds = _scaled_joint_limit(name, self.humanoid_joint_limit_scale)
            if bounds is not None:
                clamped = _clamp_float(value, bounds[0], bounds[1])
                if abs(clamped - value) > 1.0e-7:
                    clamp_count += 1
                value = clamped

            if limit_delta and previous is not None and index < len(previous) and max_delta > 0.0:
                old_value = float(previous[index])
                limited = old_value + _clamp_float(value - old_value, -max_delta, max_delta)
                if abs(limited - value) > 1.0e-7:
                    delta_count += 1
                value = limited

            constrained.append(value)

        self._last_joint_targets_by_articulation[articulation_path] = list(constrained)
        now = time.time()
        if (clamp_count or delta_count) and now - self._last_joint_safety_notice_at >= 0.75:
            self._last_joint_safety_notice_at = now
            print(
                "[VlaGraphIsaacSimBridge] humanoid joint safety "
                f"articulation={articulation_path} "
                f"clamped={clamp_count} delta_limited={delta_count}"
            )
        return constrained

    def _configure_humanoid_articulation(self, articulation_path: str, articulation: Any, joint_names: list[str]) -> None:
        if not joint_names:
            return

        try:
            import numpy as np

            count = len(joint_names)
            stiffness = np.full(count, float(self.humanoid_drive_stiffness), dtype=np.float32)
            damping = np.full(count, float(self.humanoid_drive_damping), dtype=np.float32)
            set_gains = getattr(articulation, "set_gains", None)
            if callable(set_gains):
                try:
                    set_gains(kps=stiffness, kds=damping)
                except TypeError:
                    set_gains(stiffness, damping)

            if self.zero_joint_velocities:
                zeros = np.zeros(count, dtype=np.float32)
                set_joint_velocities = getattr(articulation, "set_joint_velocities", None)
                if callable(set_joint_velocities):
                    set_joint_velocities(zeros)
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] humanoid articulation configure skipped: {exc}")

    def _zero_articulation_velocities(self, articulation: Any, count: int) -> None:
        if not self.zero_joint_velocities:
            return
        if count <= 0:
            return
        try:
            import numpy as np

            zeros = np.zeros(count, dtype=np.float32)
            set_joint_velocities = getattr(articulation, "set_joint_velocities", None)
            if callable(set_joint_velocities):
                set_joint_velocities(zeros)
        except Exception:
            pass

    def _zero_articulation_root_velocity(self, articulation: Any) -> None:
        if not self.zero_joint_velocities:
            return
        try:
            import numpy as np

            zero3 = np.zeros(3, dtype=np.float32)
            zero6 = np.zeros(6, dtype=np.float32)
            for method_name, value in (
                ("set_linear_velocity", zero3),
                ("set_angular_velocity", zero3),
                ("set_default_state", None),
            ):
                method = getattr(articulation, method_name, None)
                if not callable(method):
                    continue
                try:
                    if method_name == "set_default_state":
                        continue
                    method(value)
                except Exception:
                    pass

            for method_name in ("set_root_velocity", "set_velocities"):
                method = getattr(articulation, method_name, None)
                if callable(method):
                    try:
                        method(zero6)
                    except Exception:
                        pass
        except Exception:
            pass

    def _zero_articulation_root_velocity_by_path(self, articulation_path: str) -> None:
        articulation = self._articulations.get(articulation_path)
        if articulation is not None:
            self._zero_articulation_root_velocity(articulation)

    def _zero_articulation_velocities_by_path(self, articulation_path: str) -> None:
        articulation = self._articulations.get(articulation_path)
        if articulation is None:
            return
        joint_names = self._articulation_joint_names(articulation)
        count = len(joint_names)
        if count <= 0:
            count = len(self._last_joint_targets_by_articulation.get(articulation_path, []))
        self._zero_articulation_velocities(articulation, count)
        self._zero_articulation_root_velocity(articulation)

    def _g1_leg_target_summary(self, joint_names: list[str], targets: list[float]) -> Dict[str, Any]:
        tracked = (
            "left_hip_pitch_joint",
            "right_hip_pitch_joint",
            "left_knee_joint",
            "right_knee_joint",
            "left_ankle_pitch_joint",
            "right_ankle_pitch_joint",
        )
        summary: Dict[str, Any] = {}
        for target_name in tracked:
            found = False
            for index, joint_name in enumerate(joint_names):
                if index >= len(targets):
                    break
                if target_name in _joint_name_candidates(joint_name):
                    value = float(targets[index])
                    summary[target_name] = {
                        "rad": round(value, 6),
                        "deg": round(math.degrees(value), 3),
                        "source_joint": str(joint_name),
                    }
                    found = True
                    break
            if not found:
                summary[target_name] = None
        return summary

    def _format_g1_leg_target_summary(self, summary: Dict[str, Any]) -> str:
        short_names = {
            "left_hip_pitch_joint": "L_hip",
            "right_hip_pitch_joint": "R_hip",
            "left_knee_joint": "L_knee",
            "right_knee_joint": "R_knee",
            "left_ankle_pitch_joint": "L_ankle",
            "right_ankle_pitch_joint": "R_ankle",
        }
        parts: list[str] = []
        for key, label in short_names.items():
            item = summary.get(key)
            if isinstance(item, dict):
                parts.append(f"{label}={item.get('rad')}rad/{item.get('deg')}deg")
            else:
                parts.append(f"{label}=missing")
        return " ".join(parts)

    def _reset_humanoid_articulation_once(
        self,
        articulation_path: str,
        articulation: Any,
        joint_names: list[str],
        *,
        force: bool = False,
    ) -> None:
        if not self.reset_humanoid_on_first_command:
            return
        if not force and articulation_path in self._articulation_reset_done:
            return
        if not joint_names:
            return

        try:
            import numpy as np

            neutral = self._neutral_joint_vector(joint_names)
            neutral = self._constrain_humanoid_joint_targets(
                articulation_path,
                joint_names,
                neutral,
                limit_delta=False,
            )
            self._zero_articulation_velocities(articulation, len(neutral))
            self._set_articulation_joint_positions(
                articulation,
                np.asarray(neutral, dtype=np.float32),
                target=False,
            )
            self._last_joint_targets_by_articulation[articulation_path] = list(neutral)
            self._articulation_reset_done.add(articulation_path)
            print(
                "[VlaGraphIsaacSimBridge] humanoid articulation reset "
                f"articulation={articulation_path} joints={len(neutral)}"
            )
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] humanoid articulation reset skipped: {exc}")

    def _maybe_log_live_joint_apply(
        self,
        candidate_path: str,
        action: Dict[str, Any],
        effective_joint_names: list[str],
        targets: list[float],
    ) -> None:
        mode = str(action.get("retarget_mode") or action.get("retargetMode") or "").strip().lower()
        if mode not in {"joint_targets", "streamed_joint_targets"}:
            return
        interval_sec = max(0.0, float(self._live_apply_notice_interval_sec))
        if interval_sec <= 0.0:
            return
        now = time.monotonic()
        if now - self._last_live_apply_notice_at < interval_sec:
            return
        self._last_live_apply_notice_at = now

        numeric_targets = [float(value) for value in targets if isinstance(value, (int, float))]
        target_count = len(numeric_targets)
        nonzero_count = sum(1 for value in numeric_targets if abs(value) > 1e-4)
        if numeric_targets:
            target_range = {
                "min": round(min(numeric_targets), 5),
                "max": round(max(numeric_targets), 5),
                "mean_abs": round(sum(abs(value) for value in numeric_targets) / target_count, 5),
            }
        else:
            target_range = {"min": 0.0, "max": 0.0, "mean_abs": 0.0}
        unmatched = action.get("_joint_target_unmatched_names")
        if isinstance(unmatched, list):
            unmatched_preview = unmatched[:8]
            unmatched_count = len(unmatched)
        else:
            unmatched_preview = []
            unmatched_count = 0
        print(
            "[VlaGraphIsaacSimBridge][LIVE_APPLY] "
            f"mode={mode} articulation={candidate_path} "
            f"usd_joints={len(effective_joint_names)} targets={target_count} "
            f"matched={action.get('_joint_target_match_count', 0)} "
            f"aliases={action.get('_joint_target_alias_matches', 0)} "
            f"nonzero={nonzero_count} range={target_range} "
            f"unmatched={unmatched_count}:{unmatched_preview} "
            f"smoothing={action.get('joint_smoothing')} "
            f"max_delta={action.get('joint_max_delta_rad')} "
            f"deadband={action.get('joint_deadband_rad')}"
        )

    def _apply_joint_targets(self, prim_path: str, action: Dict[str, Any]) -> Optional[str]:
        # Articulation 로봇일 때 joint_targets / G1 joint group을 직접 적용하는 경로입니다.
        if not self._action_has_joint_targets(action):
            return None

        try:
            import numpy as np

            is_omy3m = self._is_omy3m_action(action)
            Articulation = (
                self._resolve_omy3m_articulation_class()
                if is_omy3m
                else self._resolve_articulation_class()
            )
            if is_omy3m:
                import omni.timeline

                timeline = omni.timeline.get_timeline_interface()
                if not timeline.is_playing():
                    timeline.play()
                    raise RuntimeError(
                        "OMY3M physics timeline was stopped and has now been started. "
                        "Wait one second, then run the graph again."
                    )

            errors: list[str] = []
            for candidate_path in self._candidate_articulation_paths(prim_path):
                try:
                    articulation = self._get_or_create_articulation(Articulation, candidate_path)
                    joint_names = self._articulation_joint_names(articulation)
                    action["_articulation_waist_joint_names"] = [
                        name
                        for name in joint_names
                        if "waist" in str(name).lower() or "torso" in str(name).lower()
                    ]
                    targets = self._normalize_joint_target_vector(action, joint_names)
                    rejected_reason = action.get("_joint_target_rejected_reason")
                    if rejected_reason:
                        errors.append(f"{candidate_path}: {rejected_reason}")
                        continue
                    if not targets:
                        continue
                    if is_omy3m and int(action.get("_joint_target_match_count", 0)) <= 0:
                        errors.append(
                            f"{candidate_path}: none of the configured OMY3M joint names "
                            f"matched USD joints={joint_names}"
                        )
                        continue
                    effective_joint_names = joint_names
                    if not effective_joint_names:
                        effective_joint_names = list(_g1_joint_order_from_payload(action, len(targets)))
                    if not effective_joint_names:
                        errors.append(f"{candidate_path}: joint targets require named joints or schema order")
                        continue
                    if effective_joint_names:
                        if is_omy3m:
                            # OMY3M targets are already clamped by the graph
                            # node. Never run humanoid/G1 articulation setup.
                            self._last_joint_targets_by_articulation[candidate_path] = list(targets)
                        else:
                            self._configure_humanoid_articulation(
                                candidate_path,
                                articulation,
                                effective_joint_names,
                            )
                            force_reset = bool(action.get("_force_reset_humanoid"))
                            if force_reset:
                                self._last_joint_targets_by_articulation.pop(candidate_path, None)
                                self._articulation_reset_done.discard(candidate_path)
                            self._reset_humanoid_articulation_once(
                                candidate_path,
                                articulation,
                                effective_joint_names,
                                force=force_reset,
                            )
                            targets = self._constrain_humanoid_joint_targets(
                                candidate_path,
                                effective_joint_names,
                                targets,
                            )
                            if force_reset:
                                self._zero_articulation_velocities(articulation, len(targets))
                    leg_summary = (
                        self._g1_leg_target_summary(effective_joint_names, targets)
                        if effective_joint_names
                        else {}
                    )
                    leg_summary_text = self._format_g1_leg_target_summary(leg_summary)
                    if self.print_g1_leg_targets and leg_summary:
                        print(
                            "[VlaGraphIsaacSimBridge] G1 leg targets "
                            f"articulation={candidate_path} {leg_summary_text}"
                        )
                    self._maybe_log_live_joint_apply(
                        candidate_path,
                        action,
                        effective_joint_names,
                        list(targets),
                    )
                    target_array = np.asarray(targets, dtype=np.float32)
                    retarget_mode = str(
                        action.get("retarget_mode") or action.get("retargetMode") or ""
                    ).strip().lower()
                    direct_joint_pose = retarget_mode in {
                        "procedural_walk",
                        "joint_trajectory",
                        "streamed_joint_targets",
                    }
                    if is_omy3m:
                        # Imported OMY USDs do not always carry usable drive
                        # stiffness. Write the articulation state so movement is
                        # visible, then set the drive target so physics holds it.
                        if not self._call_first_articulation_method(
                            articulation,
                            ("set_joint_positions", "set_dof_positions", "set_joint_position"),
                            target_array,
                        ):
                            raise AttributeError("OMY3M articulation has no direct joint position setter")
                        self._call_first_articulation_method(
                            articulation,
                            (
                                "set_joint_position_targets",
                                "set_dof_position_targets",
                                "set_joint_position_target",
                            ),
                            target_array,
                        )
                    else:
                        self._set_articulation_joint_positions(
                            articulation,
                            target_array,
                            target=not direct_joint_pose,
                        )
                    self.articulation_prim_path = candidate_path
                    return (
                        f"{'OMY3M ' if is_omy3m else ''}joint targets applied "
                        f"articulation={candidate_path} joints={len(targets)} "
                        f"matched={action.get('_joint_target_match_count', 0)} "
                        f"nonzero={action.get('_joint_target_nonzero_count', 0)} "
                        f"mode={'direct' if direct_joint_pose else 'target'} "
                        f"legs=({leg_summary_text})"
                    )
                except Exception as exc:
                    if self._is_invalidated_simulation_view_error(exc):
                        self._discard_articulation_cache(candidate_path, str(exc))
                    errors.append(f"{candidate_path}: {exc}")

            attempted = "; ".join(errors) if errors else "no articulation candidate accepted targets"
            raise RuntimeError(attempted)
        except Exception as exc:
            raise RuntimeError(f"Could not apply joint targets to {prim_path}: {exc}") from exc

    def _joint_trajectory_frames(self, action: Dict[str, Any]) -> list[Dict[str, Any]]:
        trajectory = (
            action.get("g1_joint_trajectory")
            or action.get("g1JointTrajectory")
            or action.get("joint_trajectory")
            or action.get("jointTrajectory")
            or []
        )
        if not isinstance(trajectory, list):
            return []

        frames: list[Dict[str, Any]] = []
        for index, frame in enumerate(trajectory):
            if not isinstance(frame, dict):
                continue
            joint_targets = (
                frame.get("joint_targets")
                or frame.get("jointTargets")
                or frame.get("joint_positions")
                or frame.get("jointPositions")
            )
            named_targets = (
                frame.get("g1_named_targets")
                or frame.get("g1NamedTargets")
                or frame.get("named_joint_targets")
                or frame.get("namedJointTargets")
                or frame.get("joint_target_map")
                or frame.get("jointTargetMap")
            )
            if joint_targets is None and not isinstance(named_targets, dict):
                continue
            try:
                time_sec = float(frame.get("time_sec", frame.get("timeSec", index * 0.0667)))
            except Exception:
                time_sec = index * 0.0667
            item = {
                "time_sec": max(0.0, time_sec),
                "joint_names": frame.get("joint_names") or frame.get("jointNames"),
            }
            for key in (
                "joint_schema",
                "jointSchema",
                "joint_order",
                "jointOrder",
                "joint_order_source",
                "jointOrderSource",
                "retarget_mode",
                "retargetMode",
            ):
                if key in frame:
                    item[key] = frame.get(key)
            if joint_targets is not None:
                item["joint_targets"] = _as_float_sequence(joint_targets)
            if isinstance(named_targets, dict):
                item["g1_named_targets"] = {
                    str(name): float(value)
                    for name, value in named_targets.items()
                    if isinstance(value, (int, float))
                }
            root_translation = frame.get("root_translation") or frame.get("rootTranslation")
            if isinstance(root_translation, (list, tuple)):
                item["root_translation"] = _as_float_sequence(root_translation)[:3]
                item["rootTranslation"] = item["root_translation"]
            root_translation_mode = frame.get("root_translation_mode") or frame.get("rootTranslationMode")
            if root_translation_mode is not None:
                item["root_translation_mode"] = str(root_translation_mode)
                item["rootTranslationMode"] = str(root_translation_mode)
            root_rotation = frame.get("root_rotation") or frame.get("rootRotation")
            if isinstance(root_rotation, (list, tuple)):
                item["root_rotation"] = _as_float_sequence(root_rotation)[:3]
                item["rootRotation"] = item["root_rotation"]
            root_delta_rotation = frame.get("root_delta_rotation") or frame.get("rootDeltaRotation")
            if isinstance(root_delta_rotation, (list, tuple)):
                item["root_delta_rotation"] = _as_float_sequence(root_delta_rotation)[:3]
                item["rootDeltaRotation"] = item["root_delta_rotation"]
            _annotate_g1_action_schema(item)
            frames.append(item)

        frames.sort(key=lambda item: float(item.get("time_sec", 0.0)))
        return frames

    def _is_fanuc_action(self, action: Dict[str, Any]) -> bool:
        action_type = str(action.get("type") or "").strip().lower()
        target = str(action.get("target_embodiment") or action.get("targetEmbodiment") or "").strip().lower()
        command = str(action.get("command") or "").strip().lower()
        return "fanuc" in action_type or "fanuc" in target or command == "apply_fanuc_trajectory"

    def _apply_fanuc_external_axis_translation(self, prim_path: str, action: Dict[str, Any]) -> Optional[str]:
        if not self._is_fanuc_action(action):
            return None
        if not self.apply_trajectory_root_translation:
            return "FANUC externalAxisJoint kept virtual; root translation disabled"
        root_translation = action.get("root_translation") or action.get("rootTranslation")
        if root_translation is None:
            return None
        translation = _as_float_list(root_translation)
        root_target_path = self._root_motion_target_path(prim_path)
        if not root_target_path or not self._valid_prim_path(root_target_path):
            return f"FANUC externalAxisJoint skipped; root prim not found: {root_target_path or '<empty>'}"
        mode = str(
            action.get("root_translation_mode")
            or action.get("rootTranslationMode")
            or "offset_from_initial"
        )
        time_sec = _as_float_value(action.get("time_sec", action.get("timeSec", 0.0)), 0.0)
        applied = self._set_root_translation_sample(
            root_target_path,
            translation,
            time_sec,
            mode=mode,
        )
        if not applied:
            return f"FANUC externalAxisJoint skipped; could not set Ink3 translation: {root_target_path}"

        now = time.time()
        if now - float(self._last_fanuc_external_axis_log_at or 0.0) > 1.0:
            self._last_fanuc_external_axis_log_at = now
            named_targets = action.get("g1_named_targets") if isinstance(action.get("g1_named_targets"), dict) else {}
            external_value = (
                named_targets.get("externalAxisJoint")
                if isinstance(named_targets, dict)
                else None
            )
            print(
                "[VlaGraphIsaacSimBridge] FANUC externalAxisJoint applied "
                f"prim={root_target_path} translation={translation} "
                f"mode={mode} time_sec={time_sec} target={external_value}"
            )
        return f"FANUC externalAxisJoint moved prim={root_target_path} translation={translation}"

    def _apply_joint_frame_root_translation(self, prim_path: str, action: Dict[str, Any]) -> Optional[str]:
        fanuc_message = self._apply_fanuc_external_axis_translation(prim_path, action)
        if fanuc_message is not None:
            return fanuc_message

        if not self.apply_trajectory_root_translation and str(action.get("retarget_mode") or "").strip().lower() in {
            "joint_trajectory",
            "streamed_joint_targets",
        }:
            return "root translation skipped; robotis.omy.motion owns humanoid root"

        has_root_translation = any(
            key in action
            for key in (
                "root_translation",
                "rootTranslation",
                "root_delta_position",
                "rootDeltaPosition",
                "target_position",
            )
        )
        if not has_root_translation:
            return None

        root_target_path = self._root_motion_target_path(prim_path)
        current_position = list(
            self._root_pose_position_by_prim.get(root_target_path)
            or self._prim_position(root_target_path)
        )
        current_rotation = self._rotation_by_prim.get(root_target_path, [0.0, 0.0, 0.0])
        translation, root_rotation, target_position, target_rotation, stop = self._extract_motion(action)
        if stop:
            return None
        retarget_mode = str(action.get("retarget_mode") or action.get("retargetMode") or "").strip().lower()
        if retarget_mode in {"joint_trajectory", "streamed_joint_targets"}:
            # MotionRetarget emits root_rotation in radians; USD Xform Euler
            # rotation is expressed in degrees.
            root_rotation = [math.degrees(float(value)) for value in root_rotation[:3]]

        mode = str(
            action.get("root_translation_mode")
            or action.get("rootTranslationMode")
            or ""
        ).strip().lower()
        if mode in {
            "offset_from_initial",
            "initial_offset",
            "base_offset",
            "foot_locked_pelvis_offset",
            "pelvis_foot_lock",
        }:
            initial_position = self._initial_root_position(root_target_path)
            _initial_position, initial_rotation = self._initial_root_pose_by_prim.get(
                root_target_path,
                (list(initial_position or current_position), list(current_rotation)),
            )
            base_position = self._root_base_position_by_prim.setdefault(
                root_target_path,
                list(initial_position or current_position),
            )
            position = target_position or [
                base_position[0] + translation[0],
                base_position[1] + translation[1],
                base_position[2] + translation[2],
            ]
            base_rotation = list(initial_rotation or current_rotation)
        else:
            position = target_position or [
                current_position[0] + translation[0],
                current_position[1] + translation[1],
                current_position[2] + translation[2],
            ]
            base_rotation = list(current_rotation)
        max_vertical = max(0.0, float(self.trajectory_root_max_vertical_m))
        if max_vertical > 0.0:
            initial_position = self._initial_root_position(root_target_path) or current_position
            position[2] = max(
                float(initial_position[2]) - max_vertical,
                min(float(initial_position[2]) + max_vertical, float(position[2])),
            )
        if self.lock_trajectory_root_rotation and retarget_mode in {"joint_trajectory", "streamed_joint_targets"}:
            _initial_position, initial_rotation = self._initial_root_pose_by_prim.get(
                root_target_path,
                (current_position, current_rotation),
            )
            rotation = list(target_rotation or initial_rotation or current_rotation)
        else:
            rotation = target_rotation or [
                base_rotation[0] + root_rotation[0],
                base_rotation[1] + root_rotation[1],
                base_rotation[2] + root_rotation[2],
            ]
        target_text = self._set_articulation_root_pose(root_target_path, position, rotation)
        applied_targets = [target_text or f"prim={root_target_path}"]
        message = f"root translation applied={translation} target={'+'.join(applied_targets)}"
        print(
            "[VlaGraphIsaacSimBridge] joint frame root translation "
            f"prim={prim_path} root={root_target_path} position={position} "
            f"mode={mode or '<delta>'} translation={translation}"
        )
        return message

    def _joint_trajectory_diagnostics(self, frames: list[Dict[str, Any]]) -> Dict[str, Any]:
        joint_values: list[float] = []
        root_values: list[float] = []
        waist_values: dict[str, list[float]] = {
            joint: [] for joint in G1_WAIST_3DOF_JOINTS
        }
        max_step_delta = 0.0
        previous: list[float] | None = None
        for frame in frames:
            targets = _as_float_sequence(
                frame.get("joint_targets")
                or frame.get("jointTargets")
                or frame.get("joint_positions")
                or frame.get("jointPositions")
            )
            named_targets = (
                frame.get("g1_named_targets")
                or frame.get("g1NamedTargets")
                or frame.get("named_joint_targets")
                or frame.get("namedJointTargets")
                or frame.get("joint_target_map")
                or frame.get("jointTargetMap")
            )
            if isinstance(named_targets, dict):
                for value in named_targets.values():
                    with contextlib.suppress(Exception):
                        targets.append(float(value))
            joint_values.extend(targets)
            if previous is not None:
                for before, after in zip(previous, targets):
                    max_step_delta = max(max_step_delta, abs(float(after) - float(before)))
            previous = targets
            root_translation = frame.get("root_translation") or frame.get("rootTranslation")
            if isinstance(root_translation, (list, tuple)):
                root_values.extend(_as_float_list(root_translation)[:3])
            named = (
                frame.get("g1_named_targets")
                or frame.get("named_joint_targets")
                or frame.get("joint_target_map")
            )
            if isinstance(named, dict):
                for joint in G1_WAIST_3DOF_JOINTS:
                    if joint in named:
                        waist_values[joint].append(_as_float_value(named[joint]))
        def range_for(values: list[float]) -> Dict[str, Any]:
            finite = [float(value) for value in values if math.isfinite(float(value))]
            if not finite:
                return {"count": 0, "min": None, "max": None, "abs_max": None}
            return {
                "count": len(finite),
                "min": round(min(finite), 6),
                "max": round(max(finite), 6),
                "abs_max": round(max(abs(value) for value in finite), 6),
            }
        return {
            "frames": len(frames),
            "joint_target_rad": range_for(joint_values),
            "root_translation": range_for(root_values),
            "waist_3dof_rad": {
                joint: range_for(values)
                for joint, values in waist_values.items()
            },
            "max_step_delta_rad": round(max_step_delta, 6),
        }

    def _start_joint_trajectory(self, prim_path: str, action: Dict[str, Any]) -> Optional[str]:
        if self._is_omy3m_action(action):
            return self._start_omy3m_trajectory(prim_path, action)
        frames = self._joint_trajectory_frames(action)
        if not frames:
            return None
        trajectory_debug = self._joint_trajectory_diagnostics(frames)
        print(
            "[VlaGraphIsaacSimBridge] trajectory root config "
            f"build={BRIDGE_BUILD} "
            f"apply_translation={self.apply_trajectory_root_translation} "
            f"lock_rotation={self.lock_trajectory_root_rotation} "
            f"max_vertical_m={self.trajectory_root_max_vertical_m} "
            f"debug={trajectory_debug}"
        )
        self._humanoid_frozen = False
        self._last_motion_command_time = time.time()
        self._ensure_articulation_timeline_playing()

        if self.reset_humanoid_each_trajectory:
            self._active_joint_trajectory = None
            self._pose_animation_active_until = 0.0
            self._humanoid_frozen = False
            self._set_root_motion_baseline_to_current(prim_path)
            if self.articulation_prim_path:
                self._zero_articulation_velocities_by_path(self.articulation_prim_path)
            print(
                "[VlaGraphIsaacSimBridge] joint trajectory reset "
                f"prim={prim_path} articulation={self.articulation_prim_path or '<none>'} "
                f"debug={trajectory_debug}"
            )

        self._active_joint_trajectory = {
            "prim_path": prim_path,
            "frames": frames,
            "start_time": time.time(),
            "elapsed": 0.0,
            "last_advance_at": time.time(),
            "next_index": 0,
            "last_index": -1,
            "action_type": action.get("type"),
            "target_embodiment": action.get("target_embodiment")
            or action.get("targetEmbodiment"),
            "command": action.get("command"),
        }
        first_action = dict(action)
        first_action.update(frames[0])
        if self.reset_humanoid_each_trajectory:
            first_action["_force_reset_humanoid"] = True
        self._apply_joint_targets(prim_path, first_action)
        print(
            "[VlaGraphIsaacSimBridge] trajectory waist apply "
            f"articulation_joints={first_action.get('_articulation_waist_joint_names', [])} "
            f"matched={first_action.get('_joint_target_match_count', 0)} "
            f"aliases={first_action.get('_joint_target_alias_matches', [])} "
            f"unmatched={first_action.get('_joint_target_unmatched_names', [])}"
        )
        root_message = self._apply_joint_frame_root_translation(prim_path, first_action)
        self._active_joint_trajectory["next_index"] = 1
        duration = float(frames[-1].get("time_sec", 0.0)) if frames else 0.0
        action_type = str(action.get("type") or "").strip().lower()
        target_embodiment = str(action.get("target_embodiment") or "").strip().lower()
        if "fanuc" in action_type or "fanuc" in target_embodiment:
            trajectory_name = "FANUC"
        else:
            trajectory_name = "OMY3M" if self._is_omy3m_action(action) else "G1"
        message = (
            f"{trajectory_name} joint trajectory started "
            f"frames={len(frames)} duration={duration:.3f}s"
        )
        if root_message:
            message = f"{message}; {root_message}"
        return message

    def _start_omy3m_trajectory(self, prim_path: str, action: Dict[str, Any]) -> Optional[str]:
        """Start an OMY3M-only articulation trajectory.

        This path never invokes humanoid reset, neutral pose, root motion,
        animation, G1 joint aliases, or G1 safety constraints.
        """
        frames = self._joint_trajectory_frames(action)
        if not frames:
            return None
        if str(action.get("target_embodiment") or "").strip().lower() not in {
            "omy3m",
            "omy3m_6dof",
            "omy_3m",
        }:
            raise RuntimeError("OMY3M trajectory rejected: target_embodiment is not OMY3M")

        import omni.timeline

        timeline = omni.timeline.get_timeline_interface()
        timeline_was_playing = bool(timeline.is_playing())
        physics_ready_at = time.time()
        if not timeline_was_playing:
            timeline.play()
            # Articulation simulation views are created on a subsequent
            # physics update. Queue frame zero instead of failing the command.
            physics_ready_at += 0.75
        self._active_joint_trajectory = None
        self._active_omy3m_trajectory = {
            "prim_path": prim_path,
            "frames": frames,
            "start_time": physics_ready_at,
            "physics_ready_at": physics_ready_at,
            "elapsed": 0.0,
            "last_advance_at": physics_ready_at,
            "next_index": 0,
            "last_index": -1,
        }
        message = "queued until physics is initialized"
        if timeline_was_playing:
            first_action = dict(frames[0])
            first_action.update(
                {
                    "type": "omy3m_joint_target",
                    "target_embodiment": "OMY3M_6DOF",
                    "command": "apply_joint_targets",
                    "retarget_mode": "omy3m_joint_trajectory",
                }
            )
            message = self._apply_joint_targets(prim_path, first_action)
            self._active_omy3m_trajectory["next_index"] = 1
        duration = float(frames[-1].get("time_sec", 0.0))
        print(
            "[VlaGraphIsaacSimBridge] OMY3M trajectory started "
            f"prim={prim_path} frames={len(frames)} first_apply={message}"
        )
        state = "started" if timeline_was_playing else "queued; physics auto-started"
        return (
            f"OMY3M joint trajectory {state} "
            f"frames={len(frames)} duration={duration:.3f}s"
        )

    def _advance_omy3m_trajectory(self, now: float) -> bool:
        trajectory = self._active_omy3m_trajectory
        if not isinstance(trajectory, dict):
            return False
        frames = trajectory.get("frames")
        if not isinstance(frames, list) or not frames:
            self._active_omy3m_trajectory = None
            return False
        if float(now) < float(trajectory.get("physics_ready_at", 0.0)):
            return False

        elapsed = self._advance_trajectory_elapsed(trajectory, now)
        index = int(trajectory.get("next_index", 0))
        prim_path = str(trajectory.get("prim_path") or self.robot_prim_path)
        applied = False
        while index < len(frames) and float(frames[index].get("time_sec", 0.0)) <= elapsed:
            frame_action = dict(frames[index])
            frame_action.update(
                {
                    "type": "omy3m_joint_target",
                    "target_embodiment": "OMY3M_6DOF",
                    "command": "apply_joint_targets",
                    "retarget_mode": "omy3m_joint_trajectory",
                }
            )
            try:
                self._ensure_articulation_timeline_playing()
                self._apply_joint_targets(prim_path, frame_action)
                trajectory["last_index"] = index
                applied = True
            except Exception as exc:
                self._active_omy3m_trajectory = None
                print(
                    "[VlaGraphIsaacSimBridge] OMY3M trajectory aborted "
                    f"index={index} error={exc}"
                )
                return False
            index += 1

        trajectory["next_index"] = index
        if index >= len(frames):
            self._active_omy3m_trajectory = None
            print(f"[VlaGraphIsaacSimBridge] OMY3M trajectory finished frames={len(frames)}")
        return applied

    def _advance_joint_trajectory(self, now: float) -> bool:
        trajectory = self._active_joint_trajectory
        if not isinstance(trajectory, dict):
            return False
        frames = trajectory.get("frames")
        if not isinstance(frames, list) or not frames:
            self._active_joint_trajectory = None
            return False

        elapsed = self._advance_trajectory_elapsed(trajectory, now)
        index = int(trajectory.get("next_index", 0))
        prim_path = str(trajectory.get("prim_path") or self.robot_prim_path)
        applied = False
        while index < len(frames) and float(frames[index].get("time_sec", 0.0)) <= elapsed:
            frame_action = dict(frames[index])
            frame_action.setdefault("type", trajectory.get("action_type"))
            frame_action.setdefault("target_embodiment", trajectory.get("target_embodiment"))
            frame_action.setdefault("command", trajectory.get("command") or "apply_joint_targets")
            frame_action.setdefault("retarget_mode", "joint_trajectory")
            try:
                self._ensure_articulation_timeline_playing()
                self._apply_joint_targets(prim_path, frame_action)
                self._apply_joint_frame_root_translation(prim_path, frame_action)
                trajectory["last_index"] = index
                applied = True
            except Exception as exc:
                print(f"[VlaGraphIsaacSimBridge] joint trajectory frame skipped index={index}: {exc}")
            index += 1

        trajectory["next_index"] = index
        if index >= len(frames):
            self._active_joint_trajectory = None
            print(f"[VlaGraphIsaacSimBridge] joint trajectory finished frames={len(frames)}")
            if self.reset_humanoid_after_trajectory:
                self._freeze_humanoid_motion("trajectory_complete")
        return applied

    def _advance_trajectory_elapsed(self, trajectory: Dict[str, Any], now: float) -> float:
        raw_now = float(now)
        last_advance_at = trajectory.get("last_advance_at")
        if not isinstance(last_advance_at, (int, float)) or not math.isfinite(float(last_advance_at)):
            last_advance_at = raw_now
        raw_step = max(0.0, raw_now - float(last_advance_at))
        max_step = float(self.trajectory_max_step_sec or 0.05)
        if not math.isfinite(max_step) or max_step <= 0.0:
            max_step = 0.05
        step = min(raw_step, max_step)
        trajectory["last_advance_at"] = raw_now
        elapsed = float(trajectory.get("elapsed", 0.0)) + step
        trajectory["elapsed"] = elapsed
        return elapsed

    def _prepare_humanoid_motion(self, action: Dict[str, Any]) -> str:
        # full-body retarget이 들어온 경우 pose_sequence 적용을 최우선으로 처리합니다.
        if self._action_has_joint_targets(action):
            return "humanoid motion uses articulation joint targets"

        pose_sequence_message = self._apply_pose_sequence(action)
        if pose_sequence_message:
            return pose_sequence_message

        # pose_sequence가 없으면 기존 SkelAnimation을 다시 재생하는 fallback입니다.
        if not bool(action.get("animation", True)):
            return "humanoid motion prepared"

        if not self._setup_skeletal_animation():
            return "humanoid motion prepared without skeletal animation"

        try:
            stage = self._get_stage()
            animation_prim = stage.GetPrimAtPath(self.animation_prim_path)
            if animation_prim and animation_prim.IsValid():
                start_time, end_time = self._animation_time_range(animation_prim)
                self._play_animation_timeline(start_time, end_time)
                return "humanoid motion prepared and animation timeline replayed"
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] humanoid animation replay skipped: {exc}")

        return "humanoid motion prepared"

    def _action_has_joint_targets(self, action: Dict[str, Any]) -> bool:
        if any(
            key in action
            for key in (
                "joint_targets",
                "jointTargets",
                "joint_positions",
                "jointPositions",
                "joint_target_map",
                "jointTargetMap",
                "named_joint_targets",
                "namedJointTargets",
                "g1_named_targets",
                "g1NamedTargets",
                "joint_groups",
                "jointGroups",
                *G1_GROUP_ORDER,
            )
        ):
            return True
        trajectory = (
            action.get("g1_joint_trajectory")
            or action.get("g1JointTrajectory")
            or action.get("joint_trajectory")
            or action.get("jointTrajectory")
            or []
        )
        if not isinstance(trajectory, list):
            return False
        for frame in trajectory:
            if not isinstance(frame, dict):
                continue
            if any(
                key in frame
                for key in (
                    "joint_targets",
                    "jointTargets",
                    "joint_positions",
                    "jointPositions",
                    "joint_target_map",
                    "jointTargetMap",
                    "named_joint_targets",
                    "namedJointTargets",
                    "g1_named_targets",
                    "g1NamedTargets",
                )
            ):
                return True
        return False

    @staticmethod
    def _is_omy3m_action(action: Dict[str, Any]) -> bool:
        action_type = str(action.get("type") or "").strip().lower()
        embodiment = str(
            action.get("target_embodiment")
            or action.get("targetEmbodiment")
            or ""
        ).strip().lower()
        return action_type == "omy3m_joint_target" or embodiment in {
            "omy3m",
            "omy3m_6dof",
            "omy_3m",
        }

    def _record_prompt_context(
        self,
        payload: Dict[str, Any],
        action: Dict[str, Any],
        *,
        prim_path: str,
        command: str,
    ) -> Dict[str, str]:
        # prompt/instruction은 로봇 제어와 별개로 state payload에 계속 실어 보냅니다.
        # 그래프에서 "Omniverse가 프롬프트를 받았는지" 확인할 때 이 값들이 기준이 됩니다.
        context = _extract_prompt_context(payload, action)
        if not context:
            return {}

        now = time.time()
        prompt = context.get("prompt", "")
        instruction = context.get("instruction", "")
        model_prompt = context.get("model_prompt", "")
        changed = (
            prompt != self._last_prompt
            or instruction != self._last_instruction
            or model_prompt != self._last_model_prompt
        )

        if prompt:
            self._last_prompt = prompt
        if instruction:
            self._last_instruction = instruction
        if model_prompt:
            self._last_model_prompt = model_prompt
        self._last_prompt_timestamp = now

        self._prompt_history.append(
            {
                "timestamp": now,
                "request_id": payload.get("request_id"),
                "prim_path": prim_path,
                "command": command,
                "prompt": prompt,
                "instruction": instruction,
                "model_prompt": model_prompt,
            }
        )
        self._prompt_history = self._prompt_history[-20:]

        # prompt가 실제로 바뀐 경우에만 console에 출력해서 반복 실행 로그가 과도해지지 않게 합니다.
        if self.print_prompts and changed:
            print(
                "[VlaGraphIsaacSimBridge] prompt received "
                f"request_id={payload.get('request_id')} "
                f"command={command} text={_preview_text(prompt or instruction or model_prompt)}"
            )
        return context

    def _apply_command(self, payload: Dict[str, Any]) -> Dict[str, Any]:
        # backend Robot Execute 노드가 보낸 하나의 omniverse_command를 실제 Isaac Sim 동작으로 바꿉니다.
        request_id = payload.get("request_id")
        prim_path = str(payload.get("prim_path") or self.robot_prim_path)
        if self._valid_prim_path(prim_path):
            if payload.get("prim_path") and prim_path != self.robot_prim_path:
                old_robot_path = self.robot_prim_path
                self.robot_prim_path = prim_path
                self._refresh_robot_dependent_paths(old_robot_path)
                print(
                    "[VlaGraphIsaacSimBridge] command prim selected "
                    f"from={old_robot_path} to={self.robot_prim_path}"
                )
        else:
            resolved_robot_path = self._resolve_robot_prim_path()
            if self._valid_prim_path(resolved_robot_path):
                print(
                    "[VlaGraphIsaacSimBridge] command prim auto-remapped "
                    f"from={prim_path} to={resolved_robot_path}"
                )
                prim_path = resolved_robot_path
        command = str(payload.get("command") or "execute_policy")
        command_key = command.strip().lower()
        action = payload.get("action") if isinstance(payload.get("action"), dict) else {}
        if action:
            _annotate_g1_action_schema(action)

        # prompt를 먼저 기록합니다. 이후 state publish에서 last_prompt로 확인할 수 있습니다.
        prompt_context = self._record_prompt_context(
            payload,
            action,
            prim_path=prim_path,
            command=command,
        )
        action_diagnostics = self._action_value_diagnostics(action)
        self._last_command_diagnostics = action_diagnostics

        if command_key == "scan_scene":
            scan = self._scan_scene(payload)
            return {
                "ok": bool(scan.get("ok")),
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": False,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "scan": _json_safe(scan),
                "objects": _json_safe(scan.get("objects", [])),
                "robot": _json_safe(scan.get("robot", {})),
                "message": f"Scanned Isaac Sim scene objects={scan.get('object_count', 0)}.",
            }

        if command_key in {"inspect_humanoid", "diagnose_humanoid", "diagnostics"}:
            runtime_diagnostics = self._inspect_humanoid_runtime(prim_path)
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": False,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "diagnostics": runtime_diagnostics,
                "action_diagnostics": action_diagnostics,
                "message": runtime_diagnostics.get("recommended_retarget_mode", "diagnostics ready"),
            }

        if command_key in {"init", "initialize", "reset_scene", "init_g1", "init_gr00t"}:
            old_robot_path = self.robot_prim_path
            if self._valid_prim_path(prim_path):
                self.robot_prim_path = prim_path
                self._refresh_robot_dependent_paths(old_robot_path)
            self._active_joint_trajectory = None
            self._active_omy3m_trajectory = None
            self._pose_animation_active_until = 0.0
            self._last_joint_targets_by_articulation.clear()
            self._articulation_reset_done.clear()
            self._held_root_motion_pose = {}
            self._last_root_motion_debug = {}
            home_applied = self._apply_explicit_home_root_pose(self.robot_prim_path)
            initialized = self._initialize_humanoid_articulation_neutral()
            if home_applied:
                self._apply_explicit_home_root_pose(self.robot_prim_path)
            else:
                self._reset_root_motion_baseline(self.robot_prim_path)
            root_target_path = self._root_motion_target_path(self.robot_prim_path)
            state = self._build_state_payload(time.time())
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": True,
                "stage": payload.get("stage"),
                "prim_path": self.robot_prim_path,
                "root_target_path": root_target_path,
                "command": command,
                "initialized_articulation": initialized,
                "home_applied": home_applied,
                "state": _json_safe(state),
                "message": (
                    "Isaac G1 bridge initialized "
                    f"prim={self.robot_prim_path} root={root_target_path} "
                    f"articulation={self.articulation_prim_path or '<none>'}"
                ),
            }

        # prompt만 저장하는 명령은 로봇을 움직이지 않고 ack만 반환합니다.
        if command.strip().lower() in {"prompt", "set_prompt", "instruction"} and not action:
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": False,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "prompt": prompt_context.get("prompt", self._last_prompt),
                "instruction": prompt_context.get("instruction", self._last_instruction),
                "message": "Prompt received and stored.",
            }

        if command_key in {"stop", "cancel", "pause", "freeze"} or bool(
            action.get("stop", False)
            or action.get("terminate", False)
            or action.get("cancel", False)
            or action.get("freeze", False)
        ):
            self._active_omy3m_trajectory = None
            if self._valid_prim_path(prim_path) or self._articulations:
                self._freeze_humanoid_motion(command_key or "stop")
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": True,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "prompt": prompt_context.get("prompt", self._last_prompt),
                "instruction": prompt_context.get("instruction", self._last_instruction),
                "action_diagnostics": action_diagnostics,
                "message": "Humanoid motion stopped and frozen.",
            }

        if not self._valid_prim_path(prim_path):
            return {
                "ok": False,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": False,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "prompt": prompt_context.get("prompt", self._last_prompt),
                "instruction": prompt_context.get("instruction", self._last_instruction),
                "action_diagnostics": action_diagnostics,
                "message": (
                    f"Robot prim not found: {prim_path}. "
                    "Load the humanoid USD stage first or set VLAGRAPH_ROBOT_PRIM."
                ),
            }

        humanoid_motion_message = ""
        # apply_humanoid_motion은 full-body pose_sequence 또는 기존 animation 재생을 준비합니다.
        if command_key == "apply_humanoid_motion":
            humanoid_motion_message = self._prepare_humanoid_motion(action)

        # joint target이 있으면 Articulation API 경로가 우선입니다.
        trajectory_message = self._start_joint_trajectory(prim_path, action)
        if trajectory_message:
            root_target_path = self._root_motion_target_path(prim_path)
            position = self._cached_or_prim_position(root_target_path)
            rotation = self._rotation_by_prim.get(root_target_path, [0.0, 0.0, 0.0])
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": True,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "message": trajectory_message,
                "action_diagnostics": action_diagnostics,
                "position": position,
                "rotation": rotation,
                "root_target_path": root_target_path,
            }

        # joint target이 있으면 Articulation API 경로가 우선입니다.
        joint_message = self._apply_joint_targets(prim_path, action)
        if joint_message:
            root_target_path = self._root_motion_target_path(prim_path)
            position = self._cached_or_prim_position(root_target_path)
            rotation = self._rotation_by_prim.get(root_target_path, [0.0, 0.0, 0.0])
            has_explicit_root_translation = any(
                key in action
                for key in (
                    "root_translation",
                    "rootTranslation",
                    "root_delta_position",
                    "rootDeltaPosition",
                    "target_position",
                )
            )
            if has_explicit_root_translation:
                current_position = position
                current_rotation = rotation
                translation, root_rotation, target_position, target_rotation, stop = self._extract_motion(action)
                if not stop:
                    mode = str(
                        action.get("root_translation_mode")
                        or action.get("rootTranslationMode")
                        or ""
                    ).strip().lower()
                    if mode in {"offset_from_initial", "initial_offset", "base_offset"}:
                        base_position = self._root_base_position_by_prim.setdefault(
                            root_target_path,
                            list(current_position),
                        )
                        position = target_position or [
                            base_position[0] + translation[0],
                            base_position[1] + translation[1],
                            base_position[2] + translation[2],
                        ]
                    else:
                        position = target_position or [
                            current_position[0] + translation[0],
                            current_position[1] + translation[1],
                            current_position[2] + translation[2],
                        ]
                    rotation = target_rotation or [
                        current_rotation[0] + root_rotation[0],
                        current_rotation[1] + root_rotation[1],
                        current_rotation[2] + root_rotation[2],
                    ]
                    target_text = self._set_articulation_root_pose(root_target_path, position, rotation)
                    joint_message = (
                        f"{joint_message}; root translation applied={translation} "
                        f"target={target_text or root_target_path}"
                    )
            return {
                "ok": True,
                "type": "omniverse_ack",
                "request_id": request_id,
                "applied": True,
                "stage": payload.get("stage"),
                "prim_path": prim_path,
                "command": command,
                "message": joint_message,
                "action_diagnostics": action_diagnostics,
                "position": position,
                "rotation": rotation,
                "root_target_path": root_target_path,
                "prompt": prompt_context.get("prompt", self._last_prompt),
                "instruction": prompt_context.get("instruction", self._last_instruction),
            }

        # joint target이 없으면 root prim 이동/회전으로 처리합니다.
        # G1 skill 명령은 실제 articulation root가 있으면 그 경로를 우선 이동합니다.
        root_target_path = self._root_motion_target_path(prim_path)
        if command_key == "apply_g1_skill" and not self.articulation_prim_path:
            self._initialize_humanoid_articulation_neutral()
            root_target_path = self._root_motion_target_path(prim_path)
        current_position = self._cached_or_prim_position(root_target_path)
        current_rotation = self._rotation_by_prim.get(root_target_path, [0.0, 0.0, 0.0])
        translation, rotation, target_position, target_rotation, stop = self._extract_motion(action)
        if not target_position and bool(action.get("local_frame", action.get("localFrame", False))):
            translation = self._rotate_planar_translation_by_yaw(translation, current_rotation[2] if len(current_rotation) >= 3 else 0.0)

        # stop 명령이면 현재 pose를 유지합니다.
        if stop or command.lower() == "stop":
            next_position = current_position
            next_rotation = current_rotation
        else:
            # target_position이 있으면 절대 이동, 없으면 현재 위치에 delta를 더합니다.
            mode = str(
                action.get("root_translation_mode")
                or action.get("rootTranslationMode")
                or ""
            ).strip().lower()
            if mode in {"offset_from_initial", "initial_offset", "base_offset"}:
                base_position = self._root_base_position_by_prim.setdefault(
                    root_target_path,
                    list(current_position),
                )
                next_position = target_position or [
                    base_position[0] + translation[0],
                    base_position[1] + translation[1],
                    base_position[2] + translation[2],
                ]
            else:
                next_position = target_position or [
                    current_position[0] + translation[0],
                    current_position[1] + translation[1],
                    current_position[2] + translation[2],
                ]
            next_rotation = target_rotation or [
                current_rotation[0] + rotation[0],
                current_rotation[1] + rotation[1],
                current_rotation[2] + rotation[2],
            ]

        target_text = self._set_articulation_root_pose(root_target_path, next_position, next_rotation)

        # backend는 이 응답을 Robot Execute 결과로 받아 그래프 로그에 표시합니다.
        return {
            "ok": True,
            "type": "omniverse_ack",
            "request_id": request_id,
            "applied": True,
            "stage": payload.get("stage"),
            "prim_path": prim_path,
            "command": command,
            "action": _json_safe(action),
            "position": next_position,
            "rotation": next_rotation,
            "root_target_path": root_target_path,
            "translation": translation,
            "root_target": target_text or root_target_path,
            "skill": action.get("skill"),
            "gesture": action.get("gesture"),
            "duration_sec": action.get("duration_sec"),
            "prompt": prompt_context.get("prompt", self._last_prompt),
            "instruction": prompt_context.get("instruction", self._last_instruction),
            "action_diagnostics": action_diagnostics,
            "message": humanoid_motion_message or "Isaac Sim command applied.",
        }

    def _build_state_payload(self, now: float) -> Dict[str, Any]:
        # backend /api/unity/state 및 unity_state 노드가 읽는 현재 로봇 상태 JSON입니다.
        try:
            self._resolve_robot_prim_path()
            stage = self._get_stage()
            if stage is None:
                raise ValueError("USD stage is not available")
            scene = str(stage.GetRootLayer().identifier or "")
            prim = self._get_prim(self.robot_prim_path)
            root_motion_target_path = self._root_motion_target_path(self.robot_prim_path)
            root_motion_position = (
                list(
                    self._root_pose_position_by_prim.get(root_motion_target_path)
                    or self._prim_position(root_motion_target_path)
                )
                if root_motion_target_path and self._valid_prim_path(root_motion_target_path)
                else []
            )
            position = list(root_motion_position) if root_motion_position else self._prim_position(self.robot_prim_path)
            rotation = self._rotation_by_prim.get(
                root_motion_target_path,
                self._rotation_by_prim.get(self.robot_prim_path, [0.0, 0.0, 0.0]),
            )
            object_name = prim.GetName()
            articulation_position = (
                list(
                    self._root_pose_position_by_prim.get(self.articulation_prim_path)
                    or self._prim_position(self.articulation_prim_path)
                )
                if self.articulation_prim_path and self._valid_prim_path(self.articulation_prim_path)
                else []
            )
        except Exception as exc:
            return {
                "type": "robot_state",
                "client_id": self.client_id,
                "source": "isaacsim_bridge",
                "message": f"state read failed: {exc}",
                "timestamp": now,
            }

        # Unity 호환 schema를 유지하되 source는 isaacsim_bridge로 표시합니다.
        return {
            "type": "robot_state",
            "client_id": self.client_id,
            "source": "isaacsim_bridge",
            "bridge_build": BRIDGE_BUILD,
            "scene": scene,
            "object_name": object_name,
            "robot_type": self.robot_type,
            "controller_mode": self.controller_mode,
            "position": position,
            "rotation": rotation,
            "root_motion_prim_path": root_motion_target_path,
            "root_motion_position": root_motion_position,
            "hold_root_motion_pose": self.hold_root_motion_pose,
            "restore_root_on_stop": self.restore_root_on_stop,
            "root_pose_use_articulation_api": self.root_pose_use_articulation_api,
            "home_root_position": list(self.home_root_position),
            "home_root_rotation": list(self.home_root_rotation),
            "articulation_prim_path": self.articulation_prim_path,
            "articulation_position": articulation_position,
            "eef_pose": position + rotation,
            "gripper": "noop",
            "last_prompt": self._last_prompt,
            "last_instruction": self._last_instruction,
            "last_model_prompt": self._last_model_prompt,
            "last_prompt_timestamp": self._last_prompt_timestamp,
            "prompt_history_size": len(self._prompt_history),
            "animation_enabled": self.enable_animation,
            "animation_ready": self._animation_setup_done,
            "animation_prim_path": self.animation_prim_path,
            "skeleton_prim_path": self.skeleton_prim_path,
            "humanoid_frozen": self._humanoid_frozen,
            "last_root_motion": _json_safe(self._last_root_motion_debug),
            "last_action_diagnostics": _json_safe(self._last_command_diagnostics),
            "timestamp": now,
        }

    def _setup_rgb_annotator(self) -> bool:
        # Replicator RGB annotator를 준비합니다.
        # 최초 한 번만 render product를 만들고 이후에는 같은 annotator를 재사용합니다.
        if not self.camera_prim_path:
            return False
        if self._rgb_annotator is not None:
            return True
        if not self._ensure_camera_prim():
            return False

        try:
            import omni.replicator.core as rep

            # render_product는 지정한 Camera prim을 실제 이미지 버퍼로 렌더링하는 Replicator 객체입니다.
            self._render_product = rep.create.render_product(
                self.camera_prim_path,
                (self.camera_width, self.camera_height),
            )
            self._rgb_annotator = rep.AnnotatorRegistry.get_annotator("rgb")
            self._rgb_annotator.attach([self._render_product])
            return True
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] RGB capture disabled: {exc}")
            self.send_image = False
            return False

    def _ensure_camera_prim(self) -> bool:
        # KyleRobot 하위에 카메라 prim이 없으면 만들고, 있으면 그 prim을 재사용합니다.
        if self._camera_ready:
            return True

        try:
            self._resolve_robot_prim_path()
            from pxr import Gf, UsdGeom

            stage = self._get_stage()
            robot_prim = stage.GetPrimAtPath(self.robot_prim_path)
            if not robot_prim or not robot_prim.IsValid():
                raise ValueError(f"Robot prim not found: {self.robot_prim_path}")

            camera_prim = stage.GetPrimAtPath(self.camera_prim_path)
            created = False
            if not camera_prim or not camera_prim.IsValid():
                # 자동 카메라 생성 경로입니다. 기본 위치/회전은 환경 변수로 조정할 수 있습니다.
                camera = UsdGeom.Camera.Define(stage, self.camera_prim_path)
                camera_prim = camera.GetPrim()
                created = True
            elif camera_prim.GetTypeName() != "Camera":
                raise ValueError(
                    f"Camera prim path exists but is not a Camera: {self.camera_prim_path}"
                )
            else:
                camera = UsdGeom.Camera(camera_prim)

            if created or self.camera_update_pose:
                # camera_update_pose=false이면 사용자가 UI에서 조정한 카메라 transform을 보존합니다.
                xform_api = UsdGeom.XformCommonAPI(camera_prim)
                xform_api.SetTranslate(Gf.Vec3d(*self.camera_local_position))
                xform_api.SetRotate(
                    Gf.Vec3f(*self.camera_local_rotation_deg),
                    UsdGeom.XformCommonAPI.RotationOrderXYZ,
                )
                xform_api.SetScale(Gf.Vec3f(*self.camera_local_scale))

            # 카메라 렌즈 파라미터는 viewport와 무관하게 RGB capture 결과에 영향을 줍니다.
            camera.GetFocalLengthAttr().Set(float(self.camera_focal_length))
            camera.GetHorizontalApertureAttr().Set(float(self.camera_horizontal_aperture))

            self._camera_ready = True
            if not self._camera_notice_printed:
                action = "created" if created else "using"
                print(
                    "[VlaGraphIsaacSimBridge] "
                    f"{action} camera={self.camera_prim_path} "
                    f"robot={self.robot_prim_path} "
                    f"resolution={self.camera_width}x{self.camera_height}"
                )
                self._camera_notice_printed = True
            return True
        except Exception as exc:
            if not self._camera_notice_printed:
                print(f"[VlaGraphIsaacSimBridge] camera setup failed: {exc}")
                self._camera_notice_printed = True
            return False

    def _build_image_payload(self, now: float) -> Optional[Dict[str, Any]]:
        # 현재 카메라 이미지를 JPEG data URI로 만들어 backend WebSocket에 보낼 payload를 만듭니다.
        if not self._setup_rgb_annotator():
            return None

        try:
            from PIL import Image
            import numpy as np

            data = self._rgb_annotator.get_data()
            if isinstance(data, dict):
                data = data.get("data")
            array = np.asarray(data)
            if array.ndim != 3:
                return None
            if array.shape[2] == 4:
                # Replicator가 RGBA를 줄 때 backend/브라우저 표시용으로 RGB만 사용합니다.
                array = array[:, :, :3]
            rgb_array = np.ascontiguousarray(array.astype("uint8"))
            image = Image.fromarray(rgb_array)
            buffer = io.BytesIO()
            # 품질 80은 네트워크 부하와 시각 품질 사이의 타협값입니다.
            image.save(buffer, format="JPEG", quality=80)
            encoded = base64.b64encode(buffer.getvalue()).decode("ascii")
            return {
                "client_id": self.client_id,
                "image": f"data:image/jpeg;base64,{encoded}",
                "content_type": "image/jpeg",
                "width": int(array.shape[1]),
                "height": int(array.shape[0]),
                "scene": "",
                "object_name": self.robot_prim_path.rsplit("/", 1)[-1],
                "source": "isaacsim_bridge",
                "timestamp": now,
            }
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] RGB frame failed: {exc}")
            return None


_BRIDGE: Optional[VlaGraphIsaacSimBridge] = None
_BRIDGE_REGISTRY_KEY = "_VLAGRAPH_ISAACSIM_BRIDGE"


def startup(port: Optional[int] = None) -> VlaGraphIsaacSimBridge:
    # Script Editor에서 exec(...)를 여러 번 실행할 수 있으므로,
    # builtins에 이전 bridge를 보관했다가 새 실행 전에 정리합니다.
    global _BRIDGE
    existing = getattr(builtins, _BRIDGE_REGISTRY_KEY, None)
    if existing is not None:
        try:
            existing.stop()
            print("[VlaGraphIsaacSimBridge] stopped previous bridge instance")
            time.sleep(0.25)
        except Exception as exc:
            print(f"[VlaGraphIsaacSimBridge] previous bridge stop failed: {exc}")
    _BRIDGE = None

    if port is not None:
        os.environ["VLAGRAPH_OMNIVERSE_PORT"] = str(int(port))

    # 새 bridge를 만들고 builtins에도 등록해 다음 실행에서 찾을 수 있게 합니다.
    _BRIDGE = VlaGraphIsaacSimBridge()
    setattr(builtins, _BRIDGE_REGISTRY_KEY, _BRIDGE)
    _BRIDGE.start()
    if _BRIDGE._network_failed.is_set():
        if hasattr(builtins, _BRIDGE_REGISTRY_KEY):
            delattr(builtins, _BRIDGE_REGISTRY_KEY)
        failed = _BRIDGE
        _BRIDGE = None
        return failed
    return _BRIDGE


def shutdown() -> None:
    # 수동으로 bridge를 멈추고 싶을 때 Script Editor에서 shutdown()을 호출합니다.
    global _BRIDGE
    bridge = _BRIDGE or getattr(builtins, _BRIDGE_REGISTRY_KEY, None)
    if bridge is not None:
        bridge.stop()
    _BRIDGE = None
    if hasattr(builtins, _BRIDGE_REGISTRY_KEY):
        delattr(builtins, _BRIDGE_REGISTRY_KEY)


if __name__ == "__main__":
    # exec(open(...).read())로 실행하면 __name__ == "__main__" 이므로 바로 시작됩니다.
    startup()
