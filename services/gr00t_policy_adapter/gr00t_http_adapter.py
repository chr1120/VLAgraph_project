from __future__ import annotations

import base64
import io
import json
import math
import os
from functools import lru_cache
from pathlib import Path
from typing import Any, Dict, Optional
from urllib.parse import urlparse

import numpy as np
import requests
from fastapi import FastAPI, HTTPException
from PIL import Image, ImageOps
from pydantic import BaseModel, ConfigDict, Field


app = FastAPI(title="gr00t-policy-http-adapter", version="1.1.0")

_CLIENT: Any = None
_CLIENT_ERROR = ""

_REAL_G1_KEY = "real_g1_relative_eef_relative_joints"
_EMBODIMENT_ALIASES = {
    "real_g1": _REAL_G1_KEY,
    "g1": _REAL_G1_KEY,
    "humanoid": _REAL_G1_KEY,
}

_G1_JOINT_GROUPS = {
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
    "waist": ("torso_joint",),
}


class InferRequest(BaseModel):
    model_config = ConfigDict(extra="allow")

    instruction: str = ""
    prompt: str = ""
    image: Any = ""
    image_url: str = ""
    state: Dict[str, Any] = Field(default_factory=dict)
    robot_state: Dict[str, Any] = Field(default_factory=dict)
    model_name: str = "nvidia/GR00T-N1.7-3B"
    embodiment_tag: str = "REAL_G1"
    inference_mode: str = "pytorch"
    action_horizon: int = 40
    action_format: str = ""
    require_complete_state: bool = False
    require_image: bool = False
    unity_command: str = ""
    waypoint_speed: Any = None


def _env_bool(name: str, default: bool = False) -> bool:
    value = os.getenv(name)
    if value is None:
        return default
    return value.strip().lower() in {"1", "true", "yes", "y", "on"}


def _env_int(name: str, default: int) -> int:
    value = os.getenv(name)
    if value is None:
        return default
    try:
        return int(value)
    except ValueError:
        return default


def _model_path() -> Path:
    return Path(os.getenv("GR00T_MODEL_PATH", "/models/GR00T-N1.7-3B"))


@lru_cache(maxsize=1)
def _processor_config() -> Dict[str, Any]:
    path = _model_path() / "processor_config.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


@lru_cache(maxsize=1)
def _statistics() -> Dict[str, Any]:
    path = _model_path() / "statistics.json"
    with path.open("r", encoding="utf-8") as fh:
        return json.load(fh)


def _processor_modality_configs() -> Dict[str, Any]:
    return (
        _processor_config()
        .get("processor_kwargs", {})
        .get("modality_configs", {})
    )


def _resolve_embodiment_key(tag: Any) -> str:
    configured = os.getenv("GR00T_EMBODIMENT_KEY", "").strip()
    configs = _processor_modality_configs()
    if configured in configs:
        return configured

    tag_text = str(tag or "").strip()
    if tag_text in configs:
        return tag_text

    lowered = tag_text.lower()
    if lowered in _EMBODIMENT_ALIASES and _EMBODIMENT_ALIASES[lowered] in configs:
        return _EMBODIMENT_ALIASES[lowered]

    for key in configs:
        if key.lower() == lowered:
            return key

    if _REAL_G1_KEY in configs:
        return _REAL_G1_KEY

    return next(iter(configs), _REAL_G1_KEY)


def _modality_config(embodiment_key: str, modality: str) -> Dict[str, Any]:
    value = _processor_modality_configs().get(embodiment_key, {}).get(modality, {})
    return value if isinstance(value, dict) else {}


def _modality_keys(embodiment_key: str, modality: str, fallback: str) -> list[str]:
    keys = _modality_config(embodiment_key, modality).get("modality_keys", [])
    if isinstance(keys, list) and keys:
        return [str(key) for key in keys]
    return [fallback]


def _horizon(embodiment_key: str, modality: str, default: int = 1) -> int:
    delta = _modality_config(embodiment_key, modality).get("delta_indices", [])
    if isinstance(delta, list) and delta:
        return len(delta)
    return default


def _json_safe(value: Any) -> Any:
    if isinstance(value, np.ndarray):
        return value.tolist()
    if isinstance(value, np.generic):
        return value.item()
    if isinstance(value, (str, int, float, bool)) or value is None:
        return value
    if isinstance(value, dict):
        return {str(key): _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return str(value)


def _get_policy_client() -> Any:
    global _CLIENT, _CLIENT_ERROR
    if _CLIENT is not None:
        return _CLIENT

    try:
        from gr00t.policy.server_client import PolicyClient
    except Exception as exc:
        _CLIENT_ERROR = (
            "Could not import gr00t.policy.server_client.PolicyClient. "
            "Run this adapter inside the Isaac-GR00T runtime image."
        )
        raise RuntimeError(f"{_CLIENT_ERROR} Detail: {exc}") from exc

    host = os.getenv("GR00T_POLICY_HOST", "127.0.0.1")
    port = _env_int("GR00T_POLICY_PORT", 5555)
    strict = _env_bool("GR00T_POLICY_STRICT", True)
    try:
        _CLIENT = PolicyClient(host=host, port=port, strict=strict)
    except TypeError:
        _CLIENT = PolicyClient(host=host, port=port)
    return _CLIENT


def _load_image_bytes(image_ref: Any) -> Optional[bytes]:
    if not isinstance(image_ref, str) or not image_ref.strip():
        return None

    value = image_ref.strip()
    if value.startswith("data:"):
        _, _, encoded = value.partition(",")
        if not encoded:
            return None
        return base64.b64decode(encoded)

    if value.startswith("base64:"):
        return base64.b64decode(value.split(":", 1)[1])

    parsed = urlparse(value)
    if parsed.scheme in {"http", "https"}:
        response = requests.get(value, timeout=float(os.getenv("GR00T_IMAGE_TIMEOUT_SEC", "10")))
        response.raise_for_status()
        return response.content

    if os.path.exists(value):
        with open(value, "rb") as fh:
            return fh.read()

    return None


def _image_to_observation_array(image_ref: Any, image_url: Any, horizon: int, *, strict: bool = False) -> np.ndarray:
    size = _env_int("GR00T_IMAGE_SIZE", 256)
    raw = _load_image_bytes(image_ref) or _load_image_bytes(image_url)

    if raw:
        with Image.open(io.BytesIO(raw)) as image:
            rgb = ImageOps.exif_transpose(image).convert("RGB")
            rgb = rgb.resize((size, size), Image.Resampling.BILINEAR)
            array = np.asarray(rgb, dtype=np.uint8)
    elif strict:
        raise ValueError("A valid robot-bound camera image is required; no blank image substitution")
    else:
        array = np.zeros((size, size, 3), dtype=np.uint8)

    batched = array[None, None, :, :, :]
    return np.repeat(batched, max(1, horizon), axis=1)


def _float_list(value: Any) -> list[float]:
    if isinstance(value, dict):
        ordered = ["x", "y", "z", "roll", "pitch", "yaw", "w"]
        out = []
        for key in ordered:
            if key not in value:
                continue
            try:
                out.append(float(value[key]))
            except Exception:
                out.append(0.0)
        return out

    if isinstance(value, (list, tuple, np.ndarray)):
        out = []
        for item in list(value):
            try:
                out.append(float(item))
            except Exception:
                out.append(0.0)
        return out

    return []


def _state_source(state: Dict[str, Any]) -> Dict[str, Any]:
    if not isinstance(state, dict):
        return {}
    nested = state.get("state")
    if isinstance(nested, dict):
        return nested
    return state


def _stat_mean(embodiment_key: str, key: str) -> list[float]:
    stats = _statistics().get(embodiment_key, {}).get("state", {}).get(key, {})
    mean = stats.get("mean", [])
    return _float_list(mean)


def _state_dim(embodiment_key: str, key: str) -> int:
    mean = _stat_mean(embodiment_key, key)
    if mean:
        return len(mean)
    fallback_dims = {
        "left_wrist_eef_9d": 9,
        "right_wrist_eef_9d": 9,
        "left_hand": 7,
        "right_hand": 7,
        "left_arm": 7,
        "right_arm": 7,
        "waist": 3,
    }
    return fallback_dims.get(key, _env_int("GR00T_STATE_DIM", 14))


def _pad_vector(values: list[float], template: list[float], dim: int) -> list[float]:
    padded = list(template[:dim])
    while len(padded) < dim:
        padded.append(0.0)
    for index, value in enumerate(values[:dim]):
        padded[index] = float(value)
    return padded


def _joint_group_vector(state: Dict[str, Any], key: str) -> list[float]:
    names = _G1_JOINT_GROUPS.get(key, ())
    named = state.get("named_joint_positions", {})
    if not isinstance(named, dict):
        return []

    values = []
    for name in names:
        if name in named:
            try:
                values.append(float(named[name]))
            except Exception:
                values.append(0.0)
    return values


def _state_array_for_key(
    state: Dict[str, Any],
    embodiment_key: str,
    key: str,
    horizon: int,
) -> np.ndarray:
    source = _state_source(state)
    dim = _state_dim(embodiment_key, key)
    template = _stat_mean(embodiment_key, key)

    values = _float_list(source.get(key))
    if not values:
        values = _joint_group_vector(source, key)

    vector = _pad_vector(values, template, dim)
    array = np.asarray(vector, dtype=np.float32)[None, None, :]
    return np.repeat(array, max(1, horizon), axis=1)


def _build_observation(req: InferRequest) -> tuple[Dict[str, Any], Dict[str, Any]]:
    prompt = req.instruction or req.prompt
    state = req.robot_state or req.state or {}
    if not isinstance(state, dict):
        state = {}

    embodiment_key = _resolve_embodiment_key(req.embodiment_tag)
    if req.require_complete_state:
        configs = _processor_modality_configs()
        requested = str(req.embodiment_tag).strip()
        requested_key = next((key for key in configs if key.lower() == requested.lower()), None)
        requested_key = requested_key or _EMBODIMENT_ALIASES.get(requested.lower())
        if requested_key not in configs or requested_key != embodiment_key:
            raise ValueError("Requested robot embodiment is unavailable or differs from the configured policy")
    video_horizon = _horizon(embodiment_key, "video", 1)
    state_horizon = _horizon(embodiment_key, "state", 1)
    language_horizon = _horizon(embodiment_key, "language", 1)

    image = _image_to_observation_array(req.image, req.image_url, video_horizon,
                                      strict=req.require_complete_state and req.require_image)
    video_keys = _modality_keys(
        embodiment_key,
        "video",
        os.getenv("GR00T_CAMERA_KEY", "ego_view"),
    )
    state_keys = _modality_keys(embodiment_key, "state", "joint_positions")
    if req.require_complete_state:
        source = _state_source(state)
        for key in state_keys:
            values = source.get(key)
            if (not isinstance(values, (list, tuple)) or len(values) != _state_dim(embodiment_key, key) or
                    any(isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) for v in values)):
                raise ValueError(f"Complete measured state required for {key}; padding and truncation are disabled")
    language_keys = _modality_keys(
        embodiment_key,
        "language",
        os.getenv("GR00T_LANGUAGE_KEY", "annotation.human.task_description"),
    )

    observation = {
        "video": {key: image for key in video_keys},
        "state": {
            key: _state_array_for_key(state, embodiment_key, key, state_horizon)
            for key in state_keys
        },
        "language": {
            key: [[prompt for _ in range(max(1, language_horizon))]]
            for key in language_keys
        },
    }
    debug = {
        "embodiment_key": embodiment_key,
        "video_keys": video_keys,
        "state_keys": state_keys,
        "language_keys": language_keys,
        "video_horizon": video_horizon,
        "state_horizon": state_horizon,
        "language_horizon": language_horizon,
        "state_source_keys": sorted(str(key) for key in _state_source(state).keys()),
    }
    return observation, debug


def _first_action(actions: Any) -> Any:
    if isinstance(actions, dict):
        for key in ("action", "actions", "joint_positions", "joint_targets", "continuous_action"):
            if key in actions:
                return _first_action(actions[key])
        if len(actions) == 1:
            return _first_action(next(iter(actions.values())))
        return actions

    if isinstance(actions, np.ndarray):
        array = actions
        while array.ndim > 1 and array.shape[0] > 0:
            array = array[0]
        return array

    if isinstance(actions, list):
        if not actions:
            return []
        if all(isinstance(item, (int, float)) for item in actions):
            return actions
        return _first_action(actions[0])

    return actions


@app.get("/health")
def health() -> Dict[str, Any]:
    host = os.getenv("GR00T_POLICY_HOST", "127.0.0.1")
    port = _env_int("GR00T_POLICY_PORT", 5555)
    embodiment_key = ""
    model_error = ""
    try:
        embodiment_key = _resolve_embodiment_key(os.getenv("GR00T_EMBODIMENT_TAG", "REAL_G1"))
    except Exception as exc:
        model_error = str(exc)

    try:
        client = _get_policy_client()
        alive = bool(client.ping())
        return {
            "ok": alive and not model_error,
            "service": "gr00t-policy-http-adapter",
            "policy_host": host,
            "policy_port": port,
            "policy_alive": alive,
            "model_path": str(_model_path()),
            "embodiment_key": embodiment_key,
            "model_error": model_error,
            "client_error": "",
        }
    except Exception as exc:
        return {
            "ok": False,
            "service": "gr00t-policy-http-adapter",
            "policy_host": host,
            "policy_port": port,
            "policy_alive": False,
            "model_path": str(_model_path()),
            "embodiment_key": embodiment_key,
            "model_error": model_error,
            "client_error": str(exc),
        }


@app.post("/infer")
def infer(req: InferRequest) -> Dict[str, Any]:
    prompt = req.instruction or req.prompt
    try:
        observation, observation_debug = _build_observation(req)
    except Exception as exc:
        raise HTTPException(
            status_code=422,
            detail=f"Failed to build GR00T observation: {exc}",
        ) from exc

    options = {
        "model_name": req.model_name,
        "embodiment_tag": req.embodiment_tag,
        "inference_mode": req.inference_mode,
        "action_horizon": req.action_horizon,
        "action_format": req.action_format,
        "unity_command": req.unity_command,
        "waypoint_speed": req.waypoint_speed,
    }

    try:
        client = _get_policy_client()
        try:
            actions, info = client.get_action(observation, options=options)
        except TypeError:
            actions, info = client.get_action(observation)
    except Exception as exc:
        raise HTTPException(
            status_code=503,
            detail=f"GR00T PolicyClient request failed: {exc}",
        ) from exc

    first_action = _first_action(actions)
    return {
        "ok": True,
        "service": "gr00t-policy-http-adapter",
        "model": req.model_name,
        "model_name": req.model_name,
        "embodiment_tag": req.embodiment_tag,
        "inference_mode": req.inference_mode,
        "action_horizon": req.action_horizon,
        "action": _json_safe(first_action),
        "action_chunk": _json_safe(actions),
        "info": _json_safe(info),
        "debug": {
            "instruction": prompt,
            **observation_debug,
        },
    }
