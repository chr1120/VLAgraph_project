"""Versioned serial-arm profiles. Loading a profile never certifies execution."""
from __future__ import annotations

import copy
import json
import math
import re
from urllib.parse import urlparse

import numpy as np

from .fanuc_preflight import action_fingerprint


M710_ID = "fanuc_m710ic20l"
M710_PROFILE = {
    "schema": "vlagraph.serial_arm_profile.v1", "id": M710_ID, "revision": "1",
    "manufacturer": "FANUC", "model": "M-710iC/20L", "joint_names": [f"J{i}" for i in range(1, 7)],
    "origins_m": [[0, 0, .565], [.15, 0, 0], [0, 0, 1.15], [0, 0, .19], [1.8, 0, 0], [0, 0, 0]],
    "axes": [[0, 0, 1], [0, 1, 0], [0, -1, 0], [-1, 0, 0], [0, -1, 0], [-1, 0, 0]],
    "limits_deg": [[-180, 180], [-90, 135], [-80.5, 205], [-200, 200], [-140, 140], [-450, 450]],
    "tool_offset_m": [.1, 0, 0], "source": "project_m710_configuration",
}


def object_value(value):
    if isinstance(value, str):
        value = json.loads(value)
    if not isinstance(value, dict):
        raise ValueError("Expected a JSON object")
    return copy.deepcopy(value)


def validate_profile(value=None):
    profile = copy.deepcopy(M710_PROFILE) if value is None else object_value(value)
    if set(profile) != set(M710_PROFILE) or profile.get("schema") != M710_PROFILE["schema"]:
        raise ValueError("Profile requires exactly the serial-arm v1 fields")
    for field in ("id", "revision", "manufacturer", "model", "source"):
        if not isinstance(profile[field], str) or not profile[field].strip():
            raise ValueError(f"Profile {field} must be nonempty")
    if profile["joint_names"] != M710_PROFILE["joint_names"]:
        raise ValueError("This adapter requires six canonical revolute axes J1..J6")
    for field, shape in (("origins_m", (6, 3)), ("axes", (6, 3)), ("limits_deg", (6, 2)), ("tool_offset_m", (3,))):
        raw = profile[field]
        def numbers(item):
            if isinstance(item, list):
                return all(numbers(v) for v in item)
            return isinstance(item, (int, float)) and not isinstance(item, bool)
        if not numbers(raw):
            raise ValueError(f"Profile {field} must contain finite numbers")
        array = np.asarray(raw, dtype=float)
        if array.shape != shape or not np.isfinite(array).all():
            raise ValueError(f"Invalid profile {field} dimensions or finite values")
        if field == "axes" and not np.allclose(np.linalg.norm(array, axis=1), 1, atol=1e-6, rtol=0):
            raise ValueError("Joint axes must be unit vectors")
        if field == "limits_deg" and (np.any(array[:, 0] >= array[:, 1]) or np.max(np.abs(array)) > 10000):
            raise ValueError("Joint lower limits must be less than upper limits")
        if field in {"origins_m", "tool_offset_m"} and np.max(np.abs(array)) > 100:
            raise ValueError("Link geometry exceeds the supported meter range")
    if profile["id"] == M710_ID and profile != M710_PROFILE:
        raise ValueError("Modified M-710 geometry must use a custom profile ID")
    return profile


def profile_arrays(value=None):
    p = validate_profile(value)
    return np.asarray(p["origins_m"]), np.asarray(p["axes"]), np.radians(p["limits_deg"]), np.asarray(p["tool_offset_m"])


def validate_prepare_settings(value):
    settings = object_value(value)
    allowed = {"uframeTransforms", "utoolTransforms", "ikSeedDeg", "externalAxisVectors", "externalAxisJointNames",
               "externalAxisRootMotionEnabled", "externalAxisAffectsIk", "timingMode", "secondsPerFrame",
               "totalDurationSec", "trajectoryTimeScale"}
    if set(settings) - allowed:
        raise ValueError(f"Unsupported prepare settings: {sorted(set(settings)-allowed)}")
    def vector(raw, size):
        return isinstance(raw, list) and len(raw) == size and all(
            not isinstance(v, bool) and isinstance(v, (float, int)) and math.isfinite(v) for v in raw)
    for key, value in settings.items():
        if key in {"externalAxisRootMotionEnabled", "externalAxisAffectsIk"}:
            if not isinstance(value, bool):
                raise ValueError(f"{key} requires a JSON boolean")
        elif key == "timingMode":
            if value not in {"source_speed", "fixed_total"}:
                raise ValueError("timingMode must be source_speed or fixed_total")
        elif key in {"secondsPerFrame", "totalDurationSec", "trajectoryTimeScale"}:
            if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value <= 0:
                raise ValueError(f"{key} must be a finite positive number")
        elif key == "ikSeedDeg":
            if not vector(value, 6):
                raise ValueError("ikSeedDeg requires six finite degrees")
        elif key in {"uframeTransforms", "utoolTransforms"}:
            if not isinstance(value, dict):
                raise ValueError(f"{key} requires an indexed JSON object")
            for index, pose in value.items():
                if (not str(index).isdigit() or not isinstance(pose, dict) or
                        not (vector(pose.get("xyzMm"), 3) or vector(pose.get("xyzM"), 3)) or not vector(pose.get("wprDeg"), 3)):
                    raise ValueError(f"{key} requires numbered xyzMm/xyzM and wprDeg poses")
        else:
            if not isinstance(value, dict) or any(not isinstance(name, str) or not re.fullmatch(r"E\d+", name) for name in value):
                raise ValueError(f"{key} requires E-axis keys")
            if key == "externalAxisVectors" and any(not vector(v, 3) or np.linalg.norm(v) == 0 for v in value.values()):
                raise ValueError("External axis directions must be nonzero finite vectors")
            if key == "externalAxisJointNames" and (len(value) > 1 or any(v != "externalAxisJoint" for v in value.values())):
                raise ValueError("This adapter supports one mapped externalAxisJoint")
    return settings


def profile_from_inventory(inventory, *, profile_id, model, tool_offset_m):
    from .fanuc_usd_proxy import _check_inventory, _joint_frame, _axis, validate_fanuc_usd_binding
    units = _check_inventory(inventory)
    profile = copy.deepcopy(M710_PROFILE)
    profile.update(id=profile_id, model=model, source="usd_joint_frames", origins_m=[], axes=[], limits_deg=[],
                   tool_offset_m=tool_offset_m)
    for i in range(1, 7):
        joint = inventory["joint_bindings"][f"J{i}"]
        frame = _joint_frame(joint, 0, units)
        profile["origins_m"].append(frame[:3, 3].tolist())
        profile["axes"].append((frame[:3, :3] @ _axis(joint)).tolist())
        profile["limits_deg"].append([joint["lower_limit_deg"], joint["upper_limit_deg"]])
    profile = validate_profile(profile)
    if not validate_fanuc_usd_binding(inventory, profile=profile)["kinematics_match"]:
        raise ValueError("USD chain is not supported by the serial-arm adapter; arbitrary fixed joint rotations are not inferred")
    return profile


def validate_robot_config(value):
    config = object_value(value)
    if set(config) != {"schema", "instance_id", "profile", "mode", "connection", "memory_path", "prepare", "asset_inventory_digest", "digest"}:
        raise ValueError("Unexpected or missing robot configuration fields")
    if config.get("schema") != "vlagraph.robot_config.v1":
        raise ValueError("Unsupported robot configuration")
    digest = config.pop("digest", None)
    if digest != action_fingerprint(config):
        raise ValueError("Robot configuration digest mismatch")
    validate_profile(config["profile"])
    validate_prepare_settings(config["prepare"])
    if not isinstance(config.get("memory_path"), str) or not config["memory_path"].strip():
        raise ValueError("A Collection memory path is required")
    if not isinstance(config.get("connection"), dict) or set(config["connection"]) != {"endpoint", "robot_prim_path"}:
        raise ValueError("Robot connection requires endpoint and robot_prim_path")
    endpoint, prim = config["connection"]["endpoint"], config["connection"]["robot_prim_path"]
    if not isinstance(endpoint, str):
        raise ValueError("Robot endpoint must be a WebSocket URL")
    if config.get("mode") not in {"preset", "auto", "manual"}:
        raise ValueError("Invalid robot settings mode")
    parsed = urlparse(endpoint)
    if parsed.scheme not in {"ws", "wss"} or not parsed.hostname or parsed.username or parsed.password:
        raise ValueError("Robot bridge requires a ws/wss URL without embedded credentials")
    if not isinstance(prim, str) or not prim.startswith("/") or prim == "/" or ".." in prim.split("/"):
        raise ValueError("An absolute robot prim path is required")
    if not isinstance(config.get("instance_id"), str) or not config["instance_id"].strip():
        raise ValueError("A robot instance ID is required")
    config["digest"] = digest
    return config


def configured_data(node, context, edges, stage):
    """A connected setting always takes precedence; missing connected data fails closed."""
    links = [e for e in edges if e.get("target") == node["id"] and e.get("targetHandle") == "robot_config"]
    data = copy.deepcopy(node.get("data", {}))
    if not links:
        return data, None
    if len(links) != 1:
        raise ValueError("Connect exactly one robot settings node")
    edge = links[0]
    config = validate_robot_config(context.get(edge["source"], {}).get(edge.get("sourceHandle", "robot_config")))
    connection = config["connection"]
    data.update(omniverseEndpoint=connection["endpoint"], robotPrimPath=connection["robot_prim_path"],
                omniversePrimPath=connection["robot_prim_path"])
    if stage == "execute":
        data["clientId"] = config["instance_id"]
    if stage == "prepare":
        data.update(config["prepare"])
    if stage == "world":
        data["memoryPath"] = config["memory_path"]
    return data, config
