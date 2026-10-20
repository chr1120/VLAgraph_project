from __future__ import annotations

import math
import re
from typing import Any, Dict, List

from backend.executors.common_nodes import _incoming_values
from backend.executors.g1_action_schema import G1_NAVIGATION_KEYS, G1_POLICY_GROUPS, g1_schema_payload


SUPPORTED_G1_SKILLS: tuple[str, ...] = (
    "stop",
    "reset",
    "stand",
    "walk",
    "run",
    "sidestep",
    "turn",
    "go_to_pose",
    "wave",
    "dance",
    "crouch",
    "bow",
    "kick",
    "punch",
    "stretch",
    "raise_arms",
    "lower_arms",
    "reach",
    "push",
    "pull",
    "clap",
    "nod",
    "shake_head",
    "look_left",
    "look_right",
    "look_up",
    "look_down",
    "jump",
)

_LOCOMOTION_SKILLS = {"stand", "walk", "run", "sidestep", "turn", "go_to_pose"}

_STOP_WORDS = ("stop", "halt", "pause", "freeze")
_RESET_WORDS = ("reset", "home", "origin", "init")
_WALK_WORDS = ("walk", "go forward", "go straight", "step forward", "forward", "straight")
_RUN_WORDS = ("run", "dash", "sprint")
_SIDESTEP_WORDS = ("sidestep", "side step", "strafe")
_TURN_WORDS = ("turn", "rotate", "spin")
_STAND_WORDS = ("stand", "idle", "pose", "stay")
_WAVE_WORDS = ("wave", "hello", "hi", "greet")
_DANCE_WORDS = ("dance",)
_JUMP_WORDS = ("jump", "hop")
_CROUCH_WORDS = ("crouch", "squat", "sit")
_BOW_WORDS = ("bow", "bend")
_KICK_WORDS = ("kick",)
_PUNCH_WORDS = ("punch", "jab")
_STRETCH_WORDS = ("stretch",)
_RAISE_ARMS_WORDS = ("raise arms", "arms up", "hands up")
_LOWER_ARMS_WORDS = ("lower arms", "arms down", "hands down")
_REACH_WORDS = ("reach", "point", "touch", "grab", "pick")
_PUSH_WORDS = ("push", "press")
_PULL_WORDS = ("pull", "drag")
_CLAP_WORDS = ("clap",)
_NOD_WORDS = ("nod",)
_SHAKE_HEAD_WORDS = ("shake head",)
_LOOK_LEFT_WORDS = ("look left",)
_LOOK_RIGHT_WORDS = ("look right",)
_LOOK_UP_WORDS = ("look up",)
_LOOK_DOWN_WORDS = ("look down",)

_NUMBER_PATTERN = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?"

_SKILL_ALIASES = {
    "walk_forward": "walk",
    "go_forward": "walk",
    "move_forward": "walk",
    "walk_backward": "walk",
    "move_backward": "walk",
    "run_forward": "run",
    "turn_left": "turn",
    "turn_right": "turn",
    "sidestep_left": "sidestep",
    "sidestep_right": "sidestep",
    "strafe_left": "sidestep",
    "strafe_right": "sidestep",
    "raise_arm": "raise_arms",
    "raise_hands": "raise_arms",
    "lower_arm": "lower_arms",
    "lower_hands": "lower_arms",
    "shake": "shake_head",
    "look": "look_left",
}


def exec_g1_skill_decode(
    node: Dict[str, Any],
    context: Dict[str, Any],
    edges: List[Dict[str, Any]],
) -> Dict[str, Any]:
    data = node.get("data", {}) or {}
    inputs = _incoming_values(node["id"], edges, context)
    action = inputs.get("action", {})
    prompt = _text(inputs.get("prompt") or data.get("prompt") or _prompt_from_action(action))
    state = inputs.get("state", {})

    skill_command = decode_g1_skill_command(prompt=prompt, action=action, state=state, props=data)
    return {
        "action": skill_command,
        "skill": skill_command.get("skill", "unknown"),
        "intent": skill_command.get("intent", {}),
        "velocity": skill_command.get("velocity", {}),
        "target_pose": skill_command.get("target_pose"),
        "supported_skills": list(SUPPORTED_G1_SKILLS),
    }


def decode_g1_skill_command(
    *,
    prompt: Any = "",
    action: Any = None,
    state: Any = None,
    props: Dict[str, Any] | None = None,
) -> Dict[str, Any]:
    props = props or {}
    action_payload = _extract_action_payload(action)
    prompt_text = _text(prompt or _prompt_from_action(action_payload))
    control_prompt = _control_prompt_text(prompt_text)
    normalized_prompt = _normalize(control_prompt)

    duration = _duration_from_action(action_payload)
    if duration is None:
        duration = _parse_duration(normalized_prompt)
    duration = _bounded(duration, 0.25, 30.0, 3.0)

    speed = _float_from_action(action_payload, "speed", "waypoint_speed", "waypointSpeed")
    if speed is None:
        speed = _parse_speed(normalized_prompt)
    default_speed = _to_float(props.get("defaultSpeed"), 0.55)
    speed = _bounded(speed, 0.05, 2.0, default_speed)

    amplitude = _float_from_action(action_payload, "amplitude")
    if amplitude is None:
        amplitude = _parse_amplitude(normalized_prompt)
    amplitude = _bounded(amplitude, 0.05, 2.0, 1.0)

    explicit_skill = _canonical_skill(_first_text(action_payload, "skill", "motion_skill", "motionSkill", "gesture"))
    approach_waypoint = _extract_approach_waypoint(control_prompt, state)
    target_pose = (
        _target_pose_from_action(action_payload)
        or (approach_waypoint.get("target_pose") if approach_waypoint else None)
        or _extract_target_pose(control_prompt)
    )

    nav = _navigation_from_action(action_payload)
    move_x, move_z, turn_yaw = _move_from_action(action_payload)
    if approach_waypoint and abs(move_x) <= 0.05 and abs(move_z) <= 0.05:
        move_x = approach_waypoint["move_x"]
        move_z = approach_waypoint["move_z"]
        turn_yaw = approach_waypoint["turn_yaw"]

    prompt_skill = _skill_from_prompt(normalized_prompt)
    prompt_has_turn = _contains(normalized_prompt, _TURN_WORDS)
    prompt_has_walk = _contains(normalized_prompt, _WALK_WORDS)
    skill = explicit_skill or prompt_skill
    requested_skill = bool(skill)
    turn_while_moving = not explicit_skill and prompt_skill == "turn" and prompt_has_walk
    if turn_while_moving:
        skill = "walk"
        requested_skill = True
    if target_pose is not None and skill in {"", "walk", "stand"}:
        skill = "go_to_pose"
        requested_skill = True
    if not skill:
        skill = _skill_from_motion_values(move_x, move_z, turn_yaw, nav) or "stand"

    direction_x, direction_y = _direction_from_prompt(normalized_prompt)
    motion_values_can_guide = skill in _LOCOMOTION_SKILLS and skill != "go_to_pose" and not prompt_skill
    if nav is not None and motion_values_can_guide:
        direction_y = _clamp(nav[0], -1.0, 1.0)
        direction_x = _clamp(nav[1], -1.0, 1.0)
        turn_yaw = nav[2]
        if not requested_skill and (abs(direction_x) > 0.05 or abs(direction_y) > 0.05):
            skill = "sidestep" if abs(direction_x) > abs(direction_y) else "walk"
        elif not requested_skill and abs(turn_yaw) > 0.05:
            skill = "turn"
    elif motion_values_can_guide and (abs(move_x) > 0.05 or abs(move_z) > 0.05):
        direction_x = _clamp(move_x, -1.0, 1.0)
        direction_y = _clamp(move_z, -1.0, 1.0)
        if not requested_skill:
            skill = "sidestep" if abs(move_x) > abs(move_z) else "walk"
    elif motion_values_can_guide and abs(turn_yaw) > 0.05 and not requested_skill:
        skill = "turn"

    if "back" in normalized_prompt or "backward" in normalized_prompt:
        direction_y = -abs(direction_y or 1.0)
        if skill == "stand":
            skill = "walk"
    if skill == "sidestep" and abs(direction_x) < 0.05:
        direction_x = -1.0 if _has_left(normalized_prompt) else 1.0
        direction_y = 0.0
    if skill in {"walk", "run"} and abs(direction_x) < 0.05 and abs(direction_y) < 0.05:
        direction_y = 1.0

    turn_velocity = 0.0
    if skill == "turn":
        turn_velocity = _turn_velocity(normalized_prompt, turn_yaw, duration)
        direction_x = 1.0 if turn_velocity >= 0.0 else -1.0
        direction_y = 0.0
    elif skill in {"walk", "run", "go_to_pose"} and prompt_has_turn:
        turn_velocity = _turn_velocity(normalized_prompt, turn_yaw, duration)
        if not _contains(normalized_prompt, _SIDESTEP_WORDS):
            direction_x = 0.0
            if abs(direction_y) > 0.01:
                direction_y = 1.0 if direction_y > 0.0 else -1.0
            elif prompt_has_walk:
                direction_y = 1.0

    planar_mag = math.hypot(direction_x, direction_y)
    if planar_mag > 1.0:
        direction_x /= planar_mag
        direction_y /= planar_mag

    forward_velocity = direction_y * speed if skill in {"walk", "run", "sidestep", "go_to_pose"} else 0.0
    lateral_velocity = direction_x * speed if skill in {"walk", "run", "sidestep", "go_to_pose"} else 0.0
    if skill == "run":
        forward_velocity = direction_y * max(speed, 0.95)
        lateral_velocity = direction_x * max(speed, 0.95)

    if not approach_waypoint and skill in {"walk", "run", "sidestep", "go_to_pose"}:
        if abs(move_x) <= 0.05 and abs(move_z) <= 0.05:
            move_x = direction_x
            move_z = direction_y
        if abs(turn_yaw) <= 0.05 and abs(turn_velocity) > 1.0e-6:
            turn_yaw = _clamp(math.degrees(turn_velocity * duration) / 15.0, -1.0, 1.0)

    stop = skill == "stop"
    reset = skill == "reset"
    use_motion_command = skill in _LOCOMOTION_SKILLS and not stop and not reset
    action_command = {
        "type": "g1_skill_command",
        "command": "apply_humanoid_motion" if use_motion_command else "apply_humanoid_step",
        "robot_type": "humanoid",
        "controller_mode": "locomotion",
        "skill": skill,
        "gesture": skill,
        "animation": bool(use_motion_command),
        "prompt": prompt_text,
        "duration_sec": duration,
        "speed": speed,
        "amplitude": amplitude,
        "direction": {
            "x": direction_x,
            "y": direction_y,
        },
        "velocity": {
            "forward": forward_velocity,
            "lateral": lateral_velocity,
            "yaw": turn_velocity,
        },
        "forward_velocity": forward_velocity,
        "lateral_velocity": lateral_velocity,
        "turn_velocity": turn_velocity,
        "move_x": move_x,
        "move_z": move_z,
        "turn_yaw": turn_yaw,
        "local_frame": bool(approach_waypoint),
        "target_pose": target_pose,
        "stop": stop,
        "reset": reset,
        "terminate": stop,
        "intent": {
            "skill": skill,
            "source": "g1_skill_decode",
            "prompt": prompt_text,
            "supported": skill in SUPPORTED_G1_SKILLS,
        },
        "joint_schema": g1_schema_payload(),
        "raw_action_summary": _raw_action_summary(action_payload),
    }
    if approach_waypoint:
        action_command["duration_sec"] = approach_waypoint["duration_sec"]
        action_command["intent"]["approach_waypoint"] = approach_waypoint["waypoint"]
        action_command["intent"]["target_position"] = approach_waypoint["target_position"]
        action_command["intent"]["target_rotation"] = approach_waypoint["target_rotation"]
        action_command["intent"]["distance_to_waypoint"] = approach_waypoint["distance"]
        action_command["intent"]["heading_error_rad"] = approach_waypoint["heading_error_rad"]
    if use_motion_command:
        trajectory = _procedural_g1_locomotion_trajectory(
            skill=skill,
            duration_sec=float(action_command["duration_sec"]),
            speed=speed,
            move_x=move_x,
            move_z=move_z,
            turn_yaw=turn_yaw,
            heading_error_rad=approach_waypoint.get("heading_error_rad") if approach_waypoint else None,
            world_delta=approach_waypoint.get("world_delta") if approach_waypoint else None,
            rotate_then_move=bool(approach_waypoint),
        )
        if trajectory:
            action_command["g1_joint_trajectory"] = trajectory
            action_command["joint_trajectory_count"] = len(trajectory)
            action_command["joint_trajectory_duration_sec"] = trajectory[-1]["time_sec"]
            action_command["retarget_mode"] = "procedural_walk"
            action_command["root_translation_mode"] = "relative"
            action_command["motion_summary"] = {
                "source": "procedural_g1_locomotion",
                "frames": len(trajectory),
                "duration_sec": trajectory[-1]["time_sec"],
                "move_x": move_x,
                "move_z": move_z,
                "turn_yaw": turn_yaw,
            }
    return action_command


def _procedural_g1_locomotion_trajectory(
    *,
    skill: str,
    duration_sec: float,
    speed: float,
    move_x: float,
    move_z: float,
    turn_yaw: float,
    heading_error_rad: float | None = None,
    world_delta: list[float] | tuple[float, float] | None = None,
    rotate_then_move: bool = False,
) -> list[dict[str, Any]]:
    if skill not in {"walk", "run", "sidestep", "turn", "go_to_pose"}:
        return []

    duration = _clamp(float(duration_sec or 0.0), 0.8, 6.0)
    fps = 12.0
    frame_count = max(10, min(72, int(round(duration * fps)) + 1))
    step_amp = 0.20 if skill != "run" else 0.30
    knee_amp = 0.34 if skill != "run" else 0.48
    ankle_amp = 0.18 if skill != "run" else 0.24
    lateral_amp = 0.10 if abs(move_x) > 0.05 or skill == "sidestep" else 0.035
    yaw_amp = 0.08 if abs(turn_yaw) > 0.05 or skill == "turn" else 0.025
    arm_amp = 0.22 if skill != "run" else 0.34
    world_dx = world_dy = None
    if isinstance(world_delta, (list, tuple)) and len(world_delta) >= 2:
        try:
            world_dx = float(world_delta[0])
            world_dy = float(world_delta[1])
        except Exception:
            world_dx = world_dy = None
    if world_dx is not None and world_dy is not None:
        x_total = _clamp(world_dx, -3.0, 3.0)
        y_total = _clamp(world_dy, -3.0, 3.0)
    else:
        travel = min(max(abs(speed) * duration, 0.12), 3.0)
        planar_mag = max(0.1, math.hypot(move_x, move_z))
        x_total = travel * _clamp(move_z / planar_mag, -1.0, 1.0)
        y_total = travel * _clamp(move_x / planar_mag, -1.0, 1.0)
    yaw_total = (
        math.degrees(float(heading_error_rad))
        if heading_error_rad is not None
        else _clamp(turn_yaw, -1.0, 1.0) * 12.0
    )
    yaw_total = _clamp(yaw_total, -120.0, 120.0)
    turn_fraction = 0.0
    if rotate_then_move and abs(yaw_total) > 3.0 and math.hypot(x_total, y_total) > 0.05:
        turn_fraction = _clamp(abs(yaw_total) / 90.0, 0.25, 0.45)

    frames: list[dict[str, Any]] = []
    previous_x = 0.0
    previous_y = 0.0
    previous_yaw = 0.0
    for index in range(frame_count):
        t = index / max(1, frame_count - 1)
        phase = 2.0 * math.pi * t * max(1.0, duration * (1.3 if skill == "run" else 0.95))
        swing = math.sin(phase)
        lift_left = max(0.0, swing)
        lift_right = max(0.0, -swing)
        lean = _clamp(move_z, -1.0, 1.0) * 0.04
        lateral = _clamp(move_x, -1.0, 1.0) * lateral_amp
        yaw = _clamp(turn_yaw, -1.0, 1.0) * yaw_amp

        named_targets = {
            "left_hip_pitch_joint": -step_amp * swing + lean,
            "left_hip_roll_joint": lateral,
            "left_hip_yaw_joint": yaw,
            "left_knee_joint": knee_amp * lift_left,
            "left_ankle_pitch_joint": -ankle_amp * lift_left + 0.05 * swing,
            "left_ankle_roll_joint": -0.5 * lateral,
            "right_hip_pitch_joint": step_amp * swing + lean,
            "right_hip_roll_joint": -lateral,
            "right_hip_yaw_joint": yaw,
            "right_knee_joint": knee_amp * lift_right,
            "right_ankle_pitch_joint": -ankle_amp * lift_right - 0.05 * swing,
            "right_ankle_roll_joint": 0.5 * lateral,
            "torso_joint": 0.03 * math.sin(phase * 0.5) + 0.04 * _clamp(turn_yaw, -1.0, 1.0),
            "waist_yaw_joint": 0.04 * _clamp(turn_yaw, -1.0, 1.0),
            "left_shoulder_pitch_joint": arm_amp * swing,
            "left_shoulder_roll_joint": 0.05,
            "left_shoulder_yaw_joint": 0.03 * _clamp(turn_yaw, -1.0, 1.0),
            "left_elbow_pitch_joint": -0.18 + 0.05 * lift_right,
            "right_shoulder_pitch_joint": -arm_amp * swing,
            "right_shoulder_roll_joint": -0.05,
            "right_shoulder_yaw_joint": 0.03 * _clamp(turn_yaw, -1.0, 1.0),
            "right_elbow_pitch_joint": -0.18 + 0.05 * lift_left,
        }
        if turn_fraction > 0.0:
            turn_progress = _smoothstep(_clamp(t / turn_fraction, 0.0, 1.0))
            move_progress = _smoothstep(_clamp((t - turn_fraction) / max(1.0e-6, 1.0 - turn_fraction), 0.0, 1.0))
        else:
            turn_progress = t
            move_progress = t
        x_offset = x_total * move_progress
        y_offset = y_total * move_progress
        yaw_offset = yaw_total * turn_progress
        delta_x = x_offset - previous_x
        delta_y = y_offset - previous_y
        delta_yaw = yaw_offset - previous_yaw
        previous_x = x_offset
        previous_y = y_offset
        previous_yaw = yaw_offset
        frames.append(
            {
                "time_sec": round(duration * t, 4),
                "g1_named_targets": {key: round(float(value), 6) for key, value in named_targets.items()},
                "root_translation": [round(delta_x, 6), round(delta_y, 6), 0.0],
                "root_delta_rotation": [0.0, 0.0, round(delta_yaw, 6)],
                "root_translation_mode": "relative",
                "root_translation_frame": "world",
                "retarget_mode": "procedural_walk",
            }
        )
    return frames


def _smoothstep(value: float) -> float:
    value = _clamp(value, 0.0, 1.0)
    return value * value * (3.0 - 2.0 * value)


def _extract_action_payload(action: Any) -> Any:
    if isinstance(action, dict):
        if "safe_action" in action:
            return action["safe_action"]
        if "decoded" in action:
            return action["decoded"]
        if "action" in action and isinstance(action["action"], dict):
            return action["action"]
    return action


def _raw_action_summary(action: Any) -> Dict[str, Any]:
    if not isinstance(action, dict):
        return {"type": type(action).__name__}

    keys = sorted(str(key) for key in action.keys())
    policy_keys: List[str] = []
    for key in G1_POLICY_GROUPS:
        if key in action or f"action.{key}" in action:
            policy_keys.append(key)
    for key in G1_NAVIGATION_KEYS:
        if key in action or f"action.{key}" in action:
            policy_keys.append(key)

    return {
        "key_count": len(keys),
        "keys": keys[:40],
        "policy_keys": policy_keys,
        "embodiment_tag": _first_text(action, "embodiment_tag", "embodimentTag"),
    }


def _prompt_from_action(action: Any) -> str:
    if not isinstance(action, dict):
        return ""
    return _first_text(action, "prompt", "instruction", "task", "goal", "text", "model_prompt", "prompt_with_context")


def _skill_from_prompt(text: str) -> str:
    if _contains(text, _STOP_WORDS) and not _negates_stop(text):
        return "stop"
    if _contains(text, _RESET_WORDS):
        return "reset"
    checks = (
        ("dance", _DANCE_WORDS),
        ("wave", _WAVE_WORDS),
        ("bow", _BOW_WORDS),
        ("stretch", _STRETCH_WORDS),
        ("raise_arms", _RAISE_ARMS_WORDS),
        ("lower_arms", _LOWER_ARMS_WORDS),
        ("reach", _REACH_WORDS),
        ("push", _PUSH_WORDS),
        ("pull", _PULL_WORDS),
        ("clap", _CLAP_WORDS),
        ("crouch", _CROUCH_WORDS),
        ("kick", _KICK_WORDS),
        ("punch", _PUNCH_WORDS),
        ("nod", _NOD_WORDS),
        ("shake_head", _SHAKE_HEAD_WORDS),
        ("look_left", _LOOK_LEFT_WORDS),
        ("look_right", _LOOK_RIGHT_WORDS),
        ("look_up", _LOOK_UP_WORDS),
        ("look_down", _LOOK_DOWN_WORDS),
        ("jump", _JUMP_WORDS),
    )
    for skill, words in checks:
        if _contains(text, words):
            return skill
    if _contains(text, _TURN_WORDS):
        return "turn"
    if _contains(text, _SIDESTEP_WORDS):
        return "sidestep"
    if _contains(text, _RUN_WORDS):
        return "run"
    if _contains(text, _STAND_WORDS):
        return "stand"
    if _contains(text, _WALK_WORDS) or _has_direction(text) or _parse_distance(text) is not None:
        return "walk"
    return ""


def _skill_from_motion_values(move_x: float, move_z: float, turn_yaw: float, nav: list[float] | None) -> str:
    if nav is not None:
        move_z, move_x, turn_yaw = nav[0], nav[1], nav[2]
    if abs(move_x) > 0.05 or abs(move_z) > 0.05:
        return "sidestep" if abs(move_x) > abs(move_z) else "walk"
    if abs(turn_yaw) > 0.05:
        return "turn"
    return ""


def _canonical_skill(value: Any) -> str:
    text = _normalize(value).replace("-", "_").replace(" ", "_")
    if not text:
        return ""
    text = _SKILL_ALIASES.get(text, text)
    return text if text in SUPPORTED_G1_SKILLS else ""


def _navigation_from_action(action: Any) -> list[float] | None:
    if not isinstance(action, dict):
        return None
    nav = action.get("navigate_command") or action.get("action.navigate_command")
    if nav is None and isinstance(action.get("action"), dict):
        nav = action["action"].get("navigate_command")
    values = _flat_float_list(nav)
    if len(values) >= 3:
        return [values[0], values[1], values[2]]
    return None


def _move_from_action(action: Any) -> tuple[float, float, float]:
    if not isinstance(action, dict):
        return 0.0, 0.0, 0.0
    translation = _flat_float_list(action.get("translation") or action.get("delta_position"))
    rotation = _flat_float_list(action.get("rotation") or action.get("delta_rotation"))
    move_x = _to_float(action.get("move_x", action.get("moveX", translation[0] if len(translation) >= 1 else 0.0)), 0.0)
    move_z = _to_float(action.get("move_z", action.get("moveZ", translation[2] if len(translation) >= 3 else 0.0)), 0.0)
    turn_yaw = _to_float(action.get("turn_yaw", action.get("turnYaw", rotation[1] if len(rotation) >= 2 else 0.0)), 0.0)
    return move_x, move_z, turn_yaw


def _duration_from_action(action: Any) -> float | None:
    return _float_from_action(action, "duration_sec", "durationSec", "duration")


def _float_from_action(action: Any, *keys: str) -> float | None:
    if not isinstance(action, dict):
        return None
    for key in keys:
        if key in action:
            try:
                return float(action[key])
            except Exception:
                return None
    return None


def _target_pose_from_action(action: Any) -> list[float] | None:
    if not isinstance(action, dict):
        return None
    target = (
        action.get("target_pose")
        or action.get("targetPose")
        or action.get("target_position")
        or action.get("targetPosition")
    )
    values = _flat_float_list(target)
    if len(values) >= 3:
        return values[:6] if len(values) >= 6 else values[:3]
    return None


def _extract_target_pose(text: str) -> list[float] | None:
    if not text:
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
        rf"[\[\(]\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})(?:\s*,\s*({_NUMBER_PATTERN}))?\s*[\]\)]"
    )
    bracket_match = bracket_pattern.search(text)
    if bracket_match:
        values = [float(bracket_match.group(i)) for i in (1, 2, 3)]
        if bracket_match.group(4) is not None:
            values.append(float(bracket_match.group(4)))
        return values
    return None


def _extract_approach_waypoint(text: str, state: Any) -> Dict[str, Any] | None:
    if not text:
        return None
    explicit_pattern = re.compile(
        rf"approach\s+waypoint\s+is\s+"
        rf"x\s*[:=]\s*({_NUMBER_PATTERN})\D+"
        rf"y\s*[:=]\s*({_NUMBER_PATTERN})\D+"
        rf"yaw\s*[:=]\s*({_NUMBER_PATTERN})",
        re.IGNORECASE,
    )
    match = explicit_pattern.search(text)
    if match:
        waypoint_x = float(match.group(1))
        waypoint_y = float(match.group(2))
        waypoint_yaw = float(match.group(3))
    else:
        bracket_pattern = re.compile(
            rf"(?:stored_)?approach[_\s-]*waypoint\s*[:=]\s*"
            rf"[\[\(]\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})\s*,\s*({_NUMBER_PATTERN})",
            re.IGNORECASE,
        )
        bracket_match = bracket_pattern.search(text)
        if not bracket_match:
            return None
        waypoint_x = float(bracket_match.group(1))
        waypoint_y = float(bracket_match.group(2))
        waypoint_yaw = float(bracket_match.group(3))

    return _approach_waypoint_command(waypoint_x, waypoint_y, waypoint_yaw, state)


def _approach_waypoint_command(waypoint_x: float, waypoint_y: float, waypoint_yaw: float, state: Any) -> Dict[str, Any] | None:
    try:
        waypoint_x = float(waypoint_x)
        waypoint_y = float(waypoint_y)
        waypoint_yaw = float(waypoint_yaw)
    except Exception:
        return None

    current_position = _state_position(state)
    current_x = current_position[0] if current_position else 0.0
    current_y = current_position[1] if current_position else 0.0
    current_z = current_position[2] if current_position and len(current_position) >= 3 else 0.0
    current_yaw = _state_yaw_rad(state)

    dx = waypoint_x - current_x
    dy = waypoint_y - current_y
    distance = math.hypot(dx, dy)
    desired_heading = math.atan2(dy, dx) if distance > 1.0e-6 else current_yaw
    heading_error = _normalize_angle_rad(desired_heading - current_yaw)
    abs_heading_error = abs(heading_error)

    magnitude = _clamp(min(distance / 0.75, 1.0), 0.0, 1.0)
    local_forward = math.cos(heading_error)
    local_lateral = math.sin(heading_error)
    if distance < 0.2:
        move_x = 0.0
        move_z = 0.0
        turn_yaw = _clamp(_normalize_angle_rad(waypoint_yaw - current_yaw) / math.radians(15.0), -1.0, 1.0)
    else:
        move_x = _clamp(magnitude * local_lateral, -1.0, 1.0)
        move_z = _clamp(magnitude * max(0.25, local_forward), 0.0, 1.0)
        turn_yaw = _clamp(heading_error / math.radians(15.0), -1.0, 1.0)

    duration = _clamp(distance / 0.55 if distance > 1.0e-6 else 1.0, 0.5, 12.0)
    yaw_degrees = math.degrees(waypoint_yaw)
    return {
        "waypoint": [waypoint_x, waypoint_y, waypoint_yaw],
        "target_pose": [waypoint_x, waypoint_y, current_z, 0.0, 0.0, yaw_degrees],
        "target_position": [waypoint_x, waypoint_y, current_z],
        "target_rotation": [0.0, 0.0, yaw_degrees],
        "world_delta": [dx, dy],
        "move_x": move_x,
        "move_z": move_z,
        "turn_yaw": turn_yaw,
        "duration_sec": duration,
        "distance": distance,
        "heading_error_rad": heading_error,
    }


def _state_position(state: Any) -> list[float] | None:
    if not isinstance(state, dict):
        return None
    raw_position = (
        state.get("position")
        or state.get("translation")
        or state.get("root_position")
        or state.get("rootPosition")
    )
    values = _flat_float_list(raw_position)
    if len(values) >= 3:
        return values[:3]
    raw_state = state.get("state")
    if isinstance(raw_state, dict) and raw_state is not state:
        return _state_position(raw_state)
    return None


def _state_yaw_rad(state: Any) -> float:
    if not isinstance(state, dict):
        return 0.0
    raw_rotation = (
        state.get("rotation")
        or state.get("euler")
        or state.get("root_rotation")
        or state.get("rootRotation")
    )
    values = _flat_float_list(raw_rotation)
    if len(values) >= 3:
        yaw = values[2]
        if abs(yaw) > math.tau:
            return math.radians(yaw)
        return yaw
    raw_state = state.get("state")
    if isinstance(raw_state, dict) and raw_state is not state:
        return _state_yaw_rad(raw_state)
    return 0.0


def _normalize_angle_rad(value: float) -> float:
    while value > math.pi:
        value -= math.tau
    while value < -math.pi:
        value += math.tau
    return value


def _direction_from_prompt(text: str) -> tuple[float, float]:
    x = 0.0
    y = 1.0
    if "back" in text or "backward" in text:
        y = -1.0
    if _has_left(text):
        x = -1.0
        if not _contains(text, _TURN_WORDS):
            y = 0.0
    if _has_right(text):
        x = 1.0
        if not _contains(text, _TURN_WORDS):
            y = 0.0
    if "forward" in text or "straight" in text:
        y = 1.0 if abs(x) < 0.01 else y
    length = math.hypot(x, y)
    if length <= 1.0e-6:
        return 0.0, 1.0
    return x / length, y / length


def _turn_velocity(text: str, turn_yaw: float, duration: float) -> float:
    if abs(turn_yaw) > 0.05:
        return math.radians(_clamp(turn_yaw * 45.0, -90.0, 90.0)) / max(0.25, duration)
    sign = -1.0 if _has_right(text) else 1.0
    degrees = _parse_degrees(text)
    if degrees is None:
        if any(word in text for word in ("small", "gentle", "slight", "little")):
            degrees = 30.0
        elif any(word in text for word in ("big", "wide", "large", "strong", "sharp")):
            degrees = 120.0
        else:
            degrees = 90.0
    return math.radians(sign * degrees) / max(0.25, duration)


def _parse_duration(text: str) -> float:
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:seconds?|secs?|sec|s)", text)
    if match:
        return float(match.group(1))
    distance = _parse_distance(text)
    if distance is not None:
        return max(0.5, min(20.0, abs(distance) / 0.55))
    return 3.0


def _parse_distance(text: str) -> float | None:
    match = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:meters?|meter|m)\b", text)
    if match:
        return float(match.group(1))
    match = re.search(r"(-?\d+(?:\.\d+)?)\s*(?:centimeters?|centimeter|cm)\b", text)
    if match:
        return float(match.group(1)) / 100.0
    return None


def _parse_degrees(text: str) -> float | None:
    match = re.search(r"(\d+(?:\.\d+)?)\s*(?:degrees?|deg)\b", text)
    if not match:
        return None
    return _clamp(float(match.group(1)), 1.0, 1080.0)


def _parse_speed(text: str) -> float:
    if any(word in text for word in ("fast", "quick", "rapid")):
        return 0.9
    if any(word in text for word in ("slow", "slowly", "careful")):
        return 0.25
    return 0.55


def _parse_amplitude(text: str) -> float:
    if any(word in text for word in ("big", "wide", "large", "strong")):
        return 1.35
    if any(word in text for word in ("small", "gentle", "slight", "little")):
        return 0.65
    return 1.0


def _contains(text: str, words: tuple[str, ...]) -> bool:
    for word in words:
        if not word:
            continue
        pattern = r"(?<![a-z0-9_])" + re.escape(word) + r"(?![a-z0-9_])"
        if re.search(pattern, text):
            return True
    return False


def _negates_stop(text: str) -> bool:
    return _contains(text, ("do not stop", "don't stop", "dont stop", "never stop", "keep going", "keep walking"))


def _has_left(text: str) -> bool:
    return _contains(text, ("left",))


def _has_right(text: str) -> bool:
    return _contains(text, ("right",))


def _has_direction(text: str) -> bool:
    return _has_left(text) or _has_right(text) or _contains(text, ("forward", "front", "back", "backward"))


def _normalize(value: Any) -> str:
    return re.sub(r"\s+", " ", str(value or "").strip().lower())


def _text(value: Any) -> str:
    return str(value or "").strip()


def _control_prompt_text(value: Any) -> str:
    text = _text(value)
    if not text:
        return ""
    marker = re.search(r"\bUser\s+task\s*:\s*", text, flags=re.IGNORECASE)
    if marker:
        return text[marker.end() :].strip()
    if text.lower().startswith("known spatial context:"):
        return ""
    return text


def _first_text(mapping: Any, *keys: str) -> str:
    if not isinstance(mapping, dict):
        return ""
    for key in keys:
        value = mapping.get(key)
        if isinstance(value, str) and value.strip():
            return value.strip()
    return ""


def _flat_float_list(value: Any) -> list[float]:
    if value is None or isinstance(value, (str, bytes, dict)):
        return []
    if not isinstance(value, (list, tuple)):
        try:
            return [float(value)]
        except Exception:
            return []
    result: list[float] = []
    for item in value:
        if isinstance(item, (list, tuple)):
            result.extend(_flat_float_list(item))
        else:
            try:
                result.append(float(item))
            except Exception:
                result.append(0.0)
    return result


def _to_float(value: Any, default: float) -> float:
    try:
        return float(value)
    except Exception:
        return default


def _bounded(value: Any, lower: float, upper: float, default: float) -> float:
    try:
        number = float(value)
    except Exception:
        number = default
    return _clamp(number, lower, upper)


def _clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, float(value)))
