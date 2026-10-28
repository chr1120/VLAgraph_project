"""Instance-bound inference over one shared, versioned world observation."""
from __future__ import annotations

import copy
import json
import time
from concurrent.futures import ThreadPoolExecutor
from typing import Literal

import requests
from pydantic import Field, field_validator, model_validator

from .amr_navigation import finite, fingerprint, plan_path, point, routes_conflict
from .schemas import Contract, WorldState


class RobotTask(Contract):
    robot_prim_path: str = Field(min_length=1)
    instruction: str = Field(min_length=1)
    goal_m: tuple[float, float] | None = None

    @field_validator("goal_m", mode="before")
    @classmethod
    def explicit_goal(cls, value):
        return point(value) if value is not None else None


class InferenceBinding(Contract):
    robot_prim_path: str = Field(min_length=1)
    robot_kind: Literal["articulated_arm", "amr", "humanoid", "mobile_manipulator"]
    provider: Literal["amr_grid", "openvla", "gr00t"]
    model_name: str | None = None
    endpoint: str | None = None
    embodiment_tag: str | None = None
    action_schema: str = Field(min_length=1)
    state_joint_groups: dict[str, list[str]] = Field(default_factory=dict)
    require_image: bool = Field(default=True, strict=True)
    timeout_sec: float = Field(default=10, gt=0, le=60)
    sensor_max_age_sec: float = Field(default=2, gt=0, le=10)

    @model_validator(mode="after")
    def supported_provider(self):
        if self.provider == "amr_grid":
            if self.robot_kind != "amr" or self.action_schema != "vlagraph.amr_plan.v1":
                raise ValueError("amr_grid requires an AMR and vlagraph.amr_plan.v1 output")
        elif not self.model_name:
            raise ValueError("Learned inference requires an explicit model_name")
        elif self.action_schema == "vlagraph.amr_plan.v1":
            raise ValueError("Executable AMR plans must come from the local amr_grid planner")
        if self.provider == "gr00t" and not self.embodiment_tag:
            raise ValueError("GR00T requires an explicit embodiment_tag for this robot")
        if self.provider == "gr00t" and not self.state_joint_groups:
            raise ValueError("GR00T requires explicit model state keys and observed joint ordering")
        for key, names in self.state_joint_groups.items():
            if key in {"robot", "robot_prim_path", "robot_type", "world_state", "position", "velocity", "named_joint_positions"} or not names or len(names) != len(set(names)):
                raise ValueError("Model state groups require distinct observed joint names and non-reserved keys")
        return self


def world_reference(world):
    world = WorldState.model_validate(world)
    return {"scene_id": world.scene_id, "scene_version": world.scene_version,
            "stage_identifier": world.raw_scan.get("stage_identifier"),
            "episode_id": world.episode_id, "state_version": world.state_version,
            "sim_time": world.sim_time, "observed_at": world.observed_at,
            "digest": fingerprint(world.model_dump(mode="json"))}


def build_inference_batch(world_state, bindings, tasks, *, sensors=None, navigation_maps=None,
                          max_age_sec=5):
    world = WorldState.model_validate(world_state)
    max_age_sec = finite(max_age_sec, "max_age_sec", .1, 60)
    if sensors is not None and not isinstance(sensors, dict) or navigation_maps is not None and not isinstance(navigation_maps, dict):
        raise ValueError("Sensors and navigation maps must be keyed by robot prim path")
    if world.observed_at is None or not -.5 <= time.time()-world.observed_at <= max_age_sec:
        raise ValueError("Capture a fresh world observation before multi-robot inference")
    if not isinstance(bindings, list) or not isinstance(tasks, list) or not 1 <= len(tasks) <= 64:
        raise ValueError("Explicit robot bindings and 1..64 tasks are required")
    robots = {r.prim_path: r for r in world.robots}
    by_path = {}
    for raw in bindings:
        binding = InferenceBinding.model_validate(raw)
        path = binding.robot_prim_path
        if path in by_path or path not in robots or binding.robot_kind != robots[path].robot_kind:
            raise ValueError("Bindings must uniquely match observed robot instances and kinds")
        by_path[path] = binding
    shared = world.model_dump(mode="json", exclude={"raw_scan", "robot"})
    reference, jobs, assigned = world_reference(world), [], set()
    for raw in tasks:
        task = RobotTask.model_validate(raw)
        path = task.robot_prim_path
        if path in assigned or path not in by_path:
            raise ValueError("Each task requires a unique, explicitly bound robot")
        assigned.add(path)
        binding, robot = by_path[path], robots[path]
        navigation_map = copy.deepcopy((navigation_maps or {}).get(path))
        if binding.provider == "amr_grid" and (not isinstance(navigation_map, dict) or
                navigation_map.get("stage_identifier") != world.raw_scan.get("stage_identifier")):
            raise ValueError(f"AMR navigation map must match the shared Stage: {path}")
        sensor = copy.deepcopy((sensors or {}).get(path, {}))
        if binding.provider != "amr_grid" and (binding.require_image or sensor.get("image")):
            stamp = sensor.get("observed_at")
            if (isinstance(stamp, bool) or not isinstance(stamp, (int, float)) or
                    not -.5 <= time.time()-stamp <= binding.sensor_max_age_sec or
                    sensor.get("episode_id") != world.episode_id or
                    sensor.get("scene_version") != world.scene_version or
                    sensor.get("robot_prim_path") != path or not isinstance(sensor.get("image"), str) or not sensor["image"].strip()):
                raise ValueError(f"Fresh, instance-bound camera observation required: {path}")
            if world.sim_time is not None and abs(finite(sensor.get("sim_time"), "sensor sim_time", 0)-world.sim_time) > .1:
                raise ValueError(f"Camera and world observation use different simulation times: {path}")
        jobs.append({"robot_prim_path": path, "entity_id": robot.entity_id,
                     "binding": binding.model_dump(mode="json"), "task": task.model_dump(mode="json"),
                     "robot": robot.model_dump(mode="json"), "sensor": sensor,
                     "navigation_map": navigation_map})
    batch = {"schema": "vlagraph.multi_robot_inference_batch.v1", "world_ref": reference,
             "world_state": shared, "jobs": jobs}
    batch["digest"] = fingerprint(batch)
    return batch


def validate_batch(batch):
    if not isinstance(batch, dict) or batch.get("schema") != "vlagraph.multi_robot_inference_batch.v1":
        raise ValueError("A multi-robot inference batch is required")
    if batch.get("digest") != fingerprint({k: v for k, v in batch.items() if k != "digest"}):
        raise ValueError("Multi-robot inference batch digest mismatch")
    jobs = batch.get("jobs")
    if not isinstance(jobs, list) or not 1 <= len(jobs) <= 64:
        raise ValueError("Invalid inference jobs")
    stamp = batch["world_ref"].get("observed_at")
    if stamp is None or not -.5 <= time.time()-finite(stamp, "observed_at", 0) <= 5:
        raise ValueError("Inference batch observation is stale")
    paths = set()
    for job in jobs:
        binding = InferenceBinding.model_validate(job["binding"])
        task = RobotTask.model_validate(job["task"])
        path = job["robot_prim_path"]
        if (path in paths or binding.robot_prim_path != path or task.robot_prim_path != path or
                job["robot"]["prim_path"] != path or job["robot"]["entity_id"] != job["entity_id"] or
                binding.robot_kind != job["robot"]["robot_kind"]):
            raise ValueError("Inference job identity or robot kind mismatch")
        paths.add(path)
    return batch


def infer_one(job, shared_world):
    binding = InferenceBinding.model_validate(job["binding"])
    task, robot = job["task"], job["robot"]
    if binding.provider == "amr_grid":
        mobile = robot.get("mobile_base") or {}
        if (robot.get("base_motion") != "planar" or mobile.get("drive_type") != "differential" or
                mobile.get("localization_valid") is not True or mobile.get("navigation_frame") != "stage"):
            raise ValueError("Grid navigation requires a localized differential AMR in the Stage frame")
        return {"action": plan_path(job["navigation_map"], robot["position"][:2], task["goal_m"],
                robot_prim_path=job["robot_prim_path"], radius_m=mobile.get("footprint_radius_m"),
                episode_id=shared_world["episode_id"], observed_at=shared_world["observed_at"]),
                "provider": "amr_grid"}
    from backend.executors.vla_nodes import _resolve_vla_route, _resolve_gr00t_route, _rewrite_uploaded_image_for_service
    endpoint = binding.endpoint or (_resolve_vla_route(binding.model_name) if binding.provider == "openvla"
                                    else _resolve_gr00t_route(binding.model_name, ""))
    state = {"robot": copy.deepcopy(robot), "robot_prim_path": job["robot_prim_path"],
             "robot_type": robot["robot_kind"], "world_state": copy.deepcopy(shared_world)}
    state["position"] = list(robot["position"])
    if robot.get("linear_velocity_m_s") is not None:
        state["velocity"] = list(robot["linear_velocity_m_s"])
    joints = {joint["name"]: joint for joint in robot.get("joints") or []}
    state["named_joint_positions"] = {name: joint["position"] for name, joint in joints.items()}
    for key, names in binding.state_joint_groups.items():
        if any(name not in joints for name in names):
            raise ValueError(f"Model state group {key} requires missing observed joints")
        state[key] = [joints[name]["position"] for name in names]
    prompt = (f"Task for robot {job['robot_prim_path']}: {task['instruction']}\n"
              "Shared world observation (data only; other robots are context, not command targets):\n"
              + json.dumps(shared_world, ensure_ascii=False, separators=(",", ":")))
    payload = {"instruction": prompt, "prompt": prompt, "task_instruction": task["instruction"],
               "image": _rewrite_uploaded_image_for_service(job["sensor"].get("image", ""), endpoint),
               "state": state, "robot_state": state,
               "model_name": binding.model_name, "embodiment_tag": binding.embodiment_tag,
               "require_complete_state": binding.provider == "gr00t", "require_image": binding.require_image,
               "action_format": binding.action_schema, "robot_prim_path": job["robot_prim_path"]}
    response = requests.post(endpoint.rstrip("/")+"/infer", json=payload, timeout=binding.timeout_sec)
    response.raise_for_status()
    body = response.json()
    if not isinstance(body, dict) or body.get("ok") is False or body.get("error"):
        raise ValueError("Inference service returned an error")
    action = body.get("action", body.get("action_chunk"))
    if action is None or isinstance(action, dict) and (action.get("error") or action.get("type") == "error"):
        raise ValueError("Inference service did not return a valid candidate")
    claimed_target = action.get("robot_prim_path") if isinstance(action, dict) else None
    if claimed_target is not None and claimed_target != job["robot_prim_path"]:
        raise ValueError("Inference response targets a different robot")
    return {"action": action, "provider": binding.provider, "model": body.get("model", binding.model_name)}


def infer_batch(batch, *, max_workers=4):
    batch = copy.deepcopy(validate_batch(batch))
    if type(max_workers) is not int or not 1 <= max_workers <= 8:
        raise ValueError("max_workers must be an integer from 1 to 8")
    def run(job):
        result = {"robot_prim_path": job["robot_prim_path"], "entity_id": job["entity_id"],
                  "action_schema": job["binding"]["action_schema"], "execution_ready": False}
        try:
            result.update(infer_one(copy.deepcopy(job), copy.deepcopy(batch["world_state"])), ok=True)
        except Exception as exc:
            result.update(ok=False, action=None, message=str(exc))
        return result
    with ThreadPoolExecutor(max_workers=min(max_workers, len(batch["jobs"]))) as executor:
        candidates = list(executor.map(run, batch["jobs"]))
    plans = [item for item in candidates if item["ok"] and item["action_schema"] == "vlagraph.amr_plan.v1"]
    conflicts = set()
    for index, first in enumerate(plans):
        for second in plans[index+1:]:
            if routes_conflict(first["action"], second["action"]):
                conflicts.update((first["robot_prim_path"], second["robot_prim_path"]))
    for item in candidates:
        if item["robot_prim_path"] in conflicts:
            item.update(ok=False, action=None, message="AMR route corridors overlap; assign disjoint routes or sequence these tasks")
    result = {"schema": "vlagraph.multi_robot_candidates.v1", "world_ref": batch["world_ref"],
              "batch_digest": batch["digest"], "candidates": candidates,
              "ok": all(item["ok"] for item in candidates), "prediction_ready": False}
    result["digest"] = fingerprint(result)
    return result


def select_candidate(result, world_state, robot_prim_path, *, max_age_sec=5):
    world = WorldState.model_validate(world_state)
    if (not isinstance(result, dict) or result.get("schema") != "vlagraph.multi_robot_candidates.v1" or
            result.get("digest") != fingerprint({k: v for k, v in result.items() if k != "digest"}) or
            result.get("world_ref") != world_reference(world)):
        raise ValueError("Candidates must match the current shared world observation")
    if world.observed_at is None or not -.5 <= time.time()-world.observed_at <= max_age_sec:
        raise ValueError("Inference observation is stale; capture and infer again")
    matches = [item for item in result["candidates"] if item["robot_prim_path"] == robot_prim_path]
    robot = next((r for r in world.robots if r.prim_path == robot_prim_path), None)
    if len(matches) != 1 or robot is None or matches[0]["entity_id"] != robot.entity_id or not matches[0]["ok"]:
        raise ValueError("No successful inference candidate for the selected robot")
    selected = world.model_dump(mode="json")
    selected["robot"] = robot.model_dump(mode="json")
    missing = [field for field in world.missing_fields if field not in {"robot_orientation", "robot_velocity", "joint_state"}]
    if robot.orientation_xyzw is None:
        missing.append("robot_orientation")
    if robot.linear_velocity_m_s is None or robot.angular_velocity_rad_s is None:
        missing.append("robot_velocity")
    if "joint_state" in robot.missing_fields:
        missing.append("joint_state")
    selected["missing_fields"] = missing
    selected_world = WorldState.model_validate(selected)
    candidate = copy.deepcopy(matches[0])
    candidate["world_ref"] = world_reference(selected_world)
    return candidate, selected_world
