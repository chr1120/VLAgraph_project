"""Optional observation/completion adapter for the dedicated FANUC extension.

Installed next to isaac_fanuc_m710.vlagraph_bridge. All calls run on the
extension's update thread; never create/reset physics or move the robot here.
"""
from __future__ import annotations

import copy
import hashlib
import json
import math
import time
import uuid


class FanucWorldModelAdapter:
    def __init__(self, bridge, *, clock=None):
        self.bridge = bridge
        self.clock = clock or self._simulation_clock
        self.episode_id = uuid.uuid4().hex
        self.version = 0
        self.last_clock = None
        self.records = {}
        self.active_id = None
        self.last_snapshot = None
        self.stage_identifier = None

    @staticmethod
    def _simulation_clock():
        from isaacsim.core.simulation_manager import SimulationManager
        return float(SimulationManager.get_simulation_time()), int(SimulationManager.get_num_physics_steps())

    def snapshot(self):
        scene = self.bridge.scene
        before = self.clock()
        if not math.isfinite(before[0]) or before[0] < 0 or before[1] < 0:
            raise ValueError("Invalid simulator clock")
        if self.last_clock is not None and (before[0] < self.last_clock[0] or before[1] < self.last_clock[1]):
            self._cancel("simulation_reset")
            self.episode_id = uuid.uuid4().hex
            self.version = 0
        state = copy.deepcopy(scene.get_vlagraph_state())
        if not state.get("physics_ready"):
            raise ValueError("FANUC physics is not ready")
        velocities = scene._articulation.get_joint_velocities()
        if velocities is None:
            raise ValueError("Joint velocities unavailable")
        axes = []
        positions = state["joint_positions_rad"]
        targets = state["joint_target_positions_rad"]
        mapping = {item["command_name"]: item for item in state["command_dof_map"]}
        if len(positions) != len(state["joint_names"]) or len(targets) != len(positions):
            raise ValueError("Incomplete command-ordered joint state")
        for index, name in enumerate(state["joint_names"]):
            item = mapping[name]
            if item["virtual"]:
                velocity = None
            else:
                velocity = float(velocities[item["dof_index"]])
            values = [float(positions[index]), float(targets[index])]
            if not all(math.isfinite(v) for v in values) or (velocity is not None and not math.isfinite(velocity)):
                raise ValueError("Non-finite joint observation")
            axes.append({"name": name, "position": values[0], "target_position": values[1],
                         "velocity": velocity, "unit": "m" if name == "externalAxisJoint" else "rad",
                         "virtual": item["virtual"]})
        geometry = self._geometry()
        if self.stage_identifier is not None and self.stage_identifier != geometry["stage_identifier"]:
            self._cancel("stage_changed")
            self.episode_id = uuid.uuid4().hex
            self.version = 0
        self.stage_identifier = geometry["stage_identifier"]
        after = self.clock()
        if before != after:
            raise ValueError("Physics advanced during capture; retry snapshot")
        self.version += 1
        self.last_clock = after
        snapshot = {"schema": "vlagraph.fanuc_snapshot.v1", "episode_id": self.episode_id,
                    "state_version": self.version, "sim_time": before[0], "physics_step": before[1],
                    "prim_path": state["prim_path"], "observed_at": time.time(), "axes": axes,
                    "synchronized_joint_snapshot": True, "geometry": geometry,
                    "prediction_ready": False, "raw_state": state}
        self.last_snapshot = copy.deepcopy(snapshot)
        return snapshot

    def _geometry(self):
        from pxr import Usd, UsdGeom
        stage = self.bridge._stage()
        units = float(UsdGeom.GetStageMetersPerUnit(stage))
        axis = str(UsdGeom.GetStageUpAxis(stage))
        if not math.isfinite(units) or units <= 0 or axis not in ("Y", "Z"):
            raise ValueError("Unknown stage coordinate system")
        result = {"meters_per_unit": units, "up_axis": axis,
                  "stage_identifier": str(stage.GetRootLayer().identifier),
                  "pose_source": "usd_default_time", "physics_synchronized": False,
                  "tcp_calibrated": False}
        for key, path in (("base_pose", self.bridge.scene.config.model_path),
                          ("flange_pose", self.bridge.scene.config.end_effector_path)):
            prim = stage.GetPrimAtPath(path)
            if not prim or not prim.IsValid():
                result[key] = None
                continue
            matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
            position = matrix.ExtractTranslation()
            quaternion = matrix.ExtractRotationQuat()
            result[key] = {"prim_path": path, "position_stage": [float(v) for v in position],
                           "orientation_wxyz": [float(quaternion.GetReal()), *[float(v) for v in quaternion.GetImaginary()]]}
        return result

    def _cancel(self, reason):
        if self.active_id:
            record = self.records[self.active_id]
            record.update(status="cancelled", reason=reason, completion_verified=False)
            self.active_id = None

    def tick(self):
        if not self.active_id:
            return
        record = self.records[self.active_id]
        scene = self.bridge.scene
        motion = scene._active_motion
        if motion is not None and motion is not record["_motion"]:
            self._cancel("superseded_outside_bridge")
            return
        try:
            snapshot = self.snapshot()
            if not self.active_id:  # clock reset cancelled this record
                return
            if time.monotonic() > record["_deadline"]:
                record.update(status="failed", reason="completion_timeout")
                self.active_id = None
                return
            if motion is not None:
                return
            finished = record["_motion"]
            if finished.get("elapsed", 0) < finished.get("duration", math.inf):
                self._cancel("interrupted_before_interpolation_end")
                return
            if snapshot["physics_step"] == record.get("_last_step"):
                return
            record["_last_step"] = snapshot["physics_step"]
            actual = {axis["name"]: axis for axis in snapshot["axes"]}
            stable = True
            for name, target in record["_target"].items():
                axis = actual[name]
                tolerance = 0.001 if axis["unit"] == "m" else 0.01
                speed_limit = 0.001 if axis["unit"] == "m" else 0.01
                stable &= (not axis["virtual"] and axis["velocity"] is not None
                           and abs(axis["position"] - target) <= tolerance
                           and abs(axis["velocity"]) <= speed_limit)
            record["_stable"] = record["_stable"] + 1 if stable else 0
            if record["_stable"] >= 3:
                record.update(status="completed", actual_state=snapshot, end_sim_time=snapshot["sim_time"],
                              completion_verified=True, outcome="unknown", reason="joint_target_settled")
                self.active_id = None
        except Exception as exc:
            record.update(status="failed", reason=str(exc), completion_verified=False)
            self.active_id = None

    def public_record(self, command_id):
        record = self.records.get(command_id)
        if record is None:
            raise ValueError("Unknown command_id; commands before adapter installation are not tracked")
        return copy.deepcopy({key: value for key, value in record.items() if not key.startswith("_")})

    def apply(self, payload, original):
        command = str(payload.get("command") or "").lower()
        request_id = payload.get("request_id")
        if command in {"capture_world_state", "get_command_status", "get_collision_geometry"}:
            try:
                if command == "get_collision_geometry":
                    try:
                        from .vlagraph_usd_geometry import extract_collision_inventory
                    except ImportError:
                        from backend.world_model.usd_geometry import extract_collision_inventory
                    data = {"inventory": extract_collision_inventory(
                        self.bridge._stage(), robot_prim_path=payload.get("prim_path") or self.bridge.scene.config.model_path,
                        root_path=payload.get("root_path", "/World"), max_colliders=payload.get("max_colliders", 4096))}
                else:
                    data = ({"snapshot": self.snapshot()} if command == "capture_world_state"
                            else {"execution": self.public_record(payload.get("command_id"))})
                return {"ok": True, "applied": False, "request_id": request_id, "command": command, **data}
            except Exception as exc:
                return {"ok": False, "applied": False, "request_id": request_id, "command": command, "message": str(exc)}
        command_id = payload.get("command_id")
        if command == "apply_fanuc_trajectory" and command_id:
            if not isinstance(command_id, str) or len(command_id) > 128:
                raise ValueError("command_id must be a string of at most 128 characters")
            digest = hashlib.sha256(json.dumps({"command": command, "action": payload.get("action"),
                                                "prim_path": payload.get("prim_path")}, sort_keys=True, allow_nan=False).encode()).hexdigest()
            if command_id in self.records:
                if self.records[command_id]["_digest"] != digest:
                    raise ValueError("command_id already used with different action")
                return {"ok": True, "applied": False, "request_id": request_id, "command": command,
                        "execution": self.public_record(command_id), "duplicate": True}
            if len(self.records) >= 256:
                raise ValueError("Command history full; restart adapter for a new session")
            if self.active_id or self.bridge.scene._active_motion is not None:
                raise ValueError("Robot already has active motion")
            start = self.snapshot()
            response = original(payload)
            if response.get("ok") and response.get("applied"):
                motion = self.bridge.scene._active_motion
                if motion is None:
                    raise ValueError("Bridge accepted trajectory without active motion")
                target = motion["keyframes"][-1][1]
                names = self.bridge.scene.command_joint_names
                self.records[command_id] = {
                    "command_id": command_id, "episode_id": start["episode_id"], "prim_path": start["prim_path"],
                    "status": "running", "start_sim_time": start["sim_time"], "completion_verified": False,
                    "outcome": "unknown", "_motion": motion, "_digest": digest,
                    "_target": dict(zip(names, target)), "_stable": 0,
                    "_deadline": time.monotonic() + float(motion["duration"]) + 15,
                }
                self.active_id = command_id
                response["execution"] = self.public_record(command_id)
            return response
        if command in {"stop", "hold", "init", "initialize"} or self.active_id and command not in {"get_state", "status", "scan_scene", "scene_scan"}:
            self._cancel("interrupted_by_command")
        response = original(payload)
        if command in {"scan_scene", "scene_scan"} and response.get("ok") and isinstance(response.get("scan"), dict):
            geometry = self._geometry()
            scan = response["scan"]
            scan.update(meters_per_unit=geometry["meters_per_unit"], up_axis=geometry["up_axis"],
                        truncated=len(scan.get("objects", [])) >= int(payload.get("max_objects") or 256))
        return response


def attach_world_model(bridge):
    if hasattr(bridge, "_world_model_adapter"):
        return
    adapter = FanucWorldModelAdapter(bridge)
    original_apply, original_tick = bridge._apply_command, bridge.tick
    bridge._apply_command = lambda payload: adapter.apply(payload, original_apply)

    def tick(dt):
        # Observe completion before draining new requests and preserve legacy tick.
        adapter.tick()
        original_tick(dt)

    bridge.tick = tick
    bridge._world_model_adapter = adapter
