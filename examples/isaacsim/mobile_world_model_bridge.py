"""Live mixed-robot observations and simulation-only differential AMR navigation.

Attach before bridge.start(); all physics/USD calls run on the Kit update thread.
Launch with start_world_model_bridge.py so the repository is importable.
"""
from __future__ import annotations

import copy
import math
import time
import uuid

from backend.world_model.amr_navigation import (
    OccupancyGrid, differential_command, finite, fingerprint, point, validate_plan, wheel_speeds,
)


class MobileWorldModelAdapter:
    def __init__(self, bridge, config, *, clock=None):
        self.bridge, self.config = bridge, copy.deepcopy(config)
        robots = self.config.get("robots")
        if not isinstance(robots, list) or not robots or len(robots) > 64:
            raise ValueError("Configure 1..64 explicit robot descriptors")
        self.robots = {}
        for robot in robots:
            path = robot.get("prim_path")
            if not isinstance(path, str) or not path.startswith("/") or path in self.robots:
                raise ValueError("Configured robots require unique absolute prim paths")
            self.robots[path] = robot
        self.clock = clock or self._clock
        self.articulations, self.records = {}, {}
        self.active_commands = {}
        self.episode_id = uuid.uuid4().hex
        self.last_clock = None
        self.last_wall = None
        self.stage_identifier = None

    @property
    def active_id(self):
        """Legacy single-command view; control always uses active_commands."""
        return next(iter(self.active_commands.values()), None)

    @staticmethod
    def _clock():
        from isaacsim.core.simulation_manager import SimulationManager
        return (float(SimulationManager.get_simulation_time()),
                int(SimulationManager.get_num_physics_steps()))

    def _identity(self):
        from pxr import UsdGeom
        stage = self.bridge._get_stage()
        if stage is None:
            raise ValueError("USD stage is unavailable")
        return {"stage_identifier": str(stage.GetRootLayer().identifier),
                "meters_per_unit": finite(float(UsdGeom.GetStageMetersPerUnit(stage)), "stage units", 1e-9),
                "up_axis": str(UsdGeom.GetStageUpAxis(stage))}

    def _articulation(self, descriptor):
        path = descriptor.get("articulation_path")
        if not path:
            raise ValueError("Configure an exact articulation_path for physics observations")
        if path not in self.articulations:
            cls = self.bridge._resolve_omy3m_articulation_class()
            articulation = cls(prim_path=path, name="world_model_" + uuid.uuid4().hex)
            articulation.initialize()
            self.articulations[path] = articulation
        return self.articulations[path]

    def observe_robot(self, descriptor):
        from pxr import Usd, UsdGeom
        prim = self.bridge._get_prim(descriptor["prim_path"])
        matrix = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        q = matrix.ExtractRotationQuat()
        result = {k: copy.deepcopy(descriptor[k]) for k in
                  ("prim_path", "robot_kind", "manufacturer", "model", "base_motion", "mobile_base") if k in descriptor}
        result.update(position=[float(v) for v in matrix.ExtractTranslation()],
                      orientation_xyzw=[float(v) for v in q.GetImaginary()] + [float(q.GetReal())])
        try:
            articulation = self._articulation(descriptor)
            position, quat = articulation.get_world_pose()
            result.update(position=[float(v) for v in position],
                          orientation_xyzw=[float(v) for v in quat[1:]] + [float(quat[0])],
                          linear_velocity=[float(v) for v in articulation.get_linear_velocity()],
                          angular_velocity_rad_s=[float(v) for v in articulation.get_angular_velocity()])
            names = list(articulation.dof_names)
            positions, velocities = articulation.get_joint_positions(), articulation.get_joint_velocities()
            mappings = descriptor.get("joint_bindings", [])
            joints = []
            for binding in mappings:
                index = names.index(binding["dof_name"])
                kind = binding["kind"]
                if kind not in {"revolute", "prismatic"}:
                    raise ValueError("Joint kind must be revolute or prismatic")
                scale = self._identity()["meters_per_unit"] if kind == "prismatic" else 1.
                joints.append({"name": binding["name"], "kind": kind,
                               "unit": "m" if kind == "prismatic" else "rad",
                               "position": float(positions[index])*scale,
                               "velocity": float(velocities[index])*scale})
            if mappings:
                result["joints"] = joints
            values = result["position"] + result["orientation_xyzw"] + result["linear_velocity"] + result["angular_velocity_rad_s"]
            for value in values:
                finite(value, "physics observation")
            if descriptor.get("robot_kind") in {"amr", "mobile_manipulator"}:
                result.setdefault("mobile_base", {}).update(navigation_frame="stage", localization_valid=True)
        except Exception as exc:
            # Retain USD geometry, but never turn a failed physics read into zero velocity.
            for field in ("linear_velocity", "angular_velocity_rad_s", "joints"):
                result.pop(field, None)
            if result.get("mobile_base") is not None:
                result["mobile_base"]["localization_valid"] = False
            result["observation_error"] = str(exc)
        return result

    def navigation_map(self, selected):
        """Overlay current USD collider AABBs on explicit free/unknown coverage."""
        from pxr import UsdPhysics
        identity = self._identity()
        data = copy.deepcopy(self.config.get("navigation_map"))
        grid = OccupancyGrid(data)
        if data["stage_identifier"] != identity["stage_identifier"]:
            raise ValueError("Navigation map belongs to a different stage")
        if identity["up_axis"] not in {"Y", "Z"}:
            raise ValueError("Unknown stage up-axis")
        excluded = [selected] + self.config.get("ground_prim_paths", [])
        for prim in self.bridge._get_stage().Traverse():
            path = str(prim.GetPath())
            if any(path == p or path.startswith(p.rstrip("/")+"/") for p in excluded):
                continue
            if not prim.IsActive() or not prim.HasAPI(UsdPhysics.CollisionAPI):
                continue
            if UsdPhysics.CollisionAPI(prim).GetCollisionEnabledAttr().Get() is False:
                continue
            bounds = self.bridge._prim_world_bbox(prim)
            center, size = bounds.get("center"), bounds.get("size")
            if not isinstance(center, list) or not isinstance(size, list) or len(center) != 3 or len(size) != 3:
                raise ValueError(f"Collision bounds unavailable: {path}")
            for v in center + size:
                finite(v, "collision bound")
            scale = identity["meters_per_unit"]
            indices = (0, 2) if identity["up_axis"] == "Y" else (0, 1)
            signs = (1, -1) if identity["up_axis"] == "Y" else (1, 1)
            c = [center[i]*scale*s for i, s in zip(indices, signs)]
            extent = [size[i]*scale/2 for i in indices]
            lower = grid.cell((c[0]-extent[0], c[1]-extent[1]))
            upper = grid.cell((c[0]+extent[0], c[1]+extent[1]))
            for y in range(max(0, lower[1]), min(grid.height-1, upper[1])+1):
                for x in range(max(0, lower[0]), min(grid.width-1, upper[0])+1):
                    data["cells"][y][x] = 1
        return data

    def capture(self, selected):
        if selected not in self.robots:
            raise ValueError("Selected robot is not in the world-model configuration")
        before = self.clock()
        identity = self._identity()
        self._check_epoch(before, identity["stage_identifier"])
        robots = [self.observe_robot(descriptor) for descriptor in self.robots.values()]
        scan = self.bridge._scan_scene({"prim_path": selected, "max_objects": 4096})
        scan["objects"] = [obj for obj in scan["objects"] if not any(
            obj["prim_path"] == p or obj["prim_path"].startswith(p.rstrip("/")+"/") for p in self.robots)]
        scan.update(identity, robots=robots, robot=next(r for r in robots if r["prim_path"] == selected),
                    sim_time=before[0], physics_step=before[1], physics_synchronized=False,
                    episode_id=self.episode_id, object_count=len(scan["objects"]))
        if self.config.get("navigation_map") is not None:
            scan["navigation_maps"] = {path: self.navigation_map(path) for path, descriptor in self.robots.items()
                                       if descriptor.get("robot_kind") == "amr"}
            scan["navigation_map"] = scan["navigation_maps"].get(selected)
        if before != self.clock():
            raise ValueError("Physics advanced during observation capture; retry")
        return scan

    def _check_epoch(self, clock, stage):
        reset = (self.stage_identifier is not None and stage != self.stage_identifier or
                 self.last_clock is not None and (clock[0] < self.last_clock[0] or clock[1] < self.last_clock[1]))
        if reset:
            # Physics views belong to the old stage/epoch. Never make their
            # successful use a prerequisite for discarding them.
            self.articulations.clear()
            self.episode_id = uuid.uuid4().hex
            self.last_clock, self.stage_identifier = clock, stage
            try:
                self.finish("cancelled", "stage_or_simulation_reset")
            except Exception:
                pass  # New handles may not be ready; retry zero only in tick.
        # Read-only calls must not consume steps used for completion verification.
        if not self.active_commands:
            self.last_clock = clock
            self.stage_identifier = stage
        return reset

    def _drive(self, path, speed, turn):
        descriptor = self.robots[path]
        wheels = descriptor.get("wheel_config", {})
        radius = finite(wheels.get("radius_m"), "wheel radius", .001, 2)
        separation = finite(wheels.get("separation_m"), "wheel separation", .01, 10)
        names = wheels.get("dof_names")
        signs = wheels.get("signs", [1, 1])
        if not isinstance(names, list) or len(names) != 2 or len(set(names)) != 2 or signs not in ([1, 1], [1, -1], [-1, 1], [-1, -1]):
            raise ValueError("Configure distinct left/right wheel DOF names and signs")
        articulation = self._articulation(descriptor)
        indices = [list(articulation.dof_names).index(name) for name in names]
        left, right = wheel_speeds(speed, turn, radius, separation)
        import numpy as np
        from isaacsim.core.utils.types import ArticulationAction
        articulation.apply_action(ArticulationAction(
            joint_velocities=np.asarray([left*signs[0], right*signs[1]]),
            joint_indices=np.asarray(indices)))

    def _pose(self, path):
        robot = self.observe_robot(self.robots[path])
        if robot.get("observation_error") or "linear_velocity" not in robot or "angular_velocity_rad_s" not in robot:
            raise ValueError("AMR physics observation is unavailable")
        identity = self._identity()
        scale = identity["meters_per_unit"]
        x, y, z = [v*scale for v in robot["position"]]
        qx, qy, qz, qw = robot["orientation_xyzw"]
        if identity["up_axis"] == "Y":
            position = (x, -z)
            qx, qy, qz, qw = [(qw+qx)/math.sqrt(2), (qy-qz)/math.sqrt(2),
                              (qy+qz)/math.sqrt(2), (qw-qx)/math.sqrt(2)]
        elif identity["up_axis"] == "Z":
            position = (x, y)
        else:
            raise ValueError("Unknown stage up-axis")
        roll = math.atan2(2*(qw*qx+qy*qz), 1-2*(qx*qx+qy*qy))
        pitch = math.asin(max(-1., min(1., 2*(qw*qy-qz*qx))))
        if abs(roll) > .15 or abs(pitch) > .15:
            raise ValueError("AMR is not upright; planar drive requires local +X forward and +Z up")
        yaw = math.atan2(2*(qw*qz+qx*qy), 1-2*(qy*qy+qz*qz))
        speed = math.sqrt(sum((v*scale)**2 for v in robot["linear_velocity"]))
        turn = math.sqrt(sum(v*v for v in robot["angular_velocity_rad_s"]))
        return position, yaw, speed, turn

    def public_record(self, command_id):
        if command_id not in self.records:
            raise ValueError("Unknown AMR command_id")
        return copy.deepcopy({k: v for k, v in self.records[command_id].items() if not k.startswith("_")})

    def finish(self, status, reason, *, command_id=None):
        if command_id is None:
            errors = []
            for active in list(self.active_commands.values()):
                try:
                    self.finish(status, reason, command_id=active)
                except Exception as exc:
                    errors.append(exc)
            if errors:
                raise errors[-1]
            return
        record = self.records[command_id]
        if self.active_commands.get(record["robot_prim_path"]) != command_id:
            return
        # Keep the command active if zeroing fails so subsequent ticks retry it.
        try:
            self._drive(record["robot_prim_path"], 0., 0.)
        except Exception as exc:
            record.update(status="stopping", reason=reason, completion_verified=False,
                          _terminal_status="failed" if status == "completed" else status,
                          _terminal_reason=reason, stop_error=str(exc))
            raise
        record.update(status=status, reason=reason, completion_verified=status == "completed")
        record.pop("stop_error", None)
        self.active_commands.pop(record["robot_prim_path"], None)

    def navigate(self, payload):
        plan = validate_plan(payload.get("plan"))
        command_id = payload.get("command_id")
        if not isinstance(command_id, str) or not command_id or len(command_id) > 200:
            raise ValueError("An explicit command_id is required")
        if command_id in self.records:
            if self.records[command_id]["_digest"] != plan["digest"]:
                raise ValueError("command_id was already used for a different plan")
            return self.public_record(command_id)
        if len(self.records) >= 10000:
            raise ValueError("Command history limit reached; reload the adapter")
        path = plan["robot_prim_path"]
        self._check_epoch(self.clock(), self._identity()["stage_identifier"])
        if path in self.active_commands:
            raise ValueError("Another command is active for this AMR; stop it first")
        articulation_path = self.robots.get(path, {}).get("articulation_path")
        if articulation_path and any(self.robots[p].get("articulation_path") == articulation_path for p in self.active_commands):
            raise ValueError("Robot instances cannot share an active articulation")
        if plan.get("episode_id") != self.episode_id or not -.5 <= time.time()-finite(plan.get("observed_at"), "observed_at", 0) <= 5:
            raise ValueError("AMR plan belongs to a stale observation or another episode; capture and replan")
        descriptor = self.robots.get(path, {})
        if descriptor.get("robot_kind") != "amr" or descriptor.get("base_motion") != "planar" or descriptor.get("mobile_base", {}).get("drive_type") != "differential":
            raise ValueError("Navigation currently requires an explicitly configured differential AMR")
        minimum = finite(descriptor["mobile_base"].get("footprint_radius_m"), "footprint radius", .01, 10)
        if plan["radius_m"] < minimum:
            raise ValueError("Plan underestimates the configured robot footprint")
        from backend.world_model.amr_navigation import routes_conflict
        if any(routes_conflict(plan, self.records[active]["_plan"]) for active in self.active_commands.values()):
            raise ValueError("AMR route conflicts with another robot's reserved route; wait or replan")
        grid = OccupancyGrid(self.navigation_map(path))
        if grid.digest != plan["map_digest"] or grid.data["stage_identifier"] != plan["stage_identifier"]:
            raise ValueError("Navigation map changed; capture and replan")
        position, _, _, _ = self._pose(path)
        if math.dist(position, plan["start_m"]) > .05:
            raise ValueError("Robot moved since planning; capture and replan")
        points = [position] + plan["waypoints_m"][1:]
        if not all(grid.segment_clear(a, b, plan["radius_m"]+plan["margin_m"]) for a, b in zip(points, points[1:])):
            raise ValueError("Plan intersects occupied or unknown space")
        self._drive(path, 0., 0.)
        now = self.clock()
        self.last_clock, self.last_wall = now, time.monotonic()
        self.stage_identifier = self._identity()["stage_identifier"]
        timeout = finite(payload.get("timeout_sec", 120), "timeout_sec", 1, 600)
        self.records[command_id] = {"command_id": command_id, "robot_prim_path": path,
            "episode_id": self.episode_id, "plan_digest": plan["digest"], "simulation_only": True,
            "status": "running", "completion_verified": False, "_digest": plan["digest"],
            "_plan": copy.deepcopy(plan), "_index": 1, "_stable": 0,
            "_last_clock": now, "_last_wall": time.monotonic(),
            "_deadline": time.monotonic()+timeout, "_progress_at": time.monotonic(),
            "_best_distance": float("inf")}
        self.active_commands[path] = command_id
        return self.public_record(command_id)

    def tick(self):
        if not self.active_commands:
            return
        try:
            clock, wall = self.clock(), time.monotonic()
            if self._check_epoch(clock, self._identity()["stage_identifier"]):
                return
        except Exception as exc:
            try:
                self.finish("failed", str(exc))
            except Exception:
                pass
            return
        for command_id in list(self.active_commands.values()):
            self._tick_command(command_id, clock, wall)
        self.last_clock, self.last_wall = clock, wall

    def _tick_command(self, command_id, clock, wall):
        record = self.records[command_id]
        try:
            if record["status"] == "stopping":
                try:
                    self.finish(record["_terminal_status"], record["_terminal_reason"], command_id=command_id)
                except Exception:
                    pass  # Retry zero with current-epoch handles on the next update.
                return
            if wall >= record["_deadline"]:
                self.finish("failed", "navigation_timeout", command_id=command_id)
                return
            if clock == record["_last_clock"]:
                self._drive(record["robot_prim_path"], 0., 0.)
                if wall-record["_last_wall"] > 2:
                    self.finish("cancelled", "physics_clock_stalled", command_id=command_id)
                return
            dt = clock[0]-record["_last_clock"][0]
            record["_last_clock"], record["_last_wall"] = clock, wall
            if dt <= 0 or dt > .25:
                self.finish("cancelled", "physics_step_gap", command_id=command_id)
                return
            path, plan = record["robot_prim_path"], record["_plan"]
            position, yaw, measured_speed, measured_turn = self._pose(path)
            target = plan["waypoints_m"][record["_index"]]
            distance = math.dist(position, target)
            record.update(position_m=list(position), distance_to_goal_m=math.dist(position, plan["goal_m"]))
            if distance < .03 and record["_index"] < len(plan["waypoints_m"])-1:
                record["_index"] += 1
                record["_best_distance"] = float("inf")
                record["_progress_at"] = wall
                target = plan["waypoints_m"][record["_index"]]
                distance = math.dist(position, target)
            grid = OccupancyGrid(self.navigation_map(path))
            clearance = plan["radius_m"] + plan["margin_m"]
            if not grid.segment_clear(position, target, clearance):
                self.finish("blocked", "live_obstacle_or_unknown_space; capture and replan", command_id=command_id)
                return
            if record["_index"] == len(plan["waypoints_m"])-1 and distance < .03:
                self._drive(path, 0., 0.)
                record["_stable"] = record["_stable"]+1 if measured_speed < .02 and measured_turn < .03 else 0
                if record["_stable"] >= 3:
                    self.finish("completed", "goal_reached_and_settled", command_id=command_id)
                return
            record["_stable"] = 0
            if distance < record["_best_distance"]-.01:
                record["_best_distance"], record["_progress_at"] = distance, wall
            if wall-record["_progress_at"] > 15:
                self.finish("failed", "no_progress", command_id=command_id)
                return
            speed, turn = differential_command(position, yaw, target)
            self._drive(path, speed, turn)
        except Exception as exc:
            if record["status"] == "stopping":
                return
            record.update(reason=str(exc), completion_verified=False)
            try:
                self.finish("failed", str(exc), command_id=command_id)
            except Exception:
                pass  # Preserve the original failure and retry zero on the next tick.

    def apply(self, payload, original):
        command = payload.get("command")
        if command not in {"capture_robot_observations", "amr_navigate", "amr_status", "amr_stop"}:
            if self.active_commands and command not in {"scan_scene", "get_state", "status"}:
                target = payload.get("prim_path") or getattr(self.bridge, "robot_prim_path", None)
                if command in {"reset_scene", "initialize", "init"} and not payload.get("prim_path"):
                    self.finish("cancelled", "interrupted_by_scene_command")
                elif target in self.active_commands:
                    self.finish("cancelled", "interrupted_by_other_command", command_id=self.active_commands[target])
            return original(payload)
        response = {"type": "omniverse_ack", "command": command, "request_id": payload.get("request_id"),
                    "ok": True, "applied": False}
        try:
            if command == "capture_robot_observations":
                response["scan"] = self.capture(payload.get("prim_path") or next(iter(self.robots)))
            elif command == "amr_navigate":
                response["execution"] = self.navigate(payload)
                response["applied"] = response["execution"]["status"] == "running"
            elif command == "amr_status":
                response["execution"] = self.public_record(payload.get("command_id"))
            else:
                command_id = payload.get("command_id")
                record = self.public_record(command_id)
                if self.active_commands.get(record["robot_prim_path"]) == command_id:
                    self.finish("cancelled", "explicit_stop", command_id=command_id)
                response["execution"] = self.public_record(record["command_id"])
        except Exception as exc:
            response.update(ok=False, message=str(exc))
        return response


def attach_world_model(bridge, config):
    if hasattr(bridge, "_mobile_world_model"):
        raise ValueError("Mobile world-model adapter already attached")
    adapter = MobileWorldModelAdapter(bridge, config)
    original_apply, original_update, original_stop = bridge._apply_command, bridge._on_update, bridge.stop
    bridge._apply_command = lambda payload: adapter.apply(payload, original_apply)

    def update(event):
        try:
            original_update(event)
        finally:
            adapter.tick()

    def stop():
        adapter.finish("cancelled", "bridge_shutdown")
        original_stop()

    bridge._on_update, bridge.stop = update, stop
    bridge._mobile_world_model = adapter
    return adapter
