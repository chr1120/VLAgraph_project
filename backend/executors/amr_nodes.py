"""Live observation, planning, and bounded simulation navigation graph nodes."""
from __future__ import annotations

import contextlib
import json
import time
import uuid

from backend.executors.spatial_nodes import _incoming_values, _props
from backend.world_model.amr_navigation import finite, plan_path, validate_plan
from backend.world_model.state_builder import build_world_state


def bridge_request(props, command, **fields):
    from websocket import create_connection, WebSocketException
    from backend.executors.vla_nodes import _omniverse_endpoint_candidates

    timeout = finite(props.get("timeoutSec", 5), "timeoutSec", .1, 30)
    request_id = f"world-model-{uuid.uuid4().hex}"
    request = {"type": "omniverse_command", "request_id": request_id, "command": command, **fields}
    errors = []
    for endpoint in _omniverse_endpoint_candidates(props.get("omniverseEndpoint") or "ws://host.docker.internal:9900/omniverse"):
        socket = None
        try:
            socket = create_connection(endpoint, timeout=timeout)
            socket.send(json.dumps(request, allow_nan=False))
            response = json.loads(socket.recv())
            if not isinstance(response, dict) or response.get("request_id") != request_id or response.get("command") != command:
                raise ValueError("Bridge response identity mismatch; reload the world-model bridge")
            if response.get("ok") is not True:
                # Rejections are final; never try another simulator after an explicit rejection.
                return response
            return response
        except (OSError, TimeoutError, WebSocketException) as exc:
            errors.append(f"{endpoint}: {exc}")
            # A navigation request may have reached the simulator. Do not resend
            # mutations to another endpoint after an ambiguous receive failure.
            if socket is not None and command in {"amr_navigate", "amr_stop"}:
                raise ValueError(f"Command delivery is uncertain; query amr_status using command_id: {exc}") from exc
        finally:
            if socket is not None:
                with contextlib.suppress(Exception):
                    socket.close()
    raise ValueError("World-model bridge unavailable: " + "; ".join(errors))


def exec_robot_observations_read(node, context, edges):
    props = _props(node)
    try:
        response = bridge_request(props, "capture_robot_observations", prim_path=props.get("robotPrimPath"))
        if response.get("ok") is not True:
            raise ValueError(response.get("message", "Observation rejected"))
        scan = response.get("scan")
        if not isinstance(scan, dict) or not scan.get("episode_id"):
            raise ValueError("Bridge did not provide an observation episode")
        cache = context.get(f"_robot_observation:{node['id']}", {})
        version = cache.get("version", 0)+1 if cache.get("episode") == scan["episode_id"] else 1
        world = build_world_state(scan, episode_id=scan["episode_id"],
                                  scene_version=props.get("sceneVersion") or "v1", state_version=version)
        context[f"_robot_observation:{node['id']}"] = {"version": version, "episode": scan["episode_id"]}
        return {"ok": True, "scan": scan, "world_state": world.model_dump(mode="json"),
                "navigation_map": scan.get("navigation_map"), "navigation_maps": scan.get("navigation_maps", {}),
                "prediction_ready": False}
    except (ValueError, TypeError, KeyError, OSError) as exc:
        return {"ok": False, "scan": None, "world_state": None, "navigation_map": None,
                "navigation_maps": None, "message": str(exc)}


def exec_amr_path_plan(node, context, edges):
    from backend.world_model.schemas import WorldState

    props, inputs = _props(node), _incoming_values(node["id"], edges, context)
    try:
        world = WorldState.model_validate(inputs.get("world_state"))
        robot = world.robot
        if robot.robot_kind != "amr" or robot.base_motion != "planar" or robot.mobile_base is None:
            raise ValueError("Select an AMR with an observed planar mobile base")
        if robot.mobile_base.drive_type != "differential" or robot.mobile_base.localization_valid is not True:
            raise ValueError("Differential drive and valid localization are required")
        if robot.mobile_base.navigation_frame != "stage":
            raise ValueError("AMR navigation requires Stage localization; map/odom transforms are not inferred")
        if world.observed_at is None or not -.5 <= time.time()-world.observed_at <= 5:
            raise ValueError("AMR observation is stale or has no timestamp; capture again")
        map_data = inputs.get("navigation_map")
        if not isinstance(map_data, dict) or map_data.get("stage_identifier") != world.raw_scan.get("stage_identifier"):
            raise ValueError("Navigation map and observation must belong to the same Stage")
        plan = plan_path(map_data, robot.position[:2], inputs.get("goal") or props.get("goalM"),
                         robot_prim_path=robot.prim_path, radius_m=robot.mobile_base.footprint_radius_m,
                         episode_id=world.episode_id, observed_at=world.observed_at,
                         margin_m=props.get("marginM", .1))
        return {"ok": True, "plan": plan, "simulation_only": True}
    except (ValueError, TypeError, KeyError) as exc:
        return {"ok": False, "plan": None, "message": str(exc)}


def exec_amr_navigate(node, context, edges):
    props, inputs = _props(node), _incoming_values(node["id"], edges, context)
    command_id = props.get("commandId") or f"amr-{uuid.uuid4().hex}"
    try:
        plan = validate_plan(inputs.get("plan"))
        if props.get("enableExecution") is not True:
            return {"ok": True, "started": False, "command_id": None,
                    "execution": None, "message": "AMR preview; enableExecution is required to drive the simulation."}
        response = bridge_request(props, "amr_navigate", plan=plan, command_id=command_id,
                                  timeout_sec=props.get("navigationTimeoutSec", 120))
        if response.get("ok") is not True:
            raise ValueError(response.get("message", "AMR navigation rejected"))
        execution = response.get("execution")
        if not isinstance(execution, dict) or execution.get("command_id") != command_id or execution.get("plan_digest") != plan["digest"]:
            raise ValueError("AMR execution does not match the submitted command and plan")
        return {"ok": True, "started": execution["status"] == "running", "command_id": command_id,
                "execution": execution, "completed": execution.get("completion_verified") is True}
    except (ValueError, TypeError, KeyError, OSError) as exc:
        return {"ok": False, "started": False, "command_id": command_id, "execution": None, "message": str(exc)}


def _execution_request(node, context, edges, command):
    props, inputs = _props(node), _incoming_values(node["id"], edges, context)
    command_id = inputs.get("command_id") or props.get("commandId")
    try:
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("commandId or a connected command_id is required")
        response = bridge_request(props, command, command_id=command_id)
        if response.get("ok") is not True:
            raise ValueError(response.get("message", "AMR status/stop rejected"))
        record = response.get("execution")
        if not isinstance(record, dict) or record.get("command_id") != command_id:
            raise ValueError("AMR command status identity mismatch")
        completed = record.get("status") == "completed" and record.get("completion_verified") is True
        return {"ok": True, "execution": record, "command_id": command_id, "completed": completed}
    except (ValueError, TypeError, KeyError, OSError) as exc:
        return {"ok": False, "execution": None, "command_id": command_id, "completed": False, "message": str(exc)}


def exec_amr_status(node, context, edges):
    return _execution_request(node, context, edges, "amr_status")


def exec_amr_stop(node, context, edges):
    return _execution_request(node, context, edges, "amr_stop")
