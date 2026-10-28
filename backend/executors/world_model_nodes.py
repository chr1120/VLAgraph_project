from __future__ import annotations

import contextlib
import json
import math
import time
import uuid

from backend.executors.spatial_nodes import _extract_scan, _incoming_values, _props
from backend.world_model.fanuc import FanucPhysicsSnapshot, normalize_fanuc_state
from backend.world_model.state_builder import build_world_state
from backend.world_model.fanuc_preflight import action_fingerprint, validate_fanuc_trajectory
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.usd_geometry import collision_inventory_digest, extract_collision_inventory
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
from backend.world_model.fanuc_geometry_review import build_geometry_review, apply_geometry_review
from backend.world_model.fanuc_segment_review import review_fanuc_segments
from backend.world_model.fanuc_world_context import reconcile_memory, prediction_handoff
from backend.world_model.robot_profiles import configured_data, validate_profile, M710_PROFILE


FANUC_M710_PRIM_PATH = (
    "/World/GraphicViewer/Device/LASD/Robot/HFS1/HFS1_R01_R03/"
    "HFS1_7Axis_LH/lnk3/FANUC_M710/Robot"
)


def exec_fanuc_trajectory_validate(node, context, edges):
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    try:
        if any(edge.get("target") == node["id"] and edge.get("targetHandle") == "snapshot" for edge in edges) and inputs.get("snapshot") is None:
            raise ValueError("Connected initial snapshot is missing")
        report = validate_fanuc_trajectory(inputs.get("action") or props.get("action"), config=props,
                                           snapshot=inputs.get("snapshot"))
        segments = review_fanuc_segments(inputs.get("action") or props.get("action"), config=props,
                                         snapshot=inputs.get("snapshot"))
        return {"ok": True, "report": report, "preview_action": report["preview_action"],
                "segment_report": segments,
                "static_checks_passed": report["static_checks_passed"], "execution_ready": False}
    except (ValueError, TypeError, AttributeError) as exc:
        return {"ok": False, "report": None, "segment_report": None, "preview_action": None, "execution_ready": False, "message": str(exc)}


def exec_fanuc_collision_check(node, context, edges):
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    action = inputs.get("action")
    try:
        enabled = props.get("enabled", False)
        if not isinstance(enabled, bool):
            raise ValueError("Collision enabled must be an explicit boolean")
        if not enabled:
            return {"ok": True, "collision_report": {
                "schema": "vlagraph.fanuc_collision_report.v1", "status": "not_requested",
                "action_digest": action_fingerprint(action), "physical_collision_verified": False,
                "execution_ready": False, "message": "Configure explicit proxy geometry and enable collision checking."}}
        scene_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") == "scene" for edge in edges)
        scene = inputs.get("scene") if scene_connected else props.get("scene")
        if scene_connected and not isinstance(scene, dict):
            raise ValueError("Connected collision scene is missing or invalid")
        if any(edge.get("target") == node["id"] and edge.get("targetHandle") == "snapshot" for edge in edges) and inputs.get("snapshot") is None:
            raise ValueError("Connected initial snapshot is missing")
        inventory_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") == "inventory" for edge in edges)
        if inventory_connected:
            inventory = inputs.get("inventory")
            if not isinstance(inventory, dict) or inventory.get("schema") != "vlagraph.usd_collision_inventory.v1":
                raise ValueError("USD collision inventory is missing or invalid")
            if inventory.get("robot_joint_collision_coverage_complete") is not True:
                raise ValueError(f"USD robot collision coverage is incomplete: {inventory.get('missing_joint_colliders')}")
            if inventory.get("inventory_digest") != collision_inventory_digest(inventory):
                raise ValueError("USD inventory content does not match its inventory_digest")
            if not isinstance(scene, dict) or scene.get("inventory_digest") != inventory.get("inventory_digest"):
                raise ValueError("Proxy scene must identify the source USD inventory_digest")
        report = check_fanuc_collision(action, scene,
                                       max_step_sec=props.get("maxStepSec", 0.05),
                                       margin_m=props.get("marginM", 0.01), max_samples=props.get("maxSamples", 10000),
                                       snapshot=inputs.get("snapshot"))
        return {"ok": True, "collision_report": report}
    except (ValueError, TypeError, AttributeError, KeyError, OverflowError) as exc:
        return {"ok": False, "collision_report": None, "message": str(exc)}


def exec_fanuc_usd_proxy_build(node, context, edges):
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    try:
        props, robot_config = configured_data(node, context, edges, "read")
        result = build_fanuc_usd_proxy(inputs.get("inventory"), padding_m=props.get("paddingM", 0.0),
                                       excluded_self_pairs=props.get("excludedSelfPairs", []),
                                       profile=robot_config["profile"] if robot_config else None)
        plan = build_geometry_review(inputs["inventory"], result["scene"])
        review_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") == "review" for edge in edges)
        review = inputs.get("review") if review_connected else props.get("geometryReview")
        has_review = review is not None and not (isinstance(review, dict) and not review)
        if review_connected or has_review:
            result["scene"] = apply_geometry_review(inputs["inventory"], result["scene"], review)
        return {"ok": True, **result, "coverage_report": result["scene"]["geometry_coverage"], "execution_ready": False,
                "review_plan": plan,
                "message": "USD/FK binding verified; robot proxy and coverage gaps generated from authored collision bounds."}
    except (ValueError, TypeError, KeyError, OverflowError) as exc:
        return {"ok": False, "scene": None, "binding_report": None, "coverage_report": None, "review_plan": None,
                "execution_ready": False, "message": str(exc)}


def exec_fanuc_usd_geometry_read(node, context, edges):
    props = _props(node)
    try:
        props, _ = configured_data(node, context, edges, "read")
        source = props.get("source", "bridge")
        prim_path = props.get("robotPrimPath") or FANUC_M710_PRIM_PATH
        if source == "bridge":
            response = _fanuc_observation_request(props, "get_collision_geometry", prim_path=prim_path,
                                                   root_path=props.get("rootPath", "/World"),
                                                   max_colliders=props.get("maxColliders", 4096))
            inventory = response.get("inventory")
        elif source == "file":
            from pxr import Usd
            if not isinstance(props.get("usdPath"), str) or not props["usdPath"].strip():
                raise ValueError("usdPath is required for file source")
            stage = Usd.Stage.Open(props["usdPath"])
            inventory = extract_collision_inventory(stage, robot_prim_path=prim_path,
                                                      root_path=props.get("rootPath", "/World"),
                                                      max_colliders=props.get("maxColliders", 4096))
        else:
            raise ValueError("source must be bridge or file")
        if (not isinstance(inventory, dict) or inventory.get("schema") != "vlagraph.usd_collision_inventory.v1"
                or inventory.get("robot_prim_path") != prim_path):
            raise ValueError("Invalid collision inventory or selected robot mismatch")
        if inventory.get("inventory_digest") != collision_inventory_digest(inventory):
            raise ValueError("USD inventory digest mismatch")
        return {"ok": True, "inventory": inventory, "coverage_complete": inventory["robot_joint_collision_coverage_complete"],
                "message": "USD collision inventory read; joint coverage does not authorize execution."}
    except Exception as exc:
        return {"ok": False, "inventory": None, "message": str(exc)}


def exec_fanuc_world_context(node, context, edges):
    from backend.executors.spatial_nodes import _memory_path, _read_records
    props, inputs = _props(node), _incoming_values(node["id"], edges, context)
    try:
        props, robot_config = configured_data(node, context, edges, "world")
        snapshot, scan, inventory = inputs.get("snapshot"), inputs.get("scan"), inputs.get("inventory")
        if not isinstance(snapshot, dict):
            raise ValueError("Current FANUC physics snapshot is unavailable; activate the bridge adapter first")
        observed = FanucPhysicsSnapshot.model_validate(snapshot)
        path = _memory_path(props.get("memoryPath"))
        if not path.is_file() or path.stat().st_size > 32 * 1024 * 1024:
            raise ValueError("Collection memory file is missing or exceeds 32 MiB")
        records = [r for r in _read_records(path) if isinstance(r.get("scan"), dict)
                   and (r["scan"].get("robot") or {}).get("prim_path") == observed.prim_path]
        if not records:
            raise ValueError("No Collection record matches this FANUC robot; collect its scene first")
        def stamp(record):
            value = record["scan"].get("timestamp")
            return value if isinstance(value, (float, int)) and not isinstance(value, bool) and math.isfinite(value) else -1
        record = max(records, key=stamp)
        config = {"max_memory_age_sec": props.get("maxMemoryAgeSec", 86400),
                  "max_live_age_sec": props.get("maxLiveAgeSec", 5),
                  "position_tolerance_m": props.get("positionToleranceM", .02)}
        report = reconcile_memory(record, scan, snapshot, inventory, now=time.time(), **config)
        profile = robot_config["profile"] if robot_config else None
        scene = (build_fanuc_usd_proxy(inventory, profile=profile) if profile is not None else build_fanuc_usd_proxy(inventory))["scene"] if report["status"] == "geometry_context_consistent" else None
        report.update(scene_digest=action_fingerprint(scene) if scene else None,
                      evidence={"record": record, "scan": scan, "inventory": inventory})
        if robot_config:
            report["robot_config_digest"] = robot_config["digest"]
        return {"ok": True, "context_report": report, "scene": scene, "execution_ready": False}
    except (ValueError, TypeError, AttributeError, KeyError, OSError) as exc:
        return {"ok": False, "context_report": {"schema": "vlagraph.fanuc_world_context.v1", "status": "unavailable",
                "reasons": [{"code": "world_context_input_unavailable", "message": str(exc)}], "execution_ready": False},
                "scene": None, "execution_ready": False, "message": str(exc)}


def exec_fanuc_world_prediction(node, context, edges):
    inputs = _incoming_values(node["id"], edges, context)
    try:
        report = prediction_handoff(inputs.get("action"), inputs.get("context_report"), inputs.get("collision_report"),
                                    inputs.get("snapshot"), inputs.get("scene"))
        return {"ok": True, "prediction_report": report, "execution_ready": False}
    except (ValueError, TypeError, AttributeError, KeyError) as exc:
        return {"ok": False, "prediction_report": {"schema": "vlagraph.fanuc_world_prediction.v1", "status": "blocked_inputs",
                "physical_rollout_verified": False, "execution_ready": False, "message": str(exc)},
                "execution_ready": False, "message": str(exc)}


def exec_execution_guard(node, context, edges):
    """Bind the prepared action to its report and enforce the selected scope."""
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    action, report = inputs.get("action"), inputs.get("report")
    policy = props.get("validationPolicy", "full")
    reasons = []
    verified_report = None
    collision_summary = None
    snapshot_age = None
    robot_config = None
    try:
        props, robot_config = configured_data(node, context, edges, "guard")
        if policy not in {"full", "static_simulation"}:
            raise ValueError("Unknown validationPolicy")
        if not isinstance(action, dict) or not isinstance(report, dict):
            raise ValueError("Prepared action and validation report are required")
        profile = validate_profile(action.get("robot_profile"))
        if profile != M710_PROFILE:
            reasons.append({"code": "robot_profile_execution_unverified", "profile_id": profile["id"]})
        if action.get("robot_config_digest") is not None or robot_config:
            if (robot_config is None or action.get("robot_config_digest") != robot_config["digest"]
                    or action.get("robot_profile") != robot_config["profile"]
                    or action.get("robot_instance_id") != robot_config["instance_id"]
                    or action.get("robot_prim_path") != robot_config["connection"]["robot_prim_path"]):
                raise ValueError("Prepared action and selected robot configuration differ")
        require_snapshot = props.get("requireInitialSnapshot", False)
        if not isinstance(require_snapshot, bool):
            raise ValueError("requireInitialSnapshot must be a boolean")
        snapshot_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") == "snapshot" for edge in edges)
        if (require_snapshot or snapshot_connected) and inputs.get("snapshot") is None:
            raise ValueError("Initial snapshot is required or connected but missing")
        if report.get("schema") != "vlagraph.fanuc_preflight.v1" or report.get("action_digest") != action_fingerprint(action):
            raise ValueError("Validation report does not match the prepared action")
        verified_report = validate_fanuc_trajectory(action, config=report.get("validation_config", {}), snapshot=inputs.get("snapshot"))
        if verified_report["snapshot_ref"] != report.get("snapshot_ref"):
            raise ValueError("Guard requires the same initial snapshot used by preflight")
        if inputs.get("snapshot") is not None:
            observed = FanucPhysicsSnapshot.model_validate(inputs["snapshot"])
            if robot_config and observed.prim_path != robot_config["connection"]["robot_prim_path"]:
                raise ValueError("Snapshot belongs to a different configured robot instance")
            maximum_age = props.get("maxSnapshotAgeSec", 5)
            if (isinstance(maximum_age, bool) or not isinstance(maximum_age, (float, int))
                    or not math.isfinite(maximum_age) or not 0.1 <= maximum_age <= 60):
                raise ValueError("maxSnapshotAgeSec must be 0.1..60 seconds")
            snapshot_age = time.time() - observed.observed_at
            if not -0.5 <= snapshot_age <= maximum_age:
                raise ValueError("Initial snapshot is stale or its clock is ahead of the backend")
        if not isinstance(props.get("requireWorldContext", False), bool):
            raise ValueError("requireWorldContext must be a boolean")
        world_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") in {"context_report", "prediction_report"} for edge in edges)
        if props.get("requireWorldContext", False) is not False or world_connected:
            world = inputs.get("context_report")
            if not isinstance(world, dict) or world.get("schema") != "vlagraph.fanuc_world_context.v1":
                raise ValueError("Collection reconciliation report is required")
            evidence = world.get("evidence") or {}
            checked = reconcile_memory(evidence.get("record"), evidence.get("scan"), inputs.get("snapshot"),
                                       evidence.get("inventory"), now=world.get("evaluated_at"), **world.get("config", {}))
            if any(world.get(key) != value for key, value in checked.items()):
                raise ValueError("World context report was altered or its evidence differs")
            if not -.5 <= time.time() - checked["evaluated_at"] <= checked["config"]["max_live_age_sec"]:
                raise ValueError("World context reconciliation is stale")
            if checked["status"] != "geometry_context_consistent":
                reasons.append({"code": "world_context_unresolved", "details": checked["reasons"]})
            else:
                expected_scene = (build_fanuc_usd_proxy(evidence["inventory"], profile=profile) if robot_config else build_fanuc_usd_proxy(evidence["inventory"]))["scene"]
                if world.get("scene_digest") != action_fingerprint(expected_scene):
                    raise ValueError("World context proxy scene differs from its inventory")
            if robot_config and world.get("robot_config_digest") != robot_config["digest"]:
                raise ValueError("World context and selected robot configuration differ")
            prediction = inputs.get("prediction_report")
            collision = inputs.get("collision_report")
            if (not isinstance(prediction, dict) or prediction.get("schema") != "vlagraph.fanuc_world_prediction.v1"
                    or prediction.get("action_digest") != action_fingerprint(action)
                    or prediction.get("snapshot_ref") != verified_report["snapshot_ref"]
                    or prediction.get("context_digest") != action_fingerprint(world)
                    or not isinstance(collision, dict) or prediction.get("collision_digest") != action_fingerprint(collision)
                    or prediction.get("scene_digest") != world.get("scene_digest")):
                reasons.append({"code": "world_prediction_missing_or_mismatched"})
            # No trusted physical worker exists yet; a claimed boolean cannot grant execution.
            reasons.append({"code": "physical_prediction_worker_unavailable"})
        if not verified_report["static_checks_passed"]:
            reasons.append({"code": "static_validation_failed", "errors": verified_report["errors"]})
        if report.get("static_checks_passed") is not True or report.get("errors"):
            reasons.append({"code": "preflight_report_failed"})
        source_audit = action.get("source_instruction_audit")
        if source_audit is not None:
            if (not isinstance(source_audit, dict) or source_audit.get("schema") != "vlagraph.fanuc_ls_instruction_audit.v1"
                    or not isinstance(source_audit.get("unmodeled"), list) or not isinstance(source_audit.get("issues"), list)):
                raise ValueError("Invalid source LS instruction audit")
            if source_audit["unmodeled"] or source_audit["issues"]:
                reasons.append({"code": "source_program_semantics_unverified", "unmodeled_instruction_count": len(source_audit["unmodeled"]),
                                "audit_issues": source_audit["issues"]})
        if action.get("preview_only") is not False or action.get("execution_enabled") is not True or action.get("execution_ready") is not True:
            reasons.append({"code": "prepare_did_not_enable_execution"})
        if policy == "full":
            reasons.append({"code": "full_validation_unavailable", "unchecked": verified_report["unchecked"]})
        collision_connected = any(edge.get("target") == node["id"] and edge.get("targetHandle") == "collision_report" for edge in edges)
        if collision_connected:
            collision = inputs.get("collision_report")
            if (not isinstance(collision, dict) or collision.get("schema") != "vlagraph.fanuc_collision_report.v1"
                    or collision.get("action_digest") != action_fingerprint(action)):
                raise ValueError("Collision report is missing or does not match the action")
            collision_summary = {key: collision.get(key) for key in (
                "status", "coverage", "scene_id", "scene_version", "scene_digest", "physical_collision_verified",
                "snapshot_ref", "initial_segment_checked")}
            if collision.get("status") == "clear":
                if collision.get("snapshot_ref") != verified_report["snapshot_ref"]:
                    raise ValueError("Collision and preflight reports use different initial snapshots")
                if verified_report["snapshot_ref"] is not None and collision.get("initial_segment_checked") is not True:
                    raise ValueError("Collision report did not check the initial segment")
            if collision.get("status") not in {"not_requested", "clear"}:
                reasons.append({"code": "collision_or_unresolved_geometry", "status": collision.get("status")})
            elif collision.get("status") == "clear" and collision.get("between_samples_certified") is not True:
                reasons.append({"code": "uncertified_collision_intervals"})
    except (ValueError, TypeError, AttributeError) as exc:
        reasons.append({"code": "invalid_guard_input", "message": str(exc)})
    approved = not reasons
    if approved:
        import copy
        guarded = copy.deepcopy(action)
        digest = action_fingerprint(guarded)
    else:
        guarded = {"type": "execution_blocked", "reasons": reasons}
        digest = None
    return {"ok": True, "type": "execution_guard_result", "action": guarded,
            "approved": approved, "action_digest": digest,
            "decision": {"approved": approved, "validation_policy": policy, "reasons": reasons,
                         "robot_config_digest": robot_config["digest"] if robot_config else None,
                         "collision_checked": False, "physical_rollout_checked": False,
                         "collision_geometry": collision_summary,
                         "initial_snapshot_age_sec": snapshot_age,
                         "unchecked": verified_report["unchecked"] if verified_report else []}}


def _fanuc_observation_request(props, command, **fields):
    from websocket import create_connection

    request_id = f"fanuc-world-{uuid.uuid4().hex}"
    request = {"type": "omniverse_command", "request_id": request_id, "command": command, **fields}
    timeout = float(props.get("timeoutSec", 5))
    if not 0 < timeout <= 60:
        raise ValueError("timeoutSec must be between 0 and 60")
    ws = create_connection(props.get("omniverseEndpoint") or "ws://host.docker.internal:9910/omniverse", timeout=timeout)
    try:
        ws.send(json.dumps(request))
        response = json.loads(ws.recv())
        if (not isinstance(response, dict) or response.get("request_id") != request_id
                or response.get("command") != command or response.get("ok") is not True):
            raise ValueError(f"FANUC {command} rejected or response identity mismatch: {response}")
        return response
    finally:
        with contextlib.suppress(Exception):
            ws.close()


def exec_fanuc_snapshot_read(node, context, edges):
    props = _props(node)
    try:
        props, _ = configured_data(node, context, edges, "read")
        response = _fanuc_observation_request(props, "capture_world_state")
        snapshot = FanucPhysicsSnapshot.model_validate(response.get("snapshot"))
        if snapshot.prim_path != (props.get("robotPrimPath") or FANUC_M710_PRIM_PATH):
            raise ValueError("Snapshot prim_path does not match selected robot")
        return {"ok": True, "snapshot": snapshot.model_dump(mode="json", by_alias=True), "prediction_ready": False}
    except Exception as exc:
        return {"ok": False, "snapshot": None, "message": str(exc)}


def exec_execution_observe(node, context, edges, stop_requested=None):
    """Observe or await verified completion of the exact submitted command."""
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    command_id = inputs.get("command_id") or props.get("commandId")
    wait = False
    last_execution = None
    can_stop = False
    try:
        props, _ = configured_data(node, context, edges, "read")
        wait = props.get("waitForCompletion", False)
        if not isinstance(wait, bool):
            raise ValueError("waitForCompletion must be an explicit boolean")
        result = inputs.get("result")
        if isinstance(result, dict):
            if result.get("ok") is not True or result.get("skipped") is True or result.get("dry_run") is True or result.get("preview_only") is True:
                return {"ok": True, "execution": None, "completed": False, "terminal": True,
                        "status": "not_started", "message": "실행 요청이 전송되지 않았습니다. 검증·실행 결과를 확인하세요."}
            request = result.get("request") or {}
            if (request.get("command") != "apply_fanuc_trajectory" or
                    request.get("prim_path") != (props.get("robotPrimPath") or FANUC_M710_PRIM_PATH)):
                raise ValueError("Execution result targets a different robot or command")
            submitted = request.get("command_id")
            if command_id and command_id != submitted:
                raise ValueError("Submitted and observed command IDs differ")
            command_id = submitted
            if result.get("endpoint"):
                props["omniverseEndpoint"] = result["endpoint"]
        if not isinstance(command_id, str) or not command_id:
            raise ValueError("commandId is required")
        can_stop = True
        maximum, interval = props.get("completionTimeoutSec", 120), props.get("pollIntervalSec", .25)
        for value, low, high in ((maximum, 1, 600), (interval, .05, 5)):
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
                raise ValueError("Invalid completion timeout or poll interval")
        deadline = time.monotonic() + maximum
        while True:
            if wait and stop_requested and stop_requested():
                stopped = _fanuc_observation_request(props, "stop_command", command_id=command_id,
                                                    prim_path=props.get("robotPrimPath") or FANUC_M710_PRIM_PATH)
                return {"ok": False, "execution": stopped.get("execution"), "completed": False,
                        "command_id": command_id, "message": "런타임 취소로 해당 FANUC 명령의 정지를 요청했습니다."}
            observed = _read_fanuc_execution(props, command_id)
            last_execution = observed["execution"]
            if not wait or observed["terminal"]:
                if wait and not observed["completed"]:
                    raise ValueError(f"FANUC command ended without completion: {last_execution['status']}")
                return observed
            if time.monotonic() >= deadline:
                raise ValueError("FANUC completion timeout")
            time.sleep(min(interval, max(0, deadline-time.monotonic())))
    except Exception as exc:
        stop_error = None
        if wait and command_id and can_stop:
            try:
                _fanuc_observation_request(props, "stop_command", command_id=command_id,
                                          prim_path=props.get("robotPrimPath") or FANUC_M710_PRIM_PATH)
            except Exception as stop_exc:
                stop_error = str(stop_exc)
        return {"ok": False, "execution": last_execution, "completed": False, "command_id": command_id,
                "message": str(exc), "stop_error": stop_error}


def _read_fanuc_execution(props, command_id):
    response = _fanuc_observation_request(props, "get_command_status", command_id=command_id)
    execution = response.get("execution")
    if not isinstance(execution, dict) or execution.get("command_id") != command_id:
        raise ValueError("Execution command_id mismatch")
    if execution.get("prim_path") != (props.get("robotPrimPath") or FANUC_M710_PRIM_PATH):
        raise ValueError("Execution belongs to a different robot")
    status = execution.get("status")
    if status not in {"running", "completed", "failed", "cancelled"}:
        raise ValueError("Unknown execution status")
    if status == "completed":
        actual = FanucPhysicsSnapshot.model_validate(execution.get("actual_state"))
        if (execution.get("completion_verified") is not True or actual.prim_path != execution["prim_path"]
                or actual.episode_id != execution.get("episode_id")):
            raise ValueError("Completion lacks matching episode and actual state")
    return {"ok": True, "execution": execution, "completed": status == "completed", "command_id": command_id,
            "terminal": status != "running", "outcome": "unknown"}


def exec_fanuc_state_read(node, context, edges):
    """Read only: this node sends get_state and never init, hold or motion."""
    from websocket import create_connection

    props = _props(node)
    endpoint = props.get("omniverseEndpoint") or "ws://host.docker.internal:9910/omniverse"
    prim_path = props.get("robotPrimPath") or FANUC_M710_PRIM_PATH
    request_id = f"fanuc-observe-{uuid.uuid4().hex}"
    request = {"type": "omniverse_command", "command": "get_state", "request_id": request_id,
               "prim_path": prim_path}
    ws = None
    started = time.monotonic()
    try:
        timeout = float(props.get("timeoutSec", 5))
        if not 0 < timeout <= 60:
            raise ValueError("timeoutSec must be between 0 and 60")
        ws = create_connection(endpoint, timeout=timeout)
        ws.send(json.dumps(request))
        response = json.loads(ws.recv())
        if not isinstance(response, dict) or response.get("ok") is not True:
            raise ValueError("FANUC bridge rejected get_state")
        if response.get("request_id") != request_id or response.get("command") != "get_state":
            raise ValueError("FANUC response does not match the observation request")
        state = response.get("state")
        if not isinstance(state, dict):
            raise ValueError("FANUC bridge did not return a state object")
        observation = normalize_fanuc_state(
            state, expected_prim_path=prim_path, request_id=request_id,
            observed_at=time.time(), capture_duration_sec=time.monotonic() - started,
        )
        return {"ok": True, "type": "fanuc_state_read_result", "endpoint": endpoint,
                "state": observation.model_dump(mode="json"), "raw_state": state,
                "prediction_ready": False,
                "message": "FANUC joint state read; synchronized physics and command completion are pending."}
    except Exception as exc:
        return {"ok": False, "type": "fanuc_state_read_result", "state": None,
                "endpoint": endpoint, "message": str(exc)}
    finally:
        if ws is not None:
            with contextlib.suppress(Exception):
                ws.close()


def exec_world_state_build(node, context, edges):
    """Build a read-only snapshot; episode identity must be provided explicitly."""
    props = _props(node)
    inputs = _incoming_values(node["id"], edges, context)
    scan = _extract_scan(inputs, props)
    episode_id = inputs.get("episode_id") or props.get("episodeId")
    scene_version = inputs.get("scene_version") or props.get("sceneVersion")
    if not episode_id or not scene_version:
        return {"ok": False, "world_state": None, "message": "episodeId and sceneVersion are required."}
    cache_key = f"_world_state_build:{node['id']}"
    previous = context.get(cache_key, {})
    same_episode = (
        previous.get("episode_id") == episode_id
        and previous.get("scene_version") == scene_version
        and previous.get("scene_id") == scan.get("scene_id")
    )
    version = previous.get("state_version", 0) + 1 if same_episode else 1
    try:
        state = build_world_state(scan, episode_id=episode_id, scene_version=scene_version, state_version=version)
    except (ValueError, TypeError) as exc:
        return {"ok": False, "world_state": None, "message": str(exc)}
    context[cache_key] = {
        "episode_id": episode_id, "scene_version": scene_version,
        "scene_id": state.scene_id, "state_version": version,
    }
    return {
        "ok": True, "type": "world_state_build_result", "world_state": state.model_dump(mode="json"),
        "prediction_ready": False,
        "message": "Geometric observation normalized; physics snapshot and execution validation are pending.",
    }
