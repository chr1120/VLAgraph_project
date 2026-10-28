"""Graph routing from one shared scene to isolated robot inference jobs."""
from backend.executors.spatial_nodes import _incoming_values, _props
from backend.world_model.multi_robot import build_inference_batch, infer_batch, select_candidate, world_reference


def exec_multi_robot_context(node, context, edges):
    inputs, props = _incoming_values(node["id"], edges, context), _props(node)
    try:
        batch = build_inference_batch(inputs.get("world_state"),
            inputs.get("bindings", props.get("robotBindings")), inputs.get("tasks", props.get("robotTasks")),
            sensors=inputs.get("sensors"), navigation_maps=inputs.get("navigation_maps"))
        return {"ok": True, "batch": batch}
    except (ValueError, TypeError, KeyError) as exc:
        return {"ok": False, "batch": None, "message": str(exc)}


def exec_multi_robot_infer(node, context, edges):
    inputs, props = _incoming_values(node["id"], edges, context), _props(node)
    try:
        result = infer_batch(inputs.get("batch"), max_workers=props.get("maxWorkers", 4))
        return {"ok": result["ok"], "candidates": result, "prediction_ready": False}
    except (ValueError, TypeError, KeyError) as exc:
        return {"ok": False, "candidates": None, "message": str(exc)}


def exec_robot_candidate_select(node, context, edges):
    inputs, props = _incoming_values(node["id"], edges, context), _props(node)
    try:
        result = inputs.get("candidates")
        if props.get("requireBatchSuccess", True) is not False and (not isinstance(result, dict) or result.get("ok") is not True):
            raise ValueError("The shared inference batch contains a failed robot task")
        candidate, world = select_candidate(result, inputs.get("world_state"), props.get("robotPrimPath"))
        plan = candidate["action"] if candidate["action_schema"] == "vlagraph.amr_plan.v1" else None
        # Learned policy output needs a robot-specific decoder, preparation and
        # guard. Directly wiring it to Robot Execute remains a preview.
        action = ({**candidate["action"], "robot_prim_path": world.robot.prim_path,
                   "preview_only": True, "execution_ready": False}
                  if plan is None and isinstance(candidate["action"], dict) else None)
        return {"ok": True, "candidate": candidate, "action": action, "plan": plan,
                "world_state": world.model_dump(mode="json"), "robot": world.robot.model_dump(mode="json")}
    except (ValueError, TypeError, KeyError) as exc:
        return {"ok": False, "candidate": None, "action": None, "plan": None, "world_state": None, "message": str(exc)}


def exec_fanuc_policy_prepare(node, context, edges):
    """Adapt only explicitly typed joint trajectories; retain the existing Guard."""
    import copy
    import time
    from backend.world_model.schemas import WorldState
    from backend.world_model.robot_profiles import configured_data, M710_PROFILE
    from backend.world_model.fanuc_preflight import validate_fanuc_trajectory

    try:
        props, config = configured_data(node, context, edges, "prepare")
        inputs = _incoming_values(node["id"], edges, context)
        candidate = inputs.get("candidate")
        world = WorldState.model_validate(inputs.get("world_state"))
        schema = "vlagraph.fanuc_m710ic20l.action.v1"
        if (config is None or config["profile"] != M710_PROFILE or world.robot.robot_kind != "articulated_arm" or
                not isinstance(candidate, dict) or candidate.get("ok") is not True or
                candidate.get("action_schema") != schema or candidate.get("world_ref") != world_reference(world) or
                candidate.get("robot_prim_path") != world.robot.prim_path or candidate.get("entity_id") != world.robot.entity_id or
                config["connection"]["robot_prim_path"] != world.robot.prim_path):
            raise ValueError("FANUC policy candidate, world observation and robot configuration must match")
        if world.observed_at is None or not -.5 <= time.time()-world.observed_at <= 5:
            raise ValueError("FANUC policy observation is stale; capture and infer again")
        raw = candidate.get("action")
        if not isinstance(raw, dict) or raw.get("schema") != schema or raw.get("command") != "apply_fanuc_trajectory":
            raise ValueError("The policy must return an explicit FANUC joint trajectory with declared units")
        enabled = props.get("enableExecution") is True
        action = {"schema": schema, "command": "apply_fanuc_trajectory",
                  **{key: copy.deepcopy(raw.get(key)) for key in ("unit", "joint_names", "joint_trajectory")},
                  "robot_profile": copy.deepcopy(config["profile"]), "robot_config_digest": config["digest"],
                  "robot_instance_id": config["instance_id"], "robot_prim_path": world.robot.prim_path,
                  "preview_only": not enabled, "execution_enabled": enabled, "execution_ready": enabled}
        report = validate_fanuc_trajectory(action, config=props, snapshot=inputs.get("snapshot"))
        if not report["static_checks_passed"]:
            return {"ok": False, "action": None, "report": report, "message": "Policy trajectory failed FANUC preflight"}
        return {"ok": True, "action": action, "report": report, "requires_execution_guard": True,
                "physical_rollout_verified": False}
    except (ValueError, TypeError, KeyError) as exc:
        return {"ok": False, "action": None, "report": None, "message": str(exc)}
