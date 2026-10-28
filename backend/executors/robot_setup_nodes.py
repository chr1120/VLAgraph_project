from __future__ import annotations

from pathlib import Path

from backend.world_model.robot_profiles import (M710_ID, validate_profile,
                                               validate_robot_config, profile_from_inventory, validate_prepare_settings)
from backend.world_model.fanuc_preflight import action_fingerprint


def exec_robot_setup(node, context, edges):
    props = node.get("data", {})
    try:
        model, mode = props.get("robotModel", M710_ID), props.get("settingsMode", "preset")
        if model not in {M710_ID, "custom_fanuc_6axis"} or mode not in {"preset", "auto", "manual"}:
            raise ValueError("Select a supported robot family and settings mode")
        inventory = None
        if mode == "preset":
            if model != M710_ID:
                raise ValueError("Custom robots require auto or manual settings; no unverified model preset is provided")
            profile = validate_profile()
        elif mode == "manual":
            profile = validate_profile(props.get("manualProfile", {}))
        else:
            try:
                from pxr import Usd
            except ImportError as exc:
                raise ValueError("USD automatic setup requires backend/requirements-usd.txt; preset and manual modes do not") from exc
            from backend.world_model.usd_geometry import extract_collision_inventory
            from backend.world_model.fanuc_usd_proxy import validate_fanuc_usd_binding
            path = Path(str(props.get("usdPath", "")))
            if not path.is_file():
                raise ValueError("Automatic setup requires an existing USD file; TCP is not inferred")
            inventory = extract_collision_inventory(Usd.Stage.Open(str(path.resolve())),
                                                     robot_prim_path=props.get("robotPrimPath"), root_path="/World")
            if model == M710_ID:
                if not validate_fanuc_usd_binding(inventory)["kinematics_match"]:
                    raise ValueError("USD does not match the selected M-710 profile")
                profile = validate_profile()
            else:
                tool = props.get("toolOffsetM", [])
                if isinstance(tool, str):
                    import json
                    tool = json.loads(tool)
                profile = profile_from_inventory(inventory, profile_id=props.get("customProfileId", ""),
                                                 model=props.get("customModelName", ""), tool_offset_m=tool)
        if (model == M710_ID) != (profile["id"] == M710_ID):
            raise ValueError("Selected robot model and profile ID differ")
        settings = validate_prepare_settings(props.get("prepareSettings", {}))
        config = {"schema": "vlagraph.robot_config.v1", "instance_id": props.get("instanceId", "fanuc_01"),
                  "profile": profile, "mode": mode,
                  "connection": {"endpoint": props.get("omniverseEndpoint", ""), "robot_prim_path": props.get("robotPrimPath", "")},
                  "memory_path": props.get("memoryPath", "spatial_memory/spatial_memory.jsonl"),
                  "prepare": settings, "asset_inventory_digest": inventory["inventory_digest"] if inventory else None}
        config["digest"] = action_fingerprint(config)
        config = validate_robot_config(config)
        return {"ok": True, "robot_config": config, "profile": profile,
                "summary": {"model": profile["model"], "instance_id": config["instance_id"], "mode": mode,
                            "profile_digest": action_fingerprint(profile), "execution_verified": False,
                            "automatic_fields": ["joint origins", "axes", "limits", "collision binding"] if inventory else [],
                            "pending": ["TCP and workcell calibration", "controller compatibility", "physical execution validation"]}}
    except Exception as exc:
        return {"ok": False, "robot_config": None, "profile": None, "message": str(exc)}
