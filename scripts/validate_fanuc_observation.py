"""Run the read-only FANUC observation graph and optionally save its summary.

Example: python scripts/validate_fanuc_observation.py --endpoint ws://127.0.0.1:9910/omniverse
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

PROJECT_ROOT = Path(__file__).resolve().parents[1]
if str(PROJECT_ROOT) not in sys.path:
    sys.path.insert(0, str(PROJECT_ROOT))

from backend.runtime.runner import run_graph
from backend.schemas import GraphRunRequest
from backend.executors.world_model_nodes import exec_fanuc_snapshot_read, exec_fanuc_usd_geometry_read


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--endpoint", default="ws://127.0.0.1:9910/omniverse")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--snapshot", action="store_true", help="Also validate capture_world_state (adapter required)")
    parser.add_argument("--geometry", action="store_true", help="Also read USD geometry and validate arm FK/proxy generation (adapter required)")
    args = parser.parse_args()
    graph_path = PROJECT_ROOT / "examples/graphs/fanuc_m710_world_observe_graph.json"
    graph = json.loads(graph_path.read_text(encoding="utf-8"))
    GraphRunRequest.model_validate(graph)
    if any(node["type"] not in {"fanuc_state_read", "scene_scan", "print"} for node in graph["nodes"]):
        raise ValueError("Observation graph contains a node that is not read-only")
    for node in graph["nodes"]:
        if node["type"] != "print":
            node["data"]["omniverseEndpoint"] = args.endpoint
    result = run_graph(graph)
    context = result["context"]
    arm = context["fanuc-state"].get("state") or {}
    scan = context["fanuc-scan"].get("scan") or {}
    report = {
        "schema": "vlagraph.fanuc_observation_validation.v1",
        "endpoint": args.endpoint,
        "ok": result["error_count"] == 0,
        "commands_sent": ["get_state", "scan_scene"],
        "motion_requested": False,
        "arm": {key: arm.get(key) for key in (
            "request_id", "observed_at", "capture_duration_sec", "prim_path", "physics_ready",
            "motion_active", "joints", "external_axes", "sim_time", "missing_fields",
            "prediction_ready", "completion_verified",
        )},
        "scene": {key: scan.get(key) for key in (
            "scene_id", "timestamp", "object_count", "meters_per_unit", "up_axis", "observation_quality",
        )},
        "errors": {key: context[key].get("message") for key in ("fanuc-state", "fanuc-scan")
                   if context[key].get("ok") is False},
        "limitations": [
            "get_state and scan_scene are separate observations, not a synchronized physics snapshot",
            "motion_active=false does not verify per-command completion or task success",
            "missing stage units and axis must be supplied by the bridge before world-state conversion",
        ],
    }
    if args.snapshot:
        captured = exec_fanuc_snapshot_read({"id": "snapshot", "data": {"omniverseEndpoint": args.endpoint}}, {}, [])
        report["snapshot_validation"] = captured
        report["commands_sent"].append("capture_world_state")
        report["ok"] = report["ok"] and captured["ok"]
    if args.geometry:
        geometry = exec_fanuc_usd_geometry_read({"id": "geometry", "data": {"omniverseEndpoint": args.endpoint}}, {}, [])
        report["geometry_validation"] = geometry
        report["commands_sent"].append("get_collision_geometry")
        report["ok"] = report["ok"] and geometry["ok"]
        if geometry["ok"]:
            from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
            try:
                report["usd_proxy_validation"] = {"ok": True, **build_fanuc_usd_proxy(geometry["inventory"])}
            except (ValueError, TypeError, KeyError) as exc:
                report["usd_proxy_validation"] = {"ok": False, "message": str(exc)}
                report["ok"] = False
        report["limitations"].append("USD geometry uses default-time transforms; it is not a synchronized physics rollout")
    payload = json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    print(payload)
    return 0 if report["ok"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
