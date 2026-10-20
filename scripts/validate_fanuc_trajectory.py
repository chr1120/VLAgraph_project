"""Validate an LS program offline; this script never connects to a robot bridge."""
from __future__ import annotations

import argparse
import json
import sys
from collections import Counter
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.runtime.runner import run_graph
from backend.schemas import GraphRunRequest
from backend.world_model.fanuc_collision import check_fanuc_collision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ls_path", type=Path)
    parser.add_argument("--initial-settings", type=Path, help="Initial LS/TOOLDATA settings used by the original template")
    parser.add_argument("--settings", type=Path, help="JSON with prepare and validate node settings")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--segment-output", type=Path, help="Write original-line segment review without executable actions")
    parser.add_argument("--collision-scene", type=Path, help="Run nominal motion geometry diagnostics only after full static preflight passes")
    parser.add_argument("--geometry-output", type=Path, help="Write collision diagnostic wrapper or explicit blocked result")
    args = parser.parse_args()
    if args.geometry_output and not args.collision_scene:
        parser.error("--geometry-output requires --collision-scene")
    graph = json.loads((ROOT / "examples/graphs/fanuc_m710_preflight_graph.json").read_text(encoding="utf-8"))
    nodes = {node["id"]: node for node in graph["nodes"]}
    nodes["upload"]["data"]["datasetPath"] = str(args.ls_path.resolve())
    if args.initial_settings:
        nodes["parse"]["data"]["initialSettings"] = str(args.initial_settings.resolve())
    if args.settings:
        settings = json.loads(args.settings.read_text(encoding="utf-8"))
        nodes["prepare"]["data"].update(settings.get("prepare", {}))
        nodes["validate"]["data"].update(settings.get("validate", {}))
    nodes["prepare"]["data"].update(previewOnly=True, enableExecution=False, requireAllIk=True)
    if any(node["type"] not in {"dataset_upload", "fanuc_ls_parse", "fanuc_m710_prepare", "fanuc_trajectory_validate", "print"}
           for node in graph["nodes"]):
        raise ValueError("Preflight graph contains unsupported node")
    GraphRunRequest.model_validate(graph)
    result = run_graph(graph)
    report = result["context"]["validate"].get("report")
    segment_report = result["context"]["validate"].get("segment_report")
    geometry = {"schema": "vlagraph.fanuc_ls_geometry_diagnostic.v1", "status": "not_requested",
                "execution_requested": False, "network_commands_sent": [], "action": None, "report": None}
    if args.collision_scene:
        if not report or not report["static_checks_passed"] or result["error_count"]:
            geometry.update(status="blocked_preflight", errors=report["errors"] if report else [])
        elif (not segment_report or not isinstance(segment_report.get("source_instruction_audit"), dict)
              or segment_report.get("global_issues")
              or (segment_report.get("source_instruction_audit") or {}).get("unmodeled")):
            geometry.update(status="blocked_source_semantics", message="Source instructions require review; no partial program is exported.")
        else:
            try:
                raw_scene = json.loads(args.collision_scene.read_text(encoding="utf-8"))
                scene = raw_scene.get("scene", raw_scene)
                action = report["preview_action"]
                collision = check_fanuc_collision(action, scene)
                geometry.update(status="completed", action=action, report=collision)
            except (ValueError, TypeError, KeyError, OSError) as exc:
                geometry.update(status="invalid_geometry", message=str(exc))
    summary = {"schema": "vlagraph.fanuc_preflight_validation.v1", "source_path": str(args.ls_path.resolve()),
               "initial_settings_path": str(args.initial_settings.resolve()) if args.initial_settings else None,
               "network_commands_sent": [], "execution_requested": False,
               "graph_error_count": result["error_count"], "report": report,
               "prepare_summary": result["context"]["prepare"].get("summary"),
               "segment_report": segment_report, "geometry_diagnostic": geometry,
               "node_errors": {key: value for key, value in result["context"].items()
                               if isinstance(value, dict) and value.get("ok") is False}}
    payload = json.dumps(summary, ensure_ascii=False, indent=2, allow_nan=False)
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload + "\n", encoding="utf-8")
    for destination, value in ((args.segment_output, segment_report), (args.geometry_output, geometry)):
        if destination:
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: value for key, value in summary.items() if key not in {"report", "prepare_summary", "segment_report", "geometry_diagnostic"}}, ensure_ascii=False))
    if segment_report:
        print(json.dumps({"segment_status_counts": segment_report["status_counts"], "geometry_status": geometry["status"]}, ensure_ascii=False))
    if report:
        print(json.dumps({"static_checks_passed": report["static_checks_passed"], "execution_ready": False,
                          "frame_count": len(report["normalized_trajectory"]),
                          "errors_by_code": dict(Counter(item["code"] for item in report["errors"]))}, ensure_ascii=False))
    geometry_passed = not args.collision_scene or geometry["status"] == "completed" and geometry["report"]["status"] == "clear"
    return 0 if report and report["static_checks_passed"] and not result["error_count"] and geometry_passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
