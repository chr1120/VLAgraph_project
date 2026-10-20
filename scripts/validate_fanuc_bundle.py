"""Collect FANUC calibration, USD, LS, Execute guard and optional live read checks.

Uses the existing Execute template in preview/dry-run mode. Never sends motion.
"""
from __future__ import annotations

import argparse
from collections import Counter
from datetime import datetime, timezone
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.executors.fanuc_nodes import _read_ls
from backend.runtime.runner import run_graph
from backend.schemas import GraphRunRequest
from backend.world_model.fanuc_alignment import fit_base_alignment
from backend.world_model.fanuc_calibration_review import review_calibration_candidate
from backend.world_model.fanuc_collision import check_fanuc_collision
from backend.world_model.fanuc_geometry_review import build_geometry_review
from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def review_execute(ls_path, *, world_context=False, endpoint=None, memory_path=None):
    graph = json.loads((ROOT / "examples/graphs/isaacsim_fanuc_m710ic20l_graph.json").read_text(encoding="utf-8"))
    # Live Collection integration is tested separately; this graph pass stays fully offline.
    if not world_context:
        offline_ids = {node["id"] for node in graph["nodes"] if not node["id"].startswith("fanuc-m710-world-")}
        graph["nodes"] = [node for node in graph["nodes"] if node["id"] in offline_ids]
        graph["edges"] = [edge for edge in graph["edges"] if edge["source"] in offline_ids and edge["target"] in offline_ids]
    allowed = {"dataset_upload", "fanuc_ls_parse", "fanuc_m710_prepare", "fanuc_trajectory_validate",
               "execution_guard", "robot_execute", "fanuc_collision_check", "print"}
    if world_context:
        if not endpoint or not memory_path:
            raise ValueError("World context diagnostic requires an explicit endpoint and memory path")
        allowed.update({"scene_scan", "fanuc_snapshot_read", "fanuc_usd_geometry_read", "fanuc_world_context", "fanuc_world_prediction"})
    if any(node["type"] not in allowed for node in graph["nodes"]):
        raise ValueError("Execute template contains a node unsupported by offline review")
    nodes = {node["id"]: node for node in graph["nodes"]}
    nodes["fanuc-m710-file"]["data"].update(datasetPath=str(ls_path.resolve()), initialSettingsEnabled=False,
                                             initialSettingsPath="")
    # Unreviewed controller transforms are never silently injected into Prepare.
    nodes["fanuc-m710-parse"]["data"].pop("initialSettings", None)
    nodes["fanuc-m710-prepare"]["data"].update(previewOnly=True, enableExecution=False, requireAllIk=True,
                                               uframeTransforms={}, utoolTransforms={})
    nodes["fanuc-m710-guard"]["data"]["validationPolicy"] = "full"
    nodes["fanuc-m710-guard"]["data"].update(requireWorldContext=world_context, requireInitialSnapshot=world_context)
    nodes["fanuc-m710-execute"]["data"].update(dryRun=True, requireExecutionGuard=True, mode="omniverse")
    nodes["fanuc-m710-collision"]["data"]["enabled"] = world_context
    if world_context:
        for key in ("snapshot", "scan", "geometry"):
            nodes[f"fanuc-m710-world-{key}"]["data"]["omniverseEndpoint"] = endpoint
        nodes["fanuc-m710-world-context"]["data"]["memoryPath"] = str(memory_path)
    GraphRunRequest.model_validate(graph)
    result = run_graph(graph)
    context = result["context"]
    report = context["fanuc-m710-validate"].get("report")
    guard = context["fanuc-m710-guard"]
    execute = context["fanuc-m710-execute"]
    checked = bool(isinstance(report, dict) and guard.get("approved") is False and execute.get("skipped") is True)
    return {"schema": "vlagraph.fanuc_execute_offline_review.v1",
            "boundary_status": "blocked_as_expected" if checked else "review_failed",
            "graph_error_count": result["error_count"],
            "report": report, "segment_report": context["fanuc-m710-validate"].get("segment_report"),
            "prepare_summary": context["fanuc-m710-prepare"].get("summary"),
            "guard": guard, "execute": execute,
            "network_commands_sent": ["capture_world_state", "scan_scene", "get_collision_geometry"] if world_context else [],
            "motion_requested": False,
            "calibration_candidates_applied": False,
            "live_collection_context_requested": world_context,
            "context_report": context.get("fanuc-m710-world-context", {}).get("context_report"),
            "prediction_report": context.get("fanuc-m710-world-prediction", {}).get("prediction_report"),
            "collision_report": context.get("fanuc-m710-collision", {}).get("collision_report"),
            "node_errors": {key: value for key, value in context.items()
                            if isinstance(value, dict) and (value.get("error") or value.get("ok") is False)}}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ls_path", type=Path)
    parser.add_argument("--candidate", action="append", type=Path, required=True)
    parser.add_argument("--usd", type=Path, required=True)
    parser.add_argument("--alignment", type=Path, help="Measured paired TCP poses bound to one reviewed candidate")
    parser.add_argument("--endpoint", help="Optional read-only bridge probe; omitted means fully offline")
    parser.add_argument("--world-context-memory", help="Also review the integrated Collection-to-Execute graph using this memoryPath; requires --endpoint")
    parser.add_argument("--output-dir", type=Path, required=True)
    args = parser.parse_args()
    if args.world_context_memory and not args.endpoint:
        parser.error("--world-context-memory requires --endpoint")
    out = args.output_dir.resolve()
    if out.exists() and (not out.is_dir() or any(out.iterdir())):
        parser.error("--output-dir must be a new or empty directory; existing evidence is never overwritten")
    out.mkdir(parents=True, exist_ok=True)
    phases = {}

    def phase(name, callback):
        print(f"Checking {name}...", flush=True)
        try:
            value = callback()
            write(out / f"{name}.json", value)
            phases[name] = {"status": "collected", "artifact": f"{name}.json"}
            return value
        except Exception as exc:
            phases[name] = {"status": "error", "message": str(exc)}
            write(out / f"{name}.json", phases[name])
            return None

    def calibration():
        source = _read_ls(args.ls_path)
        candidates = []
        for path in args.candidate:
            try:
                review = review_calibration_candidate(source, _read_ls(path))
                review["candidate_path"] = str(path.resolve())
            except Exception as exc:
                review = {"status": "error", "candidate_path": str(path.resolve()), "message": str(exc)}
            candidates.append(review)
        return {"candidates": candidates, "transforms_applied": False}

    calibration_result = phase("calibration", calibration)
    candidates = (calibration_result or {}).get("candidates", [])

    def alignment():
        eligible = [review for review in candidates if review["status"] == "requires_coordinate_alignment"]
        if not args.alignment:
            return {"status": "missing_reference_measurements", "calibration_verified": False}
        evidence = json.loads(args.alignment.read_text(encoding="utf-8"))
        matched = [review for review in eligible if review["candidate_sha256"] == evidence.get("candidate_sha256")]
        if not matched:
            raise ValueError("Alignment evidence does not match any applicable calibration candidate")
        return fit_base_alignment(evidence, source_sha256=matched[0]["source_sha256"],
                                  candidate_sha256=matched[0]["candidate_sha256"])

    alignment_result = phase("alignment", alignment)
    # A separate incomplete form is reviewable; no invented measurements are inserted.
    template = {"schema": "vlagraph.fanuc_alignment_references.v1", "units": "m", "source_sha256": None,
                "candidate_sha256": None, "references": [{"controller_tcp_in_base": None, "ik_tcp_in_base": None} for _ in range(3)]}
    eligible = [review for review in candidates if review["status"] == "requires_coordinate_alignment"]
    if len(eligible) == 1:
        template.update({key: eligible[0][key] for key in ("source_sha256", "candidate_sha256")})
    write(out / "alignment-input-template.json", template)

    def geometry():
        from pxr import Usd
        from backend.executors.world_model_nodes import FANUC_M710_PRIM_PATH
        from backend.world_model.usd_geometry import extract_collision_inventory
        stage = Usd.Stage.Open(str(args.usd.resolve()))
        inventory = extract_collision_inventory(stage, robot_prim_path=FANUC_M710_PRIM_PATH, root_path="/World")
        write(out / "inventory.json", inventory)
        proxy = build_fanuc_usd_proxy(inventory)
        write(out / "proxy.json", proxy)
        review = build_geometry_review(inventory, proxy["scene"])
        write(out / "geometry-review.json", review)
        return {"collider_count": len(inventory["colliders"]),
                "missing_joint_colliders": inventory["missing_joint_colliders"],
                "proxy": proxy, "execution_ready": False}

    geometry_result = phase("geometry", geometry)
    execute_result = phase("execute", lambda: review_execute(args.ls_path))

    def collision():
        report = (execute_result or {}).get("report") or {}
        segments = (execute_result or {}).get("segment_report") or {}
        audit = segments.get("source_instruction_audit") or {}
        if not report.get("static_checks_passed"):
            return {"status": "blocked_preflight", "action": None, "report": None}
        if not audit or audit.get("unmodeled") or audit.get("issues") or segments.get("global_issues"):
            return {"status": "blocked_source_semantics", "action": None, "report": None}
        if not geometry_result:
            return {"status": "blocked_missing_geometry", "action": None, "report": None}
        action = report["preview_action"]
        return {"status": "completed", "action": action,
                "report": check_fanuc_collision(action, geometry_result["proxy"]["scene"]), "execution_ready": False}

    collision_result = phase("collision", collision)

    def live():
        if not args.endpoint:
            return {"status": "not_requested", "motion_requested": False}
        # Only this established read-only script is launched; no arbitrary commands or motion nodes.
        command = [sys.executable, str(ROOT / "scripts/validate_fanuc_observation.py"), "--endpoint", args.endpoint,
                   "--snapshot", "--geometry", "--output", str(out / "live-observation.json")]
        observation_path = out / "live-observation.json"
        if observation_path.exists():
            raise ValueError("Use a fresh output directory for live evidence; an observation artifact already exists")
        process = subprocess.run(command, capture_output=True, text=True, encoding="utf-8", errors="replace", timeout=120)
        observation = json.loads(observation_path.read_text(encoding="utf-8")) if observation_path.exists() else None
        return {"status": "passed" if process.returncode == 0 and observation and observation.get("ok") else "blocked",
                "returncode": process.returncode, "observation": observation,
                "stderr": process.stderr[-4000:], "motion_requested": False}

    live_result = phase("live", live)
    world_result = None
    if args.world_context_memory:
        world_result = phase("world-context", lambda: review_execute(args.ls_path, world_context=True, endpoint=args.endpoint,
                                                                     memory_path=args.world_context_memory))
    report = (execute_result or {}).get("report") or {}
    audit = ((execute_result or {}).get("segment_report") or {}).get("source_instruction_audit") or {}
    blockers = ["controller_ik_base_tcp_external_axis_calibration_unverified",
                "tool_and_environment_collision_coverage_unverified", "physical_rollout_and_command_completion_unverified"]
    if not eligible:
        blockers.append("no_applicable_calibration_candidate")
    if (alignment_result or {}).get("status") != "reference_residuals_passed":
        blockers.append("base_alignment_reference_measurements_missing_or_failed")
    if not report.get("static_checks_passed"):
        blockers.append("source_ls_preflight_failed")
    if not audit or audit.get("unmodeled") or audit.get("issues"):
        blockers.append("source_ls_semantics_unverified")
    collision_report = (collision_result or {}).get("report") or {}
    if collision_report.get("status") != "clear":
        blockers.append("source_ls_collision_not_clear")
    if (live_result or {}).get("status") != "passed":
        blockers.append("synchronized_live_snapshot_and_geometry_unavailable")
    if args.world_context_memory:
        blockers.append("integrated_world_physical_prediction_unavailable")
        if ((world_result or {}).get("context_report") or {}).get("status") != "geometry_context_consistent":
            blockers.append("collection_current_scene_reconciliation_unavailable_or_failed")
    blockers.extend(f"phase_error:{name}" for name, result in phases.items() if result["status"] == "error")
    summary = {"schema": "vlagraph.fanuc_validation_bundle.v1", "created_at": datetime.now(timezone.utc).isoformat(),
               "status": "blocked", "source_path": str(args.ls_path.resolve()), "usd_path": str(args.usd.resolve()),
               "collection_completed": all(value["status"] == "collected" for value in phases.values()),
               "execution_validation_completed": False,
               "phases": phases, "blockers": blockers, "motion_requested": False, "execution_ready": False,
               "execute_boundary_status": (execute_result or {}).get("boundary_status"),
               "checks": {
                   "calibration_candidates": [{"path": review.get("candidate_path"), "status": review["status"]}
                                              for review in candidates],
                   "alignment": (alignment_result or {}).get("status", "error"),
                   "geometry": {"collider_count": (geometry_result or {}).get("collider_count"),
                                "full_cell_coverage_verified": False},
                   "ls_preflight": "passed" if report.get("static_checks_passed") else "failed",
                   "collision": (collision_result or {}).get("status", "error"),
                   "live": (live_result or {}).get("status", "error")},
               "read_only_commands": ((live_result or {}).get("observation") or {}).get("commands_sent", []),
               "preflight_errors_by_code": dict(Counter(item["code"] for item in report.get("errors", []))),
               "unmodeled_source_instruction_count": len(audit.get("unmodeled", []))}
    if args.world_context_memory:
        summary["checks"]["world_context"] = {"context_status": ((world_result or {}).get("context_report") or {}).get("status"),
                                               "prediction_status": ((world_result or {}).get("prediction_report") or {}).get("status"),
                                               "execute_skipped": ((world_result or {}).get("execute") or {}).get("skipped")}
    write(out / "summary.json", summary)
    lines = ["FANUC M-710iC/20L validation closeout", f"Created: {summary['created_at']}",
             f"Collection completed: {summary['collection_completed']}",
             "Execution validation completed: False", "Motion requested: False",
             f"Execute boundary: {summary['execute_boundary_status']}", "", "Unresolved conditions:"]
    lines.extend(f"- {blocker}" for blocker in blockers)
    lines.extend(["", "See summary.json for check results and individual artifact paths.",
                  "Exit code 1 means unresolved validation conditions remain."])
    (out / "SUMMARY.txt").write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(json.dumps(summary, ensure_ascii=False, indent=2))
    return 1  # The bundle is diagnostic; physical validation is still unavailable.


if __name__ == "__main__":
    raise SystemExit(main())
