from __future__ import annotations

import tempfile
import json
import unittest
from pathlib import Path

from backend.executors.spatial_nodes import (
    exec_scene_context_reset,
    exec_spatial_context_builder,
    exec_spatial_memory_save,
    exec_spatial_rag_retrieve,
    normalize_scene_scan_response,
    normalize_memory_record,
    _record_score,
)


def _edge(source: str, target: str, source_handle: str, target_handle: str) -> dict:
    return {
        "source": source,
        "target": target,
        "sourceHandle": source_handle,
        "targetHandle": target_handle,
    }


class SpatialNodesTest(unittest.TestCase):
    def test_observation_ignores_success_flag_and_acknowledgement(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            result = exec_spatial_memory_save(
                {"id": "save", "data": {
                    "memoryPath": str(Path(tmpdir) / "memory.jsonl"), "success": True,
                    "scan": {"scene_id": "lab", "objects": [{"name": "box", "position": [1, 2, 3]}]},
                    "action": {"command": "move"}, "result": {"ok": True, "applied": True},
                }}, {}, [],
            )
            record = result["record"]
            self.assertEqual(record["record_kind"], "observation")
            self.assertEqual(record["outcome"], "unknown")
            self.assertIsNone(record["metadata"]["success"])

    def test_verified_completion_required_for_success_and_failure(self) -> None:
        for outcome in ("success", "failure"):
            record = {
                "scan": {"scene_id": "lab"}, "action": {"command": "move"},
                "result": {"status": "completed", "actual_state": {"position": [1, 2, 3]},
                           "evaluation": {"verified": True, "outcome": outcome}},
            }
            self.assertEqual(normalize_memory_record(record)["outcome"], outcome)
            for missing in ("actual_state", "evaluation", "status"):
                incomplete = {**record, "result": {k: v for k, v in record["result"].items() if k != missing}}
                self.assertEqual(normalize_memory_record(incomplete)["outcome"], "unknown")

    def test_legacy_success_does_not_boost_retrieval_or_rewrite_file(self) -> None:
        legacy = {"id": "old", "text": "box", "metadata": {"scene_id": "lab", "success": True}}
        with tempfile.TemporaryDirectory() as tmpdir:
            path = Path(tmpdir) / "memory.jsonl"
            path.write_text(json.dumps(legacy) + "\n", encoding="utf-8")
            original = path.read_bytes()
            retrieved = exec_spatial_rag_retrieve(
                {"id": "retrieve", "data": {"memoryPath": str(path), "sceneId": "lab"}}, {}, [],
            )["memories"][0]
            self.assertEqual(path.read_bytes(), original)
            self.assertEqual(retrieved["outcome"], "unknown")
            self.assertNotIn("success", retrieved["score_debug"]["reasons"])
            built = exec_spatial_context_builder(
                {"id": "ctx", "data": {"memories": [legacy]}}, {}, [],
            )
            self.assertIn("outcome=unknown", built["text"])
            self.assertIsNone(built["memories"][0]["metadata"]["success"])

    def test_only_verified_success_gets_score_bonus(self) -> None:
        record = {"scan": {"scene_id": "lab"}, "action": {"command": "move"},
                  "result": {"status": "completed", "actual_state": {"position": [0, 0, 0]},
                             "evaluation": {"verified": True, "outcome": "success"}}}
        self.assertIn("success", _record_score(record, "", "", [0, 0, 0])[1]["reasons"])

    def test_failed_scan_is_not_saved_or_used_to_reset_scene(self) -> None:
        context = {"scan": {"ok": False, "scan": {"scene_id": "lab", "objects": [{"name": "box"}]}}}
        edges = [_edge("scan", "save", "scan", "scan")]
        # The scan handle itself must carry failure, as exec_scene_scan now does.
        context["scan"]["scan"]["ok"] = False
        self.assertFalse(exec_spatial_memory_save({"id": "save", "data": {}}, context, edges)["ok"])
        reset_edges = [_edge("scan", "reset", None, "scan")]
        self.assertFalse(exec_scene_context_reset({"id": "reset", "data": {}}, context, reset_edges)["ok"])
        self.assertNotIn("_spatial_scene_context", context)

    def test_scan_preserves_metadata_and_reports_uncertainty(self) -> None:
        scan = normalize_scene_scan_response({
            "scene_id": "lab", "timestamp": 123, "meters_per_unit": 0.01, "up_axis": "Y",
            "truncated": True, "scan_scope": {"max_objects": 2},
            "objects": [{"prim_path": "/Box", "position": [1, 2, 3]},
                        {"prim_path": "/Box", "position": [float("nan"), 0, 0]}],
        })
        self.assertEqual(scan["timestamp"], 123)
        self.assertEqual(scan["scan_scope"]["max_objects"], 2)
        quality = scan["observation_quality"]
        self.assertEqual(quality["coverage"], "partial")
        self.assertFalse(quality["sim_time_known"])
        self.assertTrue(quality["units_known"])
        self.assertEqual(quality["duplicate_prim_paths"], ["/Box"])
        self.assertEqual(quality["invalid_object_positions"], ["/Box"])

    def test_memory_save_retrieve_and_context_builder(self) -> None:
        with tempfile.TemporaryDirectory() as tmpdir:
            memory_path = str(Path(tmpdir) / "spatial_memory.jsonl")
            scan = {
                "scene_id": "lab_room_001",
                "robot": {"position": [0.0, 0.0, 0.0]},
                "objects": [
                    {"name": "red box", "position": [2.1, 0.4, 0.7]},
                    {"name": "blue cylinder", "position": [5.0, 1.0, 0.2]},
                ],
            }
            save_result = exec_spatial_memory_save(
                {"id": "save", "data": {"memoryPath": memory_path}},
                {
                    "scan": {"scan": scan},
                    "prompt": {"text": "red box 앞으로 이동해"},
                    "result": {"result": {"ok": True}},
                },
                [
                    _edge("scan", "save", "scan", "scan"),
                    _edge("prompt", "save", "text", "prompt"),
                    _edge("result", "save", "result", "result"),
                ],
            )

            self.assertTrue(save_result["ok"])
            self.assertTrue(Path(memory_path).exists())

            retrieve_result = exec_spatial_rag_retrieve(
                {"id": "retrieve", "data": {"memoryPath": memory_path, "topK": 3}},
                {
                    "prompt": {"text": "red box 앞으로 이동해"},
                    "scene": {"scene_id": "lab_room_001"},
                },
                [
                    _edge("prompt", "retrieve", "text", "prompt"),
                    _edge("scene", "retrieve", "scene_id", "scene_id"),
                ],
            )

            self.assertEqual(len(retrieve_result["memories"]), 1)
            self.assertIn("red box", retrieve_result["memories"][0]["text"])

            context_result = exec_spatial_context_builder(
                {"id": "context", "data": {"maxLines": 3}},
                {
                    "prompt": {"text": "red box 앞으로 이동해"},
                    "retrieve": retrieve_result,
                },
                [
                    _edge("prompt", "context", "text", "prompt"),
                    _edge("retrieve", "context", "memories", "memories"),
                ],
            )

            self.assertIn("Known spatial context:", context_result["text"])
            self.assertIn("User task:", context_result["text"])
            self.assertNotIn("Approach waypoint is", context_result["text"])
            self.assertIn("stored_approach_waypoint", context_result["text"])

    def test_scene_context_reset_detects_signature_change(self) -> None:
        context: dict = {
            "scan": {"scan": {"scene_id": "scene_a", "objects": [{"name": "box", "prim_path": "/World/Box"}]}}
        }
        first = exec_scene_context_reset(
            {"id": "reset", "data": {}},
            context,
            [_edge("scan", "reset", "scan", "scan")],
        )
        context["scan"] = {
            "scan": {"scene_id": "scene_b", "objects": [{"name": "table", "prim_path": "/World/Table"}]}
        }
        second = exec_scene_context_reset(
            {"id": "reset", "data": {}},
            context,
            [_edge("scan", "reset", "scan", "scan")],
        )

        self.assertTrue(first["ok"])
        self.assertFalse(first["reset"]["scene_changed"])
        self.assertTrue(second["reset"]["scene_changed"])

    def test_normalize_scene_scan_response_accepts_flat_bridge_response(self) -> None:
        normalized = normalize_scene_scan_response(
            {
                "ok": True,
                "scene_id": "lab_room_001",
                "robot": {"prim_path": "/World/KyleRobot"},
                "objects": [{"name": "table", "prim_path": "/World/Table"}],
            }
        )

        self.assertEqual(normalized["scene_id"], "lab_room_001")
        self.assertEqual(normalized["robot"]["prim_path"], "/World/KyleRobot")
        self.assertEqual(len(normalized["objects"]), 1)
        self.assertEqual(normalized["object_count"], 1)


if __name__ == "__main__":
    unittest.main()
