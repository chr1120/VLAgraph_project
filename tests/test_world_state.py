from __future__ import annotations

import copy
import unittest

from backend.executors.world_model_nodes import exec_world_state_build
from backend.world_model.state_builder import build_world_state


def scan_fixture():
    return {
        "scene_id": "cell", "meters_per_unit": 0.001, "up_axis": "Y", "timestamp": 1000,
        "robot": {"prim_path": "/World/Robot", "position": [1000, 2000, 3000]},
        "objects": [{"prim_path": "/World/Box", "position": [4000, 5000, 6000],
                     "bbox_center": [4000, 5000, 6000], "size": [100, 200, 300]}],
    }


class WorldStateTest(unittest.TestCase):
    def build(self, scan):
        return build_world_state(scan, episode_id="episode", scene_version="v1", state_version=1)

    def test_y_up_mm_geometry_and_unknown_physics(self):
        scan = scan_fixture()
        original = copy.deepcopy(scan)
        state = self.build(scan)
        self.assertEqual(state.robot.position, (1, -3, 2))
        self.assertEqual(state.objects[0].position, (4, -6, 5))
        self.assertEqual(state.objects[0].size, (0.1, 0.3, 0.2))
        self.assertIsNone(state.sim_time)
        self.assertEqual(state.observed_at, 1000)
        self.assertIn("sim_time", state.missing_fields)
        self.assertFalse(state.prediction_ready)
        self.assertEqual(scan, original)
        self.assertEqual(state.objects[0].entity_id, self.build(scan).objects[0].entity_id)

    def test_rejects_unknown_units_axis_duplicate_and_invalid_geometry(self):
        for field, value in (("meters_per_unit", None), ("meters_per_unit", float("nan")),
                             ("meters_per_unit", True), ("up_axis", "X"), ("ok", False)):
            with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                self.build({**scan_fixture(), field: value})
        scan = scan_fixture()
        scan["objects"].append(copy.deepcopy(scan["objects"][0]))
        with self.assertRaises(ValueError):
            self.build(scan)
        for position in (None, [0, 1], [0, float("inf"), 0]):
            scan = scan_fixture()
            scan["robot"]["position"] = position
            with self.assertRaises(ValueError):
                self.build(scan)

    def test_z_up_and_sim_time_are_preserved(self):
        scan = {**scan_fixture(), "up_axis": "Z", "meters_per_unit": 1, "sim_time": 12}
        state = self.build(scan)
        self.assertEqual(state.robot.position, (1000, 2000, 3000))
        self.assertEqual(state.sim_time, 12)
        self.assertNotIn("sim_time", state.missing_fields)

    def test_node_versions_reset_with_episode_and_failure_does_not_increment(self):
        context = {"scan": {"scan": scan_fixture()}}
        node = {"id": "build", "data": {"episodeId": "a", "sceneVersion": "v1"}}
        edges = [{"source": "scan", "target": "build", "sourceHandle": "scan", "targetHandle": "scan"}]
        first = exec_world_state_build(node, context, edges)
        self.assertEqual(first["world_state"]["state_version"], 1)
        context["scan"]["scan"]["meters_per_unit"] = None
        self.assertFalse(exec_world_state_build(node, context, edges)["ok"])
        context["scan"]["scan"] = scan_fixture()
        self.assertEqual(exec_world_state_build(node, context, edges)["world_state"]["state_version"], 2)
        node["data"]["episodeId"] = "b"
        self.assertEqual(exec_world_state_build(node, context, edges)["world_state"]["state_version"], 1)
        node["data"]["episodeId"] = ""
        self.assertIsNone(exec_world_state_build(node, context, edges)["world_state"])

    def test_registered_graph_builds_and_prints_world_state(self):
        from backend.runtime.runner import run_graph

        graph = {
            "nodes": [
                {"id": "source", "type": "robot_state", "data": {"robotState": scan_fixture()}},
                {"id": "build", "type": "world_state_build", "data": {"episodeId": "a", "sceneVersion": "v1"}},
                {"id": "out", "type": "print", "data": {}},
            ],
            "edges": [
                {"source": "source", "target": "build", "sourceHandle": "state", "targetHandle": "scan"},
                {"source": "build", "target": "out", "sourceHandle": "world_state", "targetHandle": "input"},
            ],
        }
        result = run_graph(graph)
        self.assertTrue(result["context"]["build"]["ok"])
        printed = result["context"]["out"]["printed"]
        self.assertIn("right_handed_z_up_m", str(printed))


if __name__ == "__main__":
    unittest.main()
