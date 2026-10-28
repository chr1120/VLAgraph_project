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

    def test_fanuc_and_amr_share_world_with_distinct_motion_states(self):
        scan = scan_fixture()
        arm = scan["robot"]
        arm.update(robot_kind="articulated_arm", manufacturer="FANUC", model="M-710iC/20L",
                   base_motion="rail", orientation_xyzw=[0, 0, 0, 1],
                   linear_velocity=[1000, 0, 0], angular_velocity_rad_s=[0, 1, 0],
                   joints=[{"name": f"J{i}", "kind": "revolute", "unit": "rad",
                            "position": i / 10, "velocity": 0} for i in range(1, 7)] +
                          [{"name": "externalAxisJoint", "kind": "prismatic", "unit": "m",
                            "position": 0.25, "velocity": 0.1}])
        amr = {"prim_path": "/World/AMR", "position": [2000, 0, 0], "robot_kind": "amr",
               "base_motion": "planar", "orientation_xyzw": [0, 0, 0, 1],
               "linear_velocity": [1000, 0, 0], "angular_velocity_rad_s": [0, 0.5, 0],
               "mobile_base": {"drive_type": "differential", "footprint_radius_m": 0.4,
                               "navigation_frame": "map", "localization_valid": True}}
        scan["robots"] = [amr, arm]
        original = copy.deepcopy(scan)
        state = self.build(scan)
        self.assertEqual(state.robot.model, "M-710iC/20L")
        self.assertEqual(state.robot.base_motion, "rail")
        self.assertEqual(len(state.robot.joints), 7)
        self.assertEqual(state.robot.joints[-1].position, 0.25)
        self.assertEqual(state.robot.linear_velocity_m_s, (1, 0, 0))
        self.assertEqual(state.robot.angular_velocity_rad_s, (0, 0, 1))
        self.assertAlmostEqual(state.robot.orientation_xyzw[0], 2 ** -0.5)
        self.assertAlmostEqual(state.robot.orientation_xyzw[3], 2 ** -0.5)
        self.assertEqual(state.robots[0].angular_velocity_rad_s, (0, 0, 0.5))
        self.assertEqual(state.robots[0].mobile_base.footprint_radius_m, 0.4)
        self.assertEqual(state.robots[0].missing_fields, [])
        self.assertNotIn("robot_velocity", state.missing_fields)
        self.assertIn("physics_snapshot", state.missing_fields)
        self.assertFalse(state.prediction_ready)
        self.assertEqual(scan, original)
        from backend.world_model.schemas import WorldState
        self.assertEqual(WorldState.model_validate_json(state.model_dump_json()), state)

    def test_robots_only_selects_first_and_tracks_missing_mobile_state(self):
        from backend.executors.spatial_nodes import normalize_scene_scan_response

        scan = scan_fixture()
        amr = scan.pop("robot")
        amr.update(robot_kind="amr", base_motion="planar", mobile_base={"localization_valid": False})
        scan["robots"] = [amr]
        state = self.build(scan)
        self.assertEqual(state.robot, state.robots[0])
        self.assertNotIn("joint_state", state.missing_fields)
        self.assertIn("mobile_base.footprint_radius_m", state.robot.missing_fields)
        self.assertIn("valid_localization", state.robot.missing_fields)
        self.assertIn("robots[/World/Robot].valid_localization", state.missing_fields)
        normalized = normalize_scene_scan_response({"ok": True, "scan": scan})
        self.assertEqual(self.build(normalized).robot, state.robot)
        result = exec_world_state_build(
            {"id": "mixed", "data": {"episodeId": "episode", "sceneVersion": "v1", "scan": normalized}}, {}, [])
        self.assertTrue(result["ok"])
        self.assertEqual(result["world_state"]["robots"][0]["robot_kind"], "amr")

    def test_label_does_not_infer_robot_kind_or_stationary_base(self):
        scan = scan_fixture()
        scan["robot"]["label"] = "FANUC M-710iC/20L"
        state = self.build(scan)
        self.assertEqual(state.robot.robot_kind, "unknown")
        self.assertEqual(state.robot.base_motion, "unknown")
        self.assertIn("robot_kind", state.robot.missing_fields)

    def test_mobile_manipulator_requires_both_arm_and_base_observations(self):
        scan = scan_fixture()
        scan["robot"].update(robot_kind="mobile_manipulator", base_motion="planar")
        state = self.build(scan)
        self.assertIn("joint_state", state.robot.missing_fields)
        self.assertIn("mobile_base.drive_type", state.robot.missing_fields)
        scan["robot"].update(robot_kind="amr", joints=[
            {"name": "left_wheel", "kind": "revolute", "unit": "rad", "position": 0}])
        self.assertIn("joint_velocity", self.build(scan).robot.missing_fields)

    def test_rejects_conflicting_robots_and_invalid_robot_observations(self):
        for update in (
            {"robot_kind": "typo"}, {"orientation_xyzw": [0, 0, 0, 0]},
            {"orientation_xyzw": [False, 0, 0, 1]}, {"linear_velocity": [0, float("nan"), 0]},
            {"angular_velocity_rad_s": [0, 1]},
            {"mobile_base": {"footprint_radius_m": -1}},
            {"mobile_base": {"localization_valid": "true"}},
            {"joints": [{"name": "J1", "kind": "revolute", "unit": "m", "position": 0}]},
            {"joints": [{"name": "J1", "kind": "revolute", "unit": "rad", "position": True}]},
            {"joints": [{"name": "J1", "kind": "revolute", "unit": "rad", "position": 0}] * 2},
        ):
            scan = scan_fixture()
            scan["robot"].update(update)
            with self.subTest(update=update), self.assertRaises(ValueError):
                self.build(scan)
        for robots in ([], [None], [scan_fixture()["robot"]] * 2,
                       [{"prim_path": "/World/Other", "position": [0, 0, 0]}]):
            with self.subTest(robots=robots), self.assertRaises(ValueError):
                self.build({**scan_fixture(), "robots": robots})

    def test_z_up_robot_orientation_and_velocity(self):
        scan = {**scan_fixture(), "up_axis": "Z", "meters_per_unit": 0.01}
        scan["robot"].update(orientation_xyzw=[0, 0, 0, 1], linear_velocity=[100, 200, 300],
                             angular_velocity_rad_s=[1, 2, 3])
        state = self.build(scan)
        self.assertEqual(state.robot.orientation_xyzw, (0, 0, 0, 1))
        self.assertEqual(state.robot.linear_velocity_m_s, (1, 2, 3))
        self.assertEqual(state.robot.angular_velocity_rad_s, (1, 2, 3))

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

    def test_mixed_robot_example_runs_through_registered_graph(self):
        import json
        from pathlib import Path
        from backend.runtime.runner import run_graph

        path = Path(__file__).resolve().parents[1] / "examples/graphs/mixed_robot_world_state_graph.json"
        result = run_graph(json.loads(path.read_text(encoding="utf-8")))
        output = result["context"]["build"]
        self.assertTrue(output["ok"])
        self.assertEqual([robot["robot_kind"] for robot in output["world_state"]["robots"]],
                         ["articulated_arm", "amr"])
        self.assertFalse(output["prediction_ready"])

    def test_deserialization_rejects_invalid_orientation_and_conflicting_selected_robot(self):
        from backend.world_model.schemas import WorldState

        document = self.build(scan_fixture()).model_dump(mode="json")
        for quaternion in ([0, 0, 0, 0], [0, 0, 0, 2], [False, 0, 0, 1]):
            altered = copy.deepcopy(document)
            altered["robot"]["orientation_xyzw"] = quaternion
            with self.subTest(quaternion=quaternion), self.assertRaises(ValueError):
                WorldState.model_validate(altered)
        altered = copy.deepcopy(document)
        altered["robots"][0]["position"] = [999, 0, 0]
        with self.assertRaisesRegex(ValueError, "Selected robot"):
            WorldState.model_validate(altered)

    def test_deserialization_rejects_duplicate_entities_and_joints(self):
        from backend.world_model.schemas import RobotObservation, WorldState

        document = self.build(scan_fixture()).model_dump(mode="json")
        for field in ("robots", "objects"):
            altered = copy.deepcopy(document)
            altered[field].append(copy.deepcopy(altered[field][0]))
            with self.subTest(field=field), self.assertRaises(ValueError):
                WorldState.model_validate(altered)
        robot = document["robot"]
        robot["joints"] = [{"name": "J1", "kind": "revolute", "unit": "rad", "position": 0}] * 2
        with self.assertRaisesRegex(ValueError, "Joint names"):
            RobotObservation.model_validate(robot)

    def test_legacy_world_without_robots_remains_readable_but_empty_explicit_list_is_invalid(self):
        from backend.world_model.schemas import WorldState

        document = self.build(scan_fixture()).model_dump(mode="json")
        document.pop("robots")
        restored = WorldState.model_validate_json(__import__("json").dumps(document))
        self.assertEqual(restored.robots, [restored.robot])
        document["robots"] = []
        with self.assertRaises(ValueError):
            WorldState.model_validate(document)


if __name__ == "__main__":
    unittest.main()
