from __future__ import annotations

import unittest

from backend.executors.g1_skill_nodes import decode_g1_skill_command

try:
    from backend.executors.vla_nodes import exec_action_decode
except ModuleNotFoundError:
    exec_action_decode = None


class G1SkillNodesTest(unittest.TestCase):
    def test_context_without_user_task_cannot_supply_navigation(self) -> None:
        decoded = decode_g1_skill_command(
            prompt="Known spatial context:\n- stored_approach_waypoint=[10, 20, 0]",
            action={}, state={},
        )
        self.assertFalse(decoded["intent"].get("approach_waypoint"))
        self.assertIsNone(decoded.get("target_pose"))

    def test_explicit_user_waypoint_remains_supported(self) -> None:
        decoded = decode_g1_skill_command(
            prompt="Approach waypoint is x=1 y=2 yaw=0", action={}, state={},
        )
        self.assertTrue(decoded["intent"].get("approach_waypoint"))

    def test_spatial_context_waypoint_is_not_treated_as_control_command(self) -> None:
        prompt = (
            "Known spatial context:\n"
            "- In scene old_scene, RedBox is at x=-1.9 y=2.9 z=0.1. "
            "Approach waypoint is x=-1.5 y=2.3 yaw=2.14.\n\n"
            "User task:\n"
            "walk forward toward the checkpoint and turn slightly left"
        )
        decoded = decode_g1_skill_command(
            prompt=prompt,
            action={},
            state={"position": [0.0, 0.0, 0.8], "rotation": [0.0, 0.0, 0.0]},
        )

        self.assertEqual(decoded["skill"], "walk")
        self.assertFalse(decoded["intent"].get("approach_waypoint"))
        self.assertIsNone(decoded.get("target_pose"))
        self.assertGreater(decoded["move_z"], 0.0)

    def test_gr00t_default_action_decode_emits_root_motion_for_walk(self) -> None:
        if exec_action_decode is None:
            self.skipTest("websocket-client is not installed in this Python environment")
        node = {
            "id": "decode",
            "type": "action_decode",
            "data": {"decodeMode": "default"},
        }
        context = {
            "prompt": {
                "type": "prompt",
                "text": "walk forward toward the checkpoint and turn slightly left",
            },
            "infer": {
                "action": {
                    "type": "gr00t_action",
                    "command": "apply_humanoid_motion",
                    "left_leg": [0.0] * 6,
                    "right_leg": [0.0] * 6,
                }
            },
        }
        edges = [
            {"source": "infer", "sourceHandle": "action", "target": "decode", "targetHandle": "action"},
        ]

        decoded = exec_action_decode(node, context, edges)["decoded"]

        self.assertEqual(decoded["type"], "g1_skill_command")
        self.assertEqual(decoded["command"], "apply_humanoid_motion")
        self.assertEqual(decoded["retarget_mode"], "procedural_walk")
        trajectory = decoded.get("g1_joint_trajectory")
        self.assertIsInstance(trajectory, list)
        self.assertGreater(len(trajectory), 1)
        forward_total = sum(frame["root_translation"][0] for frame in trajectory)
        self.assertGreater(forward_total, 0.1)


if __name__ == "__main__":
    unittest.main()
