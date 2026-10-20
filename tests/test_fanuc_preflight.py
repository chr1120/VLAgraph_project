import copy
import json
import math

import pytest

from backend.world_model.fanuc_preflight import validate_fanuc_trajectory
from backend.executors.fanuc_nodes import exec_fanuc_ls_parse, exec_fanuc_m710_prepare
from backend.executors.world_model_nodes import exec_fanuc_trajectory_validate
from test_fanuc_nodes import JOINT_ONLY_LS


def action_fixture():
    return {
        "schema": "vlagraph.fanuc_m710ic20l.action.v1", "command": "apply_fanuc_trajectory", "unit": "degree",
        "joint_names": [f"J{i}" for i in range(1, 7)],
        "joint_trajectory": [
            {"time_sec": 2, "joint_positions": [0] * 6, "motion_type": "J", "termination": "FINE"},
            {"time_sec": 4, "joint_positions": [10, 0, 0, 0, 0, 0], "motion_type": "J", "termination": "FINE"},
        ],
    }


def codes(report):
    return {item["code"] for item in report["errors"]}


def test_valid_static_preview_is_never_execution_approval():
    action = action_fixture()
    original = copy.deepcopy(action)
    report = validate_fanuc_trajectory(action)
    assert report["static_checks_passed"] is True
    assert report["execution_ready"] is False
    assert report["collision_checked"] is False
    assert "initial_motion_without_snapshot" in report["unchecked"]
    assert report["preview_action"]["preview_only"] is True
    assert report["preview_action"]["execution_ready"] is False
    assert math.isclose(report["normalized_trajectory"][1]["positions"]["J1"], math.radians(10))
    assert action == original


@pytest.mark.parametrize("time_value", [0, -1, float("nan"), True])
def test_invalid_times_never_produce_action(time_value):
    action = action_fixture()
    action["joint_trajectory"][0]["time_sec"] = time_value
    report = validate_fanuc_trajectory(action)
    assert report["static_checks_passed"] is False
    assert report["preview_action"] is None
    json.dumps(report, allow_nan=False)


def test_joint_limit_checks_do_not_clamp_or_repair():
    action = action_fixture()
    action["joint_trajectory"][1]["joint_positions"][0] = 181
    report = validate_fanuc_trajectory(action)
    assert "joint_position_limit" in codes(report)
    assert action["joint_trajectory"][1]["joint_positions"][0] == 181


def test_smoothstep_peak_velocity_and_acceleration():
    action = action_fixture()
    action["unit"] = "rad"
    action["joint_trajectory"][1]["joint_positions"][0] = 2
    report = validate_fanuc_trajectory(action, config={"maxVelocity": {"J1": 1}, "maxAcceleration": {"J1": 2}})
    peak = report["segments"][0]["smoothstep_peaks"]["J1"]
    assert peak == {"velocity": 1.5, "acceleration": 3}
    assert {"joint_velocity_limit", "joint_acceleration_limit"}.issubset(codes(report))


def test_external_axis_mm_is_distinct_from_arm_radians_and_aliases_must_agree():
    action = action_fixture()
    action["unit"] = "rad"
    for frame in action["joint_trajectory"]:
        frame.pop("joint_positions")
        frame["joint_target_map"] = {**{f"J{i}": 0 for i in range(1, 7)}, "E1": 1000, "externalAxisJoint": 1000}
        frame["external_axis_targets_m"] = {"E1": 1}
    report = validate_fanuc_trajectory(action)
    assert report["static_checks_passed"] is True
    assert report["normalized_trajectory"][0]["positions"]["externalAxisJoint"] == 1
    action["joint_trajectory"][1]["joint_target_map"]["E1"] = 2000
    assert "invalid_frame" in codes(validate_fanuc_trajectory(action))


def test_named_and_array_conflict_is_detected():
    action = action_fixture()
    action["joint_trajectory"][0]["joint_target_map"] = {f"J{i}": 10 for i in range(1, 7)}
    assert "named_and_array_targets_conflict" in codes(validate_fanuc_trajectory(action))


def test_missing_frames_calibration_and_ls_path_semantics_are_reported():
    action = action_fixture()
    action["skipped_unresolved_frames"] = 1
    frame = action["joint_trajectory"][1]
    frame.update(ik_solved=True, cartesian_mode="relative_uf_ut_ik", motion_type="L", termination="CNT50")
    report = validate_fanuc_trajectory(action)
    assert {"incomplete_source_program", "uf_ut_calibration_missing", "cartesian_path_fidelity_unverified",
            "cnt_blending_unverified"}.issubset(codes(report))


def test_node_reports_invalid_trajectory_without_forwarding_an_action():
    action = action_fixture()
    action["joint_trajectory"][0]["time_sec"] = 0
    result = exec_fanuc_trajectory_validate({"id": "check", "data": {"action": action}}, {}, [])
    assert result["ok"] is True
    assert result["static_checks_passed"] is False
    assert result["preview_action"] is None


def test_require_all_ik_also_blocks_missing_position_records(tmp_path):
    path = tmp_path / "incomplete.LS"
    path.write_text(JOINT_ONLY_LS.replace("3:J P[3]", "3:J P[99]"), encoding="utf-8")
    parsed = exec_fanuc_ls_parse({"id": "parse", "data": {"filePath": str(path)}}, {}, [])
    result = exec_fanuc_m710_prepare(
        {"id": "prepare", "data": {"requireAllIk": True, "enableExecution": True, "previewOnly": False}},
        {"parse": parsed}, [{"source": "parse", "target": "prepare", "sourceHandle": "program", "targetHandle": "program"}],
    )
    assert result["summary"]["skipped_unresolved_frames"] == 1
    assert result["action"]["execution_ready"] is False
    assert result["action"]["preview_only"] is True
