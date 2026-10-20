import copy
import json

import pytest

from backend.executors.fanuc_nodes import exec_fanuc_ls_parse, exec_fanuc_m710_prepare
from backend.executors.world_model_nodes import exec_execution_guard, exec_fanuc_trajectory_validate
from backend.world_model.fanuc_segment_review import audit_ls_instructions, review_fanuc_segments
from test_fanuc_preflight import action_fixture
from test_fanuc_nodes import JOINT_ONLY_LS
from test_fanuc_collision import scene_fixture


def test_segments_preserve_original_times_lines_and_source_action():
    action = action_fixture()
    for index, frame in enumerate(action["joint_trajectory"]): frame.update(source_line=10+index*10, source_position_id=index+1)
    original = copy.deepcopy(action)
    result = review_fanuc_segments(action)
    assert result["segment_count"] == 2
    assert result["segments"][0]["status"] == "requires_initial_snapshot"
    segment = result["segments"][1]
    assert segment["status"] == "static_segment_checks_passed"
    assert segment["start"]["source_line"] == 10 and segment["end"]["source_line"] == 20
    assert segment["original_start_time_sec"] == 2 and segment["original_end_time_sec"] == 4
    assert result["time_rebased"] is False and result["execution_ready"] is False
    assert action == original and "joint_trajectory" not in result


def test_cnt_at_left_endpoint_and_missing_calibration_block_adjacent_segment():
    action = action_fixture()
    action["joint_trajectory"][0]["termination"] = "CNT50"
    action["joint_trajectory"][1].update(ik_solved=True, cartesian_mode="relative_uf_ut_ik", motion_type="L")
    result = review_fanuc_segments(action)
    assert all(item["status"] == "blocked" for item in result["segments"])
    assert {"cnt_blending_unverified", "uf_ut_calibration_missing", "cartesian_path_fidelity_unverified"}.issubset(
        {issue["code"] for issue in result["segments"][1]["issues"]})


def test_incomplete_source_blocks_all_segments_without_partial_export():
    action = action_fixture()
    action["skipped_unresolved_frames"] = 1
    result = review_fanuc_segments(action)
    assert result["global_issues"]
    assert all(segment["status"] == "blocked" and not segment["geometry_diagnostic_candidate"] for segment in result["segments"])
    assert result["partial_program_execution_supported"] is False


def test_instruction_audit_distinguishes_comments_frames_and_omitted_semantics():
    source = "/MN\n1: ! comment ;\n2: UFRAME_NUM=1 ;\n3: J P[1] 25% FINE ;\n4: WAIT .3(sec) ;\n5: ENTER I-ZONE(1) ;\n6: CALL SUB ;\n7: C P[2] 50mm/sec FINE ;\n8: J P[3] 25% FINE ACC50 ;\n/POS\n"
    audit = audit_ls_instructions(source)
    assert audit["recognized_counts"] == {"frame_selection": 1, "motion": 1}
    assert [item["kind"] for item in audit["unmodeled"]] == ["wait", "process_or_io", "control_flow", "unsupported_motion", "motion_modifier"]
    assert audit["source_program_semantics_verified"] is False


@pytest.mark.parametrize("source", ["no MN section", "/MN\n1: J P[1] 25% FINE\n/POS\n"])
def test_missing_or_unparsed_mn_statements_never_look_complete(source):
    action = action_fixture()
    action["source_instruction_audit"] = audit_ls_instructions(source)
    result = review_fanuc_segments(action)
    assert any(issue["code"] == "source_instruction_audit_incomplete" for issue in result["global_issues"])


def test_control_flow_blocks_sequence_and_wait_requires_local_review():
    action = action_fixture()
    for index, frame in enumerate(action["joint_trajectory"]): frame["source_line"] = index*2+1
    action["source_instruction_audit"] = audit_ls_instructions("/MN\n1: J P[1] 25% FINE ;\n2: WAIT .3(sec) ;\n3: J P[2] 25% FINE ;\n/POS\n")
    assert review_fanuc_segments(action)["segments"][1]["status"] == "requires_source_instruction_review"
    action["source_instruction_audit"] = audit_ls_instructions("/MN\n1: CALL OTHER ;\n/POS\n")
    assert all(segment["status"] == "blocked" for segment in review_fanuc_segments(action)["segments"])


def test_parser_preserves_audit_and_prepare_propagates_it(tmp_path):
    path = tmp_path / "program.LS"
    path.write_text(JOINT_ONLY_LS.replace("1:J P[1] 25% FINE", "1:J P[1] 25% FINE ACC50"), encoding="utf-8")
    parsed = exec_fanuc_ls_parse({"id": "parse", "data": {"filePath": str(path)}}, {}, [])
    prepared = exec_fanuc_m710_prepare({"id": "prepare", "data": {"timingMode": "fixed_total"}}, {"parse": parsed},
                                      [{"source": "parse", "sourceHandle": "program", "target": "prepare", "targetHandle": "program"}])
    assert prepared["action"]["source_instruction_audit"] == parsed["program"]["instruction_audit"]
    assert prepared["action"]["source_program"]["path"] == str(path)
    report = review_fanuc_segments(prepared["action"])
    assert any(issue["code"] == "source_motion_modifier_unverified" for issue in report["segments"][1]["issues"])


def test_segment_report_cannot_replace_execution_preflight_report():
    action = action_fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True)
    segment = review_fanuc_segments(action)
    context = {"a": {"action": action}, "r": {"report": segment}}
    edges = [{"source": source, "sourceHandle": handle, "target": "g", "targetHandle": handle} for source, handle in [("a", "action"), ("r", "report")]]
    assert exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, context, edges)["approved"] is False
    assert exec_fanuc_trajectory_validate({"id": "check", "data": {"action": action}}, {}, [])["segment_report"]["schema"] == segment["schema"]


def test_unmodeled_ls_instructions_block_existing_execute_guard_even_after_joint_checks_pass():
    from backend.world_model.fanuc_preflight import validate_fanuc_trajectory

    action = action_fixture()
    action.update(preview_only=False, execution_enabled=True, execution_ready=True,
                  source_instruction_audit=audit_ls_instructions("/MN\n1: WAIT .3(sec) ;\n/POS\n"))
    report = validate_fanuc_trajectory(action)
    assert report["static_checks_passed"] is True
    context = {"a": {"action": action}, "r": {"report": report}}
    edges = [{"source": source, "sourceHandle": handle, "target": "g", "targetHandle": handle} for source, handle in [("a", "action"), ("r", "report")]]
    result = exec_execution_guard({"id": "g", "data": {"validationPolicy": "static_simulation"}}, context, edges)
    assert result["approved"] is False
    assert any(reason["code"] == "source_program_semantics_unverified" for reason in result["decision"]["reasons"])


@pytest.mark.parametrize("case", ["clear", "collision", "cnt", "wait"])
def test_cli_geometry_gate_preserves_preview_and_never_connects(tmp_path, monkeypatch, case):
    from scripts import validate_fanuc_trajectory as cli

    source = JOINT_ONLY_LS if case == "cnt" else JOINT_ONLY_LS.replace("CNT50", "FINE")
    if case == "wait": source = source.replace("3:J", "4:J").replace("2:J", "3:J").replace("/POS", "2: WAIT .3(sec) ;\n/POS")
    ls_path, settings_path, scene_path = tmp_path / "program.LS", tmp_path / "settings.json", tmp_path / "scene.json"
    ls_path.write_text(source, encoding="utf-8")
    settings_path.write_text(json.dumps({"prepare": {"timingMode": "fixed_total", "totalDurationSec": 4}}), encoding="utf-8")
    scene = scene_fixture()
    if case != "collision": scene["obstacles"] = [{"id": "far", "min": [20]*3, "max": [21]*3}]
    scene_path.write_text(json.dumps(scene), encoding="utf-8")
    output = tmp_path / "geometry.json"
    if case in {"cnt", "wait"}:
        monkeypatch.setattr(cli, "check_fanuc_collision", lambda *args: pytest.fail("Geometry ran despite source/preflight blockers"))
    monkeypatch.setattr("sys.argv", ["validate_fanuc_trajectory.py", str(ls_path), "--settings", str(settings_path),
                                   "--collision-scene", str(scene_path), "--geometry-output", str(output)])
    assert cli.main() == (0 if case == "clear" else 1)
    result = json.loads(output.read_text(encoding="utf-8"))
    assert result["network_commands_sent"] == [] and result["execution_requested"] is False
    if case in {"cnt", "wait"}:
        assert result["status"] == ("blocked_preflight" if case == "cnt" else "blocked_source_semantics")
        assert result["action"] is None and result["report"] is None
    else:
        assert result["status"] == "completed"
        assert result["action"]["preview_only"] is True and result["action"]["execution_enabled"] is False
        assert result["report"]["status"] == ("clear" if case == "clear" else "collision")
