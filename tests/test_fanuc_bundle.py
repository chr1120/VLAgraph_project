import json

import pytest

from backend.executors import vla_nodes
from scripts import validate_fanuc_bundle as bundle
from test_fanuc_nodes import JOINT_ONLY_LS


def test_existing_execute_template_runs_offline_and_guard_blocks_before_transport(tmp_path, monkeypatch):
    path = tmp_path / "source.LS"
    path.write_text(JOINT_ONLY_LS, encoding="utf-8")
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *a: pytest.fail("Motion transport reached"))
    result = bundle.review_execute(path)
    assert result["boundary_status"] == "blocked_as_expected"
    assert result["guard"]["approved"] is False and result["execute"]["skipped"] is True
    assert result["network_commands_sent"] == [] and not result["calibration_candidates_applied"]
    assert result["segment_report"]["source_instruction_audit"]["unmodeled"] == []


def test_bundle_collects_remaining_phases_after_bad_candidate_usd_and_alignment(tmp_path, monkeypatch):
    source, candidate, alignment = tmp_path / "source.LS", tmp_path / "tooldata.ls", tmp_path / "alignment.json"
    source.write_text(JOINT_ONLY_LS, encoding="utf-8")
    candidate.write_text("ROBOT_NAME,m20ia\nPOSE,start,0,0,0,0,0,0", encoding="utf-8")
    alignment.write_text("{}", encoding="utf-8")
    out = tmp_path / "bundle"
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *a: pytest.fail("Motion transport reached"))
    monkeypatch.setattr(bundle.subprocess, "run", lambda *a, **k: pytest.fail("Network probe launched in offline mode"))
    monkeypatch.setattr("sys.argv", ["bundle", str(source), "--candidate", str(candidate),
                                   "--candidate", str(tmp_path / "missing.ls"), "--usd", str(tmp_path / "missing.usd"),
                                   "--alignment", str(alignment), "--output-dir", str(out)])
    assert bundle.main() == 1
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert set(summary["phases"]) == {"calibration", "alignment", "geometry", "execute", "collision", "live"}
    assert summary["phases"]["geometry"]["status"] == "error"
    assert summary["phases"]["alignment"]["status"] == "error"
    assert summary["execute_boundary_status"] == "blocked_as_expected"
    assert summary["status"] == "blocked" and not summary["execution_ready"] and not summary["motion_requested"]
    assert summary["collection_completed"] is False and not summary["execution_validation_completed"]
    assert "Motion requested: False" in (out / "SUMMARY.txt").read_text(encoding="utf-8")
    reviews = json.loads((out / "calibration.json").read_text(encoding="utf-8"))["candidates"]
    assert [r["status"] for r in reviews] == ["rejected", "error"]
    assert (out / "alignment-input-template.json").exists()


def test_live_phase_launches_only_read_only_probe_and_reports_failure(tmp_path, monkeypatch):
    source, candidate = tmp_path / "source.LS", tmp_path / "tooldata.ls"
    source.write_text(JOINT_ONLY_LS, encoding="utf-8")
    candidate.write_text("ROBOT_NAME,m710ic_20l_R01\nPOSE,start,0,0,0,0,0,0", encoding="utf-8")
    out = tmp_path / "bundle"
    calls = []

    def probe(command, **kwargs):
        from pathlib import Path
        from types import SimpleNamespace
        calls.append(command)
        assert Path(command[1]).name == "validate_fanuc_observation.py"
        assert command[2:6] == ["--endpoint", "ws://localhost:9910/omniverse", "--snapshot", "--geometry"]
        bundle.write(out / "live-observation.json", {"ok": False, "commands_sent": ["get_state", "scan_scene", "capture_world_state", "get_collision_geometry"]})
        return SimpleNamespace(returncode=1, stderr="")

    monkeypatch.setattr(bundle.subprocess, "run", probe)
    monkeypatch.setattr(vla_nodes, "_omniverse_execute", lambda *a: pytest.fail("Motion transport reached"))
    monkeypatch.setattr("sys.argv", ["bundle", str(source), "--candidate", str(candidate), "--usd", str(tmp_path / "missing.usd"),
                                   "--endpoint", "ws://localhost:9910/omniverse", "--output-dir", str(out)])
    assert bundle.main() == 1 and len(calls) == 1
    summary = json.loads((out / "summary.json").read_text(encoding="utf-8"))
    assert summary["checks"]["live"] == "blocked"
    assert "apply_fanuc_trajectory" not in summary["read_only_commands"]
    # Reuse cannot mix any old evidence with a new run, or overwrite its report.
    original = (out / "summary.json").read_bytes()
    calls.clear()
    with pytest.raises(SystemExit) as exc:
        bundle.main()
    assert exc.value.code == 2 and calls == []
    assert (out / "summary.json").read_bytes() == original
