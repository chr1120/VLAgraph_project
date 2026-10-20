"""Compare LS frame requirements with TOOLDATA candidates offline; never applies settings."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.executors.fanuc_nodes import _read_ls
from backend.world_model.fanuc_calibration_review import review_calibration_candidate


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("ls_path", type=Path)
    parser.add_argument("--candidate", type=Path, action="append", required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    source = _read_ls(args.ls_path)
    reviews = []
    for path in args.candidate:
        review = review_calibration_candidate(source, _read_ls(path))
        review["candidate_path"] = str(path.resolve())
        reviews.append(review)
    result = {"schema": "vlagraph.fanuc_calibration_comparison.v1", "source_path": str(args.ls_path.resolve()),
              "execution_requested": False, "network_commands_sent": [], "candidates": reviews}
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    for review in reviews:
        print(json.dumps({key: review[key] for key in ("candidate_path", "status", "required_frames", "missing_frames", "issues")}, ensure_ascii=False))
    return 1 if any(review["status"] == "rejected" for review in reviews) else 0


if __name__ == "__main__":
    raise SystemExit(main())
