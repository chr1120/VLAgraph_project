"""Inspect selected review pairs offline using authored convex hull points."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.world_model.fanuc_hull_inspection import inspect_review_hulls
from backend.world_model.fanuc_hull_sweep import inspect_review_hull_sweep


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd_path", type=Path)
    parser.add_argument("inventory", type=Path)
    parser.add_argument("proxy", type=Path)
    parser.add_argument("diagnostic", type=Path, help="JSON wrapper containing action and report; optional snapshot")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--sweep", action="store_true", help="Check selected pairs over trajectory intervals")
    parser.add_argument("--max-step-sec", type=float, default=0.05, help="Sweep sample spacing")
    parser.add_argument("--max-samples", type=int, default=10000, help="Sweep sample budget")
    args = parser.parse_args()
    from pxr import Usd
    diagnostic, proxy = read(args.diagnostic), read(args.proxy)
    inspector = inspect_review_hull_sweep if args.sweep else inspect_review_hulls
    options = {"max_step_sec": args.max_step_sec, "max_samples": args.max_samples} if args.sweep else {}
    report = inspector(Usd.Stage.Open(str(args.usd_path.resolve())), read(args.inventory), proxy.get("scene", proxy),
                       diagnostic["action"], diagnostic["report"], snapshot=diagnostic.get("snapshot"), **options)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"pairs": len(report["pairs"]), "statuses": [item["status"] for item in report["pairs"]],
                      "motion_requested": False, "execution_ready": False}))
    return 0  # Successful evidence generation, not a collision-free result.


if __name__ == "__main__":
    raise SystemExit(main())
