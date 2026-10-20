"""Offline geometric collision diagnostics for an explicit proxy scene/action."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.world_model.fanuc_collision import check_fanuc_collision


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("input", type=Path, help="JSON object containing action and scene")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--max-step-sec", type=float, default=0.05)
    parser.add_argument("--margin-m", type=float, default=0.01)
    args = parser.parse_args()
    data = json.loads(args.input.read_text(encoding="utf-8"))
    report = check_fanuc_collision(data["action"], data["scene"], max_step_sec=args.max_step_sec,
                                   margin_m=args.margin_m, snapshot=data.get("snapshot"))
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({key: report[key] for key in ("status", "samples_checked", "min_sampled_clearance_m",
                                                  "between_samples_certified", "physical_collision_verified")}, ensure_ascii=False))
    return 0 if report["status"] == "clear" else 1


if __name__ == "__main__":
    raise SystemExit(main())
