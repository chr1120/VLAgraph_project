"""Prepare or apply an offline geometry review; no network or motion requests."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.world_model.fanuc_geometry_review import build_geometry_review, apply_geometry_review


def read(path):
    return json.loads(path.read_text(encoding="utf-8"))


def write(path, value):
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("inventory", type=Path)
    parser.add_argument("proxy", type=Path, help="Proxy export wrapper or scene JSON")
    parser.add_argument("--collision-report", type=Path, help="Matching collision report with complete pair diagnostics")
    parser.add_argument("--hull-inspection", type=Path, help="Attach matching point-convex-hull inspection evidence to a new plan")
    parser.add_argument("--hull-sweep", type=Path, help="Attach matching interval inspection evidence to a new plan")
    parser.add_argument("--review", type=Path, help="Apply explicitly edited review decisions instead of generating a plan")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--scene-output", type=Path, help="Write the reviewed scene (requires --review)")
    args = parser.parse_args()
    if args.scene_output and not args.review:
        parser.error("--scene-output requires --review")
    inventory, proxy = read(args.inventory), read(args.proxy)
    scene = proxy.get("scene", proxy)
    if args.review:
        reviewed = apply_geometry_review(inventory, scene, read(args.review))
        output = {"geometry_review": reviewed["geometry_review"], "geometry_coverage": reviewed["geometry_coverage"],
                  "motion_requested": False, "execution_ready": False}
        if args.scene_output:
            write(args.scene_output, reviewed)
    else:
        collision = read(args.collision_report) if args.collision_report else None
        if collision is not None:
            collision = collision.get("report", collision)
        output = build_geometry_review(inventory, scene, collision_report=collision,
                                       hull_inspection=read(args.hull_inspection) if args.hull_inspection else None,
                                       hull_sweep=read(args.hull_sweep) if args.hull_sweep else None)
    write(args.output, output)
    print(json.dumps({"output": str(args.output), "motion_requested": False, "execution_ready": False}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
