"""Export collision inventory from a USD file, without Isaac Sim or robot motion."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from collections import Counter

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from backend.world_model.usd_geometry import extract_collision_inventory
from backend.executors.world_model_nodes import FANUC_M710_PRIM_PATH


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("usd_path", type=Path)
    parser.add_argument("--robot-prim", default=FANUC_M710_PRIM_PATH)
    parser.add_argument("--root", default="/World")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--proxy-output", type=Path, help="Validate USD/FK binding and export robot proxy scene plus coverage/binding report")
    args = parser.parse_args()
    from pxr import Usd
    stage = Usd.Stage.Open(str(args.usd_path.resolve()))
    result = extract_collision_inventory(stage, robot_prim_path=args.robot_prim, root_path=args.root)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    if args.proxy_output:
        from backend.world_model.fanuc_usd_proxy import build_fanuc_usd_proxy
        proxy = build_fanuc_usd_proxy(result)
        args.proxy_output.parent.mkdir(parents=True, exist_ok=True)
        args.proxy_output.write_text(json.dumps(proxy, ensure_ascii=False, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(json.dumps({"collider_count": len(result["colliders"]),
                      "roles": dict(Counter(item["role"] for item in result["colliders"])),
                      "missing_joint_colliders": result["missing_joint_colliders"],
                      "robot_joint_collision_coverage_complete": result["robot_joint_collision_coverage_complete"],
                      "execution_ready": False}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
