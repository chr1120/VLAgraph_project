"""Run with runpy.run_path(absolute_path, run_name='__main__') inside Isaac Sim.

Set VLAGRAPH_WORLD_MODEL_CONFIG to an explicit JSON configuration first.
The repository root is derived from this file, not the current directory.
"""
from pathlib import Path
import builtins
import json
import os
import sys


def startup():
    root = Path(__file__).resolve().parents[2]
    if str(root) not in sys.path:
        sys.path.insert(0, str(root))
    from examples.isaacsim.VlaGraphIsaacSimBridge import VlaGraphIsaacSimBridge, _BRIDGE_REGISTRY_KEY
    from examples.isaacsim.mobile_world_model_bridge import attach_world_model

    config_path = os.environ.get("VLAGRAPH_WORLD_MODEL_CONFIG")
    if not config_path:
        raise ValueError("Set VLAGRAPH_WORLD_MODEL_CONFIG to your reviewed robot/map JSON file")
    config = json.loads(Path(config_path).read_text(encoding="utf-8-sig"))
    bridge = VlaGraphIsaacSimBridge()
    attach_world_model(bridge, config)
    previous = getattr(builtins, _BRIDGE_REGISTRY_KEY, None)
    if previous is not None:
        previous.stop()
    setattr(builtins, _BRIDGE_REGISTRY_KEY, bridge)
    bridge.start()
    return bridge


if __name__ == "__main__":
    bridge = startup()
