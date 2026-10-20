"""Run in Isaac Sim Script Editor after installing the FANUC adapter.

Attaches to the existing idle bridge without reloading the extension or physics.
"""
import gc

from isaac_fanuc_m710.vlagraph_world_model import attach_world_model


bridges = []
for item in gc.get_objects():
    if type(item).__module__ != "isaac_fanuc_m710.extension":
        continue
    bridge = getattr(item, "_vlagraph_bridge", None)
    thread = getattr(bridge, "_server_thread", None)
    if bridge is not None and getattr(bridge, "port", None) == 9910 and thread is not None and thread.is_alive():
        if all(bridge is not candidate for candidate in bridges):
            bridges.append(bridge)
if len(bridges) != 1:
    raise RuntimeError(f"Expected one live FANUC bridge on 9910, found {len(bridges)}")
bridge = bridges[0]
if bridge.scene.is_motion_active:
    raise RuntimeError("Wait for FANUC motion to finish before attaching the observer")
attach_world_model(bridge)
print("FANUC world-model snapshot and completion adapter attached; physics was not restarted")
