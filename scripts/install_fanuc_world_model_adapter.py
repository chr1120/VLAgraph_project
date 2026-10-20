"""Install the tested adapter into an existing dedicated FANUC extension.

Creates a backup, adds an idempotent attach hook and copies the adapter source.
Isaac Sim must reload the extension to activate it; this script sends no motion.
"""
from __future__ import annotations

import argparse
import ast
import hashlib
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MARKER = "        # VLAGRAPH_WORLD_MODEL_ADAPTER_V1"
ANCHOR = "        self._last_state_ok_at = 0.0"


def install(extension_root: Path):
    package = extension_root.resolve() / "python/isaac_fanuc_m710"
    target = package / "vlagraph_bridge.py"
    source = ROOT / "examples/isaacsim/fanuc_world_model_bridge.py"
    geometry_source = ROOT / "backend/world_model/usd_geometry.py"
    raw = target.read_bytes()
    text = raw.decode("utf-8")
    newline = "\r\n" if "\r\n" in text else "\n"
    if MARKER not in text:
        if text.count(ANCHOR) != 1:
            raise ValueError("Unknown FANUC bridge version; attach anchor is not unique")
        text = text.replace(ANCHOR, ANCHOR + newline + newline + newline.join([
            MARKER,
            "        from .vlagraph_world_model import attach_world_model",
            "        attach_world_model(self)",
        ]), 1)
    ast.parse(text)
    ast.parse(source.read_text(encoding="utf-8"))
    ast.parse(geometry_source.read_text(encoding="utf-8"))
    backup = target.with_suffix(".py.vlagraph-world-model.bak")
    if not backup.exists():
        backup.write_bytes(raw)
    (package / "vlagraph_usd_geometry.py").write_bytes(geometry_source.read_bytes())
    (package / "vlagraph_world_model.py").write_bytes(source.read_bytes())
    if text.encode("utf-8") != raw:
        target.write_bytes(text.encode("utf-8"))
    return {"installed": str(package / "vlagraph_world_model.py"), "backup": str(backup),
            "sha256": hashlib.sha256(source.read_bytes()).hexdigest(),
            "geometry_sha256": hashlib.sha256(geometry_source.read_bytes()).hexdigest()}


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extension-root", type=Path, required=True)
    print(install(parser.parse_args().extension_root))
