"""Export the frontend's canonical FANUC templates as importable example JSON."""
from pathlib import Path
import argparse

ROOT = Path(__file__).resolve().parents[1]
TEMPLATES = {
    "fanuc-m710.json": "isaacsim_fanuc_m710ic20l_graph.json",
    "fanuc-m710-preflight.json": "fanuc_m710_preflight_graph.json",
    "fanuc-m710-snapshot.json": "fanuc_m710_snapshot_graph.json",
    "fanuc-m710-observe.json": "fanuc_m710_world_observe_graph.json",
}


def export(*, check=False):
    for source_name, target_name in TEMPLATES.items():
        source = ROOT / "frontend/src/templates" / source_name
        target = ROOT / "examples/graphs" / target_name
        if check:
            if not target.is_file() or source.read_bytes() != target.read_bytes():
                raise ValueError(f"Stale example: {target_name}; run scripts/export_graph_templates.py")
        else:
            target.write_bytes(source.read_bytes())


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--check", action="store_true")
    export(check=parser.parse_args().check)
