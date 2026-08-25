#!/usr/bin/env python3
"""Shim for map.py forwarding to src/map_planner.py."""
import importlib.util
from pathlib import Path
import sys

_src_file = Path(__file__).resolve().parent / "src" / "map_planner.py"
_spec = importlib.util.spec_from_file_location("src.map_planner", str(_src_file))
_mod = importlib.util.module_from_spec(_spec)
sys.modules["src.map_planner"] = _mod
_spec.loader.exec_module(_mod)

globals().update({k: v for k, v in _mod.__dict__.items() if not k.startswith("__")})

if __name__ == "__main__":
    _mod.main()