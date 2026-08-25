#!/usr/bin/env python3
"""Shim for calibrate.py forwarding to src/calibrate.py."""
import importlib.util
from pathlib import Path
import sys

_src_file = Path(__file__).resolve().parent / "src" / "calibrate.py"
_spec = importlib.util.spec_from_file_location("src.calibrate", str(_src_file))
_mod = importlib.util.module_from_spec(_spec)
sys.modules["src.calibrate"] = _mod
_spec.loader.exec_module(_mod)

# Export all symbols from src.calibrate
globals().update({k: v for k, v in _mod.__dict__.items() if not k.startswith("__")})

if __name__ == "__main__":
    sys.exit(_mod.main())
