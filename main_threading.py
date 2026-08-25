#!/usr/bin/env python3
"""Shim for main_threading.py forwarding to main.py."""
import importlib.util
from pathlib import Path
import sys

_src_file = Path(__file__).resolve().parent / "main.py"
_spec = importlib.util.spec_from_file_location("main_root", str(_src_file))
_mod = importlib.util.module_from_spec(_spec)
sys.modules["main_root"] = _mod
_spec.loader.exec_module(_mod)

if __name__ == "__main__":
    sys.exit(_mod.main())
