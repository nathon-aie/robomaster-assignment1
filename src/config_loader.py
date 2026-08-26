"""Configuration loader for RoboMaster EP Autonomous Navigation System."""

import os
from pathlib import Path
from typing import Any, Dict, Optional

try:
    import yaml
except ImportError:
    yaml = None


DEFAULT_SETTINGS: Dict[str, Any] = {
    "robot": {
        "conn_type": "ap",
        "mock_mode": False,
    },
    "sensor_pipeline": {
        "sensor_rate_hz": 20.0,
        "calibration_file": "calibration_output/calibration.json",
        "wall_detect_side_mm": 280.0,
        "wall_detect_front_mm": 350.0,
        "sharp_raw_min": 50.0,
        "sharp_raw_max": 900.0,
        "median_window": 5,
        "ema_alpha": 0.35,
    },
    "controller": {
        "cell_size_m": 0.60,
        "cruising_speed": 0.25,
        "nominal_side_dist_mm": 140.0,
        "front_wall_stop_dist_mm": 150.0,
        "pause_between_steps_sec": 0.1,
        "lateral_pid": {
            "kp": 0.0018,
            "ki": 0.0,
            "kd": 0.0004,
            "max_output": 0.15,
            "deadband": 3.0,
        },
        "heading_pid": {
            "kp": 0.80,
            "ki": 0.0,
            "kd": 0.10,
            "max_output": 30.0,
        },
        "longitudinal_pid": {
            "kp": 0.0015,
            "ki": 0.0,
            "kd": 0.0002,
            "max_output": 0.20,
        },
    },
    "gripper": {
        "extend_cm": 7.0,
        "lift_cm": 10.0,
        "drop_backup_cm": 30.0,
        "action_delay_sec": 0.5,
    },
    "system": {
        "default_plan_file": "data/robot_map_plan.json",
        "telemetry_logs_dir": "telemetry_logs",
    },
}


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    merged = base.copy()
    for k, v in override.items():
        if k in merged and isinstance(merged[k], dict) and isinstance(v, dict):
            merged[k] = _deep_merge(merged[k], v)
        else:
            merged[k] = v
    return merged


def load_settings(config_path: Optional[str] = None) -> Dict[str, Any]:
    """Loads settings from YAML file, merging with default fallback values."""
    settings = DEFAULT_SETTINGS.copy()
    candidate_paths = [
        Path(config_path) if config_path else None,
        Path("config/settings.yaml"),
        Path("../config/settings.yaml"),
        Path(__file__).resolve().parent.parent / "config" / "settings.yaml",
    ]

    for p in candidate_paths:
        if p and p.exists():
            if yaml:
                try:
                    with p.open("r", encoding="utf-8") as f:
                        user_cfg = yaml.safe_load(f) or {}
                    return _deep_merge(settings, user_cfg)
                except Exception as exc:
                    print(f"[config] Warning: Failed to parse {p}: {exc}")
            break

    return settings
