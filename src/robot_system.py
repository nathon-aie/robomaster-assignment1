#!/usr/bin/env python3
"""Master Robot System Coordinator for RoboMaster EP Multi-Threading.

Manages Robot connection, Thread 1 (Sensor Collection & Filtering),
Thread 2 (Robot Motion Controller), and Telemetry Logging.
"""

import os
import signal
import sys
import time
from pathlib import Path
from typing import Any, Optional

try:
    from . import calibrate
    from .config_loader import load_settings
    from .robot_controller import RobotControllerThread
    from .sensor_pipeline import CalibrationManager, SensorCollectorThread, SensorHub
    from .telemetry import TelemetryAnalyzer, TelemetryRecorder
except (ImportError, ValueError):
    import calibrate
    from config_loader import load_settings
    from robot_controller import RobotControllerThread
    from sensor_pipeline import CalibrationManager, SensorCollectorThread, SensorHub
    from telemetry import TelemetryAnalyzer, TelemetryRecorder


def normalize_conn_type(conn_type: Optional[str]) -> str:
    """Ensures conn_type string matches DJI SDK's internal singleton ('ap'/'sta'/'rndis').

    DJI RoboMaster SDK uses identity comparisons (`if conn_type is CONNECTION_WIFI_AP:`)
    instead of equality, which fails on non-interned strings loaded from YAML or CLI args.
    """
    c = str(conn_type or "ap").strip().lower()
    if c == "sta":
        return sys.intern("sta")
    elif c == "rndis":
        return sys.intern("rndis")
    return sys.intern("ap")


class RobotSystem:
    """Master orchestrator for Step 2 Multi-Threading Architecture."""

    def __init__(
        self,
        calibration_file: Optional[str] = None,
        telemetry_dir: Optional[str] = None,
        sensor_rate_hz: Optional[float] = None,
        mock_mode: Optional[bool] = None,
        conn_type: Optional[str] = None,
        config_path: Optional[str] = None,
    ):
        cfg = load_settings(config_path)
        r_cfg = cfg.get("robot", {})
        s_cfg = cfg.get("sensor_pipeline", {})
        sys_cfg = cfg.get("system", {})

        self.mock_mode = mock_mode if mock_mode is not None else r_cfg.get("mock_mode", False)
        raw_conn = conn_type if conn_type is not None else r_cfg.get("conn_type", "ap")
        self.conn_type = normalize_conn_type(raw_conn)
        self.robot = None

        # Core subsystems
        cal_path = calibration_file or s_cfg.get("calibration_file", "calibration_output/calibration.json")
        tel_dir = telemetry_dir or sys_cfg.get("telemetry_logs_dir", "telemetry_logs")
        self.sensor_rate_hz = sensor_rate_hz if sensor_rate_hz is not None else s_cfg.get("sensor_rate_hz", 20.0)

        self.calibration_mgr = CalibrationManager(cal_path)
        self.telemetry = TelemetryRecorder(output_dir=tel_dir)
        self.sensor_hub = SensorHub(max_history=1000)

        # Multi-threading workers
        self.thread_1_sensor: Optional[SensorCollectorThread] = None
        self.thread_2_controller: Optional[RobotControllerThread] = None

    def connect_robot(self) -> bool:
        """Initializes connection to RoboMaster EP hardware."""
        if self.mock_mode:
            print("[RobotSystem] Running in SIMULATION / MOCK mode (No physical hardware needed).")
            return True

        conn_str = normalize_conn_type(self.conn_type)
        print(f"[RobotSystem] Connecting to RoboMaster EP via {conn_str.upper()}...")
        try:
            robot_mod = calibrate.load_robot_sdk()
            self.robot = robot_mod.Robot()
            self.robot.initialize(conn_type=conn_str)
            print("[RobotSystem] Successfully connected to RoboMaster EP!")
            return True
        except Exception as exc:
            print(f"[RobotSystem] Connection failed: {exc}")
            if "proxy_addr" in str(exc):
                print("\n⚠️ [สาเหตุที่เป็นไปได้ / Checklist]:")
                print("  1. คอมพิวเตอร์ยังไม่ได้เชื่อมต่อ Wi-Fi เข้ากับหุ่นยนต์ RoboMaster EP (SSID เช่น 'RM_...' หรือ 'EP_...')")
                print("  2. สวิตช์โหมดการเชื่อมต่อหลังตัวหุ่นยนต์ยังไม่ได้สับไปที่ตำแหน่ง Direct (AP Mode)")
                print("  3. หุ่นยนต์ปิดอยู่ หรือแบตเตอรี่หมด")
                print("  💡 ลองตรวจสอบ Wi-Fi ในเครื่องและทดสอบ `ping 192.168.2.1` ดูครับ\n")
            print("[RobotSystem] Switching to MOCK mode fallback.")
            self.mock_mode = True
            self.robot = None
            return False

    def setup_threads(self, plan_file: Optional[str] = None):
        """Spawns Thread 1 and Thread 2 with thread-safe shared memory."""
        # Thread 1: Sensor Collection + Filtering
        self.thread_1_sensor = SensorCollectorThread(
            sensor_hub=self.sensor_hub,
            robot=self.robot,
            calibration_manager=self.calibration_mgr,
            telemetry_recorder=self.telemetry,
            update_rate_hz=self.sensor_rate_hz,
            mock_mode=self.mock_mode,
        )

        # Thread 2: Robot Motion Controller
        self.thread_2_controller = RobotControllerThread(
            sensor_hub=self.sensor_hub,
            robot=self.robot,
            mock_mode=self.mock_mode,
        )

        if self.mock_mode and self.thread_2_controller.mock_actuator:
            self.thread_2_controller.mock_actuator.collector = self.thread_1_sensor

        if plan_file:
            try:
                self.thread_2_controller.load_plan_from_file(plan_file)
            except Exception as e:
                print(f"[RobotSystem] Plan loading warning: {e}")

    def start(self):
        """Starts both Thread 1 and Thread 2."""
        if not self.thread_1_sensor or not self.thread_2_controller:
            self.setup_threads()

        print("[RobotSystem] Starting Thread 1 (Sensor Collection & Filtering)...")
        self.thread_1_sensor.start_collecting()

        # Let sensor buffers warm up
        time.sleep(0.3)

        print("[RobotSystem] Starting Thread 2 (Robot Motion Controller)...")
        self.thread_2_controller.start_running()

    def wait_for_completion(self, timeout: Optional[float] = None) -> bool:
        """Blocks until Thread 2 finishes all plan commands or timeout."""
        start_t = time.time()
        while True:
            if self.thread_2_controller and self.thread_2_controller.plan_completed:
                return True
            if timeout and (time.time() - start_t) > timeout:
                return False
            time.sleep(0.1)

    def shutdown(self, save_telemetry: bool = True, run_analysis: bool = True):
        """Gracefully shuts down both threads and exports run telemetry."""
        print("\n[RobotSystem] Shutting down multi-threading workers...")
        if self.thread_2_controller:
            self.thread_2_controller.stop_running()
        if self.thread_1_sensor:
            self.thread_1_sensor.stop_collecting()

        if self.robot is not None:
            try:
                self.robot.close()
                print("[RobotSystem] RoboMaster SDK connection closed.")
            except Exception:
                pass

        if save_telemetry:
            json_p, csv_p = self.telemetry.export()
            if run_analysis:
                TelemetryAnalyzer.analyze_file(str(json_p), save_plot=True)
