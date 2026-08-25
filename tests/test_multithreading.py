import json
import math
import os
import shutil
import sys
import tempfile
import time
import unittest
from pathlib import Path

# Add src and root to path
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "src"))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from src.robot_controller import MockRobotActuators, RobotControllerThread
from src.robot_system import RobotSystem
from src.sensor_pipeline import (
    CalibrationManager,
    ExponentialMovingAverageFilter,
    MedianFilter,
    MovingAverageFilter,
    OutlierRejectionFilter,
    RobotSensorSnapshot,
    SensorCollectorThread,
    SensorFilterPipeline,
    SensorHub,
)
from src.telemetry import TelemetryAnalyzer, TelemetryRecorder


class TestCalibrationManager(unittest.TestCase):
    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())
        self.cal_file = self.temp_dir / "calibration.json"
        cal_data = {
            "schema": 1,
            "sensors": {
                "sharp_left": {
                    "degree": 2,
                    "coefficients": [0.001, -1.0, 300.0],
                    "reference_min_mm": 40.0,
                    "reference_max_mm": 300.0,
                }
            },
        }
        with self.cal_file.open("w", encoding="utf-8") as f:
            json.dump(cal_data, f)
        self.mgr = CalibrationManager(str(self.cal_file))

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_polynomial_eval(self):
        # 0.001 * 100^2 - 1.0 * 100 + 300 = 10 - 100 + 300 = 210
        val = self.mgr.raw_to_mm("sharp_left", 100.0)
        self.assertAlmostEqual(val, 210.0, places=2)

    def test_missing_sensor_fallback(self):
        val = self.mgr.raw_to_mm("tof", 150.0)
        self.assertEqual(val, 150.0)


class TestFilters(unittest.TestCase):
    def test_moving_average(self):
        f = MovingAverageFilter(window_size=3)
        self.assertEqual(f.update(10.0), 10.0)
        self.assertEqual(f.update(20.0), 15.0)
        self.assertEqual(f.update(30.0), 20.0)
        self.assertEqual(f.update(40.0), 30.0)  # (20+30+40)/3

    def test_median_filter(self):
        f = MedianFilter(window_size=5)
        f.update(10.0)
        f.update(11.0)
        f.update(100.0)  # Spike
        f.update(12.0)
        out = f.update(10.0)
        # Sorted [10, 10, 11, 12, 100] -> median is 11
        self.assertEqual(out, 11.0)

    def test_outlier_rejection(self):
        f = OutlierRejectionFilter(min_valid=20.0, max_valid=1000.0)
        val, valid = f.update(500.0)
        self.assertTrue(valid)
        self.assertEqual(val, 500.0)

        val, valid = f.update(5.0)  # Below min
        self.assertFalse(valid)
        self.assertEqual(val, 500.0)  # Held previous


class TestSensorHubThreadSafety(unittest.TestCase):
    def test_concurrent_access(self):
        hub = SensorHub()
        received = []

        def consumer():
            for _ in range(50):
                st = hub.get_latest_state()
                received.append(st.frame_index)
                time.sleep(0.005)

        import threading

        t_cons = threading.Thread(target=consumer)
        t_cons.start()

        for i in range(1, 51):
            snap = RobotSensorSnapshot(frame_index=i, yaw=float(i))
            hub.update_state(snap)
            time.sleep(0.005)

        t_cons.join()
        self.assertGreater(len(received), 0)
        latest = hub.get_latest_state()
        self.assertEqual(latest.frame_index, 50)


class TestFullMultiThreadingExecution(unittest.TestCase):
    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_simulation_workflow(self):
        sys_runner = RobotSystem(
            telemetry_dir=str(self.temp_dir / "telemetry"),
            mock_mode=True,
            sensor_rate_hz=30.0,
        )
        sys_runner.setup_threads()
        # Set small test commands
        sys_runner.thread_2_controller.set_commands([
            "Move Forward: 1 cells",
            "Turn Right (90 deg)",
            "Gripper Open",
            "Gripper Close"
        ])

        sys_runner.start()
        finished = sys_runner.wait_for_completion(timeout=10.0)
        self.assertTrue(finished)

        # Shutdown and verify files
        json_path, csv_path = sys_runner.telemetry.export(custom_name="test_run")
        self.assertTrue(json_path.exists())
        self.assertTrue(csv_path.exists())

        # Test TelemetryAnalyzer
        stats = TelemetryAnalyzer.analyze_file(str(json_path), save_plot=True)
        self.assertIn("sample_count", stats)
        self.assertGreater(stats["sample_count"], 5)
        plot_path = Path(stats["plot_path"])
        self.assertTrue(plot_path.exists())

        sys_runner.shutdown(save_telemetry=False)


if __name__ == "__main__":
    unittest.main()
