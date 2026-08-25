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

from src.pid_controller import PIDController, PIDGains, WallCenteringPID
from src.robot_system import RobotSystem
from src.sensor_pipeline import RobotSensorSnapshot


class TestPIDController(unittest.TestCase):
    def test_pid_deadband(self):
        gains = PIDGains(kp=1.0, deadband=20.0)
        pid = PIDController(gains)
        # Within deadband (error < 20) -> output 0.0
        self.assertEqual(pid.compute(15.0), 0.0)
        self.assertEqual(pid.compute(-19.0), 0.0)
        # Outside deadband -> non-zero
        self.assertGreater(pid.compute(25.0), 0.0)

    def test_pid_clamping(self):
        gains = PIDGains(kp=10.0, max_output=0.5, min_output=-0.5)
        pid = PIDController(gains)
        out = pid.compute(100.0)
        self.assertEqual(out, 0.5)
        out_neg = pid.compute(-100.0)
        self.assertEqual(out_neg, -0.5)


class TestStep3WallCentering8Cases(unittest.TestCase):
    def setUp(self):
        self.wall_pid = WallCenteringPID(
            nominal_side_dist_mm=140.0,
            tolerance_mm=20.0,  # 2cm
            front_target_mm=150.0,
        )

    # 1. Front Wall Cases
    def test_case_1_1_front_and_both_walls(self):
        """Case 1.1: Front Wall + Both Side Walls (|L-R| < 2cm)."""
        # Robot shifted right (L=160, R=120) -> error = R - L = -40mm (strafe left, vy < 0)
        snap = RobotSensorSnapshot(
            sharp_left_mm=160.0,
            sharp_left_valid=True,
            sharp_right_mm=120.0,
            sharp_right_valid=True,
            tof_filtered_mm=250.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 11)
        self.assertEqual(err_y, -40.0)

        # Centered: |L-R| = 10mm (< 20mm) -> deadband gives vy = 0
        snap_centered = RobotSensorSnapshot(
            sharp_left_mm=145.0,
            sharp_left_valid=True,
            sharp_right_mm=135.0,
            sharp_right_valid=True,
            tof_filtered_mm=250.0,
            tof_valid=True,
        )
        _, vy, _, _, case_id, err_centered = self.wall_pid.compute_control_speeds(snap_centered, 0.0)
        self.assertEqual(case_id, 11)
        self.assertEqual(err_centered, -10.0)
        self.assertEqual(vy, 0.0)  # within deadband

    def test_case_1_2_front_and_left_wall(self):
        """Case 1.2: Front Wall + Left Wall Only (L +- 2cm)."""
        # L is 180mm, nominal is 140mm -> error = Nominal - L = 140 - 180 = -40mm (strafe left)
        snap = RobotSensorSnapshot(
            sharp_left_mm=180.0,
            sharp_left_valid=True,
            sharp_right_mm=500.0,  # Open right
            sharp_right_valid=False,
            tof_filtered_mm=200.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 12)
        self.assertEqual(err_y, -40.0)

    def test_case_1_3_front_and_right_wall(self):
        """Case 1.3: Front Wall + Right Wall Only (R +- 2cm)."""
        # R is 100mm, nominal is 140mm -> robot too close to right, error = R - Nominal = 100 - 140 = -40mm (strafe left)
        snap = RobotSensorSnapshot(
            sharp_left_mm=500.0,  # Open left
            sharp_left_valid=False,
            sharp_right_mm=100.0,
            sharp_right_valid=True,
            tof_filtered_mm=200.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 13)
        self.assertEqual(err_y, -40.0)

    def test_case_1_4_front_and_no_side_walls(self):
        """Case 1.4: Front Wall + No Side Walls."""
        snap = RobotSensorSnapshot(
            sharp_left_mm=500.0,
            sharp_left_valid=False,
            sharp_right_mm=500.0,
            sharp_right_valid=False,
            tof_filtered_mm=200.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 14)
        self.assertEqual(err_y, 0.0)

    # 2. No Front Wall Cases
    def test_case_2_1_no_front_and_both_walls(self):
        """Case 2.1: No Front Wall + Both Side Walls (|L-R| < 2cm)."""
        # L=110, R=170 (closer to left) -> error = R - L = 170 - 110 = +60mm (strafe right, vy > 0)
        snap = RobotSensorSnapshot(
            sharp_left_mm=110.0,
            sharp_left_valid=True,
            sharp_right_mm=170.0,
            sharp_right_valid=True,
            tof_filtered_mm=800.0,  # Open front
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 21)
        self.assertEqual(err_y, 60.0)

    def test_case_2_2_no_front_and_left_wall(self):
        """Case 2.2: No Front Wall + Left Wall Only."""
        # L=120, Nominal=140 -> error = Nominal - L = 140 - 120 = +20mm (strafe right)
        snap = RobotSensorSnapshot(
            sharp_left_mm=120.0,
            sharp_left_valid=True,
            sharp_right_mm=600.0,
            sharp_right_valid=False,
            tof_filtered_mm=1000.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 22)
        self.assertEqual(err_y, 20.0)

    def test_case_2_3_no_front_and_right_wall(self):
        """Case 2.3: No Front Wall + Right Wall Only."""
        # R=180, Nominal=140 -> error = R - Nominal = 180 - 140 = +40mm (strafe right)
        snap = RobotSensorSnapshot(
            sharp_left_mm=600.0,
            sharp_left_valid=False,
            sharp_right_mm=180.0,
            sharp_right_valid=True,
            tof_filtered_mm=1000.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 23)
        self.assertEqual(err_y, 40.0)

    def test_case_2_4_open_space(self):
        """Case 2.4: Open Space."""
        snap = RobotSensorSnapshot(
            sharp_left_mm=None,
            sharp_left_valid=False,
            sharp_right_mm=None,
            sharp_right_valid=False,
            tof_filtered_mm=1500.0,
            tof_valid=True,
        )
        err_y, case_name, case_id = self.wall_pid.compute_lateral_error(snap)
        self.assertEqual(case_id, 24)
        self.assertEqual(err_y, 0.0)


class TestStep3GridNavigationIntegration(unittest.TestCase):
    def setUp(self):
        self.temp_dir = Path(tempfile.mkdtemp())

    def tearDown(self):
        shutil.rmtree(self.temp_dir)

    def test_step_test_execution(self):
        sys_runner = RobotSystem(
            telemetry_dir=str(self.temp_dir / "telemetry"),
            mock_mode=True,
            sensor_rate_hz=30.0,
        )
        sys_runner.setup_threads()
        sys_runner.thread_2_controller.set_commands([
            "Move Forward: 2 cells",
            "Turn Right (90 deg)",
            "Move Forward: 1 cells"
        ])
        sys_runner.start()
        finished = sys_runner.wait_for_completion(timeout=15.0)
        self.assertTrue(finished)

        # Check telemetry exported
        json_p, csv_p = sys_runner.telemetry.export(custom_name="step3_test_run")
        self.assertTrue(json_p.exists())
        sys_runner.shutdown(save_telemetry=False)


if __name__ == "__main__":
    unittest.main()
