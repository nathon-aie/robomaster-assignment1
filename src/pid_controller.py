#!/usr/bin/env python3
"""PID Controllers and Wall-Centering Logic for Step 3 Grid Navigation.

Step 3 Requirements (REQ.md):
- เดินทีละ Grid (Grid size 60x60 cm, Wall thickness 7.5 cm)
- PID Control ปรับการเคลื่อนที่แกน Y ให้อยู่ตรงกลางระหว่างกำแพง (ดึงค่าจาก Thread 1)
- 8 Cases:
  1. มีกำแพงข้างหน้า (วัดระยะจาก ToF ว่าอยู่ตรงกลาง Grid)
     1.1 มีกำแพง 2 ข้าง: Sharp |L - R| < 2 cm (20 mm)
     1.2 มีกำแพงแค่ข้างซ้าย: Sharp L +- 2 cm จากค่าปกติ
     1.3 มีกำแพงแค่ข้างขวา: Sharp R +- 2 cm จากค่าปกติ
     1.4 ไม่มีกำแพง
  2. ไม่มีกำแพงข้างหน้า (เดินไปข้างหน้า 1 Grid 60 cm)
     2.1 มีกำแพง 2 ข้าง: Sharp |L - R| < 2 cm (20 mm)
     2.2 มีกำแพงแค่ข้างซ้าย: Sharp L +- 2 cm จากค่าปกติ
     2.3 มีกำแพงแค่ข้างขวา: Sharp R +- 2 cm จากค่าปกติ
     2.4 ไม่มีกำแพง
"""

import math
import time
from dataclasses import dataclass
from typing import Optional, Tuple

try:
    from .config_loader import load_settings
    from .sensor_pipeline import RobotSensorSnapshot
except (ImportError, ValueError):
    from config_loader import load_settings
    from sensor_pipeline import RobotSensorSnapshot


@dataclass
class PIDGains:
    """PID Gain parameters."""
    kp: float = 1.0
    ki: float = 0.0
    kd: float = 0.0
    max_output: float = 0.5
    min_output: float = -0.5
    integral_limit: float = 0.2
    deadband: float = 0.0  # Output 0 if |error| < deadband


class PIDController:
    """General purpose PID Controller with Anti-Windup and Deadband."""

    def __init__(self, gains: PIDGains):
        self.gains = gains
        self.reset()

    def reset(self):
        self.integral = 0.0
        self.last_error: Optional[float] = None
        self.last_time: Optional[float] = None

    def compute(self, error: float, dt: Optional[float] = None) -> float:
        # Apply deadband
        if abs(error) < self.gains.deadband:
            return 0.0

        current_time = time.monotonic()
        if dt is None:
            if self.last_time is not None:
                dt = current_time - self.last_time
            else:
                dt = 0.05  # Default nominal dt (20Hz)
        self.last_time = current_time

        # P term
        p_term = self.gains.kp * error

        # I term with Anti-Windup clamping
        if dt > 0:
            self.integral += error * dt
            # Clamp integral
            self.integral = max(
                -self.gains.integral_limit,
                min(self.gains.integral_limit, self.integral),
            )
        i_term = self.gains.ki * self.integral

        # D term
        d_term = 0.0
        if self.last_error is not None and dt > 0:
            d_term = self.gains.kd * (error - self.last_error) / dt
        self.last_error = error

        raw_output = p_term + i_term + d_term
        # Clamp output
        return max(self.gains.min_output, min(self.gains.max_output, raw_output))


class WallCenteringPID:
    """Step 3 PID Controller implementing 8 Wall Decision Cases."""

    def __init__(
        self,
        nominal_side_dist_mm: Optional[float] = None,
        tolerance_mm: Optional[float] = None,
        front_target_mm: Optional[float] = None,
        lateral_kp: Optional[float] = None,
        lateral_ki: Optional[float] = None,
        lateral_kd: Optional[float] = None,
        max_lateral_speed: Optional[float] = None,
        yaw_kp: Optional[float] = None,
        yaw_ki: Optional[float] = None,
        yaw_kd: Optional[float] = None,
        max_yaw_speed: Optional[float] = None,
        config_path: Optional[str] = None,
    ):
        cfg = load_settings(config_path).get("controller", {})
        lat_cfg = cfg.get("lateral_pid", {})
        yaw_cfg = cfg.get("heading_pid", {})

        self.nominal_side_dist_mm = nominal_side_dist_mm if nominal_side_dist_mm is not None else cfg.get("nominal_side_dist_mm", 140.0)
        self.tolerance_mm = tolerance_mm if tolerance_mm is not None else lat_cfg.get("deadband", 3.0)
        self.front_target_mm = front_target_mm if front_target_mm is not None else cfg.get("front_wall_stop_dist_mm", 150.0)
        self.WALL_DETECT_THRESHOLD_MM = 280.0

        # Lateral (Y-axis) PID: error in mm -> vy in m/s
        lat_max = max_lateral_speed if max_lateral_speed is not None else lat_cfg.get("max_output", 0.15)
        self.pid_lateral = PIDController(
            PIDGains(
                kp=lateral_kp if lateral_kp is not None else lat_cfg.get("kp", 0.0018),
                ki=lateral_ki if lateral_ki is not None else lat_cfg.get("ki", 0.0),
                kd=lateral_kd if lateral_kd is not None else lat_cfg.get("kd", 0.0004),
                max_output=lat_max,
                min_output=-lat_max,
                integral_limit=30.0,
                deadband=self.tolerance_mm,
            )
        )

        # Yaw Heading PID: error in deg -> vz in deg/s
        yaw_max = max_yaw_speed if max_yaw_speed is not None else yaw_cfg.get("max_output", 30.0)
        self.pid_yaw = PIDController(
            PIDGains(
                kp=yaw_kp if yaw_kp is not None else yaw_cfg.get("kp", 0.80),
                ki=yaw_ki if yaw_ki is not None else yaw_cfg.get("ki", 0.0),
                kd=yaw_kd if yaw_kd is not None else yaw_cfg.get("kd", 0.10),
                max_output=yaw_max,
                min_output=-yaw_max,
                integral_limit=20.0,
                deadband=0.0,
            )
        )

    def reset(self):
        self.pid_lateral.reset()
        self.pid_yaw.reset()

    def classify_wall_state(self, state: RobotSensorSnapshot) -> Tuple[bool, bool, bool]:
        """Classifies (has_front_wall, has_left_wall, has_right_wall)."""
        has_left = (
            state.sharp_left_valid
            and state.sharp_left_mm is not None
            and state.sharp_left_mm < self.WALL_DETECT_THRESHOLD_MM
        )
        has_right = (
            state.sharp_right_valid
            and state.sharp_right_mm is not None
            and state.sharp_right_mm < self.WALL_DETECT_THRESHOLD_MM
        )
        has_front = (
            state.tof_valid
            and state.tof_filtered_mm is not None
            and state.tof_filtered_mm < 350.0
        )
        return has_front, has_left, has_right

    def compute_lateral_error(self, state: RobotSensorSnapshot) -> Tuple[float, str, int]:
        """Calculates lateral error (mm), description, and case id (1.1 - 2.4)."""
        has_front, has_left, has_right = self.classify_wall_state(state)
        l_mm = state.sharp_left_mm
        r_mm = state.sharp_right_mm

        if has_front:
            # Case 1: Front Wall present
            if has_left and has_right and l_mm is not None and r_mm is not None:
                # Case 1.1: Walls on both sides -> error = R - L
                # If L < R (closer to left), error > 0 -> strafe right (vy > 0)
                # If L > R (closer to right), error < 0 -> strafe left (vy < 0)
                error_y = r_mm - l_mm
                case_name = "Case 1.1: Front Wall + Both Side Walls (|L-R| < 2cm)"
                case_id = 11
            elif has_left and l_mm is not None:
                # Case 1.2: Left wall only -> error = Nominal - L
                # If L < Nominal (too close to left), error > 0 -> strafe right (vy > 0)
                error_y = self.nominal_side_dist_mm - l_mm
                case_name = "Case 1.2: Front Wall + Left Wall Only (L +- 2cm)"
                case_id = 12
            elif has_right and r_mm is not None:
                # Case 1.3: Right wall only -> error = R - Nominal
                # If R < Nominal (too close to right), error < 0 -> strafe left (vy < 0)
                error_y = r_mm - self.nominal_side_dist_mm
                case_name = "Case 1.3: Front Wall + Right Wall Only (R +- 2cm)"
                case_id = 13
            else:
                # Case 1.4: Front wall only, no side walls
                error_y = 0.0
                case_name = "Case 1.4: Front Wall + No Side Walls"
                case_id = 14
        else:
            # Case 2: No Front Wall
            if has_left and has_right and l_mm is not None and r_mm is not None:
                # Case 2.1: Walls on both sides -> error = R - L
                error_y = r_mm - l_mm
                case_name = "Case 2.1: No Front Wall + Both Side Walls (|L-R| < 2cm)"
                case_id = 21
            elif has_left and l_mm is not None:
                # Case 2.2: Left wall only -> error = Nominal - L
                error_y = self.nominal_side_dist_mm - l_mm
                case_name = "Case 2.2: No Front Wall + Left Wall Only (L +- 2cm)"
                case_id = 22
            elif has_right and r_mm is not None:
                # Case 2.3: Right wall only -> error = R - Nominal
                error_y = r_mm - self.nominal_side_dist_mm
                case_name = "Case 2.3: No Front Wall + Right Wall Only (R +- 2cm)"
                case_id = 23
            else:
                # Case 2.4: Open space (no walls)
                error_y = 0.0
                case_name = "Case 2.4: Open Space (No Side Walls)"
                case_id = 24

        return error_y, case_name, case_id

    def compute_control_speeds(
        self,
        state: RobotSensorSnapshot,
        target_yaw_deg: float,
        base_vx: float = 0.0,
        dt: Optional[float] = None,
    ) -> Tuple[float, float, float, str, int, float]:
        """Calculates (vx, vy, vz, case_name, case_id, error_y).

        - vx: Forward speed (m/s)
        - vy: Lateral correction speed (m/s) from PID
        - vz: Angular yaw correction speed (deg/s) from Heading PID
        """
        error_y, case_name, case_id = self.compute_lateral_error(state)

        # Compute lateral correction speed vy
        vy = self.pid_lateral.compute(error_y, dt=dt)

        # Compute heading error (wrap to [-180, 180])
        raw_yaw_diff = target_yaw_deg - state.yaw
        yaw_error = (raw_yaw_diff + 180.0) % 360.0 - 180.0
        vz = self.pid_yaw.compute(yaw_error, dt=dt)

        # Longitudinal speed vx: if front wall detected and close, decelerate
        vx = base_vx
        has_front, _, _ = self.classify_wall_state(state)
        if has_front and state.tof_filtered_mm is not None:
            dist_to_stop = state.tof_filtered_mm - self.front_target_mm
            if dist_to_stop <= 20.0:
                vx = 0.0  # Reached front wall stop point
            elif dist_to_stop < 150.0:
                # Proportional deceleration near front wall
                vx = min(vx, max(0.05, base_vx * (dist_to_stop / 150.0)))

        return vx, vy, vz, case_name, case_id, error_y
