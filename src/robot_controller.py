#!/usr/bin/env python3
"""Thread 2: Robot Controller and Grid-by-Grid Motion Execution with PID Control.

Step 3 Requirements (REQ.md):
- เดินทีละ Grid (60x60 cm, Wall 7.5 cm)
- PID Control ปรับการเคลื่อนที่แกน Y ให้อยู่ตรงกลางระหว่างกำแพง (ดึงค่าจาก Thread 1)
- 8 Wall Alignment Cases (มี/ไม่มีกำแพงหน้า x 2ข้าง/ซ้าย/ขวา/ไม่มี)
"""

import json
import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Tuple

try:
    from .pid_controller import WallCenteringPID
    from .sensor_pipeline import RobotSensorSnapshot, SensorHub
except (ImportError, ValueError):
    from pid_controller import WallCenteringPID
    from sensor_pipeline import RobotSensorSnapshot, SensorHub


class MockRobotActuators:
    """Mock actuators for running simulation or dry-runs without physical EP."""

    def __init__(self, sensor_collector: Optional[Any] = None, speed_mult: float = 2.0):
        self.collector = sensor_collector
        self.speed_mult = max(0.1, speed_mult)
        self.cur_x = 0.0
        self.cur_y = 0.0
        self.cur_yaw = 0.0
        self.gripper_state = "closed"

    def move(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, xy_speed: float = 0.5, z_speed: float = 30.0):
        """Simulates linear movement (x forward/backward, y left/right, z rotation deg)."""
        duration = max(0.1, ((abs(x) + abs(y)) / max(0.1, xy_speed) + abs(z) / max(1.0, z_speed)) / self.speed_mult)
        steps = max(2, int(duration * 20))
        dx = x / steps
        dy = y / steps
        yaw_change = 180.0 if abs(z) == 180.0 else -z
        dz = yaw_change / steps

        step_delay = duration / steps
        for _ in range(steps):
            time.sleep(step_delay)
            self.cur_x += dx
            self.cur_y += dy
            self.cur_yaw = (self.cur_yaw + dz + 180.0) % 360.0 - 180.0
            if self.collector:
                self.collector.inject_mock_data(
                    sharp_left_adc=350.0,
                    sharp_right_adc=350.0,
                    tof_dist=500.0,
                    yaw=self.cur_yaw,
                    pos_x=self.cur_x,
                    pos_y=self.cur_y,
                    gripper_status=self.gripper_state,
                )

    def drive_speed(self, x: float = 0.0, y: float = 0.0, z: float = 0.0, timeout: Optional[float] = None):
        """Simulates holonomic drive_speed."""
        dt = timeout if timeout else 0.05
        # Simulate position shift
        rad = math.radians(self.cur_yaw)
        self.cur_x += (x * math.cos(rad) - y * math.sin(rad)) * dt
        self.cur_y += (x * math.sin(rad) + y * math.cos(rad)) * dt
        self.cur_yaw = (self.cur_yaw - z * dt + 180.0) % 360.0 - 180.0

        if self.collector:
            self.collector.inject_mock_data(
                sharp_left_adc=350.0,
                sharp_right_adc=350.0,
                tof_dist=500.0,
                yaw=self.cur_yaw,
                pos_x=self.cur_x,
                pos_y=self.cur_y,
                gripper_status=self.gripper_state,
            )
        if timeout:
            time.sleep(timeout / self.speed_mult)

    def stop(self):
        self.drive_speed(0, 0, 0)

    def open_gripper(self, power: int = 50):
        time.sleep(0.15 / self.speed_mult)
        self.gripper_state = "opened"
        if self.collector:
            self.collector.inject_mock_data(gripper_status=self.gripper_state)

    def close_gripper(self, power: int = 50):
        time.sleep(0.15 / self.speed_mult)
        self.gripper_state = "closed"
        if self.collector:
            self.collector.inject_mock_data(gripper_status=self.gripper_state)

    def pause_gripper(self):
        self.gripper_state = "normal"
        if self.collector:
            self.collector.inject_mock_data(gripper_status=self.gripper_state)


class RobotControllerThread(threading.Thread):
    """Thread 2: Consumes filtered sensor data from Thread 1 (SensorHub) and
    executes step-by-step Grid navigation with PID centering.
    """

    def __init__(
        self,
        sensor_hub: SensorHub,
        robot: Any = None,
        mock_mode: bool = False,
        grid_size_m: float = 0.60,
        nominal_side_dist_mm: float = 140.0,
        base_speed: float = 0.25,
    ):
        super().__init__(name="RobotControllerThread-2", daemon=True)
        self.sensor_hub = sensor_hub
        self.robot = robot
        self.mock_mode = mock_mode
        self.grid_size_m = grid_size_m
        self.base_speed = base_speed
        self.target_heading_deg = 0.0

        self._running = threading.Event()
        self._pause_event = threading.Event()
        self._pause_event.set()

        # Step 3 PID controller for 8 wall cases
        self.wall_pid = WallCenteringPID(
            nominal_side_dist_mm=nominal_side_dist_mm,
            tolerance_mm=20.0,  # 2cm tolerance as per REQ
        )

        # Chassis watchdog: drive_speed is a *continuous* command - the wheels
        # keep the last commanded velocity until told otherwise.  Passing a
        # timeout makes the SDK stop the chassis if no new speed command
        # arrives within this many seconds, so a stalled control loop brakes
        # instead of driving into a wall.  (The timer lives in this process, so
        # it does not survive a hard kill - see the atexit stop in panel/.)
        self.drive_watchdog_sec: float = 0.4

        self.mock_actuator = MockRobotActuators() if mock_mode else None
        self.command_queue: List[str] = []
        self.current_action: str = "IDLE"
        self.current_step: int = 0
        self.plan_completed: bool = False
        self.step_pause_sec: float = 0.05 if mock_mode else 1.0  # 1.0s pause between states on live robot

    def load_plan_from_file(self, json_path: str):
        """Loads execution plan commands from robot_map_plan.json."""
        path = Path(json_path)
        if not path.exists():
            data_candidate = Path("data") / path.name
            if data_candidate.exists():
                path = data_candidate
            elif (Path("..") / "data" / path.name).exists():
                path = Path("..") / "data" / path.name
        with path.open("r", encoding="utf-8") as f:
            data = json.load(f)
        self.command_queue = data.get("commands", [])
        print(f"[RobotControllerThread] Loaded {len(self.command_queue)} commands from {path}")

    def set_commands(self, commands: List[str]):
        self.command_queue = list(commands)

    # -----------------------------------------------------------------------
    # Actuator Low-level Drivers
    # -----------------------------------------------------------------------

    def drive_speed(self, vx: float, vy: float, vz: float):
        """Drives robot chassis with holonomic velocities (m/s, m/s, deg/s)."""
        if self.mock_mode or self.robot is None:
            if self.mock_actuator:
                self.mock_actuator.drive_speed(x=vx, y=vy, z=vz, timeout=0.05)
        else:
            try:
                # RoboMaster EP chassis drive_speed
                # Note: drive_speed accepts x=vx(m/s), y=vy(m/s), z=vz(deg/s)
                # timeout= arms the SDK's auto-stop watchdog (chassis.py).
                self.robot.chassis.drive_speed(
                    x=vx, y=vy, z=vz, timeout=self.drive_watchdog_sec
                )
            except Exception as e:
                print(f"[Controller] drive_speed error: {e}")

    def stop_chassis(self):
        """Stops chassis motors."""
        if self.mock_mode or self.robot is None:
            if self.mock_actuator:
                self.mock_actuator.stop()
        else:
            try:
                self.robot.chassis.drive_speed(x=0, y=0, z=0)
            except Exception:
                pass

    def operate_gripper(self, action_name: str, power: int = 50):
        """Opens or closes Gripper."""
        self.current_action = f"GRIPPER_{action_name.upper()}"
        print(f"[Controller] Gripper action: {action_name}...")

        if self.mock_mode or self.robot is None:
            if self.mock_actuator:
                if action_name.lower() == "open":
                    self.mock_actuator.open_gripper(power=power)
                elif action_name.lower() == "close":
                    self.mock_actuator.close_gripper(power=power)
                else:
                    self.mock_actuator.pause_gripper()
        else:
            if hasattr(self.robot, "gripper"):
                if action_name.lower() == "open":
                    self.robot.gripper.open(power=power)
                elif action_name.lower() == "close":
                    self.robot.gripper.close(power=power)
                else:
                    self.robot.gripper.pause()
                time.sleep(0.5)

    # -----------------------------------------------------------------------
    # Step 3: Grid-by-Grid Navigation & PID Centering
    # -----------------------------------------------------------------------

    def align_at_cell_center(self, duration_sec: float = 0.4):
        """In-place PID fine alignment to ensure robot is centered (|L-R| < 2cm or L/R +- 2cm)."""
        t_end = time.monotonic() + duration_sec
        self.wall_pid.reset()

        while time.monotonic() < t_end and self._running.is_set():
            state = self.sensor_hub.get_latest_state()
            _, vy, vz, case_name, case_id, err_y = self.wall_pid.compute_control_speeds(
                state=state,
                target_yaw_deg=self.target_heading_deg,
                base_vx=0.0,
                dt=0.05,
            )

            # If error is within 20mm deadband, vy will be 0.0
            if abs(err_y) < 20.0 and abs(vz) < 1.0:
                # Already centered!
                self.stop_chassis()
                break

            self.drive_speed(vx=0.0, vy=vy, vz=vz)
            time.sleep(0.05)

        self.stop_chassis()

    def navigate_single_grid_step(self, step_idx: int = 1, total_steps: int = 1):
        """Navigates exactly 1 grid cell (60 cm) using closed-loop PID lateral centering."""
        self.current_action = f"NAVIGATE_GRID_{step_idx}_OF_{total_steps}"
        print(f"\n  [Grid Step {step_idx}/{total_steps}] Moving 1 cell forward ({self.grid_size_m:.2f} m)...")

        # Snapshot start position & orientation from Thread 1
        initial_state = self.sensor_hub.get_latest_state()
        start_x, start_y = initial_state.pos_x, initial_state.pos_y

        self.wall_pid.reset()
        dist_traveled = 0.0
        control_loop_hz = 20.0
        dt = 1.0 / control_loop_hz
        max_duration = (self.grid_size_m / max(0.1, self.base_speed)) * 1.8 + 1.5
        t_start = time.monotonic()

        last_case_id = None

        while dist_traveled < self.grid_size_m and self._running.is_set():
            loop_t0 = time.monotonic()
            if (loop_t0 - t_start) > max_duration:
                print(f"  [Warning] Grid step reached timeout limit ({max_duration:.1f}s).")
                break

            # 1. Pull clean, pre-filtered sensor snapshot from Thread 1 (Zero hardware overhead)
            state = self.sensor_hub.get_latest_state()

            # 2. Update forward distance traveled along target heading (60 cm / 0.60 m)
            dx = state.pos_x - start_x
            dy = state.pos_y - start_y
            rad = math.radians(self.target_heading_deg)
            forward_step_m = dx * math.cos(rad) + dy * math.sin(rad)
            dist_traveled = max(0.0, forward_step_m if not self.mock_mode else math.sqrt(dx * dx + dy * dy))

            # 3. Check remaining distance to grid cell boundary (60 cm)
            rem_dist = self.grid_size_m - dist_traveled
            cur_vx = self.base_speed
            if rem_dist < 0.12:
                # Decelerate smoothly at end of 60cm cell
                cur_vx = max(0.08, self.base_speed * (rem_dist / 0.12))

            # 4. Compute PID control commands for the 8 wall cases
            vx, vy, vz, case_name, case_id, err_y = self.wall_pid.compute_control_speeds(
                state=state,
                target_yaw_deg=self.target_heading_deg,
                base_vx=cur_vx,
                dt=dt,
            )

            if case_id != last_case_id:
                print(f"  [PID Centering] {case_name} | Lat Err: {err_y:+.1f} mm | vy: {vy:+.2f} m/s | L: {state.sharp_left_mm} mm | R: {state.sharp_right_mm} mm")
                last_case_id = case_id

            # 5. Drive chassis holonomically (Forward vx + Lateral correction vy + Yaw lock vz)
            self.drive_speed(vx=vx, vy=vy, vz=vz)

            # Check if front wall reached before full 60cm (safety)
            # Only stop on ToF if robot has already moved at least 0.35m or if dangerously close (< 90mm)
            has_front, _, _ = self.wall_pid.classify_wall_state(state)
            if has_front and state.tof_filtered_mm is not None:
                if (dist_traveled >= 0.35 and state.tof_filtered_mm <= self.wall_pid.front_target_mm) or (state.tof_filtered_mm < 90.0):
                    print(f"  [Front Wall Reach] Stopped at ToF={state.tof_filtered_mm:.1f} mm (Target: {self.wall_pid.front_target_mm} mm)")
                    break

            # Sleep remaining loop dt
            loop_elapsed = time.monotonic() - loop_t0
            if dt > loop_elapsed:
                time.sleep(dt - loop_elapsed)

        self.stop_chassis()

        # Perform fine centering alignment at cell center
        self.align_at_cell_center(duration_sec=0.3)

        # Log completion state
        end_state = self.sensor_hub.get_latest_state()
        diff_str = f"{end_state.sharp_diff_mm:+.1f} mm" if (end_state.wall_left_detected and end_state.wall_right_detected) else "N/A"
        print(f"  [Grid Step {step_idx}/{total_steps} Done] Local Pos: ({end_state.pos_x:+.2f}m, {end_state.pos_y:+.2f}m) | Yaw: {end_state.yaw:+.1f}° | Sharp L: {end_state.sharp_left_mm} mm | R: {end_state.sharp_right_mm} mm | Diff: {diff_str} | ToF: {end_state.tof_filtered_mm} mm")

    def move_forward_grid(self, cells: int = 1):
        """Executes multi-cell forward motion grid-by-grid with closed-loop PID centering."""
        print(f"\n[Controller] Starting {cells}-Grid Forward Motion with Step 3 PID...")
        for i in range(1, cells + 1):
            if not self._running.is_set():
                break
            self.navigate_single_grid_step(step_idx=i, total_steps=cells)
            if i < cells and self.step_pause_sec > 0:
                print(f"[Controller] ⏸️ Pausing {self.step_pause_sec:.1f}s before next grid step...")
                time.sleep(self.step_pause_sec)

    def turn_to_relative(self, deg: float, speed: float = 45.0):
        """Closed-loop relative in-place turn (+90 Left, -90 Right, 180 Around)."""
        # In DJI SDK: z=+90 rotates CCW (yaw becomes -90°), z=-90 rotates CW (yaw becomes +90°)
        expected_yaw_delta = -deg if abs(deg) <= 90.0 else deg
        self.target_heading_deg = (self.target_heading_deg + expected_yaw_delta + 180.0) % 360.0 - 180.0
        self.current_action = f"TURN_{deg:+.0f}_DEG"
        dir_name = "Left (เลี้ยวซ้าย z=+90)" if deg > 0 else ("Right (เลี้ยวขวา z=-90)" if deg < 0 else "Around (กลับหลัง z=180)")
        print(f"\n[Controller] 🔄 Executing Turn {dir_name}: z={deg:+.0f}° -> Target Heading: {self.target_heading_deg:.0f}°...")

        if self.mock_mode or self.robot is None:
            if self.mock_actuator:
                self.mock_actuator.move(z=deg, z_speed=speed)
            else:
                time.sleep(abs(deg) / max(1.0, speed))
        else:
            # Execute turn with SDK chassis.move
            action = self.robot.chassis.move(x=0, y=0, z=deg, z_speed=speed)
            action.wait_for_completed()

        # Stop chassis and reset PID states cleanly
        self.stop_chassis()
        self.wall_pid.reset()
        time.sleep(0.20)

        # Snap target heading to nearest 90-deg grid axis of current yaw
        end_state = self.sensor_hub.get_latest_state()
        if not self.mock_mode and end_state.yaw is not None:
            snapped_target = round(end_state.yaw / 90.0) * 90.0
            self.target_heading_deg = (snapped_target + 180.0) % 360.0 - 180.0

        print(f"[Controller] ✅ Turn Completed: Current Yaw = {end_state.yaw:+.1f}° (Target Grid Heading = {self.target_heading_deg:.0f}°)\n")

    def turn_left(self, deg: float = 90.0, speed: float = 45.0):
        """เลี้ยวซ้าย z = +90 องศา."""
        self.turn_to_relative(deg=+abs(deg), speed=speed)

    def turn_right(self, deg: float = 90.0, speed: float = 45.0):
        """เลี้ยวขวา z = -90 องศา."""
        self.turn_to_relative(deg=-abs(deg), speed=speed)

    def turn_around(self, speed: float = 45.0):
        """กลับหลังหัน z = 180 องศา."""
        self.turn_to_relative(deg=180.0, speed=speed)

    def emergency_stop(self):
        """Stops all robot motion immediately."""
        self.current_action = "EMERGENCY_STOP"
        self.stop_chassis()

    def start_running(self):
        self._running.set()
        self.start()

    def stop_running(self):
        self._running.clear()
        self.emergency_stop()

    def pause(self):
        self._pause_event.clear()

    def resume(self):
        self._pause_event.set()

    def execute_command(self, cmd_text: str):
        """Parses and executes a command string from plan."""
        cmd = cmd_text.strip()
        print(f"\n==================================================")
        print(f"[Thread 2 Action] Executing: {cmd}")
        print(f"==================================================")

        if "Move Forward:" in cmd:
            parts = cmd.split("Move Forward:")
            cells = int(parts[1].replace("cells", "").replace("cell", "").strip())
            self.move_forward_grid(cells=cells)
        elif "Turn Right (90 deg)" in cmd:
            self.turn_right()
        elif "Turn Left (90 deg)" in cmd:
            self.turn_left()
        elif "Turn Around (180 deg)" in cmd:
            self.turn_around()
        elif "Gripper Open" in cmd:
            self.operate_gripper("open")
        elif "Gripper Close" in cmd:
            self.operate_gripper("close")
        else:
            print(f"  [Warning] Unknown command format: {cmd}")

        # Sleep 1.0s before proceeding to next state
        if self.step_pause_sec > 0:
            print(f"[Controller] ⏸️ Pausing {self.step_pause_sec:.1f}s before next state...")
            time.sleep(self.step_pause_sec)

    def run(self):
        """Thread 2 main execution loop."""
        while self._running.is_set():
            self._pause_event.wait()

            if self.current_step < len(self.command_queue):
                cmd = self.command_queue[self.current_step]
                self.execute_command(cmd)
                self.current_step += 1
            else:
                if not self.plan_completed:
                    self.plan_completed = True
                    self.current_action = "COMPLETED"
                    print("\n[RobotControllerThread] All plan commands executed successfully with Step 3 PID!")
                time.sleep(0.1)
