#!/usr/bin/env python3
"""Simulation engine: a virtual RoboMaster driving a hidden ground-truth map.

The simulated robot lives in the *same* local metric frame as the real one
(``x`` forward, ``y`` to the right, ``yaw`` clockwise-positive), so the
coordinate transform, the state tracker, the mapper and the navigator are
shared verbatim between Simulation mode and Real Robot mode.

The ground-truth map is never handed to the mapping code - only the simulated
sensors may look at it.
"""

import copy
import math
import random
import threading
import time

from .geometry import RobotPose, wrap180
from .sensors import raycast_cells

SPEED_STEPS = (0.5, 1.0, 2.0, 5.0, 10.0)


class SimRobot(object):
    """Kinematic robot model: drives forward and turns in place, and bumps walls."""

    def __init__(self, ground_truth, transform, start_cell=(0, 0), start_dir=0,
                 base_speed_mps=0.25, turn_speed_dps=45.0, drift_deg_per_m=0.0,
                 centering_error_m=0.02, centering_error_deg=1.0, seed=None):
        self.ground_truth = ground_truth
        self.transform = transform
        self.base_speed = base_speed_mps
        self.turn_speed = turn_speed_dps
        # ponytail: a real chassis drifts; keep the knob even though default is 0
        self.drift_deg_per_m = drift_deg_per_m
        # The real robot closes the loop at every cell: WallCenteringPID centres
        # it to within ~2 cm and turn_to_relative snaps the heading to the grid
        # axis.  Model that, otherwise the simulated robot drifts in a way the
        # physical one never does.  Both residuals stay tunable.
        self.centering_error_m = centering_error_m
        self.centering_error_deg = centering_error_deg
        self._rng = random.Random(seed)

        self._lock = threading.RLock()
        self._x = 0.0
        self._y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._yaw_rate = 0.0
        self._command = None
        self._blocked = False
        self._collisions = 0
        self.place(start_cell, start_dir)

    # ------------------------------------------------------------------- setup
    def place(self, cell, direction=0):
        """Teleports the robot and re-bases the transform so odometry reads (0,0,0)."""
        with self._lock:
            self.transform.rebase(cell, direction * 90.0)
            self._x = 0.0
            self._y = 0.0
            self._yaw = 0.0
            self._vx = self._vy = self._yaw_rate = 0.0
            self._command = None
            self._blocked = False

    # ------------------------------------------------------------------- state
    def pose(self):
        with self._lock:
            return RobotPose(x_m=self._x, y_m=self._y, yaw_deg=self._yaw)

    def velocity_xy(self):
        with self._lock:
            return (self._vx, self._vy)

    def yaw_rate(self):
        with self._lock:
            return self._yaw_rate

    def is_busy(self):
        with self._lock:
            return self._command is not None

    def blocked(self):
        with self._lock:
            return self._blocked

    # ---------------------------------------------------------------- commands
    def command_move(self, distance_m):
        with self._lock:
            self._command = {"type": "move", "remaining": float(distance_m)}
            self._blocked = False

    def command_turn(self, degrees):
        with self._lock:
            self._command = {"type": "turn", "remaining": float(degrees)}
            self._blocked = False

    def stop(self):
        with self._lock:
            self._command = None
            self._vx = self._vy = self._yaw_rate = 0.0

    # ----------------------------------------------------------------- physics
    def _clearance_m(self):
        """Distance to the nearest wall straight ahead, in metres."""
        map_pose = self.transform.robot_to_map(RobotPose(self._x, self._y, self._yaw))
        cell = self.transform.cell_size_m or 0.60
        dist_cells = raycast_cells(self.ground_truth, map_pose.col, map_pose.row,
                                   map_pose.heading_deg, 6.0)
        return dist_cells * cell

    def _settle(self):
        """Emulates the per-cell wall-centring PID and heading snap."""
        if self.centering_error_m <= 0 and self.centering_error_deg <= 0:
            return
        cell_m = self.transform.cell_size_m or 0.60
        map_pose = self.transform.robot_to_map(RobotPose(self._x, self._y, self._yaw))
        cell = map_pose.cell
        if not self.ground_truth.in_bounds(cell[0], cell[1]):
            return
        jitter_cells = self.centering_error_m / cell_m
        col = cell[0] + self._rng.uniform(-jitter_cells, jitter_cells)
        row = cell[1] + self._rng.uniform(-jitter_cells, jitter_cells)
        heading = round(map_pose.heading_deg / 90.0) * 90.0
        heading += self._rng.uniform(-self.centering_error_deg, self.centering_error_deg)
        settled = self.transform.map_to_robot(col, row, heading)
        self._x, self._y, self._yaw = settled.x_m, settled.y_m, wrap180(settled.yaw_deg)

    def step(self, dt):
        """Advances the model by ``dt`` seconds of simulated time."""
        with self._lock:
            cmd = self._command
            if cmd is None:
                self._vx = self._vy = self._yaw_rate = 0.0
                return

            if cmd["type"] == "turn":
                direction = 1.0 if cmd["remaining"] >= 0 else -1.0
                delta = min(abs(cmd["remaining"]), self.turn_speed * dt) * direction
                self._yaw = wrap180(self._yaw + delta)
                self._yaw_rate = delta / dt if dt else 0.0
                cmd["remaining"] -= delta
                if abs(cmd["remaining"]) < 1e-6:
                    self._command = None
                    self._yaw_rate = 0.0
                    self._settle()
                return

            # Forward motion, with a hard stop before the wall face (~0.26 m).
            step_m = min(cmd["remaining"], self.base_speed * dt)
            clearance = self._clearance_m()
            if clearance - step_m < 0.22:
                step_m = max(0.0, clearance - 0.22)
                if step_m <= 1e-6:
                    self._command = None
                    self._blocked = True
                    self._collisions += 1
                    self._vx = self._vy = 0.0
                    self._settle()
                    return

            rad = math.radians(self._yaw)
            self._x += step_m * math.cos(rad)
            self._y += step_m * math.sin(rad)
            if self.drift_deg_per_m:
                self._yaw = wrap180(self._yaw + self.drift_deg_per_m * step_m)
            self._vx = (step_m * math.cos(rad)) / dt if dt else 0.0
            self._vy = (step_m * math.sin(rad)) / dt if dt else 0.0
            cmd["remaining"] -= step_m
            if cmd["remaining"] <= 1e-6:
                self._command = None
                self._vx = self._vy = 0.0
                self._settle()


class SimulationEngine(threading.Thread):
    """Drives the sim robot, samples the simulated sensors, feeds the shared pipeline.

    Exactly the same downstream chain as the real robot:
    sensors -> RobotStateTracker (pose/trail) -> OccupancyMapper (map).
    """

    def __init__(self, sim_robot, sensor_interface, tracker, mapper=None,
                 tick_hz=50.0, speed=1.0):
        super(SimulationEngine, self).__init__(name="SimulationEngine", daemon=True)
        self.robot = sim_robot
        self.sensors = sensor_interface
        self.tracker = tracker
        self.mapper = mapper
        self.tick_interval = 1.0 / max(1.0, tick_hz)
        self.speed = speed
        self.mapping_enabled = False
        self.on_map_update = None

        self._running = threading.Event()
        self._paused = threading.Event()
        self._paused.set()
        self._sim_time = 0.0

    # --------------------------------------------------------------- lifecycle
    def start_engine(self):
        if self.is_alive():
            return
        self._running.set()
        self.start()

    def stop_engine(self):
        self._running.clear()
        self._paused.set()
        self.robot.stop()

    def pause(self):
        self._paused.clear()
        self.robot.stop()

    def resume(self):
        self._paused.set()

    def is_paused(self):
        return not self._paused.is_set()

    def set_speed(self, speed):
        self.speed = max(0.1, float(speed))

    @property
    def sim_time(self):
        return self._sim_time

    # -------------------------------------------------------------------- loop
    def run(self):
        last = time.monotonic()
        while self._running.is_set():
            self._paused.wait(timeout=0.2)
            if not self._running.is_set():
                break
            now = time.monotonic()
            wall_dt = now - last
            last = now
            if self.is_paused():
                time.sleep(self.tick_interval)
                continue

            dt = wall_dt * self.speed
            # Keep the integrator stable when the host stalls or speed is high.
            remaining = dt
            while remaining > 1e-6:
                slice_dt = min(remaining, 0.02)
                self.robot.step(slice_dt)
                remaining -= slice_dt
            self._sim_time += dt

            reading = self.sensors.read()
            if reading is not None:
                self.tracker.update_from_pose(
                    reading.pose,
                    velocity_xy=reading.velocity_xy,
                    yaw_rate=reading.yaw_rate,
                    timestamp=reading.timestamp,
                    frame_index=reading.frame_index,
                )
                if self.mapping_enabled and self.mapper is not None:
                    update = self.mapper.integrate(reading)
                    if update.changed and self.on_map_update:
                        try:
                            self.on_map_update(update)
                        except Exception as exc:  # pragma: no cover - UI callback
                            print("[SimulationEngine] map callback error: {}".format(exc))

            time.sleep(self.tick_interval)


def ground_truth_from(grid):
    """Snapshot of an edited map, used as the hidden truth for the simulator.

    The Object marker is a *statement about the world*: on the real field the
    operator has physically stood a bottle on that square.  The simulator has
    to materialise it, or the simulated ToF sweeps an empty square and the
    robot reports finding nothing.
    """
    truth = copy.deepcopy(grid)
    truth.mark_all_known()
    if getattr(truth, "object_cell", None) is not None:
        truth.objects.add(truth.object_cell)
    return truth
