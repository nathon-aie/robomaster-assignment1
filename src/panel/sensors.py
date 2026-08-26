#!/usr/bin/env python3
"""Sensor abstraction shared by the real RoboMaster and the simulator.

    Real RoboMaster sensors  ---\\
                                 >--- SensorInterface ---> mapping / UI
    Simulated sensors        ---/

Only sensors that this project already implements are exposed:

    front  ToF distance sensor      (robot.sensor.sub_distance, mm, ~4 m)
    left   Sharp IR on adapter id1  (sensor_adaptor ADC -> calibration curve)
    right  Sharp IR on adapter id2  (sensor_adaptor ADC -> calibration curve)

There is no rear sensor on this robot, so ``back_mm`` is always ``None`` and
the UI hides it.  No LiDAR, no external ultrasonics, no extra cameras.
"""

import math
import random
import time
from dataclasses import dataclass, field

from .geometry import RobotPose
from .occupancy import OBSTACLE, WALL


@dataclass
class SensorSpec:
    """Static description of one onboard range sensor (used for visualisation)."""

    name: str
    label: str
    angle_deg: float       # bearing relative to robot heading, clockwise
    min_range_m: float
    max_range_m: float
    fov_deg: float

    @property
    def max_range_mm(self):
        return self.max_range_m * 1000.0


#: The three range sensors that physically exist on this build.
SENSOR_SPECS = (
    SensorSpec("front", "Front (ToF)", 0.0, 0.10, 4.00, 15.0),
    SensorSpec("left", "Left (Sharp IR)", -90.0, 0.04, 0.40, 6.0),
    SensorSpec("right", "Right (Sharp IR)", 90.0, 0.04, 0.40, 6.0),
)

SENSOR_BY_NAME = dict((s.name, s) for s in SENSOR_SPECS)


@dataclass
class SensorReading:
    """One synchronised set of range readings plus the pose they were taken at."""

    front_mm: float = None
    left_mm: float = None
    right_mm: float = None
    back_mm: float = None          # no rear sensor on this robot - always None

    front_valid: bool = False
    left_valid: bool = False
    right_valid: bool = False

    wall_front: bool = False
    wall_left: bool = False
    wall_right: bool = False

    pose: RobotPose = field(default_factory=RobotPose)
    velocity_xy: tuple = (0.0, 0.0)
    yaw_rate: float = 0.0
    timestamp: float = 0.0
    monotonic: float = 0.0
    frame_index: int = 0
    source: str = "real"

    def distance(self, name):
        return getattr(self, "{}_mm".format(name), None)

    def is_valid(self, name):
        return bool(getattr(self, "{}_valid".format(name), False))

    def as_display(self):
        """Ordered ``[(label, text)]`` for the debug panel; unavailable sensors dropped."""
        out = []
        for spec in SENSOR_SPECS:
            value = self.distance(spec.name)
            if value is None:
                out.append((spec.label, "--"))
            elif not self.is_valid(spec.name):
                out.append((spec.label, "invalid"))
            else:
                out.append((spec.label, "{:.2f} m".format(value / 1000.0)))
        return out


class SensorInterface(object):
    """Common surface every sensor source implements."""

    source = "abstract"

    def read(self):
        raise NotImplementedError

    def available(self):
        return True

    def close(self):
        pass


# --------------------------------------------------------------------------
# Real hardware
# --------------------------------------------------------------------------

class RealSensorInterface(SensorInterface):
    """Reads the filtered snapshots Thread 1 already publishes on ``SensorHub``.

    No extra hardware calls are made - Thread 1 owns the SDK subscriptions, this
    class only consumes the shared clean state (and pushes it onward through
    ``SensorHub.add_listener`` when a callback is registered).
    """

    source = "real"

    def __init__(self, sensor_hub, wall_threshold_mm=280.0, front_threshold_mm=350.0):
        self.hub = sensor_hub
        self.wall_threshold_mm = wall_threshold_mm
        self.front_threshold_mm = front_threshold_mm
        self._listener = None

    def _to_reading(self, snap):
        return SensorReading(
            front_mm=snap.tof_filtered_mm,
            left_mm=snap.sharp_left_mm,
            right_mm=snap.sharp_right_mm,
            back_mm=None,
            front_valid=bool(snap.tof_valid),
            left_valid=bool(snap.sharp_left_valid),
            right_valid=bool(snap.sharp_right_valid),
            wall_front=bool(snap.wall_front_detected),
            wall_left=bool(snap.wall_left_detected),
            wall_right=bool(snap.wall_right_detected),
            pose=RobotPose(x_m=snap.pos_x, y_m=snap.pos_y, yaw_deg=snap.yaw),
            velocity_xy=(snap.vel_vx, snap.vel_vy),
            yaw_rate=snap.gyro_z,
            timestamp=snap.timestamp,
            monotonic=snap.monotonic_time,
            frame_index=snap.frame_index,
            source="real",
        )

    def read(self):
        if self.hub is None:
            return None
        snap = self.hub.get_latest_state()
        if snap is None:
            return None
        return self._to_reading(snap)

    def subscribe(self, callback):
        """Asynchronous push - preferred over polling, per the SDK's callback model."""
        if self.hub is None:
            return

        def _forward(snapshot):
            callback(self._to_reading(snapshot))

        self._listener = _forward
        self.hub.add_listener(_forward)

    def available(self):
        return self.hub is not None


# --------------------------------------------------------------------------
# Simulation
# --------------------------------------------------------------------------

def raycast_cells(grid, col, row, heading_deg, max_cells, step=0.02,
                  object_radius=0.12):
    """Distance in *cells* from a continuous position to the first wall.

    Marches along the ray, honouring both the edge walls between cells and
    blocked cell states.  Returns ``max_cells`` when nothing is hit.
    """
    rad = math.radians(heading_deg)
    # Map heading is clockwise from North (screen up): N -> (0, -1).
    d_col = math.sin(rad)
    d_row = -math.cos(rad)

    cur_cell = (int(math.floor(col + 0.5)), int(math.floor(row + 0.5)))
    travelled = 0.0
    c, r = col, row
    # A graspable object stands in the middle of its cell, so the beam stops
    # short of the wall behind it - which is exactly what lets the detector
    # tell "something is there" from "that is the wall".
    objects = getattr(grid, "objects", None) or ()
    while travelled < max_cells:
        c += d_col * step
        r += d_row * step
        travelled += step
        cell = (int(math.floor(c + 0.5)), int(math.floor(r + 0.5)))
        if cell in objects and math.hypot(c - cell[0], r - cell[1]) <= object_radius:
            return travelled
        if cell == cur_cell:
            continue
        d_c = cell[0] - cur_cell[0]
        d_r = cell[1] - cur_cell[1]
        if abs(d_c) + abs(d_r) != 1:
            # Diagonal slip through a corner: step the dominant axis only.
            cell = (cur_cell[0] + d_c, cur_cell[1]) if abs(d_col) >= abs(d_row) else (cur_cell[0], cur_cell[1] + d_r)
            d_c = cell[0] - cur_cell[0]
            d_r = cell[1] - cur_cell[1]
        direction = (0 if d_r < 0 else 2) if d_r else (1 if d_c > 0 else 3)
        if grid.has_wall(cur_cell[0], cur_cell[1], direction):
            return travelled
        if not grid.in_bounds(cell[0], cell[1]):
            return travelled
        if grid.get(cell[0], cell[1]) in (WALL, OBSTACLE):
            return travelled
        cur_cell = cell
    return max_cells


class SimulatedSensorInterface(SensorInterface):
    """Ray-casts the hidden ground-truth map with realistic sensor imperfections.

    Deliberately *not* perfect: each sensor has its real range window, a blind
    spot below the minimum range, Gaussian noise, an occasional dropped
    reading, and it only refreshes at its nominal rate.
    """

    source = "sim"

    def __init__(self, ground_truth, sim_robot, transform, noise=True, seed=None,
                 update_rate_hz=20.0, dropout_prob=0.02):
        self.ground_truth = ground_truth
        self.robot = sim_robot
        self.transform = transform
        self.noise = noise
        self.rng = random.Random(seed)
        self.update_interval = 1.0 / max(1.0, update_rate_hz)
        self.dropout_prob = dropout_prob
        self._last_reading = None
        self._last_time = 0.0
        self._frame = 0
        self.noise_sigma_mm = {"front": 4.0, "left": 9.0, "right": 9.0}
        #: Distance (m) at which a carried object sits in the front beam.
        #: Set while the gripper is loaded so the simulator reproduces the
        #: real blinding instead of pretending the ToF stays clear.
        self.payload_distance_m = None

    def _measure(self, spec, col, row, heading_deg):
        cell_size = self.transform.cell_size_m or 0.60
        max_cells = spec.max_range_m / cell_size
        # A little slack so a wall just past max range still reads "far".
        dist_cells = raycast_cells(self.ground_truth, col, row, heading_deg + spec.angle_deg,
                                   max_cells * 1.3)
        dist_mm = dist_cells * cell_size * 1000.0

        if self.noise:
            if self.rng.random() < self.dropout_prob:
                return None, False
            dist_mm += self.rng.gauss(0.0, self.noise_sigma_mm.get(spec.name, 5.0))

        if dist_mm < spec.min_range_m * 1000.0:
            # Blind spot: the sensor reports something, but it cannot be trusted.
            return spec.min_range_m * 1000.0, False
        if dist_mm > spec.max_range_mm:
            # Out of range: a real Sharp/ToF saturates rather than reporting infinity.
            return spec.max_range_mm, False
        return dist_mm, True

    def read(self):
        now = time.monotonic()
        if self._last_reading is not None and (now - self._last_time) < self.update_interval:
            return self._last_reading

        pose = self.robot.pose()
        map_pose = self.transform.robot_to_map(pose)
        values = {}
        for spec in SENSOR_SPECS:
            values[spec.name] = self._measure(spec, map_pose.col, map_pose.row, map_pose.heading_deg)

        self._frame += 1
        front_mm, front_ok = values["front"]
        if self.payload_distance_m is not None:
            front_mm = self.payload_distance_m * 1000.0
            front_ok = True
        left_mm, left_ok = values["left"]
        right_mm, right_ok = values["right"]

        reading = SensorReading(
            front_mm=front_mm,
            left_mm=left_mm,
            right_mm=right_mm,
            back_mm=None,
            front_valid=front_ok,
            left_valid=left_ok,
            right_valid=right_ok,
            wall_front=bool(front_ok and front_mm is not None and front_mm < 350.0),
            wall_left=bool(left_ok and left_mm is not None and left_mm < 280.0),
            wall_right=bool(right_ok and right_mm is not None and right_mm < 280.0),
            pose=pose,
            velocity_xy=self.robot.velocity_xy(),
            yaw_rate=self.robot.yaw_rate(),
            timestamp=time.time(),
            monotonic=now,
            frame_index=self._frame,
            source="sim",
        )
        self._last_reading = reading
        self._last_time = now
        return reading
