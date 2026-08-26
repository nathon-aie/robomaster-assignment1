#!/usr/bin/env python3
"""Normalised robot state, movement trail and tracking-timeout supervision.

Whatever the source - the physical RoboMaster via ``SensorHub``, the mock
actuators, or the simulator - it is funnelled into a single ``RobotState``
so the UI, the mapper and the navigator never care where the pose came from.
"""

import math
import threading
import time
from dataclasses import dataclass

from .geometry import CoordinateTransform, RobotPose, wrap180


class RobotStatus(object):
    """Robot lifecycle states (string constants keep them JSON/print friendly)."""

    DISCONNECTED = "DISCONNECTED"
    CONNECTING = "CONNECTING"
    CONNECTED = "CONNECTED"
    READY = "READY"
    RUNNING = "RUNNING"
    MOVING = "MOVING"
    PAUSED = "PAUSED"
    STOPPED = "STOPPED"
    MAPPING = "MAPPING"
    NAVIGATING = "NAVIGATING"
    ERROR = "ERROR"
    EMERGENCY_STOP = "EMERGENCY STOP"
    TRACKING_LOST = "TRACKING LOST"

    ALL = (
        DISCONNECTED, CONNECTING, CONNECTED, READY, RUNNING, MOVING, PAUSED,
        STOPPED, MAPPING, NAVIGATING, ERROR, EMERGENCY_STOP, TRACKING_LOST,
    )

    #: States in which the robot must not be commanded to move.
    BLOCKING = (DISCONNECTED, CONNECTING, ERROR, EMERGENCY_STOP, TRACKING_LOST)


@dataclass
class RobotState:
    """One coherent snapshot of where the robot is and what it is doing."""

    # Robot-frame pose (metres / degrees), straight from odometry+IMU.
    x: float = 0.0
    y: float = 0.0
    heading: float = 0.0

    # Map-frame pose, produced by the coordinate transform.
    map_col: float = 0.0
    map_row: float = 0.0
    map_heading: float = 0.0

    velocity: float = 0.0          # m/s, magnitude
    vx: float = 0.0
    vy: float = 0.0
    yaw_rate: float = 0.0          # deg/s

    status: str = RobotStatus.DISCONNECTED
    timestamp: float = 0.0         # wall clock of the underlying sensor frame
    monotonic: float = 0.0
    frame_index: int = 0

    current_target: object = None      # (col, row) cell the robot is driving to
    current_checkpoint: object = None  # label of the mission waypoint in progress
    valid: bool = False                # False until real data has arrived

    @property
    def cell(self):
        return (int(math.floor(self.map_col + 0.5)), int(math.floor(self.map_row + 0.5)))

    def to_dict(self):
        return {
            "x": self.x, "y": self.y, "heading": self.heading,
            "map_col": self.map_col, "map_row": self.map_row,
            "map_heading": self.map_heading, "velocity": self.velocity,
            "status": self.status, "timestamp": self.timestamp,
            "current_target": self.current_target,
            "current_checkpoint": self.current_checkpoint,
        }


class RobotStateTracker(object):
    """Thread-safe holder for the live robot state plus its actual trajectory.

    Fed asynchronously (``SensorHub.add_listener`` on real hardware, the
    simulation loop in simulation mode), read by the UI at frame rate.
    """

    def __init__(self, transform=None, tracking_timeout_s=1.5, trail_limit=4000,
                 trail_min_step_cells=0.06):
        self.transform = transform or CoordinateTransform()
        self.tracking_timeout_s = tracking_timeout_s
        self.trail_min_step_cells = trail_min_step_cells
        self._lock = threading.RLock()
        self._updated = threading.Event()
        self._state = RobotState()
        self._status = RobotStatus.DISCONNECTED
        self._last_update_monotonic = 0.0
        self._trail = []
        self._trail_limit = trail_limit
        self._last_pos = None
        self._timeout_flagged = False
        self.show_trail = True

    # ------------------------------------------------------------------ input
    def update_from_pose(self, pose, velocity_xy=(0.0, 0.0), yaw_rate=0.0,
                         timestamp=None, frame_index=0):
        """Feeds a raw robot-frame pose; converts to map frame and records it."""
        now_mono = time.monotonic()
        map_pose = self.transform.robot_to_map(pose)
        vx, vy = velocity_xy
        speed = math.hypot(vx, vy)
        with self._lock:
            self._state = RobotState(
                x=pose.x_m,
                y=pose.y_m,
                heading=wrap180(pose.yaw_deg),
                map_col=map_pose.col,
                map_row=map_pose.row,
                map_heading=map_pose.heading_deg,
                velocity=speed,
                vx=vx,
                vy=vy,
                yaw_rate=yaw_rate,
                status=self._status,
                timestamp=timestamp if timestamp is not None else time.time(),
                monotonic=now_mono,
                frame_index=frame_index,
                current_target=self._state.current_target,
                current_checkpoint=self._state.current_checkpoint,
                valid=True,
            )
            self._last_update_monotonic = now_mono
            self._timeout_flagged = False
            self._append_trail(map_pose.col, map_pose.row)
        self._updated.set()

    def wait_for_update(self, timeout=1.0, after_frame=None):
        """Blocks until a pose frame *newer* than ``after_frame`` arrives.

        Motion decisions must never be taken on a pose measured before the
        motion happened - that is how a robot ends up turning twice because the
        first turn had not been reported yet.  Returns the fresh state, or
        ``None`` on timeout.
        """
        if after_frame is None:
            after_frame = self.get().frame_index
        deadline = time.monotonic() + timeout
        while True:
            state = self.get()
            if state.valid and state.frame_index != after_frame:
                return state
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                return None
            self._updated.clear()
            self._updated.wait(timeout=min(remaining, 0.05))

    def update_from_snapshot(self, snapshot):
        """Adapter for ``RobotSensorSnapshot`` coming out of Thread 1."""
        pose = RobotPose(x_m=snapshot.pos_x, y_m=snapshot.pos_y, yaw_deg=snapshot.yaw)
        self.update_from_pose(
            pose,
            velocity_xy=(snapshot.vel_vx, snapshot.vel_vy),
            yaw_rate=snapshot.gyro_z,
            timestamp=snapshot.timestamp,
            frame_index=snapshot.frame_index,
        )

    def _append_trail(self, col, row):
        if self._last_pos is not None:
            if math.hypot(col - self._last_pos[0], row - self._last_pos[1]) < self.trail_min_step_cells:
                return
        self._last_pos = (col, row)
        self._trail.append((col, row))
        if len(self._trail) > self._trail_limit:
            del self._trail[: len(self._trail) - self._trail_limit]

    # ----------------------------------------------------------------- output
    def get(self):
        with self._lock:
            state = self._state
            # Status is authoritative on read so a stale frame never lies about it.
            if state.status != self._status:
                state.status = self._status
            return state

    def trail(self):
        with self._lock:
            return list(self._trail)

    def clear_trail(self):
        with self._lock:
            self._trail = []
            self._last_pos = None

    # ----------------------------------------------------------------- status
    def set_status(self, status):
        with self._lock:
            self._status = status
            self._state.status = status

    def get_status(self):
        with self._lock:
            return self._status

    def set_target(self, cell, checkpoint_label=None):
        with self._lock:
            self._state.current_target = cell
            self._state.current_checkpoint = checkpoint_label

    # --------------------------------------------------------------- tracking
    def age(self):
        """Seconds since the last pose update (``inf`` if none yet)."""
        with self._lock:
            if not self._last_update_monotonic:
                return float("inf")
            return time.monotonic() - self._last_update_monotonic

    def tracking_ok(self):
        return self.age() <= self.tracking_timeout_s

    def check_timeout(self):
        """Returns True exactly once per timeout event, so callers can react."""
        with self._lock:
            if not self._state.valid:
                return False
            if self._last_update_monotonic and (
                time.monotonic() - self._last_update_monotonic > self.tracking_timeout_s
            ):
                if not self._timeout_flagged:
                    self._timeout_flagged = True
                    return True
            return False

    def mark_stale(self):
        """Forgets the last update so tracking is reported as lost immediately."""
        with self._lock:
            self._last_update_monotonic = 0.0
            self._state.valid = False
            self._timeout_flagged = False

    def reset(self):
        with self._lock:
            self._state = RobotState(status=self._status)
            self._trail = []
            self._last_pos = None
            self._last_update_monotonic = 0.0
            self._timeout_flagged = False
