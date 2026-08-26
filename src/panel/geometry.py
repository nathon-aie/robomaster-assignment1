#!/usr/bin/env python3
"""Coordinate transformation between RoboMaster odometry and map grid cells.

The robot reports its pose in the *local* frame that ``SensorCollectorThread``
already establishes (see ``src/sensor_pipeline.py``):

    pos_x, pos_y   metres, zeroed at startup and rotated so the initial heading
                   lies along +x
    yaw            degrees, zeroed at startup

The project convention (``RobotControllerThread.navigate_single_grid_step``)
is that the forward unit vector in that frame is ``(cos(yaw), sin(yaw))`` and
that a left turn *decreases* yaw.  Therefore +y is the robot's right-hand side
and increasing yaw is a clockwise rotation.

The map frame is the usual screen grid: ``col`` increases to the right, ``row``
increases downward, direction index 0=N, 1=E, 2=S, 3=W (clockwise).

Nothing here assumes robot (0,0) is map (0,0): origin cell, cell size, start
heading and handedness are all configurable.
"""

import math
from dataclasses import dataclass

# Direction index -> (d_col, d_row).  Clockwise: N, E, S, W.
DIR_VECTORS = ((0, -1), (1, 0), (0, 1), (-1, 0))
DIR_NAMES = ("N", "E", "S", "W")
DIR_LONG = ("North", "East", "South", "West")

# Edge-wall key used by the existing map planner, per direction index.
DIR_WALL_KEY = ("top", "right", "bottom", "left")
OPPOSITE_WALL_KEY = {"top": "bottom", "bottom": "top", "left": "right", "right": "left"}


def wrap180(deg):
    """Wraps an angle to [-180, 180)."""
    return (deg + 180.0) % 360.0 - 180.0


def dir_from_delta(d_col, d_row):
    """Direction index for a unit grid step. Raises ValueError if not a unit step."""
    try:
        return DIR_VECTORS.index((d_col, d_row))
    except ValueError:
        raise ValueError("not a unit grid step: ({}, {})".format(d_col, d_row))


def heading_to_dir(heading_deg):
    """Snaps a map heading (deg clockwise from North) to the nearest direction index."""
    return int(round(wrap180(heading_deg) / 90.0)) % 4


def turn_delta(from_dir, to_dir):
    """Signed turn in degrees (positive = clockwise/right) between direction indices."""
    return wrap180((to_dir - from_dir) * 90.0)


@dataclass
class RobotPose:
    """Robot pose in the robot's own local metric frame."""

    x_m: float = 0.0
    y_m: float = 0.0
    yaw_deg: float = 0.0


@dataclass
class MapPose:
    """Robot pose expressed in continuous map-grid coordinates."""

    col: float = 0.0
    row: float = 0.0
    heading_deg: float = 0.0  # clockwise from North (screen up)

    @property
    def cell(self):
        return (int(math.floor(self.col + 0.5)), int(math.floor(self.row + 0.5)))


@dataclass
class CoordinateTransform:
    """Configurable robot-frame <-> map-frame transformation.

    Attributes:
        origin_col/origin_row: map cell whose *centre* corresponds to robot
            local (0, 0).  Normally the cell the robot stood in when odometry
            was zeroed - not necessarily map (0, 0).
        cell_size_m: physical size of one grid cell (0.60 m on the EP field).
        start_dir: map direction the robot's local +x axis points at.
        handedness: +1 if increasing yaw turns clockwise on the map (project
            default), -1 to mirror.  Flips the lateral axis with it so the pair
            stays consistent.
        yaw_offset_deg: extra yaw bias applied before mapping.
    """

    origin_col: float = 0.0
    origin_row: float = 0.0
    cell_size_m: float = 0.60
    start_dir: int = 0
    handedness: int = 1
    yaw_offset_deg: float = 0.0

    # ---------------------------------------------------------------- basis
    def _basis(self):
        fwd = DIR_VECTORS[self.start_dir % 4]
        right = DIR_VECTORS[(self.start_dir + 1) % 4]
        return fwd, right

    # ------------------------------------------------------------- forwards
    def robot_to_map(self, pose):
        """Robot local metres/degrees -> continuous map cell coordinates."""
        fwd, right = self._basis()
        cs = self.cell_size_m if self.cell_size_m else 1.0
        u = pose.x_m / cs
        v = (pose.y_m / cs) * self.handedness
        col = self.origin_col + u * fwd[0] + v * right[0]
        row = self.origin_row + u * fwd[1] + v * right[1]
        heading = wrap180(
            self.start_dir * 90.0 + (pose.yaw_deg + self.yaw_offset_deg) * self.handedness
        )
        return MapPose(col=col, row=row, heading_deg=heading)

    def robot_to_cell(self, pose):
        return self.robot_to_map(pose).cell

    # ------------------------------------------------------------ backwards
    def map_to_robot(self, col, row, heading_deg=0.0):
        """Continuous map cell coordinates -> robot local metres/degrees."""
        fwd, right = self._basis()
        cs = self.cell_size_m if self.cell_size_m else 1.0
        d_col = col - self.origin_col
        d_row = row - self.origin_row
        u = d_col * fwd[0] + d_row * fwd[1]
        v = d_col * right[0] + d_row * right[1]
        yaw = wrap180(
            (wrap180(heading_deg) - self.start_dir * 90.0) * self.handedness - self.yaw_offset_deg
        )
        return RobotPose(x_m=u * cs, y_m=v * cs * self.handedness, yaw_deg=yaw)

    # ----------------------------------------------------------------- misc
    def metres_between(self, cell_a, cell_b):
        return math.hypot(cell_b[0] - cell_a[0], cell_b[1] - cell_a[1]) * self.cell_size_m

    def rebase(self, cell, heading_deg):
        """Re-zeroes the transform so the given cell/heading becomes robot (0, 0, 0).

        Call this together with re-zeroing the odometry on the robot side so the
        two frames stay in sync.
        """
        self.origin_col = float(cell[0])
        self.origin_row = float(cell[1])
        self.start_dir = heading_to_dir(heading_deg)
        self.yaw_offset_deg = 0.0

    def to_dict(self):
        return {
            "origin_col": self.origin_col,
            "origin_row": self.origin_row,
            "cell_size_m": self.cell_size_m,
            "start_dir": self.start_dir,
            "handedness": self.handedness,
            "yaw_offset_deg": self.yaw_offset_deg,
        }

    @classmethod
    def from_dict(cls, data):
        fields = cls.__dataclass_fields__
        return cls(**{k: v for k, v in (data or {}).items() if k in fields})
