#!/usr/bin/env python3
"""Detecting a graspable object (bottle / can) in front of the robot.

There is no object *recognition* available here and this module does not
pretend otherwise.  The DJI vision module only classifies gestures, lines,
markers, people and robots - there is no bottle or can class - and this
project deliberately stubs out the camera codec (see
``src/calibrate.load_robot_sdk``), so no camera stream exists either.

What the robot really has is the front ToF.  That is enough for the decision
that actually matters - *is there something graspable directly ahead* -
because the map already says how far away the wall in front should be:

    measured front distance  <<  distance the map predicts   ->  something
                                                                 is there

A free-standing bottle reads far closer than the wall behind it, and closer
than the "no wall, open corridor" case by even more.  Requiring several
consistent frames keeps sensor noise from inventing objects.

While the gripper is holding something, the payload sits directly in the ToF
beam and would otherwise read as a permanent obstacle a few centimetres away.
Detection is therefore suppressed while carrying, and callers are expected to
mask the front sensor as well (see ``front_sensor_blinded``).
"""

from dataclasses import dataclass

from .geometry import DIR_VECTORS, heading_to_dir


@dataclass
class ObjectDetection:
    """Outcome of one detection attempt."""

    present: bool = False
    distance_m: float = 0.0
    cell: object = None          # (col, row) the object appears to occupy
    confidence: int = 0          # consecutive agreeing frames
    reason: str = ""

    def __bool__(self):
        return self.present

    __nonzero__ = __bool__


class ObjectDetector(object):
    """Front-ToF object detector, gated on the map's own expectation."""

    def __init__(self, grid, transform, detect_min_m=0.08, detect_max_m=0.75,
                 clearance_margin_m=0.12, confirm_frames=3):
        self.grid = grid
        self.transform = transform
        #: Window in which an echo can plausibly be a graspable object rather
        #: than a wall.  The robot pulls up in the cell *next to* the object,
        #: so a 60 cm grid puts it a little over half a metre away - the upper
        #: bound has to cover that, not just the arm's own reach.
        self.detect_min_m = detect_min_m
        self.detect_max_m = detect_max_m
        #: How much closer than predicted a reading must be to count.
        self.clearance_margin_m = clearance_margin_m
        self.confirm_frames = max(1, confirm_frames)
        self._streak = 0
        self._last_cell = None

    @property
    def cell_size_m(self):
        return self.transform.cell_size_m or 0.60

    def reset(self):
        self._streak = 0
        self._last_cell = None

    # ------------------------------------------------------------------ helpers
    def expected_clear_m(self, cell, direction):
        """How far the front beam should run if nothing but the map is there."""
        cell_size = self.cell_size_m
        # Distance from the cell centre to the shared edge.
        distance = cell_size / 2.0
        col, row = cell
        d_col, d_row = DIR_VECTORS[direction % 4]
        for _ in range(8):
            if self.grid.has_wall(col, row, direction):
                return distance
            col, row = col + d_col, row + d_row
            if not self.grid.in_bounds(col, row):
                return distance
            distance += cell_size
        return distance

    def front_sensor_blinded(self, carrying):
        """True when the front ToF cannot be trusted because of the payload."""
        return bool(carrying)

    # ---------------------------------------------------------------- detection
    def detect(self, reading, carrying=False):
        """Looks for a graspable object straight ahead.

        Returns an ``ObjectDetection``.  Always negative while carrying - the
        object in the gripper is not a new thing to pick up.
        """
        if carrying:
            self.reset()
            return ObjectDetection(reason="carrying - front sensor ignored")
        if reading is None or reading.front_mm is None or not reading.front_valid:
            self.reset()
            return ObjectDetection(reason="no valid front reading")

        distance = reading.front_mm / 1000.0
        if not (self.detect_min_m <= distance <= self.detect_max_m):
            self.reset()
            return ObjectDetection(distance_m=distance,
                                   reason="out of detection range")

        map_pose = self.transform.robot_to_map(reading.pose)
        cell = map_pose.cell
        if not self.grid.in_bounds(cell[0], cell[1]):
            self.reset()
            return ObjectDetection(distance_m=distance, reason="outside the map")

        direction = heading_to_dir(map_pose.heading_deg)
        expected = self.expected_clear_m(cell, direction)
        if distance > expected - self.clearance_margin_m:
            # As far away as the map says the wall is: that is the wall.
            self.reset()
            return ObjectDetection(distance_m=distance,
                                   reason="matches the mapped wall distance")

        d_col, d_row = DIR_VECTORS[direction]
        object_cell = (cell[0] + d_col, cell[1] + d_row)
        if object_cell != self._last_cell:
            self._streak = 0
            self._last_cell = object_cell
        self._streak += 1

        return ObjectDetection(
            present=self._streak >= self.confirm_frames,
            distance_m=distance,
            cell=object_cell,
            confidence=self._streak,
            reason="{:.2f} m ahead, map predicts {:.2f} m".format(distance, expected),
        )
