#!/usr/bin/env python3
"""Detecting a bottle / can in front of the robot with the sensors it has.

There is no camera classifier here and no extra hardware.  The only forward
sensor is the ToF, which cannot tell a bottle from a wall on its own - but the
*map* can: it already knows where the field partitions are.  So an object is
whatever the ToF sees that the map says should not be there.

    expected clearance  = distance to the mapped wall straight ahead
    measured clearance  = front ToF reading
    object present      = measured is clearly shorter than expected,
                          and lies inside the range the gripper can reach

While the robot is already carrying something the detector reports nothing:
the object in the gripper is what matters, and the sensors would otherwise
keep re-triggering on whatever is next in front.
"""

from .geometry import heading_to_dir, wrap180
from .sensors import raycast_cells


class ObjectSighting(object):
    """One positive detection, with the numbers behind it."""

    __slots__ = ("distance_m", "expected_m", "cell", "direction", "confidence")

    def __init__(self, distance_m, expected_m, cell, direction, confidence):
        self.distance_m = distance_m
        self.expected_m = expected_m
        self.cell = cell
        self.direction = direction
        self.confidence = confidence

    @property
    def gap_m(self):
        """How much nearer than the mapped wall the object sits."""
        return self.expected_m - self.distance_m

    def __repr__(self):
        return "<ObjectSighting {:.2f} m (wall at {:.2f} m) cell={}>".format(
            self.distance_m, self.expected_m, self.cell)


class ObjectDetector(object):
    """Front-facing object detector built on the ToF plus the occupancy map."""

    def __init__(self, grid, transform,
                 grab_min_m=0.10, grab_max_m=0.45,
                 clearance_margin_m=0.14, heading_tolerance_deg=12.0,
                 confirm_frames=2):
        self.grid = grid
        self.transform = transform
        #: Window in which the arm can actually reach the object.
        self.grab_min_m = grab_min_m
        self.grab_max_m = grab_max_m
        #: How much nearer than the mapped wall a reading must be to count.
        #: Covers ToF noise, the 7.5 cm partition and an off-centre robot.
        self.clearance_margin_m = clearance_margin_m
        self.heading_tolerance_deg = heading_tolerance_deg
        #: Consecutive agreeing frames before reporting - one noisy sample
        #: should not send the arm out.
        self.confirm_frames = max(1, confirm_frames)
        self._streak = 0
        self.last_sighting = None

    @property
    def cell_size_m(self):
        return self.transform.cell_size_m or 0.60

    def reset(self):
        self._streak = 0
        self.last_sighting = None

    def expected_clearance_m(self, map_pose):
        """Distance to the wall the map says is straight ahead."""
        max_cells = 6.0
        cells = raycast_cells(self.grid, map_pose.col, map_pose.row,
                              map_pose.heading_deg, max_cells)
        return cells * self.cell_size_m

    def inspect(self, reading, carrying=False):
        """Returns an ``ObjectSighting`` when something graspable is ahead.

        ``carrying`` suppresses detection entirely - see the module docstring.
        """
        if carrying:
            self.reset()
            return None
        if reading is None or reading.front_mm is None or not reading.front_valid:
            self._streak = 0
            return None

        map_pose = self.transform.robot_to_map(reading.pose)
        cell = map_pose.cell
        if not self.grid.in_bounds(cell[0], cell[1]):
            self._streak = 0
            return None

        # Off-axis the ray-cast against a square grid is not comparable to the
        # ToF beam, so no claim is made either way.
        heading_dir = heading_to_dir(map_pose.heading_deg)
        if abs(wrap180(map_pose.heading_deg - heading_dir * 90.0)) > self.heading_tolerance_deg:
            self._streak = 0
            return None

        measured = reading.front_mm / 1000.0
        if not (self.grab_min_m <= measured <= self.grab_max_m):
            self._streak = 0
            return None

        expected = self.expected_clearance_m(map_pose)
        if measured >= expected - self.clearance_margin_m:
            # Whatever the ToF is seeing is where the map already has a wall.
            self._streak = 0
            return None

        self._streak += 1
        if self._streak < self.confirm_frames:
            return None

        gap = expected - measured
        confidence = min(1.0, gap / max(self.cell_size_m, 1e-6))
        sighting = ObjectSighting(measured, expected, cell, heading_dir, confidence)
        self.last_sighting = sighting
        return sighting

    def status_text(self, carrying=False):
        """Short line for the sensor panel."""
        if carrying:
            return "HOLDING (ignoring front)"
        if self.last_sighting is None:
            return "none"
        return "object {:.2f} m".format(self.last_sighting.distance_m)
