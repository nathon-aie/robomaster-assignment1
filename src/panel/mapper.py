#!/usr/bin/env python3
"""Sensor readings -> occupancy grid.

This is the single mapping implementation.  It consumes ``SensorReading``
objects and does not know or care whether they came from the physical
RoboMaster or from the simulator - only the sensor source differs between
Real Robot mode and Simulation mode.

Geometry, for a 60 cm cell with 7.5 cm partitions: standing at a cell centre
the wall face is ~26 cm away, and the *next* wall is ~86 cm away.  The Sharp IR
sensors saturate at 40 cm, so for them "saturated" reliably means "no wall on
that side"; the ToF reaches 4 m and can clear several cells ahead at once.
"""

import math
import time

from .geometry import DIR_VECTORS, heading_to_dir, wrap180
from .occupancy import FREE, OBSTACLE, UNKNOWN
from .sensors import SENSOR_SPECS

#: Sensor bearing (deg, clockwise) -> direction offset in quarter turns.
_SENSOR_DIR_OFFSET = {0.0: 0, 90.0: 1, 180.0: 2, -90.0: 3, 270.0: 3}


class MapUpdate(object):
    """What one integration step changed - drives replanning and UI redraw."""

    __slots__ = ("new_walls", "new_free", "obstacles", "cells_touched", "changed")

    def __init__(self):
        self.new_walls = []     # [(col, row, direction)]
        self.new_free = []      # [(col, row)]
        self.obstacles = []     # [(col, row)]
        self.cells_touched = 0
        self.changed = False


class OccupancyMapper(object):
    """Integrates range readings into an occupancy grid."""

    def __init__(self, grid, transform, heading_tolerance_deg=8.0,
                 center_tolerance_cells=0.18, max_free_run=6,
                 gate_margin_cells=0.12, range_epsilon_cells=0.05,
                 wall_confirm_votes=2):
        self.grid = grid
        self.transform = transform
        self.heading_tolerance_deg = heading_tolerance_deg
        self.center_tolerance_cells = center_tolerance_cells
        self.max_free_run = max_free_run
        #: How far past the shared edge a reading may land and still count as a
        #: wall on that edge (covers sensor noise and the 7.5 cm partition).
        self.gate_margin_cells = gate_margin_cells
        #: Guards the free-run count against landing exactly on a cell boundary.
        self.range_epsilon_cells = range_epsilon_cells
        #: How many independent sensor frames must agree before a wall is
        #: committed - one noisy sample should not create a wall that blocks
        #: navigation.  Boundary walls are geometric and bypass this.
        self.wall_confirm_votes = max(1, wall_confirm_votes)
        self._votes = {}
        self._last_frame = None
        #: Set while the gripper holds something: the payload sits in the front
        #: ToF beam and would otherwise be mapped as a wall a few centimetres
        #: ahead, everywhere the robot goes.
        self.ignore_front = False
        self.last_update = MapUpdate()
        self.updates = 0
        self.skipped = 0
        self.last_integration_time = 0.0

    # ------------------------------------------------------------------ helpers
    @property
    def cell_size_m(self):
        return self.transform.cell_size_m or 0.60

    @property
    def wall_gate_m(self):
        """Range below which a reading means "wall on the shared edge".

        The shared edge sits half a cell away (0.30 m) and the *next* edge a
        cell further (0.90 m), so 0.6 cells separates the two cases with plenty
        of margin for sensor noise and an off-centre robot.
        """
        return self.cell_size_m * 0.6

    def _sensor_direction(self, heading_dir, spec):
        offset = _SENSOR_DIR_OFFSET.get(spec.angle_deg)
        if offset is None:
            offset = int(round(spec.angle_deg / 90.0)) % 4
        return (heading_dir + offset) % 4

    def _classify(self, spec, value_mm, valid):
        """Returns ``(kind, distance_m)`` where kind is 'hit', 'clear' or 'none'."""
        if value_mm is None or not math.isfinite(value_mm):
            return "none", 0.0
        dist_m = value_mm / 1000.0
        saturated = value_mm >= spec.max_range_mm * 0.95
        if valid and not saturated:
            return "hit", dist_m
        if saturated:
            # A saturated Sharp/ToF is real information: nothing within its range.
            return "clear", spec.max_range_m
        return "none", 0.0

    # -------------------------------------------------------------- integration
    def integrate(self, reading):
        """Folds one sensor reading into the grid.  Returns a ``MapUpdate``."""
        update = MapUpdate()
        self.last_update = update
        if reading is None:
            return update

        # Never integrate the same sensor frame twice: the engine polls faster
        # than the sensors refresh, and a repeated frame is not new evidence.
        frame_id = (reading.source, reading.frame_index)
        if reading.frame_index and frame_id == self._last_frame:
            return update
        self._last_frame = frame_id

        map_pose = self.transform.robot_to_map(reading.pose)
        col_f, row_f = map_pose.col, map_pose.row
        cell = map_pose.cell
        if not self.grid.in_bounds(cell[0], cell[1]):
            self.skipped += 1
            return update

        # The robot's own cell is free by construction - it is standing in it.
        if self.grid.get(cell[0], cell[1]) != FREE:
            self.grid.set(cell[0], cell[1], FREE)
            update.new_free.append(cell)
            update.changed = True

        # Edge assignment only makes sense when the robot is squared up on a
        # cell.  Mid-turn and mid-cell frames are skipped rather than guessed at:
        # a beam 20 deg off-axis lands on a different edge than the one assumed.
        heading = map_pose.heading_deg
        heading_dir = heading_to_dir(heading)
        if abs(wrap180(heading - heading_dir * 90.0)) > self.heading_tolerance_deg:
            self.skipped += 1
            return update
        off_centre = max(abs(col_f - cell[0]), abs(row_f - cell[1]))
        if off_centre > self.center_tolerance_cells:
            self.skipped += 1
            return update

        for spec in SENSOR_SPECS:
            if spec.name == "front" and self.ignore_front:
                continue
            value = reading.distance(spec.name)
            valid = reading.is_valid(spec.name)
            kind, dist_m = self._classify(spec, value, valid)
            if kind == "none":
                continue
            direction = self._sensor_direction(heading_dir, spec)
            d_col, d_row = DIR_VECTORS[direction]

            # Measure against the *actual* pose, not the cell centre: the robot
            # is rarely perfectly centred, and half a cell of assumed offset is
            # enough to place a wall one cell too far away.
            offset = (col_f - cell[0]) * d_col + (row_f - cell[1]) * d_row
            edge_cells = 0.5 - offset
            dist_cells = dist_m / self.cell_size_m

            if kind == "hit" and dist_cells <= edge_cells + self.gate_margin_cells:
                self._write_wall(cell, direction, update)
                continue

            # Contrary evidence decays an unconfirmed wall vote on this edge.
            self._votes.pop(self.grid.edge_id(cell[0], cell[1], direction), None)

            # Free run: how many whole cells the beam cleared past this one.
            free_cells = int(dist_cells - edge_cells - self.range_epsilon_cells) + 1
            free_cells = max(0, min(self.max_free_run, free_cells))
            self._write_free_run(cell, direction, free_cells, update)
            if kind == "hit":
                # Genuine echo at the end of the run: that edge is a wall.
                end = (cell[0] + d_col * free_cells, cell[1] + d_row * free_cells)
                if self.grid.in_bounds(end[0], end[1]):
                    self._write_wall(end, direction, update)

        self.updates += 1
        self.last_integration_time = time.monotonic()
        return update

    def _write_wall(self, cell, direction, update, confirm=True):
        if self.grid.has_wall(cell[0], cell[1], direction):
            self.grid.mark_edge_known(cell[0], cell[1], direction)
            return
        if confirm and self.wall_confirm_votes > 1:
            key = self.grid.edge_id(cell[0], cell[1], direction)
            votes = self._votes.get(key, 0) + 1
            self._votes[key] = votes
            if votes < self.wall_confirm_votes:
                return
        self.grid.set_wall(cell[0], cell[1], direction, True, known=True)
        update.new_walls.append((cell[0], cell[1], direction))
        update.changed = True

    def _write_free_run(self, cell, direction, count, update):
        d_col, d_row = DIR_VECTORS[direction]
        cur = cell
        for _ in range(count):
            nxt = (cur[0] + d_col, cur[1] + d_row)
            if not self.grid.in_bounds(nxt[0], nxt[1]):
                # Field boundary: geometric certainty, no vote needed.
                self._write_wall(cur, direction, update, confirm=False)
                return
            if self.grid.has_wall(cur[0], cur[1], direction):
                # Previously mapped wall contradicts this beam; trust the map.
                return
            self.grid.mark_edge_known(cur[0], cur[1], direction)
            if self.grid.get(nxt[0], nxt[1]) == UNKNOWN:
                self.grid.set(nxt[0], nxt[1], FREE)
                update.new_free.append(nxt)
                update.changed = True
            cur = nxt
            update.cells_touched += 1

    # ---------------------------------------------------------------- obstacles
    def mark_obstacle(self, cell):
        """Flags a cell as a dynamic obstacle (discovered, not user-painted)."""
        if not self.grid.in_bounds(cell[0], cell[1]):
            return False
        if self.grid.get(cell[0], cell[1]) == OBSTACLE:
            return False
        self.grid.set(cell[0], cell[1], OBSTACLE)
        return True

    def obstacle_ahead(self, reading, stop_distance_mm=200.0):
        """True when the front ToF sees something inside the hard-stop envelope."""
        if self.ignore_front:
            return False   # that is the carried object, not an obstacle
        if reading is None or reading.front_mm is None or not reading.front_valid:
            return False
        return reading.front_mm <= stop_distance_mm

    def blocking_cell_ahead(self, reading, transform=None):
        """Cell the front beam is currently blocked by, or ``None``."""
        if self.ignore_front:
            return None
        if reading is None or reading.front_mm is None or not reading.front_valid:
            return None
        transform = transform or self.transform
        map_pose = transform.robot_to_map(reading.pose)
        heading_dir = heading_to_dir(map_pose.heading_deg)
        if reading.front_mm / 1000.0 > self.wall_gate_m:
            return None
        d_col, d_row = DIR_VECTORS[heading_dir]
        cell = map_pose.cell
        nxt = (cell[0] + d_col, cell[1] + d_row)
        return nxt if self.grid.in_bounds(nxt[0], nxt[1]) else None
