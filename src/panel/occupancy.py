#!/usr/bin/env python3
"""Occupancy grid with edge walls, markers and JSON map storage.

Two things are tracked, because the RoboMaster field has both:

* **Cell state** - UNKNOWN / FREE / WALL / OBSTACLE for the cell itself.
  ``WALL`` is a user-painted blocked cell, ``OBSTACLE`` is something the robot
  discovered at runtime.
* **Edge walls** - the 7.5 cm partitions *between* cells that the Sharp/ToF
  sensors actually see, using the same ``top/right/bottom/left`` model as the
  existing ``src/map_planner.py``.

Edges are tri-state (UNKNOWN / OPEN / WALL) so auto-mapping can distinguish
"no wall there" from "never looked".  Manual editing marks edges as known.

The JSON format is a superset of ``data/robot_map_plan.json`` so existing plan
files load unchanged.
"""

import json
import random
from pathlib import Path

from .geometry import DIR_VECTORS, DIR_WALL_KEY

# ----------------------------------------------------------------- cell states
UNKNOWN = 0
FREE = 1
WALL = 2
OBSTACLE = 3

CELL_STATE_NAMES = {UNKNOWN: "UNKNOWN", FREE: "FREE", WALL: "WALL", OBSTACLE: "OBSTACLE"}

# ----------------------------------------------------------------- edge states
EDGE_UNKNOWN = 0
EDGE_OPEN = 1
EDGE_WALL = 2


def _edge_key(col, row, direction):
    """Canonical key for the edge on `direction` side of cell (col, row).

    Horizontal edges are ('h', col, row) = top of (col, row).
    Vertical edges are ('v', col, row) = left of (col, row).
    Both cells sharing an edge produce the same key, so walls stay symmetric.
    """
    d = direction % 4
    if d == 0:
        return ("h", col, row)
    if d == 1:
        return ("v", col + 1, row)
    if d == 2:
        return ("h", col, row + 1)
    return ("v", col, row)


class OccupancyGrid(object):
    """Dynamic-size occupancy grid.  No dimension is hardcoded."""

    def __init__(self, width=9, height=9, fill=FREE):
        self.width = 0
        self.height = 0
        self.cells = []
        self._walls = set()        # edge keys that are walls
        self._known_edges = set()  # edge keys whose state has been observed/edited
        self.start = None          # (col, row) or None
        self.goal = None
        self.checkpoints = []      # list of (col, row), ordered by the user
        self.robot_cell = None     # editor-placed robot start cell
        self.robot_dir = 0         # editor-placed robot start heading (dir index)
        self.place_cell = None     # where a carried object is put down
        self.place_dir = 0         # heading the robot faces while placing it
        self.cell_size_m = 0.60
        self.resize(width, height, fill=fill)

    # ------------------------------------------------------------------ shape
    def resize(self, width, height, fill=FREE, keep=True):
        """Resizes the grid, optionally preserving the overlapping region."""
        width = max(1, int(width))
        height = max(1, int(height))
        old_cells = self.cells if keep else []
        old_w, old_h = self.width, self.height
        new_cells = [[fill for _ in range(width)] for _ in range(height)]
        if keep and old_cells:
            for r in range(min(old_h, height)):
                for c in range(min(old_w, width)):
                    new_cells[r][c] = old_cells[r][c]
        self.cells = new_cells
        self.width = width
        self.height = height
        if keep:
            self._walls = set(k for k in self._walls if self._edge_in_bounds(k))
            self._known_edges = set(k for k in self._known_edges if self._edge_in_bounds(k))
        else:
            self._walls = set()
            self._known_edges = set()
        self._clamp_markers()

    def _edge_in_bounds(self, key):
        kind, c, r = key
        if kind == "h":
            return 0 <= c < self.width and 0 <= r <= self.height
        return 0 <= c <= self.width and 0 <= r < self.height

    def _clamp_markers(self):
        def ok(p):
            return p is not None and self.in_bounds(p[0], p[1])

        if not ok(self.start):
            self.start = None
        if not ok(self.goal):
            self.goal = None
        if not ok(self.robot_cell):
            self.robot_cell = None
        if not ok(self.place_cell):
            self.place_cell = None
        self.checkpoints = [p for p in self.checkpoints if ok(p)]

    def in_bounds(self, col, row):
        return 0 <= col < self.width and 0 <= row < self.height

    # ------------------------------------------------------------------ cells
    def get(self, col, row):
        if not self.in_bounds(col, row):
            return WALL
        return self.cells[row][col]

    def set(self, col, row, state):
        if self.in_bounds(col, row):
            self.cells[row][col] = state

    def fill(self, state):
        for r in range(self.height):
            for c in range(self.width):
                self.cells[r][c] = state

    def is_blocked(self, col, row):
        return self.get(col, row) in (WALL, OBSTACLE)

    def passable(self, col, row, allow_unknown=False):
        """A cell may be routed through."""
        if not self.in_bounds(col, row):
            return False
        state = self.cells[row][col]
        if state in (WALL, OBSTACLE):
            return False
        if state == UNKNOWN and not allow_unknown:
            return False
        return True

    # ------------------------------------------------------------------ edges
    def edge_id(self, col, row, direction):
        """Canonical identity of an edge - the same for both cells sharing it."""
        return _edge_key(col, row, direction)

    def edge_state(self, col, row, direction):
        key = _edge_key(col, row, direction)
        if key in self._walls:
            return EDGE_WALL
        if key in self._known_edges:
            return EDGE_OPEN
        return EDGE_UNKNOWN

    def has_wall(self, col, row, direction):
        return _edge_key(col, row, direction) in self._walls

    def set_wall(self, col, row, direction, value=True, known=True):
        key = _edge_key(col, row, direction)
        if not self._edge_in_bounds(key):
            return
        if value:
            self._walls.add(key)
        else:
            self._walls.discard(key)
        if known:
            self._known_edges.add(key)

    def mark_edge_known(self, col, row, direction):
        key = _edge_key(col, row, direction)
        if self._edge_in_bounds(key):
            self._known_edges.add(key)

    def toggle_wall(self, col, row, direction):
        self.set_wall(col, row, direction, not self.has_wall(col, row, direction))

    def cell_walls(self, col, row):
        """Dict in the legacy ``{'top':..,'right':..,'bottom':..,'left':..}`` form."""
        return dict(
            (DIR_WALL_KEY[d], self.has_wall(col, row, d)) for d in range(4)
        )

    def clear_walls(self, keep_border=True):
        self._walls = set()
        self._known_edges = set()
        if keep_border:
            self.add_border()

    def add_border(self):
        for c in range(self.width):
            self.set_wall(c, 0, 0, True)
            self.set_wall(c, self.height - 1, 2, True)
        for r in range(self.height):
            self.set_wall(0, r, 3, True)
            self.set_wall(self.width - 1, r, 1, True)

    def mark_all_known(self):
        """Treats every in-bounds edge as observed (used by the manual editor)."""
        for r in range(self.height):
            for c in range(self.width):
                for d in range(4):
                    self._known_edges.add(_edge_key(c, r, d))

    # ------------------------------------------------------------ traversal
    def can_move(self, col, row, direction, allow_unknown=False, require_known_edge=False):
        """Whether the robot may step from (col,row) one cell in `direction`."""
        d_col, d_row = DIR_VECTORS[direction % 4]
        n_col, n_row = col + d_col, row + d_row
        if not self.in_bounds(n_col, n_row):
            return False
        edge = self.edge_state(col, row, direction)
        if edge == EDGE_WALL:
            return False
        if require_known_edge and edge != EDGE_OPEN:
            return False
        return self.passable(n_col, n_row, allow_unknown=allow_unknown)

    def neighbors(self, col, row, allow_unknown=False, require_known_edge=False):
        out = []
        for d in range(4):
            if self.can_move(col, row, d, allow_unknown, require_known_edge):
                d_col, d_row = DIR_VECTORS[d]
                out.append(((col + d_col, row + d_row), d))
        return out

    # ------------------------------------------------------------- frontiers
    def frontier_cells(self):
        """Known-free cells that still have something to learn next to them.

        That is either an adjacent unknown *cell* (reachable through a non-wall
        edge) or an edge whose state has never been observed - a long ToF sweep
        marks cells free from a distance without ever proving whether the walls
        between them exist, and those cells still need visiting.
        """
        out = []
        for r in range(self.height):
            for c in range(self.width):
                if self.cells[r][c] != FREE:
                    continue
                if self.information_gain(c, r) > 0:
                    out.append((c, r))
        return out

    def unknown_neighbor_count(self, col, row):
        """Adjacent unknown cells reachable through a non-wall edge."""
        n = 0
        for d in range(4):
            if self.edge_state(col, row, d) == EDGE_WALL:
                continue
            d_col, d_row = DIR_VECTORS[d]
            n_col, n_row = col + d_col, row + d_row
            if self.in_bounds(n_col, n_row) and self.cells[n_row][n_col] == UNKNOWN:
                n += 1
        return n

    def unknown_edge_count(self, col, row):
        """Edges of this cell whose state has never been observed."""
        return sum(1 for d in range(4) if self.edge_state(col, row, d) == EDGE_UNKNOWN)

    def information_gain(self, col, row):
        """How much visiting this cell would still teach us."""
        return self.unknown_neighbor_count(col, row) + self.unknown_edge_count(col, row)

    # ----------------------------------------------------------------- stats
    def stats(self):
        total = self.width * self.height
        counts = {UNKNOWN: 0, FREE: 0, WALL: 0, OBSTACLE: 0}
        for r in range(self.height):
            for c in range(self.width):
                counts[self.cells[r][c]] = counts.get(self.cells[r][c], 0) + 1
        known = total - counts[UNKNOWN]
        return {
            "total": total,
            "unknown": counts[UNKNOWN],
            "free": counts[FREE],
            "wall": counts[WALL],
            "obstacle": counts[OBSTACLE],
            "known": known,
            "wall_edges": len(self._walls),
            "progress": (float(known) / total) if total else 0.0,
        }

    # ---------------------------------------------------------------- editing
    def reset(self, fill=FREE):
        self.fill(fill)
        self._walls = set()
        self._known_edges = set()
        self.add_border()
        self.start = None
        self.goal = None
        self.checkpoints = []
        self.robot_cell = None
        self.robot_dir = 0
        self.place_cell = None
        self.place_dir = 0

    def rotate(self, quarter_turns=1):
        """Rotates the whole map in 90 degree steps (positive = clockwise).

        Cells, edge walls, edge-knowledge, start/goal/checkpoints, the placed
        robot cell and its heading all move together, and width/height swap on
        odd numbers of turns.
        """
        for _ in range(int(quarter_turns) % 4):
            self._rotate_cw()
        return self

    def _rotate_cw(self):
        old_w, old_h = self.width, self.height
        old_cells = self.cells
        old_walls = self._walls
        old_known = self._known_edges

        def move(col, row):
            """Old cell -> new cell for a clockwise quarter turn."""
            return (old_h - 1 - row, col)

        new_cells = [[UNKNOWN for _ in range(old_h)] for _ in range(old_w)]
        new_walls = set()
        new_known = set()
        for row in range(old_h):
            for col in range(old_w):
                n_col, n_row = move(col, row)
                new_cells[n_row][n_col] = old_cells[row][col]
                for d in range(4):
                    key = _edge_key(col, row, d)
                    new_key = _edge_key(n_col, n_row, (d + 1) % 4)
                    if key in old_walls:
                        new_walls.add(new_key)
                    if key in old_known:
                        new_known.add(new_key)

        self.cells = new_cells
        self.width, self.height = old_h, old_w
        self._walls = new_walls
        self._known_edges = new_known
        self.start = move(*self.start) if self.start else None
        self.goal = move(*self.goal) if self.goal else None
        self.robot_cell = move(*self.robot_cell) if self.robot_cell else None
        self.place_cell = move(*self.place_cell) if self.place_cell else None
        self.checkpoints = [move(*p) for p in self.checkpoints]
        self.robot_dir = (self.robot_dir + 1) % 4
        self.place_dir = (self.place_dir + 1) % 4

    def random_map(self, wall_density=0.28, seed=None):
        """Random maze-ish layout using interior edge walls."""
        rng = random.Random(seed)
        self.fill(FREE)
        self._walls = set()
        self._known_edges = set()
        self.add_border()
        for r in range(self.height):
            for c in range(self.width):
                if c + 1 < self.width and rng.random() < wall_density:
                    self.set_wall(c, r, 1, True)
                if r + 1 < self.height and rng.random() < wall_density:
                    self.set_wall(c, r, 2, True)
        self.mark_all_known()
        self.start = (0, 0)
        self.goal = (self.width - 1, self.height - 1)
        self.checkpoints = []
        self.robot_cell = self.start
        # Guarantee the start and goal are not sealed in.
        for cell in (self.start, self.goal):
            if all(self.has_wall(cell[0], cell[1], d) or
                   not self.in_bounds(cell[0] + DIR_VECTORS[d][0], cell[1] + DIR_VECTORS[d][1])
                   for d in range(4)):
                for d in range(4):
                    n = (cell[0] + DIR_VECTORS[d][0], cell[1] + DIR_VECTORS[d][1])
                    if self.in_bounds(n[0], n[1]):
                        self.set_wall(cell[0], cell[1], d, False)
                        break

    # ---------------------------------------------------------------- storage
    def to_dict(self):
        walls = []
        for r in range(self.height):
            for c in range(self.width):
                w = self.cell_walls(c, r)
                if any(w.values()):
                    walls.append({"pos": [c, r], "walls": w})
        return {
            "version": 2,
            "grid_info": {"rows": self.height, "cols": self.width, "cell_size_m": self.cell_size_m},
            "width": self.width,
            "height": self.height,
            "start": list(self.start) if self.start else None,
            "goal": list(self.goal) if self.goal else None,
            "checkpoints": [list(p) for p in self.checkpoints],
            "robot": {"cell": list(self.robot_cell) if self.robot_cell else None,
                      "dir": self.robot_dir},
            "place": {"cell": list(self.place_cell) if self.place_cell else None,
                      "dir": self.place_dir},
            "cells": [list(row) for row in self.cells],
            "walls": walls,
            "known_edges": [list(k) for k in sorted(self._known_edges, key=lambda x: (x[0], x[1], x[2]))],
        }

    def load_dict(self, data):
        info = data.get("grid_info", {})
        width = int(data.get("width", info.get("cols", self.width)))
        height = int(data.get("height", info.get("rows", self.height)))
        self.resize(width, height, fill=FREE, keep=False)
        if "cell_size_m" in info:
            try:
                self.cell_size_m = float(info["cell_size_m"])
            except (TypeError, ValueError):
                pass

        cells = data.get("cells")
        if cells:
            for r in range(min(height, len(cells))):
                row = cells[r]
                for c in range(min(width, len(row))):
                    self.cells[r][c] = int(row[c])

        for entry in data.get("walls", []):
            pos = entry.get("pos") or entry.get("cell")
            if not pos:
                continue
            c, r = int(pos[0]), int(pos[1])
            wdict = entry.get("walls", {})
            for d, key in enumerate(DIR_WALL_KEY):
                if wdict.get(key):
                    self.set_wall(c, r, d, True, known=True)

        known = data.get("known_edges")
        if known:
            self._known_edges = set(
                (str(k[0]), int(k[1]), int(k[2])) for k in known if len(k) == 3
            )
            self._known_edges |= set(self._walls)
        elif not cells:
            # Legacy plan file: everything in it was drawn by hand, so it is known.
            self.mark_all_known()
        else:
            self._known_edges = set(self._walls)

        start = data.get("start")
        goal = data.get("goal")
        self.start = (int(start[0]), int(start[1])) if start else None
        self.goal = (int(goal[0]), int(goal[1])) if goal else None
        self.checkpoints = [(int(p[0]), int(p[1])) for p in data.get("checkpoints", [])]
        robot = data.get("robot") or {}
        rcell = robot.get("cell")
        self.robot_cell = (int(rcell[0]), int(rcell[1])) if rcell else self.start
        self.robot_dir = int(robot.get("dir", 0)) % 4
        place = data.get("place") or {}
        pcell = place.get("cell")
        self.place_cell = (int(pcell[0]), int(pcell[1])) if pcell else None
        self.place_dir = int(place.get("dir", 0)) % 4
        self._clamp_markers()
        return self

    def save(self, path):
        target = Path(path)
        target.parent.mkdir(parents=True, exist_ok=True)
        with target.open("w", encoding="utf-8") as fh:
            json.dump(self.to_dict(), fh, indent=2, ensure_ascii=False)
        return target

    @classmethod
    def load(cls, path):
        with Path(path).open("r", encoding="utf-8") as fh:
            data = json.load(fh)
        grid = cls(1, 1)
        return grid.load_dict(data)

    # -------------------------------------------------------------- debugging
    def ascii_art(self, robot_cell=None):
        """Compact text rendering, handy for tests and the debug console."""
        glyph = {UNKNOWN: "?", FREE: ".", WALL: "#", OBSTACLE: "X"}
        lines = []
        for r in range(self.height):
            top = ""
            mid = ""
            for c in range(self.width):
                top += "+" + ("---" if self.has_wall(c, r, 0) else "   ")
                mid += ("|" if self.has_wall(c, r, 3) else " ")
                ch = glyph.get(self.cells[r][c], "?")
                if robot_cell == (c, r):
                    ch = "R"
                elif self.start == (c, r):
                    ch = "S"
                elif self.goal == (c, r):
                    ch = "G"
                elif (c, r) in self.checkpoints:
                    ch = "C"
                elif self.place_cell == (c, r):
                    ch = "P"
                mid += " " + ch + " "
            top += "+"
            mid += "|" if self.has_wall(self.width - 1, r, 1) else " "
            lines.append(top)
            lines.append(mid)
        lines.append("".join("+" + ("---" if self.has_wall(c, self.height - 1, 2) else "   ")
                             for c in range(self.width)) + "+")
        return "\n".join(lines)
