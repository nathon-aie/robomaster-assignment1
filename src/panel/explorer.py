#!/usr/bin/env python3
"""Frontier-based exploration planner for auto-mapping.

A *frontier* is a known-free cell that touches an unknown cell through an edge
that is not a wall.  Driving to frontiers - rather than wandering randomly -
is what makes auto-mapping terminate.

Frontier selection balances:

    distance          shorter trips first
    information gain  prefer frontiers touching several unknown cells
    turns             turning is slow and costs odometry accuracy
    safety            avoid squeezing into cells hemmed in by walls
"""

from collections import deque

from .geometry import DIR_VECTORS
from .occupancy import FREE
from .pathfinding import astar, count_turns


class ExplorationTarget(object):
    """A chosen frontier plus the route to it."""

    __slots__ = ("cell", "path", "score", "gain", "distance", "turns")

    def __init__(self, cell, path, score, gain, distance, turns):
        self.cell = cell
        self.path = path
        self.score = score
        self.gain = gain
        self.distance = distance
        self.turns = turns

    def __repr__(self):
        return "<ExplorationTarget {} d={} gain={} score={:.2f}>".format(
            self.cell, self.distance, self.gain, self.score
        )


class FrontierExplorer(object):
    def __init__(self, grid, gain_weight=2.5, turn_weight=0.8, safety_weight=1.0,
                 max_candidates=40):
        self.grid = grid
        self.gain_weight = gain_weight
        self.turn_weight = turn_weight
        self.safety_weight = safety_weight
        self.max_candidates = max_candidates
        self.blacklist = set()

    # ------------------------------------------------------------- frontiers
    def frontiers(self):
        return [c for c in self.grid.frontier_cells() if c not in self.blacklist]

    def reachable_distances(self, origin):
        """BFS over known-free cells; returns ``{cell: steps}``."""
        if not self.grid.in_bounds(origin[0], origin[1]):
            return {}
        dist = {origin: 0}
        queue = deque([origin])
        while queue:
            cell = queue.popleft()
            for d in range(4):
                if not self.grid.can_move(cell[0], cell[1], d, allow_unknown=False):
                    continue
                d_col, d_row = DIR_VECTORS[d]
                nxt = (cell[0] + d_col, cell[1] + d_row)
                if nxt in dist:
                    continue
                dist[nxt] = dist[cell] + 1
                queue.append(nxt)
        return dist

    def _safety_penalty(self, cell):
        """Cells with walls on three sides (dead ends) are less attractive."""
        walls = sum(1 for d in range(4) if self.grid.has_wall(cell[0], cell[1], d))
        return max(0, walls - 1)

    # --------------------------------------------------------------- selection
    def select(self, from_cell, from_dir=None, visited=None):
        """Picks the next place to drive to, or ``None`` when there is nothing left.

        Information frontiers come first.  When they run out and ``visited`` is
        supplied, any reachable free cell the robot has not actually stood in
        becomes a target, so auto-mapping ends having driven every reachable
        square rather than only enough of them to resolve the unknowns.
        """
        target = self._select_frontier(from_cell, from_dir)
        if target is not None:
            return target
        if visited is None:
            return None
        return self._select_unvisited(from_cell, from_dir, visited)

    def _select_unvisited(self, from_cell, from_dir, visited):
        """Nearest reachable free cell that has never been driven through."""
        dist = self.reachable_distances(from_cell)
        best = None
        for cell, steps in sorted(dist.items(), key=lambda item: item[1]):
            if cell in visited or cell in self.blacklist:
                continue
            if self.grid.get(cell[0], cell[1]) != FREE:
                continue
            path = astar(self.grid, from_cell, cell, allow_unknown=False)
            if not path:
                continue
            turns = count_turns(path, start_dir=from_dir)
            score = steps + self.turn_weight * turns
            candidate = ExplorationTarget(cell, path, score, 0, steps, turns)
            if best is None or candidate.score < best.score:
                best = candidate
            # Distances are ascending, so the first few are already the closest.
            if best is not None and steps > best.distance + 2:
                break
        return best

    def _select_frontier(self, from_cell, from_dir=None):
        candidates = self.frontiers()
        if not candidates:
            return None

        dist = self.reachable_distances(from_cell)
        scored = []
        for cell in candidates:
            steps = dist.get(cell)
            if steps is None:
                continue
            gain = self.grid.information_gain(cell[0], cell[1])
            if gain <= 0:
                continue
            scored.append((steps, -gain, cell))
        if not scored:
            return None

        scored.sort()
        best = None
        for steps, neg_gain, cell in scored[: self.max_candidates]:
            path = astar(self.grid, from_cell, cell, allow_unknown=False)
            if not path:
                continue
            turns = count_turns(path, start_dir=from_dir)
            gain = -neg_gain
            score = (
                steps
                - self.gain_weight * gain
                + self.turn_weight * turns
                + self.safety_weight * self._safety_penalty(cell)
            )
            target = ExplorationTarget(cell, path, score, gain, steps, turns)
            if best is None or target.score < best.score:
                best = target
        return best

    # ---------------------------------------------------------------- lifecycle
    def blacklist_cell(self, cell):
        """Drops an unreachable/failed frontier so exploration does not loop on it."""
        self.blacklist.add(cell)

    def reset(self):
        self.blacklist = set()

    def is_complete(self, from_cell, visited=None):
        return self.select(from_cell, visited=visited) is None

    def progress(self):
        stats = self.grid.stats()
        return stats["progress"], stats
