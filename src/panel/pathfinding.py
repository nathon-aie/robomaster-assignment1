#!/usr/bin/env python3
"""A* pathfinding over the occupancy grid, plus multi-checkpoint ordering.

Paths are grid-cell sequences.  Walls (both blocked cells and the edge
partitions between cells) are never crossed, so an invalid path cannot be
produced: if no route exists the result carries ``ok == False`` and the reason
``NO VALID PATH``.
"""

import heapq
import itertools
import math
from dataclasses import dataclass, field

from .geometry import DIR_VECTORS, dir_from_delta

NO_PATH = "NO VALID PATH"


@dataclass
class PathResult:
    """Outcome of a planning request."""

    cells: list = field(default_factory=list)
    ok: bool = False
    reason: str = ""
    distance_m: float = 0.0
    steps: int = 0
    turns: int = 0
    est_time_s: float = 0.0
    order: list = field(default_factory=list)          # checkpoint visit order (indices)
    waypoints: list = field(default_factory=list)      # [(label, cell), ...] in visit order
    segment_ends: list = field(default_factory=list)   # index into `cells` where each leg ends

    def __bool__(self):
        return self.ok

    __nonzero__ = __bool__  # py2-style guard, harmless


def _heuristic(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def astar(
    grid,
    start,
    goal,
    allow_unknown=False,
    require_known_edge=False,
    turn_penalty=0.0,
    start_dir=None,
):
    """A* over cells, optionally penalising turns.

    Returns the cell list including both endpoints, or ``None`` when no route
    exists.  ``allow_unknown`` lets the search cross UNKNOWN cells (off by
    default - "safe navigation").  ``require_known_edge`` additionally demands
    that every edge crossed has actually been observed as open.
    """
    if start is None or goal is None:
        return None
    if not grid.in_bounds(*start) or not grid.in_bounds(*goal):
        return None
    if grid.is_blocked(*start) or grid.is_blocked(*goal):
        return None
    if start == goal:
        return [start]

    counter = itertools.count()
    start_state = (start, start_dir)
    open_heap = [(_heuristic(start, goal), next(counter), start_state)]
    g_score = {start_state: 0.0}
    came_from = {}
    closed = set()

    while open_heap:
        _, _, state = heapq.heappop(open_heap)
        if state in closed:
            continue
        closed.add(state)
        cell, cur_dir = state

        if cell == goal:
            path = [cell]
            while state in came_from:
                state = came_from[state]
                path.append(state[0])
            path.reverse()
            return path

        base_g = g_score[state]
        for direction in range(4):
            if not grid.can_move(cell[0], cell[1], direction, allow_unknown, require_known_edge):
                continue
            d_col, d_row = DIR_VECTORS[direction]
            nxt = (cell[0] + d_col, cell[1] + d_row)
            step_cost = 1.0
            if turn_penalty and cur_dir is not None and cur_dir != direction:
                quarter_turns = abs(direction - cur_dir)
                if quarter_turns == 3:
                    quarter_turns = 1
                step_cost += turn_penalty * quarter_turns
            n_state = (nxt, direction)
            tentative = base_g + step_cost
            if tentative < g_score.get(n_state, float("inf")):
                g_score[n_state] = tentative
                came_from[n_state] = state
                f = tentative + _heuristic(nxt, goal)
                heapq.heappush(open_heap, (f, next(counter), n_state))
    return None


def count_turns(cells, start_dir=None):
    """Number of 90-degree turns required to walk a cell path."""
    turns = 0
    cur = start_dir
    for i in range(len(cells) - 1):
        d_col = cells[i + 1][0] - cells[i][0]
        d_row = cells[i + 1][1] - cells[i][1]
        try:
            direction = dir_from_delta(d_col, d_row)
        except ValueError:
            continue
        if cur is not None and direction != cur:
            quarter = abs(direction - cur)
            if quarter == 3:
                quarter = 1
            turns += quarter
        cur = direction
    return turns


def estimate_time(steps, turns, cell_size_m=0.60, speed_mps=0.25, turn_time_s=2.0, per_cell_overhead_s=1.0):
    """Rough mission duration from the same numbers the PID controller uses."""
    if speed_mps <= 0:
        speed_mps = 0.25
    return steps * (cell_size_m / speed_mps + per_cell_overhead_s) + turns * turn_time_s


def path_to_commands(cells, start_dir=0):
    """Converts a cell path into the command strings ``RobotControllerThread`` understands."""
    commands = []
    if not cells or len(cells) < 2:
        return commands
    cur_dir = start_dir
    forward = 0
    for i in range(len(cells) - 1):
        d_col = cells[i + 1][0] - cells[i][0]
        d_row = cells[i + 1][1] - cells[i][1]
        try:
            target = dir_from_delta(d_col, d_row)
        except ValueError:
            continue
        if target != cur_dir:
            if forward:
                commands.append("Move Forward: {} cells".format(forward))
                forward = 0
            diff = (target - cur_dir) % 4
            if diff == 1:
                commands.append("Turn Right (90 deg)")
            elif diff == 2:
                commands.append("Turn Around (180 deg)")
            elif diff == 3:
                commands.append("Turn Left (90 deg)")
            cur_dir = target
        forward += 1
    if forward:
        commands.append("Move Forward: {} cells".format(forward))
    return commands


# --------------------------------------------------------------------------
# Multi-checkpoint mission planning
# --------------------------------------------------------------------------

def _order_nearest_neighbour(dist, n):
    remaining = set(range(n))
    order = []
    cur = "start"
    while remaining:
        best = None
        best_d = float("inf")
        for idx in remaining:
            d = dist.get((cur, idx))
            if d is not None and d < best_d:
                best_d = d
                best = idx
        if best is None:
            best = sorted(remaining)[0]
        order.append(best)
        remaining.discard(best)
        cur = best
    return order


def _two_opt(order, dist):
    """Cheap 2-opt improvement over a nearest-neighbour tour."""

    def tour_cost(seq):
        total = 0.0
        cur = "start"
        for idx in seq:
            d = dist.get((cur, idx))
            if d is None:
                return float("inf")
            total += d
            cur = idx
        d = dist.get((cur, "goal"))
        if d is None:
            return float("inf")
        return total + d

    best = list(order)
    best_cost = tour_cost(best)
    improved = True
    while improved:
        improved = False
        for i in range(len(best) - 1):
            for j in range(i + 1, len(best)):
                candidate = best[:i] + best[i:j + 1][::-1] + best[j + 1:]
                cost = tour_cost(candidate)
                if cost < best_cost - 1e-9:
                    best, best_cost = candidate, cost
                    improved = True
    return best, best_cost


def plan_mission(
    grid,
    start,
    goal,
    checkpoints=None,
    allow_unknown=False,
    require_known_edge=False,
    turn_penalty=0.0,
    start_dir=None,
    optimize_order=True,
    speed_mps=0.25,
    brute_force_limit=7,
):
    """Plans Start -> [checkpoints] -> Goal, choosing an efficient visit order.

    For up to ``brute_force_limit`` checkpoints every permutation is evaluated
    exactly; beyond that a nearest-neighbour tour refined by 2-opt is used.
    Each leg is solved with A*, so walls are always respected.
    """
    checkpoints = list(checkpoints or [])
    if start is None:
        return PathResult(ok=False, reason="No start cell set")
    if goal is None:
        return PathResult(ok=False, reason="No goal cell set")

    nodes = {"start": start, "goal": goal}
    for i, cp in enumerate(checkpoints):
        nodes[i] = cp

    kwargs = dict(
        allow_unknown=allow_unknown,
        require_known_edge=require_known_edge,
        turn_penalty=turn_penalty,
    )

    leg_cache = {}

    def leg(a_key, b_key):
        if (a_key, b_key) in leg_cache:
            return leg_cache[(a_key, b_key)]
        cells = astar(grid, nodes[a_key], nodes[b_key], **kwargs)
        leg_cache[(a_key, b_key)] = cells
        return cells

    dist = {}
    keys = ["start"] + list(range(len(checkpoints))) + ["goal"]
    for a in keys:
        for b in keys:
            if a == b:
                continue
            cells = leg(a, b)
            if cells is not None:
                dist[(a, b)] = len(cells) - 1

    # Decide visit order.
    if not checkpoints:
        order = []
    elif not optimize_order:
        order = list(range(len(checkpoints)))
    elif len(checkpoints) <= brute_force_limit:
        best_order, best_cost = None, float("inf")
        for perm in itertools.permutations(range(len(checkpoints))):
            cur = "start"
            cost = 0.0
            valid = True
            for idx in perm:
                d = dist.get((cur, idx))
                if d is None:
                    valid = False
                    break
                cost += d
                cur = idx
            if valid:
                d = dist.get((cur, "goal"))
                if d is None:
                    valid = False
                else:
                    cost += d
            if valid and cost < best_cost:
                best_order, best_cost = list(perm), cost
        order = best_order if best_order is not None else list(range(len(checkpoints)))
    else:
        order = _order_nearest_neighbour(dist, len(checkpoints))
        order, _ = _two_opt(order, dist)

    # Stitch the legs together.
    sequence = ["start"] + list(order) + ["goal"]
    cells = []
    segment_ends = []
    for i in range(len(sequence) - 1):
        piece = leg(sequence[i], sequence[i + 1])
        if piece is None:
            label = "checkpoint {}".format(sequence[i + 1] + 1) if isinstance(sequence[i + 1], int) else sequence[i + 1]
            return PathResult(ok=False, reason="{} (unreachable: {})".format(NO_PATH, label), order=list(order))
        if cells:
            piece = piece[1:]
        cells.extend(piece)
        segment_ends.append(len(cells) - 1)

    turns = count_turns(cells, start_dir=start_dir)
    steps = max(0, len(cells) - 1)
    cell_size = getattr(grid, "cell_size_m", 0.60)
    waypoints = [("Start", start)]
    for idx in order:
        waypoints.append(("Checkpoint {}".format(idx + 1), checkpoints[idx]))
    waypoints.append(("Goal", goal))

    return PathResult(
        cells=cells,
        ok=True,
        reason="OK",
        distance_m=steps * cell_size,
        steps=steps,
        turns=turns,
        est_time_s=estimate_time(steps, turns, cell_size_m=cell_size, speed_mps=speed_mps),
        order=list(order),
        waypoints=waypoints,
        segment_ends=segment_ends,
    )


def path_is_valid(grid, cells, allow_unknown=False, require_known_edge=False):
    """Re-validates a path against the current grid (used before/while executing)."""
    if not cells:
        return False
    for i, cell in enumerate(cells):
        if not grid.in_bounds(cell[0], cell[1]):
            return False
        if grid.is_blocked(cell[0], cell[1]):
            return False
        if not allow_unknown and grid.get(cell[0], cell[1]) == 0:  # UNKNOWN
            return False
        if i + 1 < len(cells):
            d_col = cells[i + 1][0] - cell[0]
            d_row = cells[i + 1][1] - cell[1]
            try:
                direction = dir_from_delta(d_col, d_row)
            except ValueError:
                return False
            if not grid.can_move(cell[0], cell[1], direction, allow_unknown, require_known_edge):
                return False
    return True


def nearest_path_index(cells, col, row):
    """Index of the path cell closest to a continuous map position."""
    best_i, best_d = -1, float("inf")
    for i, (c, r) in enumerate(cells):
        d = math.hypot(c - col, r - row)
        if d < best_d:
            best_i, best_d = i, d
    return best_i, best_d


def deviation_from_path(cells, col, row):
    """Perpendicular-ish deviation (in cells) of a position from a path."""
    if not cells:
        return float("inf")
    _, d = nearest_path_index(cells, col, row)
    return d
