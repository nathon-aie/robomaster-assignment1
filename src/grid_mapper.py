#!/usr/bin/env python3
"""Autonomous Grid Mapping and Maze Exploration Module for RoboMaster EP.

Task Requirements (mapping.txt):
1. Autonomous Maze Exploration: Robot explores an unknown maze without prior map knowledge.
2. Map Export: Discovers the maze grid & wall boundaries and exports the resulting map (JSON / ASCII).
3. Sensor Integration: Uses existing sensors (Sharp IR Left id1/port1, Sharp IR Right id2/port2,
   ToF Front, IMU Yaw, Odometry) via Thread 1 (SensorPipeline) and Thread 2 (RobotController).
"""

import collections
import heapq
import json
import math
import os
import sys
import time
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Dict, List, Optional, Set, Tuple

try:
    from .pid_controller import WallCenteringPID
    from .robot_controller import RobotControllerThread
    from .robot_system import RobotSystem
    from .sensor_pipeline import RobotSensorSnapshot, SensorHub
except (ImportError, ValueError):
    from pid_controller import WallCenteringPID
    from robot_controller import RobotControllerThread
    from robot_system import RobotSystem
    from sensor_pipeline import RobotSensorSnapshot, SensorHub

# ---------------------------------------------------------------------------
# Global Direction Constants
# ---------------------------------------------------------------------------
# 0: North (up, dy=-1, dx=0, 'top')
# 1: East  (right, dy=0, dx=+1, 'right')
# 2: South (down, dy=+1, dx=0, 'bottom')
# 3: West  (left, dy=0, dx=-1, 'left')
NORTH = 0
EAST = 1
SOUTH = 2
WEST = 3

DIR_VECTORS: Dict[int, Tuple[int, int]] = {
    NORTH: (0, -1),
    EAST: (1, 0),
    SOUTH: (0, 1),
    WEST: (-1, 0),
}

DIR_NAMES: Dict[int, str] = {
    NORTH: "North (ขึ้น)",
    EAST: "East (ขวา)",
    SOUTH: "South (ลง)",
    WEST: "West (ซ้าย)",
}

DIR_SYMBOLS: Dict[int, str] = {
    NORTH: "^",
    EAST: ">",
    SOUTH: "v",
    WEST: "<",
}

WORLD_WALL_KEYS = ["top", "right", "bottom", "left"]
OPPOSITE_DIR: Dict[int, int] = {NORTH: SOUTH, EAST: WEST, SOUTH: NORTH, WEST: EAST}
OPPOSITE_WALL: Dict[str, str] = {"top": "bottom", "bottom": "top", "left": "right", "right": "left"}


# ---------------------------------------------------------------------------
# Data Models for Discovered Map
# ---------------------------------------------------------------------------
@dataclass
class DiscoveredCell:
    """Represents a single grid cell in the discovered maze."""

    col: int
    row: int
    walls: Dict[str, Optional[bool]] = field(
        default_factory=lambda: {"top": None, "bottom": None, "left": None, "right": None}
    )
    visited: bool = False
    visit_count: int = 0
    first_visited_time: Optional[float] = None

    def has_wall(self, direction_key: str) -> bool:
        """Returns True if wall exists in specified direction."""
        return self.walls.get(direction_key) is True

    def is_wall_known(self, direction_key: str) -> bool:
        return self.walls.get(direction_key) is not None

    def to_dict(self) -> Dict[str, Any]:
        return {
            "pos": [self.col, self.row],
            "walls": {k: (bool(v) if v is not None else False) for k, v in self.walls.items()},
            "visited": self.visited,
            "visit_count": self.visit_count,
        }


class DiscoveredGridMap:
    """Represents the global 2D grid map being constructed during exploration."""

    def __init__(self, cols: int = 4, rows: int = 4, grid_size_px: int = 100, grid_size_m: float = 0.60):
        self.cols = cols
        self.rows = rows
        self.grid_size_px = grid_size_px
        self.grid_size_m = grid_size_m
        self.grid: List[List[DiscoveredCell]] = [
            [DiscoveredCell(col=c, row=r) for c in range(cols)] for r in range(rows)
        ]
        self.apply_outer_boundaries()

    def apply_outer_boundaries(self):
        """Sets external boundary walls of the arena."""
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                if r == 0:
                    cell.walls["top"] = True
                if r == self.rows - 1:
                    cell.walls["bottom"] = True
                if c == 0:
                    cell.walls["left"] = True
                if c == self.cols - 1:
                    cell.walls["right"] = True

    def get_cell(self, col: int, row: int) -> Optional[DiscoveredCell]:
        if 0 <= col < self.cols and 0 <= row < self.rows:
            return self.grid[row][col]
        return None

    def update_cell_from_sensors(
        self,
        col: int,
        row: int,
        heading_dir: int,
        front_wall: bool,
        left_wall: bool,
        right_wall: bool,
    ):
        """Updates cell walls using robot's relative sensor readings converted to world coordinates."""
        cell = self.get_cell(col, row)
        if not cell:
            return

        if not cell.visited:
            cell.visited = True
            cell.first_visited_time = time.time()
        cell.visit_count += 1

        # Relative directions in robot local frame
        # Front -> heading_dir
        # Right -> (heading_dir + 1) % 4
        # Left  -> (heading_dir + 3) % 4
        world_front_key = WORLD_WALL_KEYS[heading_dir]
        world_right_key = WORLD_WALL_KEYS[(heading_dir + 1) % 4]
        world_left_key = WORLD_WALL_KEYS[(heading_dir + 3) % 4]

        cell.walls[world_front_key] = front_wall
        cell.walls[world_left_key] = left_wall
        cell.walls[world_right_key] = right_wall

        # Propagate reciprocal walls to adjacent cells for consistency
        # Front neighbor
        dfx, dfy = DIR_VECTORS[heading_dir]
        front_neighbor = self.get_cell(col + dfx, row + dfy)
        if front_neighbor:
            front_neighbor.walls[OPPOSITE_WALL[world_front_key]] = front_wall

        # Left neighbor
        dlx, dly = DIR_VECTORS[(heading_dir + 3) % 4]
        left_neighbor = self.get_cell(col + dlx, row + dly)
        if left_neighbor:
            left_neighbor.walls[OPPOSITE_WALL[world_left_key]] = left_wall

        # Right neighbor
        drx, dry = DIR_VECTORS[(heading_dir + 1) % 4]
        right_neighbor = self.get_cell(col + drx, row + dry)
        if right_neighbor:
            right_neighbor.walls[OPPOSITE_WALL[world_right_key]] = right_wall

    def is_passable(self, c1: int, r1: int, c2: int, r2: int) -> bool:
        """Returns True if movement between adjacent cells (c1, r1) -> (c2, r2) is known to be open."""
        cell1 = self.get_cell(c1, r1)
        cell2 = self.get_cell(c2, r2)
        if not cell1 or not cell2:
            return False

        dx = c2 - c1
        dy = r2 - r1
        if abs(dx) + abs(dy) != 1:
            return False

        # Determine direction from cell1 to cell2
        target_dir = None
        for d, (vx, vy) in DIR_VECTORS.items():
            if vx == dx and vy == dy:
                target_dir = d
                break

        if target_dir is None:
            return False

        wall_key = WORLD_WALL_KEYS[target_dir]
        opp_wall_key = OPPOSITE_WALL[wall_key]

        # Blocked if either side is known to have a wall
        if cell1.walls.get(wall_key) is True or cell2.walls.get(opp_wall_key) is True:
            return False

        return True

    def get_accessible_unvisited_neighbors(
        self, col: int, row: int, current_heading: int = NORTH
    ) -> List[Tuple[int, int, int]]:
        """Returns list of (next_col, next_row, target_direction) for adjacent unvisited cells with NO wall.

        Prioritizes straight ahead (current_heading) -> right -> left -> back.
        """
        cell = self.get_cell(col, row)
        if not cell:
            return []

        # Order candidates: Front, Right, Left, Back
        candidate_dirs = [
            current_heading,
            (current_heading + 1) % 4,
            (current_heading + 3) % 4,
            (current_heading + 2) % 4,
        ]

        result = []
        for d in candidate_dirs:
            wall_key = WORLD_WALL_KEYS[d]
            # Must NOT have a wall
            if cell.walls.get(wall_key) is True:
                continue

            dx, dy = DIR_VECTORS[d]
            nc, nr = col + dx, row + dy
            neighbor = self.get_cell(nc, nr)
            if neighbor and not neighbor.visited:
                result.append((nc, nr, d))

        return result

    def find_path_bfs(
        self, start_col: int, start_row: int, goal_col: int, goal_row: int
    ) -> Optional[List[Tuple[int, int]]]:
        """Finds shortest passable path from start to goal over currently known map."""
        if start_col == goal_col and start_row == goal_row:
            return [(start_col, start_row)]

        queue = collections.deque([(start_col, start_row)])
        came_from: Dict[Tuple[int, int], Tuple[int, int]] = {}
        visited: Set[Tuple[int, int]] = {(start_col, start_row)}

        while queue:
            curr_c, curr_r = queue.popleft()
            if curr_c == goal_col and curr_r == goal_row:
                path = []
                curr = (goal_col, goal_row)
                while curr in came_from:
                    path.append(curr)
                    curr = came_from[curr]
                path.append((start_col, start_row))
                return path[::-1]

            cell = self.get_cell(curr_c, curr_r)
            if not cell:
                continue

            for d, (dx, dy) in DIR_VECTORS.items():
                wall_key = WORLD_WALL_KEYS[d]
                if cell.walls.get(wall_key) is True:
                    continue

                nc, nr = curr_c + dx, curr_r + dy
                neighbor = self.get_cell(nc, nr)
                if neighbor and (nc, nr) not in visited:
                    if self.is_passable(curr_c, curr_r, nc, nr):
                        visited.add((nc, nr))
                        came_from[(nc, nr)] = (curr_c, curr_r)
                        queue.append((nc, nr))

        return None

    def find_closest_unvisited_frontier(
        self, curr_col: int, curr_row: int
    ) -> Optional[Tuple[int, int, List[Tuple[int, int]]]]:
        """Finds the closest reachable unvisited cell from current position.

        Returns (frontier_col, frontier_row, path_to_frontier) or None if fully explored.
        """
        all_unvisited_candidates = []
        for r in range(self.rows):
            for c in range(self.cols):
                candidate = self.grid[r][c]
                if not candidate.visited:
                    path = self.find_path_bfs(curr_col, curr_row, c, r)
                    if path:
                        all_unvisited_candidates.append((len(path), c, r, path))

        if not all_unvisited_candidates:
            return None

        # Sort by shortest path length
        all_unvisited_candidates.sort(key=lambda item: item[0])
        _, best_c, best_r, best_path = all_unvisited_candidates[0]
        return best_c, best_r, best_path

    def get_statistics(self) -> Dict[str, Any]:
        """Calculates discovery completeness and metrics."""
        total_cells = self.rows * self.cols
        visited_cells = sum(1 for r in range(self.rows) for c in range(self.cols) if self.grid[r][c].visited)
        wall_count = 0
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                if cell.walls.get("top") is True:
                    wall_count += 1
                if cell.walls.get("left") is True:
                    wall_count += 1
                if r == self.rows - 1 and cell.walls.get("bottom") is True:
                    wall_count += 1
                if c == self.cols - 1 and cell.walls.get("right") is True:
                    wall_count += 1

        coverage = (visited_cells / total_cells) * 100.0 if total_cells > 0 else 0.0
        return {
            "total_cells": total_cells,
            "visited_cells": visited_cells,
            "unvisited_cells": total_cells - visited_cells,
            "coverage_percent": round(coverage, 1),
            "estimated_unique_walls": wall_count,
        }

    def render_ascii(
        self,
        robot_pos: Optional[Tuple[int, int]] = None,
        robot_heading: Optional[int] = None,
        start_pos: Optional[Tuple[int, int]] = None,
        goal_pos: Optional[Tuple[int, int]] = None,
    ) -> str:
        """Renders an ASCII visualization of the discovered grid maze."""
        lines = []
        heading_sym = DIR_SYMBOLS.get(robot_heading, "R") if robot_heading is not None else "R"

        # Top wall of first row
        top_line = "+"
        for c in range(self.cols):
            cell = self.grid[0][c]
            top_line += "---+" if cell.walls.get("top") is True else "   +"
        lines.append(top_line)

        for r in range(self.rows):
            # Mid cell line
            mid_line = ""
            for c in range(self.cols):
                cell = self.grid[r][c]
                # Left wall
                if c == 0:
                    mid_line += "|" if cell.walls.get("left") is True else " "

                # Content inside cell
                content = " "
                if robot_pos and (c, r) == robot_pos:
                    content = f"[{heading_sym}]"
                elif start_pos and (c, r) == start_pos:
                    content = " S "
                elif goal_pos and (c, r) == goal_pos:
                    content = " G "
                elif cell.visited:
                    content = " . "
                else:
                    content = " ? "

                mid_line += content

                # Right wall
                mid_line += "|" if cell.walls.get("right") is True else " "
            lines.append(mid_line)

            # Bottom wall line
            bot_line = "+"
            for c in range(self.cols):
                cell = self.grid[r][c]
                bot_line += "---+" if cell.walls.get("bottom") is True else "   +"
            lines.append(bot_line)

        return "\n".join(lines)

    def to_json_dict(
        self,
        start: Optional[Tuple[int, int]] = None,
        goal: Optional[Tuple[int, int]] = None,
        path: Optional[List[Tuple[int, int]]] = None,
        commands: Optional[List[str]] = None,
    ) -> Dict[str, Any]:
        """Converts the discovered map to the exact JSON schema required by robot_map_plan.json."""
        start_coord = list(start) if start else [0, self.rows - 1]
        goal_coord = list(goal) if goal else [self.cols - 1, 0]

        walls_data = []
        for r in range(self.rows):
            for c in range(self.cols):
                cell = self.grid[r][c]
                wall_dict = {
                    "top": bool(cell.walls.get("top", False)),
                    "bottom": bool(cell.walls.get("bottom", False)),
                    "left": bool(cell.walls.get("left", False)),
                    "right": bool(cell.walls.get("right", False)),
                }
                if any(wall_dict.values()):
                    walls_data.append({"pos": [c, r], "walls": wall_dict})

        formatted_path = [[p[0], p[1]] for p in path] if path else []
        stats = self.get_statistics()

        return {
            "grid_info": {
                "rows": self.rows,
                "cols": self.cols,
                "grid_size_px": self.grid_size_px,
                "grid_size_m": self.grid_size_m,
            },
            "start": start_coord,
            "goal": goal_coord,
            "walls": walls_data,
            "path": formatted_path,
            "commands": commands or [],
            "discovery_metadata": {
                "explored_timestamp": time.time(),
                "statistics": stats,
            },
        }

    def save_to_json(
        self,
        filepath: str,
        start: Optional[Tuple[int, int]] = None,
        goal: Optional[Tuple[int, int]] = None,
        path: Optional[List[Tuple[int, int]]] = None,
        commands: Optional[List[str]] = None,
        save_plot: bool = True,
    ) -> Path:
        """Exports the discovered map to a JSON file and optionally generates PNG plot."""
        target_path = Path(filepath)
        target_path.parent.mkdir(parents=True, exist_ok=True)
        data = self.to_json_dict(start=start, goal=goal, path=path, commands=commands)
        with open(target_path, "w", encoding="utf-8") as f:
            json.dump(data, f, indent=4, ensure_ascii=False)

        if save_plot:
            try:
                plot_discovered_map(str(target_path))
            except Exception as e:
                print(f"[Plot] Warning: Could not generate map plot: {e}")

        return target_path


def plot_discovered_map(json_file: str, output_png: Optional[str] = None) -> Optional[Path]:
    """Generates a high-quality visualization plot of the discovered grid map and saves as PNG."""
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import matplotlib.patches as patches
    except ImportError:
        print("[Warning] Matplotlib not installed; skipping map plot generation.")
        return None

    json_path = Path(json_file)
    if output_png is None:
        output_png = str(json_path.with_suffix(".png"))
    out_path = Path(output_png)

    with open(json_path, "r", encoding="utf-8") as f:
        data = json.load(f)

    grid_info = data.get("grid_info", {})
    rows = grid_info.get("rows", 6)
    cols = grid_info.get("cols", 5)
    start = data.get("start", [0, rows - 1])
    goal = data.get("goal", [cols - 1, 0])
    walls_list = data.get("walls", [])
    path_list = data.get("path", [])
    metadata = data.get("discovery_metadata", {})
    stats = metadata.get("statistics", {})

    wall_map = {}
    visited_cells = set()
    for item in walls_list:
        c, r = item["pos"]
        wall_map[(c, r)] = item.get("walls", {})
        visited_cells.add((c, r))

    fig, ax = plt.subplots(figsize=(cols * 1.5 + 3.2, rows * 1.4), dpi=180)
    fig.patch.set_facecolor("#f8f9fa")
    ax.set_facecolor("#ffffff")

    # Draw Cells
    for r in range(rows):
        for c in range(cols):
            y_plot = rows - 1 - r
            x_plot = c
            is_visited = (c, r) in visited_cells
            cell_color = "#e8f5e9" if is_visited else "#f5f5f5"
            rect = patches.Rectangle(
                (x_plot, y_plot), 1, 1,
                linewidth=0.8, edgecolor="#d0d0d0", facecolor=cell_color
            )
            ax.add_patch(rect)

            status_text = f"({c},{r})"
            ax.text(
                x_plot + 0.5, y_plot + 0.5, status_text,
                ha="center", va="center", fontsize=9,
                color="#2e7d32" if is_visited else "#9e9e9e",
                weight="bold" if is_visited else "normal"
            )

    # Draw Walls
    wall_linewidth = 4.5
    wall_color = "#0d47a1"

    for (c, r), w in wall_map.items():
        y_plot = rows - 1 - r
        x_plot = c
        if w.get("top"):
            ax.plot([x_plot, x_plot + 1], [y_plot + 1, y_plot + 1], color=wall_color, linewidth=wall_linewidth, solid_capstyle="round")
        if w.get("bottom"):
            ax.plot([x_plot, x_plot + 1], [y_plot, y_plot], color=wall_color, linewidth=wall_linewidth, solid_capstyle="round")
        if w.get("left"):
            ax.plot([x_plot, x_plot], [y_plot, y_plot + 1], color=wall_color, linewidth=wall_linewidth, solid_capstyle="round")
        if w.get("right"):
            ax.plot([x_plot + 1, x_plot + 1], [y_plot, y_plot + 1], color=wall_color, linewidth=wall_linewidth, solid_capstyle="round")

    # Draw Path
    if path_list and len(path_list) > 1:
        px = [p[0] + 0.5 for p in path_list]
        py = [rows - 1 - p[1] + 0.5 for p in path_list]
        ax.plot(px, py, color="#ff9800", linewidth=3.5, linestyle="--", zorder=4, label="Discovered Path")

    # Draw Start and Goal
    if start:
        sx, sy = start[0] + 0.5, rows - 1 - start[1] + 0.5
        ax.scatter(sx, sy, s=450, color="#4caf50", edgecolors="#1b5e20", linewidth=2.5, zorder=5, label=f"Start ({start[0]},{start[1]})")
        ax.text(sx, sy, "S", ha="center", va="center", color="white", fontsize=11, weight="bold", zorder=6)

    if goal:
        gx, gy = goal[0] + 0.5, rows - 1 - goal[1] + 0.5
        ax.scatter(gx, gy, s=450, color="#f44336", edgecolors="#b71c1c", linewidth=2.5, zorder=5, label=f"Goal ({goal[0]},{goal[1]})")
        ax.text(gx, gy, "G", ha="center", va="center", color="white", fontsize=11, weight="bold", zorder=6)

    # Format Axes: Set ticks centered at each grid cell column and row
    ax.set_xlim(0, cols)
    ax.set_ylim(0, rows)
    ax.set_aspect("equal")

    # Column ticks at cell centers: 0, 1, 2, 3, 4
    ax.set_xticks([c + 0.5 for c in range(cols)])
    ax.set_xticklabels([f"Col {c}" for c in range(cols)], fontsize=10, fontweight="bold")

    # Row ticks at cell centers: Row 0 at top, Row (rows-1) at bottom
    ax.set_yticks([rows - 1 - r + 0.5 for r in range(rows)])
    ax.set_yticklabels([f"Row {r}" for r in range(rows)], fontsize=10, fontweight="bold")

    # Minor grid line ticks at boundaries
    ax.set_xticks(range(cols + 1), minor=True)
    ax.set_yticks(range(rows + 1), minor=True)
    ax.grid(which="minor", color="#e0e0e0", linestyle=":", linewidth=0.8)

    cov = stats.get("coverage_percent", round(len(visited_cells) / (rows * cols) * 100, 1))
    vis_count = stats.get("visited_cells", len(visited_cells))
    total_cells = stats.get("total_cells", rows * cols)

    plt.title(f"RoboMaster EP Discovered Grid Map ({cols}x{rows})\nCoverage: {vis_count}/{total_cells} Cells ({cov}%)", fontsize=13, fontweight="bold", pad=14)

    grid_m = grid_info.get("grid_size_m", 0.6)
    stats_lines = [
        "Exploration Stats:",
        f"- Dimensions: {cols}x{rows} grids",
        f"- Cell Size: {grid_m:.2f} m",
        f"- Visited: {vis_count}/{total_cells} ({cov}%)",
        f"- Unique Walls: {stats.get('estimated_unique_walls', len(wall_map))}",
        f"- Start: ({start[0]}, {start[1]})",
        f"- Goal: ({goal[0]}, {goal[1]})" if goal else "- Goal: None",
    ]
    plt.gcf().text(0.80, 0.5, "\n".join(stats_lines), fontsize=9.5, verticalalignment="center", bbox=dict(boxstyle="round,pad=0.6", facecolor="#ffffff", edgecolor="#cccccc", alpha=0.95))

    plt.legend(loc="upper left", bbox_to_anchor=(1.02, 1.0), frameon=True, facecolor="#ffffff")
    plt.tight_layout()
    plt.subplots_adjust(right=0.78)

    out_path.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out_path, bbox_inches="tight", dpi=180)
    plt.close()
    print(f"[Plot] Discovered map visualization saved -> {out_path}")
    return out_path


# ---------------------------------------------------------------------------
# Ground Truth Simulator for Mock Exploration Testing
# ---------------------------------------------------------------------------
class GroundTruthMazeSimulator:
    """Provides synthetic sensor readings based on a reference ground-truth maze for dry-runs."""

    def __init__(self, ground_truth_file: Optional[str] = None, cols: int = 4, rows: int = 4):
        self.cols = cols
        self.rows = rows
        self.walls: Dict[Tuple[int, int], Dict[str, bool]] = {}

        if ground_truth_file and Path(ground_truth_file).exists():
            self.load_from_json(ground_truth_file)
        else:
            self._generate_default_maze()

    def _generate_default_maze(self):
        """Creates default 4x4 maze with perimeter walls if no reference file is given."""
        for r in range(self.rows):
            for c in range(self.cols):
                w = {"top": False, "bottom": False, "left": False, "right": False}
                if r == 0:
                    w["top"] = True
                if r == self.rows - 1:
                    w["bottom"] = True
                if c == 0:
                    w["left"] = True
                if c == self.cols - 1:
                    w["right"] = True
                self.walls[(c, r)] = w

    def load_from_json(self, json_file: str):
        with open(json_file, "r", encoding="utf-8") as f:
            data = json.load(f)
        grid_info = data.get("grid_info", {})
        self.rows = grid_info.get("rows", self.rows)
        self.cols = grid_info.get("cols", self.cols)
        self.walls.clear()

        # Initialize with outer boundaries
        self._generate_default_maze()

        for w_item in data.get("walls", []):
            c, r = w_item["pos"]
            if (c, r) in self.walls:
                self.walls[(c, r)].update(w_item.get("walls", {}))

    def get_simulated_readings(
        self, col: int, row: int, heading_dir: int
    ) -> Tuple[bool, bool, bool, float, float, float]:
        """Returns (front_wall, left_wall, right_wall, sharp_left_adc, sharp_right_adc, tof_mm)."""
        w = self.walls.get((col, row), {"top": False, "bottom": False, "left": False, "right": False})
        world_front_key = WORLD_WALL_KEYS[heading_dir]
        world_right_key = WORLD_WALL_KEYS[(heading_dir + 1) % 4]
        world_left_key = WORLD_WALL_KEYS[(heading_dir + 3) % 4]

        front_wall = bool(w.get(world_front_key, False))
        left_wall = bool(w.get(world_left_key, False))
        right_wall = bool(w.get(world_right_key, False))

        # Synthetic sensor values:
        # Sharp IR: ~420 ADC = 140mm (wall detected < 280mm), ~35 ADC = 800mm (no wall)
        # ToF: ~150mm (wall detected < 350mm), ~1200mm (no wall)
        sl_adc = 420.0 if left_wall else 35.0
        sr_adc = 420.0 if right_wall else 35.0
        tof_mm = 150.0 if front_wall else 1200.0

        return front_wall, left_wall, right_wall, sl_adc, sr_adc, tof_mm


# ---------------------------------------------------------------------------
# Autonomous Maze Explorer Orchestrator
# ---------------------------------------------------------------------------
class AutonomousMazeExplorer:
    """Master orchestrator for exploring an unknown maze, mapping grid cells & walls, and exporting."""

    def __init__(
        self,
        robot_system: RobotSystem,
        start_col: int = 0,
        start_row: int = 3,
        start_heading: int = NORTH,
        cols: int = 4,
        rows: int = 4,
        output_file: str = "data/discovered_map.json",
        sim_ground_truth_file: Optional[str] = "data/robot_map_plan.json",
        step_pause_sec: float = 0.5,
    ):
        self.sys = robot_system
        self.start_col = start_col
        self.start_row = start_row
        self.start_heading = start_heading
        self.curr_col = start_col
        self.curr_row = start_row
        self.curr_heading = start_heading
        self.output_file = output_file
        self.step_pause_sec = step_pause_sec

        self.discovered_map = DiscoveredGridMap(cols=cols, rows=rows)
        self.exploration_history: List[Dict[str, Any]] = []
        self.total_forward_steps = 0
        self.total_turns = 0

        self.ground_truth_sim = None
        if self.sys.mock_mode:
            self.ground_truth_sim = GroundTruthMazeSimulator(
                ground_truth_file=sim_ground_truth_file,
                cols=cols,
                rows=rows,
            )

    def sense_current_cell(self) -> Tuple[bool, bool, bool]:
        """Reads filtered sensors from Thread 1 (or ground truth sim in mock mode) and classifies walls."""
        if self.sys.mock_mode and self.ground_truth_sim:
            fw, lw, rw, sl_adc, sr_adc, tof_mm = self.ground_truth_sim.get_simulated_readings(
                self.curr_col, self.curr_row, self.curr_heading
            )
            # Inject into Thread 1 mock collector
            if self.sys.thread_1_sensor:
                yaw_deg = 0.0
                if self.curr_heading == EAST:
                    yaw_deg = 90.0
                elif self.curr_heading == SOUTH:
                    yaw_deg = 180.0
                elif self.curr_heading == WEST:
                    yaw_deg = -90.0

                self.sys.thread_1_sensor.inject_mock_data(
                    sharp_left_adc=sl_adc,
                    sharp_right_adc=sr_adc,
                    tof_dist=tof_mm,
                    yaw=yaw_deg,
                )
            return fw, lw, rw

        # Live robot mode: Read filtered snapshot from SensorHub
        state = self.sys.sensor_hub.get_latest_state()

        # Wall detection classification as defined in Step 2 / Step 3:
        # Side wall present in current cell if distance < 220 mm (nominal centered is ~140 mm)
        # Front wall present if ToF distance < 350 mm
        wall_front = (
            state.wall_front_detected or (
                state.tof_filtered_mm is not None and state.tof_filtered_mm < 350.0
            )
        )
        wall_left = (
            state.wall_left_detected or (
                state.sharp_left_mm is not None and state.sharp_left_mm < 220.0
            )
        )
        wall_right = (
            state.wall_right_detected or (
                state.sharp_right_mm is not None and state.sharp_right_mm < 220.0
            )
        )

        return wall_front, wall_left, wall_right

    def execute_turn_to(self, target_heading: int):
        """Turns the robot from curr_heading to target_heading using RobotControllerThread."""
        if target_heading == self.curr_heading:
            return

        diff = (target_heading - self.curr_heading) % 4
        ctrl = self.sys.thread_2_controller

        if diff == 1:
            # Turn Right (+90° in grid / -90° in z SDK)
            if ctrl:
                ctrl.turn_right()
            self.total_turns += 1
        elif diff == 2:
            # Turn Around (180°)
            if ctrl:
                ctrl.turn_around()
            self.total_turns += 1
        elif diff == 3:
            # Turn Left (-90° in grid / +90° in z SDK)
            if ctrl:
                ctrl.turn_left()
            self.total_turns += 1

        self.curr_heading = target_heading
        if self.sys.thread_1_sensor and hasattr(self.sys.thread_1_sensor, "flush_filters"):
            self.sys.thread_1_sensor.flush_filters()
        if self.step_pause_sec > 0:
            time.sleep(self.step_pause_sec)

    def execute_forward_one_cell(self):
        """Moves forward 1 grid cell (60 cm) with closed-loop PID centering."""
        ctrl = self.sys.thread_2_controller
        if ctrl:
            ctrl.navigate_single_grid_step(step_idx=1, total_steps=1)
        else:
            time.sleep(0.1)

        dx, dy = DIR_VECTORS[self.curr_heading]
        self.curr_col += dx
        self.curr_row += dy
        self.total_forward_steps += 1

        if self.sys.thread_1_sensor and hasattr(self.sys.thread_1_sensor, "flush_filters"):
            self.sys.thread_1_sensor.flush_filters()
        if self.step_pause_sec > 0:
            time.sleep(self.step_pause_sec)

    def navigate_path(self, path: List[Tuple[int, int]]):
        """Navigates step-by-step along a planned path between discovered cells."""
        for i in range(len(path) - 1):
            c1, r1 = path[i]
            c2, r2 = path[i + 1]
            dx = c2 - c1
            dy = r2 - r1

            target_dir = None
            for d, (vx, vy) in DIR_VECTORS.items():
                if vx == dx and vy == dy:
                    target_dir = d
                    break

            if target_dir is not None:
                self.execute_turn_to(target_dir)
                self.execute_forward_one_cell()

    def explore(self, max_steps: int = 150) -> Dict[str, Any]:
        """Runs the autonomous maze exploration algorithm (DFS + Frontier Backtracking)."""
        print("\n" + "=" * 70)
        print("🧭 STARTING AUTONOMOUS GRID MAZE EXPLORATION (MAPPING MODULE)")
        print(f"   Start Position: ({self.start_col}, {self.start_row}) | Heading: {DIR_NAMES[self.start_heading]}")
        print(f"   Grid Size: {self.discovered_map.cols}x{self.discovered_map.rows} cells (60x60 cm/cell)")
        print("=" * 70 + "\n")

        step_count = 0
        start_time = time.time()

        while step_count < max_steps:
            step_count += 1
            print(f"\n--- [Exploration Step {step_count}] At Grid ({self.curr_col}, {self.curr_row}) | Facing {DIR_NAMES[self.curr_heading]} ---")

            # 1. Sense environment at current cell
            fw, lw, rw = self.sense_current_cell()
            detected_str = []
            if fw:
                detected_str.append("Front")
            if lw:
                detected_str.append("Left")
            if rw:
                detected_str.append("Right")
            wall_info = ", ".join(detected_str) if detected_str else "None (Open)"
            print(f"  [Sensor Observation] Sensed Walls: [{wall_info}]")

            # 2. Update Discovered Grid Map
            self.discovered_map.update_cell_from_sensors(
                col=self.curr_col,
                row=self.curr_row,
                heading_dir=self.curr_heading,
                front_wall=fw,
                left_wall=lw,
                right_wall=rw,
            )

            # 3. Print Live ASCII Map
            ascii_map = self.discovered_map.render_ascii(
                robot_pos=(self.curr_col, self.curr_row),
                robot_heading=self.curr_heading,
                start_pos=(self.start_col, self.start_row),
            )
            print(ascii_map)

            # Record history
            self.exploration_history.append(
                {
                    "step": step_count,
                    "pos": [self.curr_col, self.curr_row],
                    "heading": DIR_NAMES[self.curr_heading],
                    "sensed_walls": detected_str,
                }
            )

            # 4. Decide next action:
            # Strategy A: Check adjacent unvisited neighbors (DFS local exploration)
            unvisited_neighbors = self.discovered_map.get_accessible_unvisited_neighbors(
                self.curr_col, self.curr_row, self.curr_heading
            )

            if unvisited_neighbors:
                # Pick the first neighbor (prioritized: straight > right > left > back)
                next_c, next_r, target_dir = unvisited_neighbors[0]
                print(f"  [Decision: DFS Forward] Moving to unvisited neighbor ({next_c}, {next_r}) facing {DIR_NAMES[target_dir]}...")
                self.execute_turn_to(target_dir)
                self.execute_forward_one_cell()
            else:
                # Strategy B: Dead-end reached or all adjacent neighbors visited -> Backtrack to closest frontier
                print("  [Decision: Backtrack] No unvisited adjacent neighbors. Searching for nearest frontier...")
                frontier_result = self.discovered_map.find_closest_unvisited_frontier(
                    self.curr_col, self.curr_row
                )

                if frontier_result is None:
                    # All reachable cells have been visited!
                    print("\n🎉 [EXPLORATION COMPLETE] All reachable cells in the maze have been fully mapped!")
                    break

                target_fc, target_fr, backtrack_path = frontier_result
                print(f"  [Frontier Found] Navigating {len(backtrack_path)-1} steps to frontier ({target_fc}, {target_fr})...")
                self.navigate_path(backtrack_path)

        elapsed_time = time.time() - start_time
        stats = self.discovered_map.get_statistics()
        stats["exploration_duration_sec"] = round(elapsed_time, 2)
        stats["total_steps_taken"] = step_count
        stats["total_forward_moves"] = self.total_forward_steps
        stats["total_turns"] = self.total_turns

        # Generate sample path from start to farthest cell or opposite corner
        goal_candidate = (self.discovered_map.cols - 1, 0)
        final_path = self.discovered_map.find_path_bfs(
            self.start_col, self.start_row, goal_candidate[0], goal_candidate[1]
        )

        # 5. Export Discovered Map to JSON file
        saved_file = self.discovered_map.save_to_json(
            filepath=self.output_file,
            start=(self.start_col, self.start_row),
            goal=goal_candidate,
            path=final_path,
        )
        print("\n" + "=" * 70)
        print(f"💾 [MAP EXPORTED] Discovered map saved to: {saved_file}")
        print(f"   Coverage: {stats['visited_cells']}/{stats['total_cells']} cells ({stats['coverage_percent']}%)")
        print(f"   Forward Steps: {self.total_forward_steps} | Turns: {self.total_turns} | Duration: {elapsed_time:.1f}s")
        print("=" * 70 + "\n")

        return {
            "statistics": stats,
            "output_file": str(saved_file),
            "history": self.exploration_history,
        }
