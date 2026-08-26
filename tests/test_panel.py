#!/usr/bin/env python3
"""Tests for the Mission Control panel: map, pathfinding, mapping, tracking, robot.

Runs headless - no pygame window, no hardware.

    python tests/test_panel.py
    python -m unittest tests.test_panel
"""

import io
import math
import os
import sys
import tempfile
import threading
import time
import unittest

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from src.panel.explorer import FrontierExplorer
from src.panel.geometry import DIR_VECTORS, CoordinateTransform, RobotPose, heading_to_dir, wrap180
from src.panel.mapper import OccupancyMapper
from src.panel.mission import MODE_SIM, MissionConfig, MissionController
from src.panel.occupancy import FREE, OBSTACLE, UNKNOWN, WALL, OccupancyGrid
from src.panel.pathfinding import (
    astar,
    count_turns,
    path_is_valid,
    path_to_commands,
    plan_mission,
)
from src.panel.robot_state import RobotStateTracker, RobotStatus
from src.panel.robot_iface import SimRobotInterface
from src.panel.sensors import SENSOR_SPECS, SimulatedSensorInterface, raycast_cells
from src.panel.simulation import SimRobot, SimulationEngine, ground_truth_from
from src.panel.ui import theme


def open_grid(width, height):
    grid = OccupancyGrid(width, height)
    grid.add_border()
    grid.mark_all_known()
    grid.start = (0, 0)
    grid.goal = (width - 1, height - 1)
    grid.robot_cell = (0, 0)
    return grid


# ==========================================================================
# Map
# ==========================================================================

class TestMap(unittest.TestCase):
    def test_arbitrary_sizes(self):
        for width, height in ((2, 2), (6, 2), (6, 6), (9, 9), (20, 20), (30, 30)):
            grid = open_grid(width, height)
            self.assertEqual((grid.width, grid.height), (width, height))
            self.assertEqual(grid.stats()["total"], width * height)
            # Border is closed on every side.
            self.assertTrue(grid.has_wall(0, 0, 0))
            self.assertTrue(grid.has_wall(0, 0, 3))
            self.assertTrue(grid.has_wall(width - 1, height - 1, 1))
            self.assertTrue(grid.has_wall(width - 1, height - 1, 2))

    def test_resize_preserves_overlap(self):
        grid = open_grid(6, 6)
        grid.set_wall(2, 2, 1, True)
        grid.resize(9, 9, keep=True)
        self.assertTrue(grid.has_wall(2, 2, 1))
        self.assertEqual(grid.width, 9)

    def test_wall_edges_are_symmetric(self):
        grid = open_grid(4, 4)
        grid.set_wall(1, 1, 1, True)          # right side of (1,1)
        self.assertTrue(grid.has_wall(2, 1, 3))  # is the left side of (2,1)
        grid.toggle_wall(2, 1, 3)
        self.assertFalse(grid.has_wall(1, 1, 1))

    def test_eraser_and_clear(self):
        grid = open_grid(5, 5)
        grid.set_wall(2, 2, 0, True)
        grid.set_wall(2, 2, 0, False)
        self.assertFalse(grid.has_wall(2, 2, 0))
        grid.set_wall(3, 3, 1, True)
        grid.clear_walls(keep_border=True)
        self.assertFalse(grid.has_wall(3, 3, 1))
        self.assertTrue(grid.has_wall(0, 0, 3))

    def test_long_wall_and_maze(self):
        grid = open_grid(9, 9)
        for row in range(1, 8):
            grid.set_wall(4, row, 1, True)
        for row in range(1, 8):
            self.assertTrue(grid.has_wall(5, row, 3))

    def test_random_map_is_solvable_or_reported(self):
        grid = OccupancyGrid(12, 12)
        grid.random_map(wall_density=0.25, seed=7)
        result = plan_mission(grid, grid.start, grid.goal)
        self.assertIn(result.ok, (True, False))
        if not result.ok:
            self.assertIn("NO VALID PATH", result.reason)

    def test_rotate_swaps_dimensions_and_moves_everything(self):
        grid = open_grid(6, 9)
        grid.start = (0, 0)
        grid.goal = (5, 8)
        grid.checkpoints = [(3, 2)]
        grid.robot_cell = (1, 4)
        grid.robot_dir = 1                     # East
        grid.set_wall(2, 4, 1, True)           # east side of (2,4)
        grid.rotate(1)                         # 90 deg clockwise

        self.assertEqual((grid.width, grid.height), (9, 6))
        # (col, row) -> (old_height - 1 - row, col)
        self.assertEqual(grid.start, (8, 0))
        self.assertEqual(grid.goal, (0, 5))
        self.assertEqual(grid.checkpoints, [(6, 3)])
        self.assertEqual(grid.robot_cell, (4, 1))
        self.assertEqual(grid.robot_dir, 2)    # East turned clockwise is South
        # The east wall of (2,4) becomes the south wall of its rotated cell.
        self.assertTrue(grid.has_wall(4, 2, 2))
        self.assertFalse(grid.has_wall(4, 2, 1))
        # Border survives the rotation intact.
        self.assertTrue(grid.has_wall(0, 0, 0))
        self.assertTrue(grid.has_wall(8, 5, 1))
        self.assertTrue(grid.has_wall(8, 5, 2))

    def test_rotate_four_times_is_identity(self):
        grid = open_grid(7, 4)
        grid.random_map(0.3, seed=5)
        grid.checkpoints = [(2, 1), (5, 3)]
        grid.robot_cell = (3, 2)
        grid.robot_dir = 3
        before = grid.to_dict()
        grid.rotate(4)
        self.assertEqual(grid.to_dict(), before)

    def test_rotate_counter_clockwise_is_three_clockwise(self):
        one = open_grid(5, 3)
        one.set_wall(1, 1, 0, True)
        other = open_grid(5, 3)
        other.set_wall(1, 1, 0, True)
        one.rotate(-1)
        other.rotate(3)
        self.assertEqual(one.to_dict(), other.to_dict())

    def test_rotate_preserves_cell_states(self):
        grid = open_grid(4, 3)
        grid.set(1, 2, OBSTACLE)
        grid.rotate(1)
        self.assertEqual(grid.get(0, 1), OBSTACLE)

    def test_save_load_roundtrip(self):
        grid = open_grid(7, 5)
        grid.set_wall(3, 2, 1, True)
        grid.checkpoints = [(1, 1), (5, 3)]
        grid.set(2, 2, OBSTACLE)
        handle, path = tempfile.mkstemp(suffix=".json")
        os.close(handle)
        try:
            grid.save(path)
            loaded = OccupancyGrid.load(path)
            self.assertEqual((loaded.width, loaded.height), (7, 5))
            self.assertTrue(loaded.has_wall(3, 2, 1))
            self.assertEqual(loaded.checkpoints, [(1, 1), (5, 3)])
            self.assertEqual(loaded.get(2, 2), OBSTACLE)
            self.assertEqual(loaded.start, (0, 0))
        finally:
            os.unlink(path)

    def test_legacy_plan_file_loads(self):
        legacy = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
                              "data", "robot_map_plan.json")
        if not os.path.exists(legacy):
            self.skipTest("legacy plan file not present")
        grid = OccupancyGrid.load(legacy)
        self.assertGreater(grid.width, 0)
        self.assertGreater(grid.height, 0)
        self.assertIsNotNone(grid.start)


# ==========================================================================
# Pathfinding
# ==========================================================================

class TestPathfinding(unittest.TestCase):
    def test_simple_path(self):
        grid = open_grid(6, 6)
        path = astar(grid, (0, 0), (5, 5))
        self.assertIsNotNone(path)
        self.assertEqual(path[0], (0, 0))
        self.assertEqual(path[-1], (5, 5))
        self.assertEqual(len(path) - 1, 10)   # Manhattan distance in an open room

    def test_walls_are_not_traversable(self):
        grid = open_grid(3, 1)
        grid.set_wall(0, 0, 1, True)
        self.assertIsNone(astar(grid, (0, 0), (2, 0)))

    def test_impossible_path_is_reported(self):
        grid = open_grid(5, 5)
        for row in range(5):
            grid.set_wall(2, row, 1, True)
        result = plan_mission(grid, (0, 0), (4, 4))
        self.assertFalse(result.ok)
        self.assertIn("NO VALID PATH", result.reason)
        self.assertEqual(result.cells, [])

    def test_maze_path_is_valid(self):
        grid = open_grid(9, 9)
        for row in range(0, 7):
            grid.set_wall(3, row, 1, True)
        for row in range(2, 9):
            grid.set_wall(6, row, 1, True)
        result = plan_mission(grid, (0, 0), (8, 8))
        self.assertTrue(result.ok, result.reason)
        self.assertTrue(path_is_valid(grid, result.cells))

    def test_multiple_checkpoints_are_all_visited(self):
        grid = open_grid(8, 8)
        checkpoints = [(6, 1), (1, 6), (4, 4)]
        result = plan_mission(grid, (0, 0), (7, 7), checkpoints=checkpoints)
        self.assertTrue(result.ok, result.reason)
        for cp in checkpoints:
            self.assertIn(cp, result.cells)
        self.assertEqual(sorted(result.order), [0, 1, 2])
        self.assertEqual(len(result.waypoints), 5)

    def test_checkpoint_order_is_optimised(self):
        grid = open_grid(10, 3)
        # Deliberately given out of order; the planner should reorder them.
        checkpoints = [(7, 1), (2, 1)]
        result = plan_mission(grid, (0, 1), (9, 1), checkpoints=checkpoints)
        self.assertTrue(result.ok)
        self.assertEqual(result.order, [1, 0])
        self.assertEqual(result.steps, 9)

    def test_unknown_cells_avoided_unless_allowed(self):
        grid = OccupancyGrid(5, 1, fill=UNKNOWN)
        grid.add_border()
        for col in range(5):
            grid.set(col, 0, FREE)
        grid.set(2, 0, UNKNOWN)
        self.assertIsNone(astar(grid, (0, 0), (4, 0), allow_unknown=False))
        self.assertIsNotNone(astar(grid, (0, 0), (4, 0), allow_unknown=True))

    def test_turn_penalty_reduces_turns(self):
        grid = open_grid(7, 7)
        plain = astar(grid, (0, 0), (6, 6), turn_penalty=0.0, start_dir=1)
        fewest = astar(grid, (0, 0), (6, 6), turn_penalty=3.0, start_dir=1)
        self.assertLessEqual(count_turns(fewest, 1), count_turns(plain, 1))

    def test_path_to_commands_matches_legacy_format(self):
        cells = [(0, 0), (1, 0), (2, 0), (2, 1)]
        commands = path_to_commands(cells, start_dir=1)
        self.assertEqual(commands, ["Move Forward: 2 cells", "Turn Right (90 deg)",
                                    "Move Forward: 1 cells"])

    def test_metrics_are_reported(self):
        grid = open_grid(5, 5)
        result = plan_mission(grid, (0, 0), (4, 4), speed_mps=0.25)
        self.assertTrue(result.ok)
        self.assertEqual(result.steps, 8)
        self.assertAlmostEqual(result.distance_m, 8 * 0.60, places=6)
        self.assertGreater(result.est_time_s, 0.0)

    def test_dynamic_obstacle_invalidates_path(self):
        grid = open_grid(5, 1)
        result = plan_mission(grid, (0, 0), (4, 0))
        self.assertTrue(result.ok)
        grid.set(2, 0, OBSTACLE)
        self.assertFalse(path_is_valid(grid, result.cells))
        replanned = plan_mission(grid, (0, 0), (4, 0))
        self.assertFalse(replanned.ok)   # 1-wide corridor: correctly reports no path


# ==========================================================================
# Coordinate transform
# ==========================================================================

class TestGeometry(unittest.TestCase):
    def test_origin_is_not_assumed_to_be_map_origin(self):
        transform = CoordinateTransform(origin_col=3, origin_row=4, cell_size_m=0.6, start_dir=0)
        pose = transform.robot_to_map(RobotPose(0.0, 0.0, 0.0))
        self.assertAlmostEqual(pose.col, 3.0)
        self.assertAlmostEqual(pose.row, 4.0)

    def test_forward_motion_follows_start_heading(self):
        transform = CoordinateTransform(origin_col=2, origin_row=5, cell_size_m=0.6, start_dir=0)
        pose = transform.robot_to_map(RobotPose(0.6, 0.0, 0.0))
        self.assertEqual(pose.cell, (2, 4))            # north is -row
        transform.start_dir = 1
        pose = transform.robot_to_map(RobotPose(0.6, 0.0, 0.0))
        self.assertEqual(pose.cell, (3, 5))            # east is +col

    def test_lateral_axis_is_robot_right(self):
        transform = CoordinateTransform(origin_col=2, origin_row=2, cell_size_m=0.6, start_dir=0)
        pose = transform.robot_to_map(RobotPose(0.0, 0.6, 0.0))
        self.assertEqual(pose.cell, (3, 2))            # facing north, right is east

    def test_yaw_maps_to_clockwise_heading(self):
        transform = CoordinateTransform(start_dir=0)
        self.assertAlmostEqual(transform.robot_to_map(RobotPose(0, 0, 90.0)).heading_deg, 90.0)
        self.assertEqual(heading_to_dir(90.0), 1)

    def test_roundtrip(self):
        transform = CoordinateTransform(origin_col=4, origin_row=1, cell_size_m=0.5,
                                        start_dir=2, handedness=1)
        for col, row in ((0, 0), (3, 7), (9, 2)):
            pose = transform.map_to_robot(col, row, 0.0)
            back = transform.robot_to_map(pose)
            self.assertAlmostEqual(back.col, col, places=6)
            self.assertAlmostEqual(back.row, row, places=6)

    def test_rebase(self):
        transform = CoordinateTransform()
        transform.rebase((5, 6), 180.0)
        self.assertEqual(transform.start_dir, 2)
        pose = transform.robot_to_map(RobotPose(0, 0, 0))
        self.assertEqual(pose.cell, (5, 6))
        self.assertAlmostEqual(abs(pose.heading_deg), 180.0)

    def test_wrap180(self):
        self.assertAlmostEqual(wrap180(370.0), 10.0)
        self.assertAlmostEqual(wrap180(-190.0), 170.0)


# ==========================================================================
# Sensors + mapping
# ==========================================================================

class TestSensorMapping(unittest.TestCase):
    def test_raycast_stops_at_wall(self):
        grid = open_grid(5, 1)
        grid.set_wall(2, 0, 1, True)
        # Facing east from cell 0: wall face after 2.5 cells.
        distance = raycast_cells(grid, 0, 0, 90.0, 6.0)
        self.assertAlmostEqual(distance, 2.5, delta=0.05)

    def test_raycast_open_run(self):
        grid = open_grid(6, 1)
        distance = raycast_cells(grid, 0, 0, 90.0, 4.0)
        self.assertAlmostEqual(distance, 4.0, delta=0.05)

    def test_mapper_writes_walls_and_free_cells(self):
        truth = open_grid(5, 5)
        discovered = OccupancyGrid(5, 5, fill=UNKNOWN)
        transform = CoordinateTransform(origin_col=2, origin_row=2, cell_size_m=0.6, start_dir=0)
        robot = SimRobot(truth, transform, start_cell=(2, 2), start_dir=0)
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False, update_rate_hz=1000.0)
        mapper = OccupancyMapper(discovered, transform)

        update = mapper.integrate(sensors.read())
        self.assertTrue(update.changed)
        self.assertEqual(discovered.get(2, 2), FREE)
        # Facing north in an open room: the cell ahead is discovered free.
        self.assertEqual(discovered.get(2, 1), FREE)

    def test_mapper_detects_a_wall_in_front(self):
        truth = open_grid(5, 5)
        truth.set_wall(2, 2, 0, True)
        discovered = OccupancyGrid(5, 5, fill=UNKNOWN)
        transform = CoordinateTransform(origin_col=2, origin_row=2, cell_size_m=0.6, start_dir=0)
        robot = SimRobot(truth, transform, start_cell=(2, 2), start_dir=0)
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False, update_rate_hz=1000.0)
        mapper = OccupancyMapper(discovered, transform)
        # One noisy frame must not create a wall; two agreeing frames must.
        mapper.integrate(sensors.read())
        self.assertFalse(discovered.has_wall(2, 2, 0))
        sensors._last_time = 0.0
        mapper.integrate(sensors.read())
        self.assertTrue(discovered.has_wall(2, 2, 0))

    def test_simulated_sensors_are_not_perfect(self):
        truth = open_grid(6, 6)
        transform = CoordinateTransform(origin_col=1, origin_row=1, cell_size_m=0.6)
        robot = SimRobot(truth, transform, start_cell=(1, 1))
        noisy = SimulatedSensorInterface(truth, robot, transform, noise=True, seed=3,
                                         update_rate_hz=100000.0)
        values = []
        for _ in range(40):
            reading = noisy.read()
            noisy._last_time = 0.0          # force a fresh sample
            values.append(reading.front_mm)
        distinct = set(v for v in values if v is not None)
        self.assertGreater(len(distinct), 1, "simulated sensor should carry noise")

    def test_sharp_sensors_saturate_beyond_range(self):
        truth = open_grid(10, 10)
        transform = CoordinateTransform(origin_col=5, origin_row=5, cell_size_m=0.6)
        robot = SimRobot(truth, transform, start_cell=(5, 5))
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False, update_rate_hz=1e6)
        reading = sensors.read()
        left = [s for s in SENSOR_SPECS if s.name == "left"][0]
        self.assertAlmostEqual(reading.left_mm, left.max_range_mm, delta=1.0)
        self.assertFalse(reading.left_valid)     # saturated, not a real echo

    def test_no_rear_sensor_is_exposed(self):
        truth = open_grid(4, 4)
        transform = CoordinateTransform(origin_col=1, origin_row=1)
        robot = SimRobot(truth, transform, start_cell=(1, 1))
        reading = SimulatedSensorInterface(truth, robot, transform, noise=False).read()
        self.assertIsNone(reading.back_mm)
        self.assertEqual([label for label, _ in reading.as_display()],
                         [spec.label for spec in SENSOR_SPECS])


# ==========================================================================
# Frontier exploration
# ==========================================================================

class TestExploration(unittest.TestCase):
    def _blank(self, width, height, robot_cell=(0, 0)):
        grid = OccupancyGrid(width, height, fill=UNKNOWN)
        grid.add_border()
        grid.set(robot_cell[0], robot_cell[1], FREE)
        return grid

    def test_frontier_is_free_cell_next_to_unknown(self):
        grid = self._blank(5, 5, (2, 2))
        frontiers = grid.frontier_cells()
        self.assertEqual(frontiers, [(2, 2)])

    def test_no_frontier_when_fully_known(self):
        grid = open_grid(4, 4)
        grid.fill(FREE)
        self.assertEqual(grid.frontier_cells(), [])
        explorer = FrontierExplorer(grid)
        self.assertIsNone(explorer.select((0, 0)))

    def test_unknown_edges_also_make_a_frontier(self):
        grid = self._blank(4, 4, (0, 0))
        grid.fill(FREE)                       # every cell known free...
        self.assertTrue(grid.frontier_cells(), "...but their walls are still unobserved")
        grid.mark_all_known()
        self.assertEqual(grid.frontier_cells(), [])

    def test_selects_reachable_frontier(self):
        grid = self._blank(6, 6, (0, 0))
        for col in range(4):
            grid.set(col, 0, FREE)
        explorer = FrontierExplorer(grid)
        target = explorer.select((0, 0), from_dir=1)
        self.assertIsNotNone(target)
        self.assertEqual(grid.get(*target.cell), FREE)
        self.assertGreater(target.gain, 0)

    def test_blacklist_is_respected(self):
        grid = self._blank(5, 5, (0, 0))
        explorer = FrontierExplorer(grid)
        target = explorer.select((0, 0))
        self.assertIsNotNone(target)
        explorer.blacklist_cell(target.cell)
        self.assertIsNone(explorer.select((0, 0)))

    def test_dead_end_penalised(self):
        grid = self._blank(5, 5, (0, 0))
        for col in range(5):
            grid.set(col, 0, FREE)
        explorer = FrontierExplorer(grid)
        self.assertGreaterEqual(explorer._safety_penalty((0, 0)), 1)


# ==========================================================================
# Robot state tracking
# ==========================================================================

class TestTracking(unittest.TestCase):
    def test_pose_updates_and_trail(self):
        transform = CoordinateTransform(origin_col=1, origin_row=4, cell_size_m=0.6, start_dir=0)
        tracker = RobotStateTracker(transform)
        tracker.update_from_pose(RobotPose(0.0, 0.0, 0.0))
        tracker.update_from_pose(RobotPose(0.6, 0.0, 0.0), velocity_xy=(0.4, 0.0))
        state = tracker.get()
        self.assertTrue(state.valid)
        self.assertEqual(state.cell, (1, 3))
        self.assertAlmostEqual(state.velocity, 0.4, places=6)
        self.assertEqual(len(tracker.trail()), 2)
        tracker.clear_trail()
        self.assertEqual(tracker.trail(), [])

    def test_heading_updates(self):
        tracker = RobotStateTracker(CoordinateTransform(start_dir=0))
        tracker.update_from_pose(RobotPose(0, 0, 90.0))
        self.assertAlmostEqual(tracker.get().map_heading, 90.0)

    def test_tracking_timeout_fires_once(self):
        tracker = RobotStateTracker(CoordinateTransform(), tracking_timeout_s=0.05)
        tracker.update_from_pose(RobotPose(0, 0, 0))
        self.assertTrue(tracker.tracking_ok())
        time.sleep(0.08)
        self.assertFalse(tracker.tracking_ok())
        self.assertTrue(tracker.check_timeout())
        self.assertFalse(tracker.check_timeout())   # only once per event

    def test_status_is_authoritative(self):
        tracker = RobotStateTracker(CoordinateTransform())
        tracker.set_status(RobotStatus.EMERGENCY_STOP)
        tracker.update_from_pose(RobotPose(0, 0, 0))
        self.assertEqual(tracker.get().status, RobotStatus.EMERGENCY_STOP)

    def test_communication_loss_marks_stale(self):
        tracker = RobotStateTracker(CoordinateTransform())
        tracker.update_from_pose(RobotPose(0, 0, 0))
        tracker.mark_stale()
        self.assertFalse(tracker.tracking_ok())
        self.assertFalse(tracker.get().valid)


# ==========================================================================
# Simulated robot + interface
# ==========================================================================

class TestSimRobot(unittest.TestCase):
    def _rig(self, grid=None, cell=(1, 1), direction=0):
        truth = grid or open_grid(5, 5)
        transform = CoordinateTransform(cell_size_m=0.6)
        robot = SimRobot(truth, transform, start_cell=cell, start_dir=direction)
        return truth, transform, robot

    def test_move_one_cell(self):
        truth, transform, robot = self._rig(cell=(2, 3), direction=0)
        robot.command_move(0.6)
        for _ in range(400):
            robot.step(0.02)
            if not robot.is_busy():
                break
        pose = transform.robot_to_map(robot.pose())
        self.assertEqual(pose.cell, (2, 2))

    def test_turn_right(self):
        truth, transform, robot = self._rig()
        robot.command_turn(90.0)
        for _ in range(400):
            robot.step(0.02)
            if not robot.is_busy():
                break
        # The heading snaps to the grid axis with the same residual the real
        # controller leaves behind, so compare against that tolerance.
        self.assertAlmostEqual(robot.pose().yaw_deg, 90.0, delta=robot.centering_error_deg + 0.1)
        self.assertEqual(heading_to_dir(transform.robot_to_map(robot.pose()).heading_deg), 1)

    def test_robot_stops_at_a_wall(self):
        grid = open_grid(4, 4)
        grid.set_wall(1, 1, 0, True)
        truth, transform, robot = self._rig(grid, cell=(1, 1), direction=0)
        robot.command_move(0.6)
        for _ in range(400):
            robot.step(0.02)
            if not robot.is_busy():
                break
        self.assertTrue(robot.blocked())

    def test_interface_reports_failure_when_blocked(self):
        grid = open_grid(4, 4)
        grid.set_wall(1, 1, 0, True)
        truth, transform, robot = self._rig(grid, cell=(1, 1), direction=0)
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False)
        tracker = RobotStateTracker(transform)
        engine = SimulationEngine(robot, sensors, tracker, speed=8.0)
        engine.start_engine()
        try:
            interface = SimRobotInterface(robot, sensors, engine, cell_size_m=0.6)
            interface.connect()
            result = interface.move_cells(1)
            self.assertFalse(result.ok)
            self.assertIn("Blocked", result.reason)
        finally:
            engine.stop_engine()

    def test_emergency_stop_blocks_motion(self):
        truth, transform, robot = self._rig()
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False)
        interface = SimRobotInterface(robot, sensors, None, cell_size_m=0.6)
        interface.connect()
        interface.emergency_stop()
        self.assertTrue(interface.emergency_stopped())
        self.assertFalse(interface.move_cells(1).ok)
        interface.clear_emergency_stop()
        self.assertFalse(interface.emergency_stopped())

    def test_disconnected_interface_refuses_motion(self):
        truth, transform, robot = self._rig()
        sensors = SimulatedSensorInterface(truth, robot, transform, noise=False)
        interface = SimRobotInterface(robot, sensors, None, cell_size_m=0.6)
        self.assertFalse(interface.move_cells(1).ok)


# ==========================================================================
# Mission controller (end to end, simulation)
# ==========================================================================

class TestMissionController(unittest.TestCase):
    def _controller(self, grid, **overrides):
        config = MissionConfig(sim_noise=False, sim_speed=10.0, tracking_timeout_s=30.0)
        for key, value in overrides.items():
            setattr(config, key, value)
        controller = MissionController(grid, config)
        controller.mode = MODE_SIM
        return controller

    def test_connect_plan_and_run_reaches_goal(self):
        grid = open_grid(6, 6)
        grid.start = (0, 0)
        grid.goal = (3, 2)
        grid.robot_cell = (0, 0)
        controller = self._controller(grid)
        try:
            ok, reason = controller.connect()
            self.assertTrue(ok, reason)
            result = controller.plan()
            self.assertTrue(result.ok, result.reason)
            self.assertTrue(controller.start_navigation())
            deadline = time.time() + 60
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            self.assertEqual(controller.navigation_status, "COMPLETE")
            self.assertEqual(controller.tracker.get().cell, (3, 2))
        finally:
            controller.shutdown()

    def test_checkpoints_are_marked_done(self):
        grid = open_grid(6, 6)
        grid.start = (0, 0)
        grid.goal = (0, 3)
        grid.checkpoints = [(2, 0)]
        grid.robot_cell = (0, 0)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            self.assertTrue(controller.start_navigation())
            deadline = time.time() + 90
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            states = [wp[2] for wp in controller.mission_waypoints]
            self.assertTrue(all(s == "done" for s in states), states)
        finally:
            controller.shutdown()

    def test_emergency_stop_aborts_mission(self):
        grid = open_grid(10, 10)
        grid.start = (0, 0)
        grid.goal = (9, 9)
        grid.robot_cell = (0, 0)
        controller = self._controller(grid, sim_speed=1.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            self.assertTrue(controller.start_navigation())
            time.sleep(0.4)
            controller.emergency_stop()
            deadline = time.time() + 20
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            self.assertFalse(controller.mission_active())
            self.assertTrue(controller.robot.emergency_stopped())
            self.assertFalse(controller.armed)
            self.assertNotEqual(controller.tracker.get().cell, (9, 9))
        finally:
            controller.shutdown()

    def test_run_without_path_is_refused(self):
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.path_result = controller.path_result.__class__()
            self.assertFalse(controller.start_navigation())
        finally:
            controller.shutdown()

    def test_auto_mapping_discovers_a_room(self):
        grid = open_grid(5, 5)
        grid.start = (0, 0)
        grid.goal = (4, 4)
        grid.robot_cell = (0, 0)
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.begin_auto_mapping())
            deadline = time.time() + 120
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.1)
            self.assertEqual(controller.mapping_status, "COMPLETE")
            stats = controller.map.stats()
            self.assertEqual(stats["unknown"], 0)
            self.assertIsNot(controller.map, controller.design_map)
            self.assertIsNotNone(controller.ground_truth)
            # The discovered walls must match the hidden ground truth exactly.
            discovered, truth = controller.map, controller.ground_truth
            mismatches = [
                (c, r, d)
                for r in range(discovered.height)
                for c in range(discovered.width)
                for d in range(4)
                if discovered.has_wall(c, r, d) != truth.has_wall(c, r, d)
            ]
            self.assertEqual(mismatches, [])
        finally:
            controller.shutdown()

    def test_auto_mapping_keeps_ground_truth_hidden(self):
        grid = open_grid(4, 4)
        controller = self._controller(grid)
        discovered = controller.prepare_auto_mapping()
        self.assertEqual(discovered.stats()["unknown"], 16)
        self.assertIsNot(discovered, controller.design_map)
        self.assertFalse(controller.reveal_ground_truth)

    def test_mode_switch_is_explicit_and_blocked_while_running(self):
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            controller.start_navigation()
            self.assertFalse(controller.set_mode("REAL ROBOT"))
        finally:
            controller.shutdown()

    def test_dynamic_obstacle_is_discovered_and_routed_around(self):
        grid = open_grid(7, 3)
        grid.start = (0, 1)
        grid.goal = (6, 1)
        grid.robot_cell = (0, 1)
        grid.robot_dir = 1                       # already facing the goal
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            self.assertTrue(controller.start_navigation())
            # Drop an obstacle in the corridor mid-run; only the world knows.
            time.sleep(0.5)
            controller.ground_truth.set_wall(3, 1, 1, True, known=True)
            deadline = time.time() + 90
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            self.assertTrue(controller.map.has_wall(3, 1, 1),
                            "the robot should have learned about the obstacle")
            self.assertGreater(controller.replan_count, 1)
            self.assertEqual(controller.tracker.get().cell, (6, 1))
        finally:
            controller.shutdown()

    def test_sealed_corridor_reports_no_valid_path(self):
        grid = open_grid(7, 1)
        grid.start = (0, 0)
        grid.goal = (6, 0)
        grid.robot_cell = (0, 0)
        grid.robot_dir = 1
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            self.assertTrue(controller.start_navigation())
            time.sleep(0.5)
            controller.ground_truth.set_wall(3, 0, 1, True, known=True)
            deadline = time.time() + 90
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            self.assertNotEqual(controller.tracker.get().cell, (6, 0))
            self.assertEqual(controller.navigation_status, "NO VALID PATH")
        finally:
            controller.shutdown()

    def test_exploration_never_invents_obstacles_in_a_static_world(self):
        """The simulated field cannot change, so any OBSTACLE is a false positive.

        Bumping a wall during exploration means "there is a wall on this edge",
        not "that square is occupied" - cells are marked free speculatively from
        long ToF sweeps, and blocking one permanently corrupts the map.
        """
        for seed in (6, 2, 4):
            grid = OccupancyGrid(5, 6)
            grid.random_map(0.30, seed=seed)
            grid.robot_cell = (0, 0)
            grid.robot_dir = 0
            controller = self._controller(grid, sim_speed=10.0)
            try:
                self.assertTrue(controller.begin_auto_mapping())
                deadline = time.time() + 120
                while controller.mission_active() and time.time() < deadline:
                    controller.tick()
                    time.sleep(0.04)
                discovered = controller.map
                obstacles = [
                    (c, r)
                    for r in range(discovered.height)
                    for c in range(discovered.width)
                    if discovered.get(c, r) == OBSTACLE
                ]
                self.assertEqual(obstacles, [],
                                 "seed {} invented an obstacle".format(seed))
            finally:
                controller.shutdown()

    def test_navigation_still_records_a_genuine_new_obstacle(self):
        """Outside exploration the inference is valid and must be kept."""
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.mapping_status = "IDLE"          # navigating a known map
            self.assertEqual(controller.map.get(2, 1), FREE)
            controller._handle_blocked((2, 2), (2, 1))
            self.assertEqual(controller.map.get(2, 1), OBSTACLE)
            self.assertTrue(controller.map.has_wall(2, 2, 0))
        finally:
            controller.shutdown()

    def test_exploration_records_the_wall_but_not_an_obstacle(self):
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.mapping_status = "ACTIVE"        # exploring
            controller._handle_blocked((2, 2), (2, 1))
            self.assertTrue(controller.map.has_wall(2, 2, 0),
                            "the blocked edge is always a wall")
            self.assertNotEqual(controller.map.get(2, 1), OBSTACLE)
        finally:
            controller.shutdown()

    def test_auto_mapping_drives_every_reachable_cell(self):
        """Full coverage: exploration is not done until each reachable square
        has actually been driven through, not merely observed from next door."""
        grid = OccupancyGrid(5, 6)
        grid.random_map(0.30, seed=0)
        grid.robot_cell = (0, 0)
        grid.robot_dir = 0
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.begin_auto_mapping())
            deadline = time.time() + 240
            while controller.mission_active() and time.time() < deadline:
                controller.tick()
                time.sleep(0.04)
            self.assertEqual(controller.mapping_status, "COMPLETE")
            driven, reachable = controller.coverage_summary()
            self.assertGreater(reachable, 1)
            self.assertGreaterEqual(driven, reachable,
                                    "every reachable cell must be driven through")
        finally:
            controller.shutdown()

    def test_route_is_planned_once_mapping_finishes(self):
        grid = open_grid(4, 4)
        grid.start = (0, 0)
        grid.goal = (3, 3)
        grid.robot_cell = (0, 0)
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.begin_auto_mapping())
            deadline = time.time() + 240
            while controller.mission_active() and time.time() < deadline:
                controller.tick()
                time.sleep(0.04)
            self.assertEqual(controller.mapping_status, "COMPLETE")
            self.assertTrue(controller.path_result.ok, controller.path_result.reason)
            self.assertEqual(controller.path_result.cells[-1], (3, 3))
        finally:
            controller.shutdown()

    def test_coverage_can_be_switched_off(self):
        grid = open_grid(4, 4)
        controller = self._controller(grid, full_coverage=False)
        self.assertIsNone(
            controller.explorer.select((0, 0), visited=None),
            "with coverage off a fully known map has nothing left to do")
        controller.shutdown()

    def test_rotate_map_keeps_the_robot_frame_in_sync(self):
        grid = open_grid(6, 9)
        grid.start = (0, 0)
        grid.goal = (5, 8)
        grid.robot_cell = (0, 0)
        grid.robot_dir = 1
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertEqual(controller.tracker.get().cell, (0, 0))
            self.assertTrue(controller.rotate_map(1))
            self.assertEqual((controller.map.width, controller.map.height), (9, 6))
            self.assertTrue(controller.wait_for_pose(2.0))
            # The live robot must land on the rotated start cell, not the old one.
            self.assertEqual(controller.map.robot_cell, (8, 0))
            self.assertEqual(controller.tracker.get().cell, (8, 0))
            self.assertTrue(controller.plan().ok)
            # Ground truth rotates with it, so the simulator agrees with the map.
            self.assertEqual((controller.ground_truth.width,
                              controller.ground_truth.height), (9, 6))
        finally:
            controller.shutdown()

    def test_rotate_map_refused_while_running(self):
        grid = open_grid(8, 8)
        controller = self._controller(grid, sim_speed=1.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            controller.start_navigation()
            self.assertFalse(controller.rotate_map(1))
            self.assertFalse(controller.rotate_robot_start(1))
            self.assertEqual((controller.map.width, controller.map.height), (8, 8))
        finally:
            controller.shutdown()

    def test_rotate_robot_start_changes_heading(self):
        grid = open_grid(5, 5)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.rotate_robot_start(1))
            self.assertEqual(controller.map.robot_dir, 1)
            self.assertTrue(controller.wait_for_pose(2.0))
            self.assertEqual(heading_to_dir(controller.tracker.get().map_heading), 1)
            controller.rotate_robot_start(-1)
            self.assertEqual(controller.map.robot_dir, 0)
        finally:
            controller.shutdown()

    def test_jog_turn_rotates_the_actual_robot(self):
        grid = open_grid(5, 5)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.jog_turn(90.0))
            deadline = time.time() + 30
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.02)
            self.assertEqual(heading_to_dir(controller.tracker.get().map_heading), 1)
            self.assertEqual(controller.tracker.get().cell, (2, 2))
        finally:
            controller.shutdown()

    def test_stop_latches_until_explicitly_cleared(self):
        grid = open_grid(6, 6)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.robot.stop()
            # A stop must not be forgotten by the next command in the queue.
            self.assertFalse(controller.robot.turn(90.0).ok)
            self.assertFalse(controller.robot.move_cells(1).ok)
            controller.robot.clear_stop()
            self.assertTrue(controller.robot.turn(90.0).ok)
        finally:
            controller.shutdown()

    def test_heading_is_verified_and_corrected_after_a_turn(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            # Simulate the chassis ending a turn 30 deg short of the grid axis.
            with controller.sim_robot._lock:
                controller.sim_robot._yaw = 30.0
            self.assertTrue(controller.tracker.wait_for_update(timeout=2.0))
            self.assertGreater(abs(controller.tracker.get().map_heading), 20.0)

            result = controller._settle_heading(0)
            self.assertTrue(result.ok, result.reason)
            error = abs(wrap180(controller.tracker.get().map_heading))
            self.assertLessEqual(error, controller.config.heading_tolerance_deg)
        finally:
            controller.shutdown()

    def test_step_refuses_to_drive_on_an_unverified_heading(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        controller = self._controller(grid)
        controller.config.heading_retries = 0        # no corrective turns allowed
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            with controller.sim_robot._lock:
                controller.sim_robot._yaw = 40.0
            self.assertTrue(controller.tracker.wait_for_update(timeout=2.0))
            result = controller._settle_heading(0)
            self.assertFalse(result.ok)
            self.assertIn("Heading not reached", result.reason)
        finally:
            controller.shutdown()

    def test_set_origin_reanchors_the_frame_after_placing_the_robot(self):
        grid = open_grid(8, 8)
        grid.robot_cell = (0, 0)
        grid.robot_dir = 0
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertEqual(controller.tracker.get().cell, (0, 0))
            # Operator picks the robot up and puts it on a different square,
            # facing a different way, then tells the panel where it is.
            controller.map.robot_cell = (5, 6)
            controller.map.robot_dir = 1
            self.assertTrue(controller.set_origin_here())
            self.assertTrue(controller.wait_for_pose(2.0))
            self.assertEqual(controller.tracker.get().cell, (5, 6))
            self.assertEqual(heading_to_dir(controller.tracker.get().map_heading), 1)
        finally:
            controller.shutdown()

    def test_squares_up_before_moving_even_when_no_turn_is_needed(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0                      # already facing the next cell
        controller = self._controller(grid)
        controller.config.heading_retries = 0   # no correction allowed
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            with controller.sim_robot._lock:
                controller.sim_robot._yaw = 25.0
            self.assertTrue(controller.tracker.wait_for_update(timeout=2.0))

            # Straight ahead, so no turn is scheduled - it must still refuse to
            # drive while crooked instead of veering into the wall.
            result = controller._step_to((2, 1))
            self.assertFalse(result.ok)
            self.assertIn("Heading not reached", result.reason)
            self.assertEqual(controller.tracker.get().cell, (2, 2), "must not have moved")
        finally:
            controller.shutdown()

    def test_crooked_heading_is_corrected_then_the_move_proceeds(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0
        controller = self._controller(grid, sim_speed=10.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            with controller.sim_robot._lock:
                controller.sim_robot._yaw = 20.0
            self.assertTrue(controller.tracker.wait_for_update(timeout=2.0))

            result = controller._step_to((2, 1))
            self.assertTrue(result.ok, result.reason)
            self.assertEqual(controller.tracker.get().cell, (2, 1))
            self.assertLessEqual(abs(wrap180(controller.tracker.get().map_heading)),
                                 controller.config.heading_tolerance_deg)
        finally:
            controller.shutdown()

    def test_front_obstacle_stops_the_step_before_driving(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        grid.robot_dir = 0
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            # Something appears 90 mm in front, inside the hard-stop envelope.
            controller._front_clearance_mm = lambda: 90.0
            result = controller._step_to((2, 1))
            self.assertFalse(result.ok)
            self.assertIn("Blocked", result.reason)
            self.assertEqual(controller.tracker.get().cell, (2, 2), "must not have moved")
        finally:
            controller.shutdown()

    def test_off_path_step_is_reported_for_replanning(self):
        grid = open_grid(6, 6)
        grid.robot_cell = (2, 2)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.wait_for_pose(2.0))
            # The leg was planned from a cell the robot is not standing in.
            result = controller._step_to((4, 4), expected_from=(4, 3))
            self.assertFalse(result.ok)
            self.assertTrue(result.reason.startswith("Off path"), result.reason)
        finally:
            controller.shutdown()

    def test_robot_speed_can_be_changed_live(self):
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.set_robot_speed(0.10)
            self.assertAlmostEqual(controller.config.base_speed_mps, 0.10)
            self.assertAlmostEqual(controller.sim_robot.base_speed, 0.10)
            controller.set_robot_speed(0.22)
            self.assertAlmostEqual(controller.sim_robot.base_speed, 0.22)
        finally:
            controller.shutdown()

    def test_set_origin_needs_a_connection(self):
        controller = self._controller(open_grid(5, 5))
        try:
            self.assertFalse(controller.set_origin_here())
        finally:
            controller.shutdown()

    def test_jog_turn_blocked_by_emergency_stop(self):
        grid = open_grid(5, 5)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.emergency_stop()
            self.assertFalse(controller.jog_turn(90.0))
        finally:
            controller.shutdown()

    def test_tracking_timeout_stops_the_mission(self):
        grid = open_grid(8, 8)
        grid.start = (0, 0)
        grid.goal = (7, 7)
        grid.robot_cell = (0, 0)
        controller = self._controller(grid, sim_speed=1.0, tracking_timeout_s=0.2)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.plan().ok)
            self.assertTrue(controller.start_navigation())
            time.sleep(0.3)
            controller.sim_engine.pause()      # simulates losing the pose stream
            controller.tracker.mark_stale()
            controller.tracker.update_from_pose(controller.sim_robot.pose())
            time.sleep(0.5)
            controller.tick()
            deadline = time.time() + 15
            while controller.mission_active() and time.time() < deadline:
                controller.tick()
                time.sleep(0.05)
            self.assertFalse(controller.mission_active())
            self.assertEqual(controller.tracker.get_status(), RobotStatus.TRACKING_LOST)
        finally:
            controller.shutdown()


# ==========================================================================
# Gripper: pick the bottle up and place it facing a marked direction
# ==========================================================================

class TestCarryMission(unittest.TestCase):
    def _controller(self, grid, **overrides):
        config = MissionConfig(sim_noise=False, sim_speed=10.0, tracking_timeout_s=30.0)
        for key, value in overrides.items():
            setattr(config, key, value)
        controller = MissionController(grid, config)
        controller.mode = MODE_SIM
        return controller

    def _field(self):
        grid = open_grid(6, 6)
        grid.start = (0, 0)
        grid.goal = (5, 2)               # the bottle
        grid.robot_cell = (0, 0)
        grid.robot_dir = 1
        grid.delivery_cell = (1, 5)         # where it must end up
        grid.delivery_dir = 2               # facing South when released
        grid.objects = {grid.goal}       # the bottle actually standing there
        grid.object_cell = grid.goal     # ...and the Object tool marks it
        return grid

    def test_place_point_survives_save_and_load(self):
        grid = self._field()
        handle, path = tempfile.mkstemp(suffix=".json")
        os.close(handle)
        try:
            grid.save(path)
            loaded = OccupancyGrid.load(path)
            self.assertEqual(loaded.delivery_cell, (1, 5))
            self.assertEqual(loaded.delivery_dir, 2)
        finally:
            os.unlink(path)

    def test_place_point_rotates_with_the_map(self):
        grid = self._field()               # 6x6, place (1,5) facing South
        grid.rotate(1)                     # 90 deg clockwise
        self.assertEqual(grid.delivery_cell, (0, 1))
        self.assertEqual(grid.delivery_dir, 3)   # South turned clockwise is West

    def test_place_point_is_dropped_when_resized_away(self):
        grid = self._field()
        grid.resize(2, 2, keep=True)
        self.assertIsNone(grid.delivery_cell)

    def test_simulated_gripper_tracks_what_it_holds(self):
        controller = self._controller(self._field())
        try:
            self.assertTrue(controller.connect()[0])
            robot = controller.robot
            # Put an object within reach of the gripper.
            controller.ground_truth.objects.add(controller.tracker.get().cell)
            self.assertTrue(robot.has_gripper())
            self.assertFalse(robot.carrying)
            self.assertTrue(robot.pick().ok)
            self.assertTrue(robot.carrying)
            self.assertFalse(robot.pick().ok, "must not pick twice")
            self.assertTrue(robot.place().ok)
            self.assertFalse(robot.carrying)
            self.assertFalse(robot.place().ok, "nothing left to place")
        finally:
            controller.shutdown()

    def test_delivery_faces_the_marked_direction_around_a_detour(self):
        grid = self._field()
        for row in range(0, 4):
            grid.set_wall(3, row, 1, True)     # force a detour
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.start_pickup_mission())
            self._run_mission(controller)
            self.assertTrue(controller.robot.carrying)

            self.assertTrue(controller.start_delivery_mission())
            self._run_mission(controller)
            state = controller.tracker.get()
            self.assertEqual(controller.navigation_status, "COMPLETE")
            self.assertEqual(state.cell, grid.delivery_cell)
            self.assertEqual(heading_to_dir(state.map_heading), grid.delivery_dir)
            self.assertFalse(controller.robot.carrying, "the object must be released")
        finally:
            controller.shutdown()

    def test_object_detector_tells_a_can_from_the_wall(self):
        """The map says how far the wall is; anything much closer is an object."""
        from src.panel.objects import ObjectDetector
        from src.panel.sensors import SensorReading
        from src.panel.geometry import RobotPose

        grid = open_grid(5, 5)
        transform = CoordinateTransform(origin_col=2, origin_row=2,
                                        cell_size_m=0.6, start_dir=0)
        detector = ObjectDetector(grid, transform, confirm_frames=2)

        def reading(distance_m):
            return SensorReading(front_mm=distance_m * 1000.0, front_valid=True,
                                 pose=RobotPose(0.0, 0.0, 0.0))

        # Wall two cells north: the beam should run ~1.5 m. A 1.5 m echo is
        # the wall, not a can.
        grid.set_wall(2, 0, 0, True)
        self.assertFalse(detector.detect(reading(1.5)).present)
        # A can standing in the next cell reads far shorter than that.
        detector.reset()
        self.assertFalse(detector.detect(reading(0.30)).present, "needs 2 frames")
        self.assertTrue(detector.detect(reading(0.30)).present)

    def test_detector_is_silent_while_carrying(self):
        """Holding something must not read as a new object to grab."""
        from src.panel.objects import ObjectDetector
        from src.panel.sensors import SensorReading
        from src.panel.geometry import RobotPose

        grid = open_grid(5, 5)
        transform = CoordinateTransform(origin_col=2, origin_row=2, cell_size_m=0.6)
        detector = ObjectDetector(grid, transform, confirm_frames=1)
        held = SensorReading(front_mm=120.0, front_valid=True,
                             pose=RobotPose(0.0, 0.0, 0.0))
        self.assertTrue(detector.detect(held, carrying=False).present)
        self.assertFalse(detector.detect(held, carrying=True).present)
        self.assertTrue(detector.front_sensor_blinded(True))

    def test_front_sensor_is_masked_while_carrying(self):
        """The payload sits in the beam, so it must not become a mapped wall."""
        grid = self._field()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertFalse(controller.mapper.ignore_front)
            controller.robot.carrying = True
            self.assertTrue(controller.sync_gripper_state())
            self.assertTrue(controller.mapper.ignore_front)
            self.assertFalse(controller.mapper.obstacle_ahead(
                controller.latest_reading(), stop_distance_mm=500.0))
            controller.robot.carrying = False
            controller.sync_gripper_state()
            self.assertFalse(controller.mapper.ignore_front)
        finally:
            controller.shutdown()

    def _run_mission(self, controller, seconds=300):
        deadline = time.time() + seconds
        while controller.mission_active() and time.time() < deadline:
            controller.tick()
            time.sleep(0.05)

    def test_object_place_then_delivery(self):
        """OBJECT PLACE grabs it; DELIVERY takes it to the aimed square."""
        controller = self._controller(self._field())
        grid = controller.map
        try:
            self.assertTrue(controller.connect()[0])

            self.assertTrue(controller.start_pickup_mission())
            self._run_mission(controller)
            self.assertEqual(controller.navigation_status, "HOLDING")
            self.assertTrue(controller.robot.carrying)
            # It pulls up beside the object rather than onto it.
            cell = controller.tracker.get().cell
            self.assertEqual(
                abs(cell[0] - grid.goal[0]) + abs(cell[1] - grid.goal[1]), 1)

            self.assertTrue(controller.start_delivery_mission())
            self._run_mission(controller)
            self.assertEqual(controller.navigation_status, "COMPLETE")
            self.assertFalse(controller.robot.carrying)
            self.assertEqual(controller.tracker.get().cell, grid.delivery_cell)
        finally:
            controller.shutdown()

    def test_delivery_refuses_with_an_empty_gripper(self):
        controller = self._controller(self._field())
        try:
            self.assertTrue(controller.connect()[0])
            self.assertFalse(controller.robot.carrying)
            self.assertFalse(controller.start_delivery_mission())
        finally:
            controller.shutdown()

    def test_object_place_refuses_when_already_loaded(self):
        controller = self._controller(self._field())
        try:
            self.assertTrue(controller.connect()[0])
            controller.robot.carrying = True
            self.assertFalse(controller.start_pickup_mission())
        finally:
            controller.shutdown()

    def test_pickup_sweeps_by_turning_until_the_object_is_ahead(self):
        """The ToF is bolted to the chassis, so finding means turning."""
        grid = self._field()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertIsNotNone(controller._approach(grid.goal, "object"))
            facing = heading_to_dir(controller.tracker.get().map_heading)
            # Deliberately face away from the object before sweeping.
            self.assertTrue(controller._face_direction((facing + 2) % 4))
            detection = controller._scan_for_object()
            self.assertIsNotNone(detection)
            self.assertTrue(detection.present)
        finally:
            controller.shutdown()

    def test_scan_gives_up_after_a_full_circle(self):
        grid = self._field()
        grid.objects = set()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.ground_truth.objects = set()
            self.assertIsNone(controller._scan_for_object())
        finally:
            controller.shutdown()

    def test_object_place_works_without_a_delivery_point(self):
        """Picking up does not depend on knowing where it will go."""
        grid = self._field()
        grid.delivery_cell = None
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.start_pickup_mission())
            self._run_mission(controller)
            self.assertTrue(controller.robot.carrying)
            self.assertEqual(controller.navigation_status, "HOLDING")
            # It pulls up beside the object rather than onto it.
            cell = controller.tracker.get().cell
            self.assertEqual(
                abs(cell[0] - grid.goal[0]) + abs(cell[1] - grid.goal[1]), 1)
        finally:
            controller.shutdown()

    def test_delivery_goes_straight_to_the_place_point(self):
        """A loaded gripper must deliver, not drive back to the goal first."""
        grid = self._field()
        grid.objects = set()          # nothing at the goal to fetch
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.ground_truth.objects = set()
            controller.robot.carrying = True      # already holding something
            controller.sync_gripper_state()

            self.assertTrue(controller.start_delivery_mission())
            self._run_mission(controller)
            self.assertEqual(controller.navigation_status, "COMPLETE")
            self.assertFalse(controller.robot.carrying)
            self.assertEqual(controller.tracker.get().cell, grid.delivery_cell)
        finally:
            controller.shutdown()

    def test_carry_reports_when_there_is_nothing_to_pick_up(self):
        grid = self._field()
        grid.objects = set()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.ground_truth.objects = set()
            self.assertTrue(controller.start_pickup_mission())
            deadline = time.time() + 300
            while controller.mission_active() and time.time() < deadline:
                controller.tick()
                time.sleep(0.05)
            self.assertFalse(controller.robot.carrying)
            self.assertEqual(controller.navigation_status, "NOTHING FOUND")
        finally:
            controller.shutdown()

    def test_back_to_start_drives_home(self):
        grid = self._field()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            # Drive somewhere else first.
            self.assertTrue(controller._drive_to((3, 3), "detour"))
            self.assertEqual(controller.tracker.get().cell, (3, 3))

            self.assertTrue(controller.start_return_to_start())
            deadline = time.time() + 300
            while controller.mission_active() and time.time() < deadline:
                controller.tick()
                time.sleep(0.05)
            self.assertEqual(controller.navigation_status, "AT START")
            self.assertEqual(controller.tracker.get().cell, grid.start)
        finally:
            controller.shutdown()

    def test_back_to_start_needs_a_start_cell(self):
        grid = self._field()
        grid.start = None
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertFalse(controller.start_return_to_start())
        finally:
            controller.shutdown()

    def test_delivery_target_grid_is_nine_by_nine(self):
        from src.panel.ui.place_dialog import DELIVERY_TARGET, SubCellTargetDialog

        self.assertEqual(SubCellTargetDialog.DIVISIONS, 9)
        grid = self._field()
        dialog = SubCellTargetDialog(grid, kind=DELIVERY_TARGET)
        # The middle sub-square is the cell centre.
        centre = dialog._offset_for(4, 4)
        self.assertAlmostEqual(centre[0], 0.0)
        self.assertAlmostEqual(centre[1], 0.0)
        # Round-tripping a chosen square selects the same square again.
        grid.delivery_offset = dialog._offset_for(7, 2)
        self.assertEqual(dialog._selected_indices(), (7, 2))

    def test_place_offset_roundtrips_and_rotates(self):
        grid = self._field()
        grid.delivery_offset = (0.3, -0.2)
        handle, path = tempfile.mkstemp(suffix=".json")
        os.close(handle)
        try:
            grid.save(path)
            loaded = OccupancyGrid.load(path)
            self.assertAlmostEqual(loaded.delivery_offset[0], 0.3)
            self.assertAlmostEqual(loaded.delivery_offset[1], -0.2)
        finally:
            os.unlink(path)
        # An aim point due East becomes due South after a clockwise turn.
        grid.delivery_offset = (0.4, 0.0)
        grid.rotate(1)
        self.assertAlmostEqual(grid.delivery_offset[0], 0.0)
        self.assertAlmostEqual(grid.delivery_offset[1], 0.4)

    def test_object_target_aims_inside_the_object_square(self):
        """TARGET: OBJECT edits the object offset, not the delivery one."""
        from src.panel.ui.place_dialog import OBJECT_TARGET, SubCellTargetDialog

        grid = self._field()
        dialog = SubCellTargetDialog(grid, kind=OBJECT_TARGET)
        self.assertFalse(dialog.has_facing, "an object has no facing to set")
        self.assertEqual(dialog.title, "TARGET: OBJECT")
        self.assertEqual(dialog.cell, grid.object_cell)

        before_delivery = grid.delivery_offset
        dialog._set_offset(dialog._offset_for(6, 1))
        self.assertEqual(dialog._selected_indices(), (6, 1))
        self.assertEqual(grid.delivery_offset, before_delivery,
                         "the delivery aim must be left alone")
        self.assertNotEqual(grid.object_offset, (0.0, 0.0))

    def test_delivery_target_aims_inside_the_delivery_square(self):
        from src.panel.ui.place_dialog import DELIVERY_TARGET, SubCellTargetDialog

        grid = self._field()
        dialog = SubCellTargetDialog(grid, kind=DELIVERY_TARGET)
        self.assertTrue(dialog.has_facing)
        self.assertEqual(dialog.title, "TARGET: DELIVERY")
        self.assertEqual(dialog.cell, grid.delivery_cell)

        before_object = grid.object_offset
        dialog._set_offset(dialog._offset_for(2, 7))
        self.assertEqual(dialog._selected_indices(), (2, 7))
        self.assertEqual(grid.object_offset, before_object,
                         "the object aim must be left alone")

    def test_object_marker_saves_rotates_and_clamps(self):
        grid = self._field()
        grid.object_cell = (4, 1)
        grid.object_offset = (0.3, -0.2)
        handle, path = tempfile.mkstemp(suffix=".json")
        os.close(handle)
        try:
            grid.save(path)
            loaded = OccupancyGrid.load(path)
            self.assertEqual(loaded.object_cell, (4, 1))
            self.assertAlmostEqual(loaded.object_offset[0], 0.3)
            self.assertAlmostEqual(loaded.object_offset[1], -0.2)
        finally:
            os.unlink(path)

        # An aim due East becomes due South after a clockwise turn.
        grid.object_offset = (0.4, 0.0)
        grid.rotate(1)
        self.assertAlmostEqual(grid.object_offset[0], 0.0)
        self.assertAlmostEqual(grid.object_offset[1], 0.4)

        grid.resize(2, 2, keep=True)
        self.assertIsNone(grid.object_cell)

    def test_pickup_uses_the_object_marker_over_the_goal(self):
        grid = self._field()
        grid.object_cell = (1, 1)        # deliberately not the goal
        grid.goal = (5, 2)
        controller = self._controller(grid)
        try:
            self.assertEqual(controller.object_square(), (1, 1))
            grid.object_cell = None
            self.assertEqual(controller.object_square(), (5, 2),
                             "older maps without an Object marker fall back")
        finally:
            controller.shutdown()

    def test_legacy_place_key_still_loads(self):
        """Maps saved before the rename used a "place" block."""
        import json

        grid = open_grid(5, 5)
        data = grid.to_dict()
        data.pop("delivery", None)
        data["place"] = {"cell": [2, 3], "dir": 1, "offset": [0.25, -0.25]}
        handle, path = tempfile.mkstemp(suffix=".json")
        os.close(handle)
        try:
            with io.open(path, "w", encoding="utf-8") as fh:
                fh.write(json.dumps(data))
            loaded = OccupancyGrid.load(path)
            self.assertEqual(loaded.delivery_cell, (2, 3))
            self.assertEqual(loaded.delivery_dir, 1)
            self.assertAlmostEqual(loaded.delivery_offset[0], 0.25)
        finally:
            os.unlink(path)

    def test_marking_an_object_puts_one_in_the_simulated_world(self):
        """The Object marker is a claim about the world, so the world gets one.

        Without this the simulated ToF sweeps an empty square and OBJECT PLACE
        reports NOTHING FOUND even though the map clearly shows an object.
        """
        grid = open_grid(6, 6)
        grid.start = (0, 0)
        grid.robot_cell = (0, 0)
        grid.robot_dir = 1
        grid.object_cell = (5, 2)          # marker only - no grid.objects entry
        self.assertEqual(grid.objects, set())

        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertIn((5, 2), controller.ground_truth.objects)

            self.assertTrue(controller.start_pickup_mission())
            self._run_mission(controller)
            self.assertEqual(controller.navigation_status, "HOLDING")
            self.assertTrue(controller.robot.carrying)
        finally:
            controller.shutdown()

    def test_object_tool_places_a_real_object(self):
        """Clicking with the Object tool marks it and puts one there."""
        import pygame
        from src.panel.ui.map_view import MapView, TOOL_OBJECT

        grid = open_grid(6, 6)
        view = MapView((0, 0, 400, 400))
        view.layout(grid)
        view.tool = TOOL_OBJECT
        centre = view.cell_center_px(3, 2)
        click = pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1,
                                   pos=(int(centre[0]), int(centre[1])))
        view.handle_event(click, grid)
        self.assertEqual(grid.object_cell, (3, 2))
        self.assertIn((3, 2), grid.objects)

        # Clicking the same square again clears both.
        view.handle_event(click, grid)
        self.assertIsNone(grid.object_cell)
        self.assertNotIn((3, 2), grid.objects)

    def _deliver(self, controller):
        """Runs pickup then delivery, returning (aim, release) in cell coords."""
        self.assertTrue(controller.start_pickup_mission())
        self._run_mission(controller)
        self.assertTrue(controller.robot.carrying)
        self.assertTrue(controller.start_delivery_mission())
        self._run_mission(controller)
        return controller.delivery_aim_point(), controller.robot.last_release_point

    def _error_cm(self, aim, release, cell_m):
        return math.hypot(release[0] - aim[0], release[1] - aim[1]) * cell_m * 100

    def test_object_lands_on_the_delivery_aim_point(self):
        """The gripper releases in front of the robot, not under it, so the
        chassis has to stand back from the aim point by exactly that much."""
        grid = self._field()
        grid.delivery_offset = (0.35, -0.35)     # well off-centre in the square
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            aim, release = self._deliver(controller)
            self.assertIsNotNone(release, "the simulator must record the release")
            self.assertLess(self._error_cm(aim, release, grid.cell_size_m), 5.0)
        finally:
            controller.shutdown()

    def test_delivery_stands_off_the_square_when_the_reach_needs_it(self):
        """Standing outside the delivery square is fine - that is where the
        robot has to be for the object to land inside it."""
        grid = self._field()
        grid.delivery_offset = (0.0, 0.0)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            plan = controller.plan_delivery_pose()
            self.assertIsNotNone(plan)
            stand, target_col, target_row = plan
            reach = controller._release_offset_cells()
            facing = DIR_VECTORS[grid.delivery_dir % 4]
            aim = controller.delivery_aim_point()
            # The planned pose is exactly one reach back from the aim point.
            self.assertAlmostEqual(target_col + facing[0] * reach, aim[0], places=6)
            self.assertAlmostEqual(target_row + facing[1] * reach, aim[1], places=6)
        finally:
            controller.shutdown()

    def test_fine_positioning_never_puts_the_chassis_into_a_wall(self):
        grid = self._field()
        grid.delivery_offset = (0.0, 0.45)       # aim hard against the south side
        grid.set_wall(grid.delivery_cell[0], grid.delivery_cell[1], 2, True)
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            aim, release = self._deliver(controller)
            state = controller.tracker.get()
            self.assertTrue(controller.position_is_clear(state.map_col, state.map_row),
                            "the chassis must stay clear of the wall it aimed at")
            self.assertIsNotNone(release)
        finally:
            controller.shutdown()

    def test_position_is_clear_rejects_hugging_a_wall(self):
        grid = self._field()
        controller = self._controller(grid)
        try:
            grid.set_wall(2, 2, 0, True)         # wall on the north side of (2,2)
            clearance = controller.config.robot_clearance_m
            cell_m = grid.cell_size_m
            # Centre of the cell is fine; right up against that wall is not.
            self.assertTrue(controller.position_is_clear(2.0, 2.0))
            hugging = 2.0 - 0.5 + (clearance * 0.4) / cell_m
            self.assertFalse(controller.position_is_clear(2.0, hugging))
        finally:
            controller.shutdown()

    def test_nudge_is_capped(self):
        grid = self._field()
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.config.max_nudge_m = 0.05
            state = controller.tracker.get()
            nudge, _ = controller._delivery_nudge(state.map_col + 3.0, state.map_row)
            self.assertLessEqual(math.hypot(*nudge), 0.05 + 1e-6)
        finally:
            controller.shutdown()

    def test_delivery_needs_a_place_point(self):
        grid = self._field()
        grid.delivery_cell = None
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            controller.robot.carrying = True
            self.assertFalse(controller.start_delivery_mission())
        finally:
            controller.shutdown()

    def test_object_place_needs_a_square_to_look_in(self):
        grid = self._field()
        grid.object_cell = None
        grid.goal = None                 # no fallback square either
        controller = self._controller(grid)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertFalse(controller.start_pickup_mission())
        finally:
            controller.shutdown()

    def test_emergency_stop_aborts_a_carry_mission(self):
        grid = self._field()
        controller = self._controller(grid, sim_speed=1.0)
        try:
            self.assertTrue(controller.connect()[0])
            self.assertTrue(controller.start_pickup_mission())
            time.sleep(0.4)
            controller.emergency_stop()
            deadline = time.time() + 30
            while controller.mission_active() and time.time() < deadline:
                time.sleep(0.05)
            self.assertFalse(controller.mission_active())
            self.assertNotEqual(controller.navigation_status, "COMPLETE")
        finally:
            controller.shutdown()


# ==========================================================================
# Chassis safety net (no hardware, no network)
# ==========================================================================

class _FakeChassis(object):
    def __init__(self):
        self.calls = []

    def drive_speed(self, x=0.0, y=0.0, z=0.0, timeout=None):
        self.calls.append({"x": x, "y": y, "z": z, "timeout": timeout})


class _FakeRobot(object):
    def __init__(self):
        self.chassis = _FakeChassis()


class TestChassisSafety(unittest.TestCase):
    """`drive_speed` is continuous - the wheels hold the last velocity forever.
    These cover the two things that stop a runaway robot."""

    def test_drive_speed_arms_the_sdk_watchdog(self):
        from src.robot_controller import RobotControllerThread
        from src.sensor_pipeline import SensorHub

        robot = _FakeRobot()
        controller = RobotControllerThread(SensorHub(), robot=robot, mock_mode=False)
        controller.drive_speed(0.2, 0.0, 0.0)

        self.assertEqual(len(robot.chassis.calls), 1)
        call = robot.chassis.calls[0]
        self.assertEqual(call["x"], 0.2)
        self.assertIsNotNone(call["timeout"],
                             "every drive command must arm the auto-stop watchdog")
        self.assertEqual(call["timeout"], controller.drive_watchdog_sec)
        self.assertGreater(call["timeout"], 0.0)
        self.assertLess(call["timeout"], 2.0, "watchdog must brake quickly")

    def test_safety_net_runs_every_registered_stopper(self):
        from src.panel.robot_iface import _ChassisSafetyNet

        net = _ChassisSafetyNet()
        fired = []
        net.register(lambda: fired.append("a"))
        net.register(lambda: fired.append("b"))
        net.stop_all()
        self.assertEqual(sorted(fired), ["a", "b"])

    def test_one_failing_stopper_does_not_block_the_others(self):
        from src.panel.robot_iface import _ChassisSafetyNet

        net = _ChassisSafetyNet()
        fired = []

        def boom():
            raise RuntimeError("comms already gone")

        net.register(boom)
        net.register(lambda: fired.append("braked"))
        net.stop_all()
        self.assertEqual(fired, ["braked"])

    def test_unregister_removes_a_stopper(self):
        from src.panel.robot_iface import _ChassisSafetyNet

        net = _ChassisSafetyNet()
        fired = []
        stopper = lambda: fired.append("x")
        net.register(stopper)
        net.unregister(stopper)
        net.stop_all()
        self.assertEqual(fired, [])

    def test_panic_stop_brakes_the_chassis_directly(self):
        from src.panel.robot_iface import RealRobotInterface

        class _FakeController(object):
            def __init__(self):
                self._running = threading.Event()
                self._running.set()

        class _FakeSystem(object):
            def __init__(self):
                self.robot = _FakeRobot()
                self.thread_2_controller = _FakeController()

        interface = RealRobotInterface()
        interface.system = _FakeSystem()
        interface._panic_stop()

        calls = interface.system.robot.chassis.calls
        self.assertTrue(calls, "panic stop must command the chassis to zero")
        self.assertEqual((calls[-1]["x"], calls[-1]["y"], calls[-1]["z"]), (0, 0, 0))
        self.assertFalse(interface.system.thread_2_controller._running.is_set(),
                         "the motion loop must be told to stop as well")

    def test_panic_stop_is_safe_when_never_connected(self):
        from src.panel.robot_iface import RealRobotInterface

        interface = RealRobotInterface()
        interface._panic_stop()          # must not raise


# ==========================================================================
# Dashboard (headless)
# ==========================================================================

class TestDashboard(unittest.TestCase):
    """Renders the real pygame dashboard against the dummy SDL driver."""

    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("SDL_VIDEODRIVER", "dummy")
        os.environ.setdefault("SDL_AUDIODRIVER", "dummy")

    def _app(self, grid=None):
        from src.panel.ui.app import MissionControlApp
        grid = grid or open_grid(9, 9)
        controller = MissionController(grid, MissionConfig(sim_noise=False, sim_speed=10.0))
        return MissionControlApp(controller, size=(1600, 1000))

    def test_every_panel_renders(self):
        import pygame
        app = self._app()
        try:
            app._draw()
            app.controller.plan()
            app._draw()
            app.notify("hello")
            app._draw()
        finally:
            pygame.quit()

    def test_wall_drawing_through_the_map_view(self):
        import pygame
        app = self._app()
        try:
            app._draw()               # forces layout so pixel maths is valid
            app.act_set_tool("Wall")
            grid = app.controller.map
            view = app.map_view
            x, y = view.cell_to_px(3, 3)
            edge_pos = (x + view.cell_px // 2, y + 2)   # near the top edge of (3,3)
            view.handle_event(
                pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=edge_pos), grid)
            self.assertTrue(grid.has_wall(3, 3, 0))
            app.act_set_tool("Eraser")
            view.handle_event(
                pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=edge_pos), grid)
            view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=edge_pos), grid)
            self.assertFalse(grid.has_wall(3, 3, 0))
        finally:
            pygame.quit()

    def test_drag_draws_a_long_wall(self):
        import pygame
        app = self._app()
        try:
            app._draw()
            app.act_set_tool("Wall")
            grid = app.controller.map
            view = app.map_view
            # Drag along the top edges of row 3, cells 1..5.
            start_x, start_y = view.cell_to_px(1, 3)
            pos = (start_x + view.cell_px // 2, start_y + 2)
            view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos), grid)
            for col in range(2, 6):
                x, y = view.cell_to_px(col, 3)
                drag = (x + view.cell_px // 2, y + 2)
                view.handle_event(pygame.event.Event(pygame.MOUSEMOTION, pos=drag, rel=(1, 0),
                                                     buttons=(1, 0, 0)), grid)
            view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONUP, button=1, pos=pos), grid)
            for col in range(1, 6):
                self.assertTrue(grid.has_wall(col, 3, 0), "wall missing at column {}".format(col))
        finally:
            pygame.quit()

    def test_marker_tools_place_start_goal_checkpoints(self):
        import pygame
        app = self._app()
        try:
            app._draw()
            grid = app.controller.map
            view = app.map_view
            centre = view.cell_center_px(4, 5)
            pos = (int(centre[0]), int(centre[1]))
            for tool, check in (
                ("Start", lambda: grid.start),
                ("Goal", lambda: grid.goal),
                ("Robot", lambda: grid.robot_cell),
            ):
                app.act_set_tool(tool)
                view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos), grid)
                self.assertEqual(check(), (4, 5))
            app.act_set_tool("Checkpoint")
            view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos), grid)
            self.assertIn((4, 5), grid.checkpoints)
            view.handle_event(pygame.event.Event(pygame.MOUSEBUTTONDOWN, button=1, pos=pos), grid)
            self.assertNotIn((4, 5), grid.checkpoints)
        finally:
            pygame.quit()

    def test_real_mode_requires_confirmation(self):
        import pygame
        app = self._app()
        try:
            app.act_set_mode("REAL ROBOT")
            self.assertIsNotNone(app.modal, "switching to real hardware must ask first")
            self.assertEqual(app.controller.mode, MODE_SIM)
            app.modal._cancel()
            self.assertEqual(app.controller.mode, MODE_SIM)
            app.act_set_mode("REAL ROBOT")
            app.modal._confirm()
            self.assertEqual(app.controller.mode, "REAL ROBOT")
        finally:
            pygame.quit()

    def test_placed_robot_is_drawn_before_connecting(self):
        import pygame
        grid = open_grid(9, 9)
        grid.robot_cell = (4, 5)
        grid.robot_dir = 1
        app = self._app(grid)
        try:
            app._draw()
            self.assertFalse(app.connected(), "this checks the disconnected view")
            view = app.map_view
            on_robot = app.screen.get_at([int(v) for v in view.cell_center_px(4, 5)])
            plain = app.screen.get_at([int(v) for v in view.cell_center_px(1, 1)])
            self.assertNotEqual(on_robot, plain,
                                "the placed robot should be visible on the map")
        finally:
            pygame.quit()

    def test_placed_robot_falls_back_to_the_start_cell(self):
        import pygame
        grid = open_grid(7, 7)
        grid.robot_cell = None
        grid.start = (3, 3)
        app = self._app(grid)
        try:
            app._draw()
            view = app.map_view
            on_start = app.screen.get_at([int(v) for v in view.cell_center_px(3, 3)])
            plain = app.screen.get_at([int(v) for v in view.cell_center_px(1, 1)])
            self.assertNotEqual(on_start, plain)
        finally:
            pygame.quit()

    def test_unexplored_edges_are_drawn_differently_from_open_ones(self):
        """A never-looked-at edge must not render like a confirmed-open one."""
        import pygame

        app = self._app(open_grid(5, 5))
        try:
            # A real discovery map: everything unknown except where we have been.
            discovered = app.controller.prepare_auto_mapping()
            discovered.set(1, 1, FREE)
            app._draw()
            view = app.map_view
            x, y = view.cell_to_px(1, 1)
            # The mark is dashed, so scan the whole edge rather than one point.
            found = False
            for dx in range(1, view.cell_px):
                for dy in range(-2, 3):
                    pixel = app.screen.get_at([int(x + dx), int(y + dy)])
                    if tuple(pixel)[:3] == theme.WALL_EDGE_UNKNOWN:
                        found = True
                        break
                if found:
                    break
            self.assertTrue(found, "unexplored edge next to a known cell must be marked")
        finally:
            pygame.quit()

    def test_gripper_status_reads_empty_or_holding(self):
        import pygame

        app = self._app()
        try:
            app._draw()
            self.assertEqual(app._gripper_text(), "--", "not connected yet")
            app._do_connect()
            self.assertEqual(app._gripper_text(), "EMPTY")
            app.controller.robot.carrying = True
            self.assertEqual(app._gripper_text(), "HOLDING OBJECT")
            app._draw()
        finally:
            app.controller.shutdown()
            pygame.quit()

    def test_gripper_state_survives_a_short_window(self):
        """The ROBOT STATE panel clips first, so the header must carry it."""
        import pygame

        app = self._app()
        try:
            app._do_connect()
            app.controller.robot.carrying = True
            app._draw()

            def header_has(colour):
                for x in range(0, app.screen.get_width(), 2):
                    for y in range(0, 52, 2):
                        if tuple(app.screen.get_at((x, y)))[:3] == colour:
                            return True
                return False

            self.assertTrue(header_has(theme.OBJECT_CARRIED),
                            "holding an object must show in the header")
        finally:
            app.controller.shutdown()
            pygame.quit()

    def test_carried_object_is_drawn_on_the_robot(self):
        """The payload must be visible travelling, not vanish until put down."""
        import pygame

        app = self._app()
        try:
            app._do_connect()
            app._draw()                       # settles the smoothed robot pose
            view = app.map_view

            def carried_pixels():
                found = 0
                board = view.board_rect(app.controller.map)
                for x in range(board.x, board.right, 2):
                    for y in range(board.y, board.bottom, 2):
                        if tuple(app.screen.get_at((x, y)))[:3] == theme.OBJECT_CARRIED:
                            found += 1
                return found

            self.assertEqual(carried_pixels(), 0, "nothing held yet")
            app.controller.robot.carrying = True
            app._draw()
            self.assertGreater(carried_pixels(), 0, "the held object should be drawn")
        finally:
            app.controller.shutdown()
            pygame.quit()

    def test_toolbar_is_grouped_and_fits_the_window(self):
        import pygame
        app = self._app()
        try:
            app._draw()
            labels = [text for _, _, text in app.toolbar_labels]
            for group in ("TOOLS", "EDIT", "ROTATE MAP", "ROTATE ROBOT",
                          "FILE", "PLAN", "TURN ROBOT NOW", "VIEW", "SIM SPEED"):
                self.assertIn(group, labels)
            self.assertTrue(app.toolbar_separators)
            for button in app.buttons:
                self.assertLessEqual(button.rect.right, app.screen.get_width(),
                                     "button {!r} runs off the window".format(button.label))
        finally:
            pygame.quit()

    def test_rotate_buttons_turn_map_and_robot(self):
        import pygame
        app = self._app(open_grid(6, 9))
        try:
            app._draw()
            app.act_rotate_robot(1)
            self.assertEqual(app.controller.map.robot_dir, 1)
            app.act_rotate_map(1)
            grid = app.controller.map
            self.assertEqual((grid.width, grid.height), (9, 6))
            # The size fields follow the rotation.
            self.assertEqual(app.width_input.value, 9)
            self.assertEqual(app.height_input.value, 6)
            app.act_rotate_map(-1)
            self.assertEqual((app.controller.map.width, app.controller.map.height), (6, 9))
            app._draw()
        finally:
            pygame.quit()

    def test_resize_map_from_toolbar(self):
        import pygame
        app = self._app()
        try:
            app.width_input.set_value(20)
            app.height_input.set_value(12)
            app.act_resize()
            self.assertEqual((app.controller.map.width, app.controller.map.height), (20, 12))
            app._draw()
        finally:
            pygame.quit()


if __name__ == "__main__":
    unittest.main(verbosity=2)
