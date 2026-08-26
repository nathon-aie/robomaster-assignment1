#!/usr/bin/env python3
"""Mission controller: the state machine that ties the panel together.

Owns the working occupancy map, the coordinate transform, the robot state
tracker, the mapper, the frontier explorer and whichever robot backend is
selected.  Everything the UI does to the robot goes through here.

Safety rules enforced in this module:

* physical motion never starts on its own - a real-robot mission requires an
  explicit ARM plus an explicit RUN;
* emergency stop wins over everything and latches until cleared;
* loss of pose updates (tracking timeout) stops the robot;
* an unexpected obstacle stops the robot, updates the map and replans;
* unknown cells are not routed through unless explicitly allowed;
* a command is only reported as successful when the robot interface confirms it.
"""

import threading
import time
from collections import deque
from dataclasses import dataclass

from .explorer import FrontierExplorer
from .geometry import (
    DIR_LONG,
    DIR_VECTORS,
    CoordinateTransform,
    dir_from_delta,
    heading_to_dir,
    wrap180,
)
from .mapper import OccupancyMapper
from .objects import ObjectDetector
from .occupancy import FREE, UNKNOWN, OccupancyGrid
from .pathfinding import (
    PathResult,
    astar,
    deviation_from_path,
    path_is_valid,
    plan_mission,
)
from .robot_state import RobotStateTracker, RobotStatus
from .sensors import RealSensorInterface
from .simulation import SimRobot, SimulationEngine, ground_truth_from
from .robot_iface import (
    MockRobotInterface,
    RealRobotInterface,
    RobotCommandResult,
    SimRobotInterface,
)
from .sensors import SimulatedSensorInterface

MODE_SIM = "SIMULATION"
MODE_REAL = "REAL ROBOT"
MODE_MOCK = "MOCK ROBOT"

MISSION_IDLE = "IDLE"
MISSION_NAVIGATE = "NAVIGATE"
MISSION_AUTOMAP = "AUTO MAP"
MISSION_JOG = "MANUAL TURN"
MISSION_PICKUP = "OBJECT PLACE"
MISSION_DELIVERY = "DELIVERY"
MISSION_RETURN = "BACK TO START"
#: Kept so older callers/tests naming the combined mission still resolve.
MISSION_CARRY = MISSION_PICKUP


@dataclass
class MissionConfig:
    """Tunables, all configurable rather than hardcoded."""

    cell_size_m: float = 0.60
    #: Cruise speed on the real robot.  Deliberately gentle: the grid is only
    #: 60 cm wide, so a fast pass leaves the PID no room to centre the chassis.
    base_speed_mps: float = 0.15
    #: In-place turn rate (deg/s).  Slower turns overshoot far less.
    turn_speed_dps: float = 30.0
    #: Pause after each cell so odometry and the chassis settle before deciding.
    step_settle_s: float = 0.35
    conn_type: str = "ap"
    calibration_file: str = "calibration_output/calibration.json"
    sensor_rate_hz: float = 20.0
    nominal_side_mm: float = 140.0

    tracking_timeout_s: float = 1.5
    #: How long to wait for a fresh pose frame before acting on the cached one.
    pose_wait_s: float = 0.6
    #: A turn counts as reached within this many degrees of the grid axis.
    heading_tolerance_deg: float = 5.0
    #: Corrective turns attempted when the heading is off.
    heading_retries: int = 3
    obstacle_stop_mm: float = 200.0
    deviation_cells: float = 0.85
    allow_unknown_cells: bool = False       # "safe navigation" - off by default
    turn_penalty: float = 0.35
    #: How far the real drop sequence reverses the chassis before opening the
    #: gripper (cm).  Matches the value tuned on the robot; the object ends up
    #: roughly this far behind the Place point.  0 releases on the spot.
    place_backoff_cm: float = 50.0
    #: Quarter turns the robot may make while sweeping the ToF for the object.
    #: 4 covers a full circle from wherever it stopped.
    object_scan_turns: int = 4
    automap_max_steps: int = 4000
    #: Keep exploring until every reachable cell has actually been driven
    #: through, not just until the unknowns are resolved.
    full_coverage: bool = True
    #: Plan Start -> Goal automatically once auto-mapping finishes.
    plan_after_mapping: bool = True
    sim_noise: bool = True
    sim_speed: float = 1.0


class MissionController(object):
    """Central coordinator for the Mission Control Center."""

    def __init__(self, grid=None, config=None):
        self.config = config or MissionConfig()
        self.design_map = grid or OccupancyGrid(9, 9)
        self.design_map.mark_all_known()
        self.map = self.design_map           # working map shown to the operator
        self.ground_truth = None             # only set for simulated auto-mapping
        self.reveal_ground_truth = False     # developer/debug toggle

        self.transform = CoordinateTransform(cell_size_m=self.config.cell_size_m)
        self.tracker = RobotStateTracker(
            transform=self.transform,
            tracking_timeout_s=self.config.tracking_timeout_s,
        )
        self.mapper = OccupancyMapper(self.map, self.transform)
        self.explorer = FrontierExplorer(self.map)
        self.detector = ObjectDetector(self.map, self.transform)
        self.last_detection = None

        self.mode = MODE_SIM
        self.robot = None
        self.sensor_source = None
        self.sim_robot = None
        self.sim_engine = None

        self.armed = False
        self.path_result = PathResult()
        self.executed_index = 0              # how far along the planned path we are
        self.mission_kind = MISSION_IDLE
        self.mission_waypoints = []          # [[label, cell, state]]
        self.navigation_status = "IDLE"
        self.mapping_status = "IDLE"
        self.last_error = ""
        self.warnings = deque(maxlen=8)
        self.events = deque(maxlen=200)
        self.replan_count = 0
        self.deviation_warning = False
        self.visited_cells = set()      # cells the robot has actually stood in

        self._worker = None
        self._stop_flag = threading.Event()
        self._pause_flag = threading.Event()
        self._pause_flag.set()
        self._lock = threading.RLock()
        self._latest_reading = None
        self._map_dirty = True
        self._map_changed_since_step = False

        self.log("Mission Control ready. Mode: {}".format(self.mode))

    # ====================================================================
    # logging
    # ====================================================================
    def log(self, text, level="info"):
        stamp = time.strftime("%H:%M:%S")
        self.events.append((stamp, level, text))
        if level in ("warn", "error"):
            self.warnings.append((stamp, text))
        if level == "error":
            self.last_error = text

    def clear_error(self):
        self.last_error = ""

    # ====================================================================
    # mode / connection
    # ====================================================================
    def set_mode(self, mode):
        """Switches backend.  Never happens implicitly - the UI confirms first."""
        if mode == self.mode:
            return True
        if self.mission_active():
            self.log("Cannot change mode while a mission is running", "warn")
            return False
        self.disconnect()
        self.mode = mode
        self.armed = False
        self.log("Mode set to {}".format(mode), "warn" if mode == MODE_REAL else "info")
        return True

    def mission_active(self):
        return self._worker is not None and self._worker.is_alive()

    def connect(self):
        if self.robot is not None and self.robot.is_connected():
            return True, "Already connected"

        self.tracker.set_status(RobotStatus.CONNECTING)
        cfg = self.config

        if self.mode == MODE_SIM:
            self._build_simulation()
            result = self.robot.connect()
        elif self.mode == MODE_MOCK:
            self.robot = MockRobotInterface(
                calibration_file=cfg.calibration_file,
                sensor_rate_hz=cfg.sensor_rate_hz,
                base_speed=cfg.base_speed_mps,
                cell_size_m=cfg.cell_size_m,
            )
            self.robot.on_status = self._on_robot_status
            result = self.robot.connect()
            if result.ok:
                self._attach_real_sensors()
        else:
            self.robot = RealRobotInterface(
                conn_type=cfg.conn_type,
                calibration_file=cfg.calibration_file,
                sensor_rate_hz=cfg.sensor_rate_hz,
                base_speed=cfg.base_speed_mps,
                nominal_side_mm=cfg.nominal_side_mm,
                cell_size_m=cfg.cell_size_m,
                turn_speed_dps=cfg.turn_speed_dps,
                place_backoff_cm=cfg.place_backoff_cm,
            )
            self.robot.on_status = self._on_robot_status
            result = self.robot.connect()
            if result.ok:
                self._attach_real_sensors()

        if not result.ok:
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Connect failed: {}".format(result.reason), "error")
            return False, result.reason

        start_cell = self.map.robot_cell or self.map.start or (0, 0)
        start_dir = self.map.robot_dir
        self.transform.cell_size_m = cfg.cell_size_m
        self.transform.rebase(start_cell, start_dir * 90.0)
        self.robot.zero_odometry(start_cell, start_dir)
        self.tracker.reset()
        self.tracker.set_status(RobotStatus.READY)
        self.log("Connected ({}). Origin cell {} facing {}".format(
            self.mode, start_cell, DIR_LONG[start_dir % 4]))
        if not self.wait_for_pose(2.5):
            self.log("Connected but no pose received yet", "warn")
        if self.robot.is_physical:
            # The frame was just anchored to wherever the robot happens to be
            # standing.  Say so, loudly, before anyone presses RUN.
            self.log("Place the robot on {} facing {}, then press SET ORIGIN".format(
                start_cell, DIR_LONG[start_dir % 4]), "warn")
        return True, "Connected"

    def wait_for_pose(self, timeout=2.0):
        """Blocks briefly until the first real pose arrives (never fabricates one)."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            if self.tracker.get().valid:
                return True
            time.sleep(0.05)
        return self.tracker.get().valid

    def disconnect(self):
        self.stop_mission(reason="disconnect")
        if self.sim_engine is not None:
            self.sim_engine.stop_engine()
            self.sim_engine = None
        if self.robot is not None:
            self.robot.disconnect()
        self.robot = None
        self.sensor_source = None
        self.sim_robot = None
        self.armed = False
        self.tracker.mark_stale()
        self.tracker.set_status(RobotStatus.DISCONNECTED)
        self.log("Disconnected")

    def _on_robot_status(self, status, message):
        self.tracker.set_status(status)
        if message:
            self.log(message, "warn" if status in RobotStatus.BLOCKING else "info")

    # ------------------------------------------------------------ simulation
    def _build_simulation(self):
        cfg = self.config
        self.transform.cell_size_m = cfg.cell_size_m
        truth = self.ground_truth if self.ground_truth is not None else ground_truth_from(self.map)
        self.ground_truth = truth
        start_cell = self.map.robot_cell or self.map.start or (0, 0)
        start_dir = self.map.robot_dir

        self.sim_robot = SimRobot(
            ground_truth=truth,
            transform=self.transform,
            start_cell=start_cell,
            start_dir=start_dir,
            base_speed_mps=cfg.base_speed_mps,
        )
        self.sensor_source = SimulatedSensorInterface(
            ground_truth=truth,
            sim_robot=self.sim_robot,
            transform=self.transform,
            noise=cfg.sim_noise,
            update_rate_hz=cfg.sensor_rate_hz,
        )
        self.sim_engine = SimulationEngine(
            sim_robot=self.sim_robot,
            sensor_interface=self.sensor_source,
            tracker=self.tracker,
            mapper=self.mapper,
            speed=cfg.sim_speed,
        )
        self.sim_engine.on_map_update = self._on_map_update
        self.robot = SimRobotInterface(
            sim_robot=self.sim_robot,
            sensor_interface=self.sensor_source,
            engine=self.sim_engine,
            cell_size_m=cfg.cell_size_m,
        )
        self.robot.on_status = self._on_robot_status
        self.sim_engine.start_engine()

    def _attach_real_sensors(self):
        """Hooks the SDK's asynchronous sensor stream, no polling."""
        self.sensor_source = self.robot.sensors()
        if isinstance(self.sensor_source, RealSensorInterface):
            self.sensor_source.subscribe(self._on_sensor_reading)

    def _on_sensor_reading(self, reading):
        """Called from Thread 1 for every filtered sensor frame."""
        self._latest_reading = reading
        self.tracker.update_from_pose(
            reading.pose,
            velocity_xy=reading.velocity_xy,
            yaw_rate=reading.yaw_rate,
            timestamp=reading.timestamp,
            frame_index=reading.frame_index,
        )
        if self.mapping_status == "ACTIVE":
            update = self.mapper.integrate(reading)
            if update.changed:
                self._on_map_update(update)

    def _on_map_update(self, update):
        self._map_dirty = True
        if update.new_walls:
            self._map_changed_since_step = True

    # ====================================================================
    # map management
    # ====================================================================
    def set_map(self, grid):
        """Installs a newly edited/loaded map as the design map."""
        self.design_map = grid
        self.map = grid
        self.ground_truth = None
        self.mapper.grid = grid
        self.explorer.grid = grid
        self.detector.grid = grid
        self.detector.reset()
        self.explorer.reset()
        self.transform.cell_size_m = grid.cell_size_m or self.config.cell_size_m
        self.path_result = PathResult()
        self.mission_waypoints = []
        self._map_dirty = True

    def use_design_map(self):
        """Back to the hand-drawn map (leaves auto-mapping results behind)."""
        self.set_map(self.design_map)
        self.mapping_status = "IDLE"

    def rotate_map(self, quarter_turns=1):
        """Rotates every map layer together (positive = clockwise).

        The working map, the design map and the hidden ground truth all turn at
        once so they stay in one frame.  If the robot is connected the run is
        restarted, which re-zeroes odometry at the rotated start cell - on real
        hardware that re-zeroes the frame only, it never drives the robot.
        """
        if self.mission_active():
            self.log("Cannot rotate the map while a mission is running", "warn")
            return False
        turns = int(quarter_turns) % 4
        if turns == 0:
            return True
        for grid in self._map_layers():
            grid.rotate(turns)
        self.path_result = PathResult()
        self.mission_waypoints = []
        self.executed_index = 0
        self._map_dirty = True
        self.log("Map rotated {} deg {}".format(
            90 * (turns if turns <= 2 else 4 - turns),
            "clockwise" if turns <= 2 else "counter-clockwise"))
        if self.robot is not None and self.robot.is_connected():
            self.restart()
        return True

    def rotate_robot_start(self, quarter_turns=1):
        """Turns the heading the robot starts the mission facing."""
        if self.mission_active():
            self.log("Cannot re-aim the robot while a mission is running", "warn")
            return False
        grid = self.map
        grid.robot_dir = (grid.robot_dir + int(quarter_turns)) % 4
        if grid is not self.design_map:
            self.design_map.robot_dir = grid.robot_dir
        self._map_dirty = True
        self.log("Robot start heading: {}".format(DIR_LONG[grid.robot_dir]))
        if self.robot is not None and self.robot.is_connected():
            self.restart()
        return True

    def _map_layers(self):
        """Every grid that must stay in the same frame, without duplicates."""
        layers = [self.map]
        for grid in (self.design_map, self.ground_truth):
            if grid is not None and not any(grid is seen for seen in layers):
                layers.append(grid)
        return layers

    def prepare_auto_mapping(self):
        """Blank discovery map + hidden ground truth (simulation only)."""
        width, height = self.design_map.width, self.design_map.height
        discovered = OccupancyGrid(width, height, fill=UNKNOWN)
        discovered.cell_size_m = self.design_map.cell_size_m
        discovered.start = self.design_map.start
        discovered.goal = self.design_map.goal
        discovered.checkpoints = list(self.design_map.checkpoints)
        discovered.robot_cell = self.design_map.robot_cell or self.design_map.start or (0, 0)
        discovered.robot_dir = self.design_map.robot_dir
        if self.mode == MODE_SIM:
            self.ground_truth = ground_truth_from(self.design_map)
        self.map = discovered
        self.mapper.grid = discovered
        self.explorer.grid = discovered
        self.detector.grid = discovered
        self.detector.reset()
        self.explorer.reset()
        self.path_result = PathResult()
        self._map_dirty = True
        return discovered

    # ====================================================================
    # planning
    # ====================================================================
    def plan(self, from_cell=None, from_dir=None):
        """Plans Start -> checkpoints -> Goal on the current map."""
        grid = self.map
        start = from_cell or grid.start
        if start is None:
            self.path_result = PathResult(ok=False, reason="No start cell set")
            self.log("Planning failed: no start cell", "warn")
            return self.path_result

        result = plan_mission(
            grid,
            start=start,
            goal=grid.goal,
            checkpoints=grid.checkpoints,
            allow_unknown=self.config.allow_unknown_cells,
            turn_penalty=self.config.turn_penalty,
            start_dir=from_dir,
            speed_mps=self.config.base_speed_mps,
        )
        self.path_result = result
        self.executed_index = 0
        if result.ok:
            self.mission_waypoints = [[label, cell, "pending"] for label, cell in result.waypoints]
            if self.mission_waypoints:
                self.mission_waypoints[0][2] = "done"
            self.log("Path found: {} steps, {:.2f} m, {} turns, ~{:.0f} s".format(
                result.steps, result.distance_m, result.turns, result.est_time_s))
        else:
            self.mission_waypoints = []
            self.log("Planning failed: {}".format(result.reason), "warn")
        return result

    def _replan_from_robot(self, remaining_targets):
        """Re-solves the route from where the robot actually is, right now."""
        self.tracker.wait_for_update(timeout=self.config.pose_wait_s)
        state = self.tracker.get()
        cell = state.cell if state.valid else self.map.start
        cur_dir = heading_to_dir(state.map_heading) if state.valid else None
        if not self.map.in_bounds(cell[0], cell[1]):
            return PathResult(ok=False, reason="Robot outside map")

        targets = list(remaining_targets)
        if not targets:
            return PathResult(ok=False, reason="No targets remaining")
        goal = targets[-1][1]
        checkpoints = [c for _, c in targets[:-1]]
        result = plan_mission(
            self.map,
            start=cell,
            goal=goal,
            checkpoints=checkpoints,
            allow_unknown=self.config.allow_unknown_cells,
            turn_penalty=self.config.turn_penalty,
            start_dir=cur_dir,
            optimize_order=False,
            speed_mps=self.config.base_speed_mps,
        )
        if result.ok:
            self.replan_count += 1
            self.path_result = result
            self.executed_index = 0
        return result

    # ====================================================================
    # mission control
    # ====================================================================
    def arm(self):
        """Explicit operator action required before any physical movement."""
        if self.robot is None or not self.robot.is_connected():
            self.log("Cannot arm: robot not connected", "warn")
            return False
        self.armed = True
        self.log("ARMED - robot may now be commanded to move", "warn")
        return True

    def disarm(self):
        self.armed = False
        self.log("Disarmed")

    def _preflight(self, need_path=True):
        if self.robot is None or not self.robot.is_connected():
            self.log("Robot not connected", "warn")
            return False
        if self.robot.emergency_stopped():
            self.log("Emergency stop is engaged - clear it first", "warn")
            return False
        if self.mission_active():
            self.log("A mission is already running", "warn")
            return False
        if self.robot.is_physical and not self.armed:
            self.log("Real robot is not ARMED - press ARM first", "warn")
            return False
        if not self.tracker.get().valid:
            self.log("No robot pose yet - waiting for sensor data", "warn")
            return False
        # A previous STOP latches on the interface; starting a new run is the
        # explicit operator action that lifts it.
        self.robot.clear_stop()
        if need_path and not self.path_result.ok:
            self.log("No valid path - generate a path first", "warn")
            return False
        return True

    def start_navigation(self):
        if not self._preflight(need_path=True):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_NAVIGATE
        self.deviation_warning = False
        self.replan_count = 0
        self._worker = threading.Thread(target=self._run_navigation, name="MissionNavigate")
        self._worker.daemon = True
        self._worker.start()
        return True

    def start_return_to_start(self):
        """Drives back to the Start cell from wherever the robot is."""
        if self.map.start is None:
            self.log("No Start cell set", "warn")
            return False
        if not self._preflight(need_path=False):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_RETURN
        self.replan_count = 0
        self._worker = threading.Thread(target=self._run_return, name="MissionReturn")
        self._worker.daemon = True
        self._worker.start()
        return True

    def _run_return(self):
        target = self.map.start
        try:
            self.navigation_status = "RETURNING"
            self.tracker.set_status(RobotStatus.NAVIGATING)
            self.log("Returning to Start at {}".format(target))
            if self._drive_to(target, "Start"):
                self.navigation_status = "AT START"
                self.tracker.set_status(RobotStatus.READY)
                self.log("Back at Start {}".format(target))
        except Exception as exc:  # pragma: no cover - defensive
            self.navigation_status = "ERROR"
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Return-to-start error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            self.mission_kind = MISSION_IDLE
            self.tracker.set_target(None)

    def detect_object(self):
        """Looks for a graspable object ahead using the front ToF."""
        reading = self.latest_reading()
        carrying = bool(self.robot and self.robot.carrying)
        detection = self.detector.detect(reading, carrying=carrying)
        self.last_detection = detection
        return detection

    def object_square(self):
        """Square the object is expected on.

        The Object tool is the explicit way to say where it is; maps drawn
        before that tool existed used the Goal marker, so fall back to it
        rather than refusing to run on an older map.
        """
        grid = self.map
        if grid.object_cell is not None:
            return grid.object_cell
        return grid.goal

    def start_pickup_mission(self):
        """OBJECT PLACE: go to the object's square, find it, grab it.

        The robot pulls up beside the square (an object occupies its cell, so
        it cannot be driven onto), then *rotates in place* sweeping the front
        ToF until the object is the thing in front of it, and closes the
        gripper.
        """
        grid = self.map
        if self.robot is not None and not self.robot.has_gripper():
            self.log("This robot backend has no gripper", "warn")
            return False
        if self.robot is not None and self.robot.carrying:
            self.log("Already holding something - use DELIVERY", "warn")
            return False
        if self.object_square() is None:
            self.log("Mark the object square with the Object tool first", "warn")
            return False
        if grid.object_cell is None:
            self.log("No Object marker set - using the Goal square at {}".format(
                grid.goal), "warn")
        if not self._preflight(need_path=False):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_PICKUP
        self.deviation_warning = False
        self.replan_count = 0
        self._worker = threading.Thread(target=self._run_pickup, name="MissionPickup")
        self._worker.daemon = True
        self._worker.start()
        return True

    def start_delivery_mission(self):
        """DELIVERY: carry what is held to the aimed sub-position and release it."""
        grid = self.map
        if self.robot is not None and not self.robot.has_gripper():
            self.log("This robot backend has no gripper", "warn")
            return False
        if self.robot is not None and not self.robot.carrying:
            self.log("Nothing in the gripper - run OBJECT PLACE first", "warn")
            return False
        if grid.delivery_cell is None:
            self.log("Set a delivery point with the Place tool first", "warn")
            return False
        if not self._preflight(need_path=False):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_DELIVERY
        self.deviation_warning = False
        self.replan_count = 0
        self._worker = threading.Thread(target=self._run_delivery, name="MissionDelivery")
        self._worker.daemon = True
        self._worker.start()
        return True

    def start_carry_mission(self):
        """Whichever of the two makes sense for what the gripper holds."""
        if self.robot is not None and self.robot.carrying:
            return self.start_delivery_mission()
        return self.start_pickup_mission()

    def _run_pickup(self):
        """Drive to the object's square, sweep for it, grab it."""
        grid = self.map
        try:
            self.navigation_status = "TO OBJECT"
            self.tracker.set_status(RobotStatus.NAVIGATING)
            target = self.object_square()
            self.log("OBJECT PLACE: heading to the square at {}".format(target))
            if self._approach(target, "object") is None:
                return

            self.navigation_status = "SCANNING"
            detection = self._scan_for_object()
            if detection is None or not detection.present:
                self.navigation_status = "NOTHING FOUND"
                self.log("Swept a full circle and found nothing to pick up", "warn")
                return

            self.navigation_status = "PICKING"
            self.log("Object {:.2f} m ahead - closing the gripper".format(
                detection.distance_m))
            result = self.robot.pick()
            if not result.ok:
                self.navigation_status = "PICK FAILED"
                self.log("Pick failed: {}".format(result.reason), "error")
                return

            self.sync_gripper_state()
            self.navigation_status = "HOLDING"
            self.tracker.set_status(RobotStatus.READY)
            self.log("Object picked up. Press DELIVERY to take it to the "
                     "delivery point")
        except Exception as exc:  # pragma: no cover - defensive
            self.navigation_status = "ERROR"
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Pickup error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            self.mission_kind = MISSION_IDLE
            self.tracker.set_target(None)

    def _scan_for_object(self):
        """Rotates in place, sweeping the front ToF, until the object is ahead.

        The ToF is bolted to the chassis, so "aim the sensor" means "turn the
        robot".  Each quarter turn puts the beam down a different axis, and the
        detector can only be trusted on an axis because that is where the map
        knows what the beam ought to hit.
        """
        for attempt in range(max(1, self.config.object_scan_turns)):
            if self._should_abort() or not self._wait_if_paused():
                return None
            detection = self._look_for_object()
            if detection.present:
                if attempt:
                    self.log("Found it after {} quarter turn(s)".format(attempt))
                return detection
            if attempt + 1 >= self.config.object_scan_turns:
                break
            self.log("Nothing ahead ({}), turning to look".format(detection.reason))
            _, cur_dir = self._robot_cell_dir(fresh=True)
            result = self.robot.turn(90.0)
            if not result.ok:
                self.log("Scan turn failed: {}".format(result.reason), "warn")
                return None
            self._settle_heading((cur_dir + 1) % 4)
        return None

    def _run_delivery(self):
        """Carry what is held to the delivery point and release it on the aim spot."""
        grid = self.map
        try:
            self.navigation_status = "TO DELIVERY"
            self.tracker.set_status(RobotStatus.NAVIGATING)
            self.log("DELIVERY: carrying to {}".format(grid.delivery_cell))
            if not self._drive_to(grid.delivery_cell, "delivery point"):
                return

            self.navigation_status = "PLACING"
            if not self._face_direction(grid.delivery_dir):
                return
            result = self.robot.place(offset_xy=self._place_offset_robot_frame())
            if not result.ok:
                self.navigation_status = "PLACE FAILED"
                self.log("Place failed: {}".format(result.reason), "error")
                return

            self.sync_gripper_state()
            self.navigation_status = "COMPLETE"
            self.tracker.set_status(RobotStatus.READY)
            off_x, off_y = getattr(grid, "delivery_offset", (0.0, 0.0))
            cell_m = grid.cell_size_m or self.config.cell_size_m
            where = "at {}".format(grid.delivery_cell)
            if off_x or off_y:
                where += " aimed {:+.0f} cm E {:+.0f} cm S in the square".format(
                    off_x * cell_m * 100, off_y * cell_m * 100)
            backoff = self.config.place_backoff_cm if self.robot.is_physical else 0.0
            if backoff:
                where += (" (released ~{:.0f} cm behind it - the drop"
                          " sequence reverses first)".format(backoff))
            self.log("Delivery complete: object placed {} facing {}".format(
                where, DIR_LONG[grid.delivery_dir % 4]))
        except Exception as exc:  # pragma: no cover - defensive
            self.navigation_status = "ERROR"
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Delivery error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            self.mission_kind = MISSION_IDLE
            self.tracker.set_target(None)

    def _place_offset_robot_frame(self):
        """Aim point for the drop, as (forward, right) metres for the robot.

        The map stores it in screen axes so it rotates with the map; the robot
        needs it relative to the way it is facing when it lets go.
        """
        grid = self.map
        off_x, off_y = getattr(grid, "delivery_offset", (0.0, 0.0))
        if not off_x and not off_y:
            return None
        cell_m = grid.cell_size_m or self.config.cell_size_m
        forward = DIR_VECTORS[grid.delivery_dir % 4]
        right = DIR_VECTORS[(grid.delivery_dir + 1) % 4]
        return (
            (off_x * forward[0] + off_y * forward[1]) * cell_m,
            (off_x * right[0] + off_y * right[1]) * cell_m,
        )

    def _approach(self, target, label):
        """Stops in a cell *next to* `target` and turns to face it.

        An object standing on a square physically occupies it, so the robot
        cannot drive onto the square - it has to pull up alongside and look at
        it, which is also the only way the front ToF can see it and the arm can
        reach it.
        """
        state = self.tracker.get()
        here = state.cell if state.valid else self.map.start
        options = []
        for direction in range(4):
            d_col, d_row = DIR_VECTORS[direction]
            stand = (target[0] - d_col, target[1] - d_row)
            if not self.map.in_bounds(stand[0], stand[1]):
                continue
            if self.map.is_blocked(stand[0], stand[1]):
                continue
            # The robot must be able to see across the shared edge.
            if self.map.has_wall(stand[0], stand[1], direction):
                continue
            route = astar(self.map, here, stand,
                          allow_unknown=self.config.allow_unknown_cells)
            if route is None:
                continue
            options.append((len(route), stand, direction))

        if not options:
            self.log("No way to pull up beside the {} at {}".format(label, target),
                     "error")
            return None
        options.sort(key=lambda item: item[0])
        _, stand, facing = options[0]

        if stand != here and not self._drive_to(stand, label):
            return None
        if not self._face_direction(facing):
            return None
        return facing

    def _look_for_object(self):
        """Watches the front ToF for a few frames before committing to a grab.

        The detector needs several agreeing frames, so give it that many rather
        than judging on one sample.
        """
        detection = self.detect_object()
        deadline = time.time() + 2.0
        while (not detection.present and time.time() < deadline
               and not self._should_abort()):
            self.tracker.wait_for_update(timeout=self.config.pose_wait_s)
            detection = self.detect_object()
        return detection

    def _drive_to(self, destination, label):
        """Replan-and-walk to one cell.  False when it could not get there."""
        guard = 0
        while not self._should_abort():
            if not self._wait_if_paused():
                return False
            guard += 1
            if guard > self.config.automap_max_steps:
                self.log("Gave up driving to the {}".format(label), "error")
                return False

            cell, _ = self._robot_cell_dir(fresh=True)
            if cell == destination:
                return True

            result = self._replan_from_robot([(label, destination)])
            if not result.ok:
                self.navigation_status = "NO VALID PATH"
                self.log("NO VALID PATH to the {}: {}".format(label, result.reason), "error")
                return False

            for i in range(1, len(result.cells)):
                if self._should_abort() or not self._wait_if_paused():
                    return False
                step = self._step_to(result.cells[i], expected_from=result.cells[i - 1])
                if not step.ok:
                    here, _ = self._robot_cell_dir()
                    if "Blocked" in step.reason or "obstacle" in step.reason.lower():
                        self._handle_blocked(here, result.cells[i])
                    else:
                        self.log("Step failed: {}".format(step.reason), "warn")
                    break
                self.executed_index = i
        return False

    def _face_direction(self, target_dir):
        """Turns in place until the robot faces the given map direction."""
        _, cur_dir = self._robot_cell_dir(fresh=True)
        quarter = (int(target_dir) - cur_dir) % 4
        if not quarter:
            return True
        degrees = {1: 90.0, 2: 180.0, 3: -90.0}[quarter]
        self.tracker.set_status(RobotStatus.MOVING)
        result = self.robot.turn(degrees)
        if not result.ok:
            self.log("Could not face {}: {}".format(
                DIR_LONG[int(target_dir) % 4], result.reason), "error")
            return False
        settled = self._settle_heading(int(target_dir))
        if not settled.ok:
            self.log(settled.reason, "warn")
        return True

    def start_automap(self):
        if not self._preflight(need_path=False):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_AUTOMAP
        self.explorer.reset()
        self.visited_cells = set()
        self.mapping_status = "ACTIVE"
        if self.sim_engine is not None:
            self.sim_engine.mapping_enabled = True
        self._worker = threading.Thread(target=self._run_automap, name="MissionAutoMap")
        self._worker.daemon = True
        self._worker.start()
        return True

    def begin_auto_mapping(self):
        """Blank discovery map, hidden ground truth, then frontier exploration.

        In simulation the backend is rebuilt so the simulated sensors read the
        ground truth while the mapping code only ever sees the discovery map.
        """
        if self.mission_active():
            self.log("A mission is already running", "warn")
            return False
        if self.mode == MODE_SIM:
            self.disconnect()
            self.prepare_auto_mapping()
            ok, reason = self.connect()
            if not ok:
                return False
        else:
            if self.robot is None or not self.robot.is_connected():
                self.log("Connect the robot before auto-mapping", "warn")
                return False
            self.prepare_auto_mapping()
            cell = self.map.robot_cell or (0, 0)
            self.transform.rebase(cell, self.map.robot_dir * 90.0)
            self.robot.zero_odometry(cell, self.map.robot_dir)
            self.tracker.clear_trail()
            if not self.wait_for_pose(2.0):
                self.log("No pose from robot - cannot start auto-mapping", "error")
                return False
        return self.start_automap()

    def jog_turn(self, degrees):
        """Turns the *actual* robot in place (positive = clockwise/right).

        Goes through the same preflight as a mission, so on real hardware it
        still needs CONNECT + ARM and respects the emergency stop.
        """
        if not self._preflight(need_path=False):
            return False
        self._stop_flag.clear()
        self._pause_flag.set()
        self.mission_kind = MISSION_JOG
        self._worker = threading.Thread(target=self._run_jog, args=(float(degrees),),
                                        name="MissionJog")
        self._worker.daemon = True
        self._worker.start()
        return True

    def _run_jog(self, degrees):
        self.tracker.set_status(RobotStatus.MOVING)
        self.log("Manual turn {:+.0f} deg".format(degrees))
        try:
            result = self.robot.turn(degrees)
            if not result.ok:
                self.log("Turn failed: {}".format(result.reason), "warn")
        except Exception as exc:  # pragma: no cover - defensive
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Turn error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            self.mission_kind = MISSION_IDLE
            if self.tracker.get_status() == RobotStatus.MOVING:
                self.tracker.set_status(RobotStatus.READY)

    def pause_mission(self):
        self._pause_flag.clear()
        if self.robot:
            self.robot.pause()
        self.tracker.set_status(RobotStatus.PAUSED)
        self.log("Mission paused")

    def resume_mission(self):
        if self.robot and self.robot.emergency_stopped():
            self.log("Cannot resume: emergency stop engaged", "warn")
            return False
        self._pause_flag.set()
        if self.robot:
            self.robot.clear_stop()
            self.robot.resume()
        self.tracker.set_status(
            RobotStatus.MAPPING if self.mission_kind == MISSION_AUTOMAP else RobotStatus.NAVIGATING
        )
        self.log("Mission resumed")
        return True

    def stop_mission(self, reason="operator stop"):
        self._stop_flag.set()
        self._pause_flag.set()
        if self.robot:
            self.robot.stop()
        worker = self._worker
        if worker is not None and worker.is_alive() and worker is not threading.current_thread():
            worker.join(timeout=3.0)
        self._worker = None
        if self.mission_kind != MISSION_IDLE:
            self.log("Mission stopped ({})".format(reason))
        self.mission_kind = MISSION_IDLE
        self.navigation_status = "STOPPED"
        if self.mapping_status == "ACTIVE":
            self.mapping_status = "STOPPED"
        if self.sim_engine is not None:
            self.sim_engine.mapping_enabled = False
        if self.robot and self.robot.is_connected() and not self.robot.emergency_stopped():
            self.tracker.set_status(RobotStatus.STOPPED)

    def restart(self):
        """Stops, puts the robot back on its start cell and clears the run history.

        On real hardware this only re-zeroes odometry - it does not drive the
        robot back, which the operator has to do themselves.
        """
        self.stop_mission(reason="restart")
        cell = self.map.robot_cell or self.map.start or (0, 0)
        direction = self.map.robot_dir
        self.transform.rebase(cell, direction * 90.0)
        if self.robot is not None and self.robot.is_connected():
            self.robot.zero_odometry(cell, direction)
        # Drop the cached pose as well: it was measured in the old frame, and
        # showing it against the new one would put the marker in the wrong cell
        # until the next sensor frame lands.
        self.tracker.reset()
        self.executed_index = 0
        self.deviation_warning = False
        self.replan_count = 0
        for wp in self.mission_waypoints:
            wp[2] = "pending"
        if self.mission_waypoints:
            self.mission_waypoints[0][2] = "done"
        self.navigation_status = "IDLE"
        if self.robot is not None and self.robot.is_physical:
            self.log("Restarted: odometry re-zeroed at {} (robot was not moved)".format(cell), "warn")
        else:
            self.log("Restarted at {} facing {}".format(cell, DIR_LONG[direction % 4]))

    def set_robot_speed(self, metres_per_second):
        """Changes the cruise speed, live, without reconnecting."""
        speed = max(0.05, float(metres_per_second))
        self.config.base_speed_mps = speed
        if self.robot is not None and hasattr(self.robot, "set_speed"):
            self.robot.set_speed(speed)
        if self.sim_robot is not None:
            self.sim_robot.base_speed = speed
        self.log("Robot speed set to {:.2f} m/s".format(speed))
        return True

    def set_origin_here(self):
        """Declares that the robot is physically on the placed start cell now,
        facing the placed heading, and re-zeroes odometry to match.

        The frame is captured when you press CONNECT.  A robot that is picked up
        and set down afterwards - the normal way of putting it on the start
        square - is still being tracked against that stale origin, so every
        planned turn comes out rotated by the difference.  This re-anchors it.
        """
        if self.mission_active():
            self.log("Cannot set the origin while a mission is running", "warn")
            return False
        if self.robot is None or not self.robot.is_connected():
            self.log("Connect the robot before setting the origin", "warn")
            return False
        cell = self.map.robot_cell or self.map.start or (0, 0)
        direction = self.map.robot_dir
        self.restart()
        self.log("Origin set: robot is at {} facing {}".format(
            cell, DIR_LONG[direction % 4]), "warn")
        return True

    def emergency_stop(self):
        """Highest priority - latches until explicitly cleared."""
        self._stop_flag.set()
        self._pause_flag.set()
        if self.robot:
            self.robot.emergency_stop()
        self.tracker.set_status(RobotStatus.EMERGENCY_STOP)
        self.navigation_status = "E-STOP"
        self.mapping_status = "STOPPED" if self.mapping_status == "ACTIVE" else self.mapping_status
        self.armed = False
        self.log("*** EMERGENCY STOP ***", "error")

    def clear_emergency_stop(self):
        if self.robot:
            self.robot.clear_emergency_stop()
        self._stop_flag.clear()
        self.clear_error()
        if self.robot and self.robot.is_connected():
            self.tracker.set_status(RobotStatus.READY)
        self.log("Emergency stop cleared (robot still disarmed)", "warn")

    def safe_stop(self, reason):
        """Stops motion without latching, used by the safety supervisor."""
        self._stop_flag.set()
        if self.robot:
            self.robot.stop()
        self.log("Safe stop: {}".format(reason), "warn")

    # ====================================================================
    # supervision - called from the UI loop every frame
    # ====================================================================
    def sync_gripper_state(self):
        """Keeps the front-sensor mask in step with what the gripper holds.

        Holding something puts it right in the ToF beam, so while carrying the
        front sensor is ignored for mapping, obstacle checks and detection -
        'ignore what is in front, mind what is in hand'.
        """
        carrying = bool(self.robot and self.robot.carrying)
        if self.mapper.ignore_front != carrying:
            self.mapper.ignore_front = carrying
            self.log("Front ToF {} (gripper {})".format(
                "ignored - object in hand" if carrying else "back in use",
                "loaded" if carrying else "empty"))
        return carrying

    def tick(self):
        if self.robot is None or not self.robot.is_connected():
            return
        self.sync_gripper_state()
        if self.tracker.check_timeout():
            self.tracker.set_status(RobotStatus.TRACKING_LOST)
            state = self.tracker.get()
            self.log(
                "ROBOT TRACKING LOST - last pose X={:.2f} m Y={:.2f} m".format(state.x, state.y),
                "error",
            )
            if self.mission_active():
                self.safe_stop("tracking lost")

    def map_dirty(self):
        dirty = self._map_dirty
        self._map_dirty = False
        return dirty

    # ====================================================================
    # mission workers
    # ====================================================================
    def _should_abort(self):
        if self._stop_flag.is_set():
            return True
        if self.robot is None or self.robot.emergency_stopped():
            return True
        if not self.tracker.tracking_ok():
            return True
        return False

    def _wait_if_paused(self):
        while not self._pause_flag.is_set():
            if self._stop_flag.is_set():
                return False
            time.sleep(0.05)
        return True

    def _robot_cell_dir(self, fresh=False):
        """Current cell and heading.  ``fresh=True`` waits for a new pose frame
        first, so a decision is never taken on a pose from before the last move."""
        if fresh:
            self.tracker.wait_for_update(timeout=self.config.pose_wait_s)
        state = self.tracker.get()
        return state.cell, heading_to_dir(state.map_heading)

    def _settle_heading(self, target_dir):
        """Confirms the robot really reached the intended heading after a turn.

        A commanded turn is not a completed turn: wheels slip, the chassis
        overshoots, and the SDK reports the action done before the yaw settles.
        Driving on an unverified heading is what sends the robot off at a right
        angle to the plan, so measure it and correct before moving.
        """
        tolerance = self.config.heading_tolerance_deg
        error = 0.0
        for attempt in range(self.config.heading_retries + 1):
            state = self.tracker.wait_for_update(timeout=self.config.pose_wait_s)
            if state is None:
                state = self.tracker.get()
            error = wrap180(target_dir * 90.0 - state.map_heading)
            if abs(error) <= tolerance:
                return RobotCommandResult(True, "Heading OK")
            if attempt >= self.config.heading_retries:
                break
            self.log("Heading off by {:+.1f} deg after turn - correcting".format(error), "warn")
            result = self.robot.turn(error)
            if not result.ok:
                return result
            if not self._wait_if_paused():
                return RobotCommandResult(False, "Stopped")
        return RobotCommandResult(
            False, "Heading not reached (off by {:+.1f} deg)".format(error))

    def _front_clearance_mm(self):
        """Live front ToF distance, or ``None`` when there is no trustworthy reading.

        Returns ``None`` while the gripper is loaded: the carried object sits a
        few centimetres in front of the sensor, so every reading would look like
        an obstacle and the robot would refuse to move in any direction.  Mind
        what is in the hand, not what the beam hits.
        """
        if self.robot is not None and self.robot.carrying:
            return None
        reading = self.latest_reading()
        if reading is None or reading.front_mm is None or not reading.front_valid:
            return None
        return reading.front_mm

    def _step_to(self, next_cell, expected_from=None):
        """Turns towards and drives into one adjacent cell.  Returns a result object."""
        cell, cur_dir = self._robot_cell_dir(fresh=True)
        if expected_from is not None and cell != expected_from:
            # The pose moved between planning and stepping; replan rather than
            # trying to drive a leg that no longer starts where we are.
            return RobotCommandResult(
                False, "Off path (at {}, planned from {})".format(cell, expected_from))

        d_col = next_cell[0] - cell[0]
        d_row = next_cell[1] - cell[1]
        try:
            target_dir = dir_from_delta(d_col, d_row)
        except ValueError:
            return RobotCommandResult(
                False, "Off path ({} is not adjacent to {})".format(next_cell, cell))

        quarter = (target_dir - cur_dir) % 4
        if quarter:
            degrees = {1: 90.0, 2: 180.0, 3: -90.0}[quarter]
            self.tracker.set_status(RobotStatus.MOVING)
            result = self.robot.turn(degrees)
            if not result.ok:
                return result
            if not self._wait_if_paused():
                return RobotCommandResult(False, "Stopped")

        # Square up before *every* move, not only after a turn.  Heading drifts
        # between cells too, and driving a cell while a few degrees crooked is
        # what walks the robot sideways into a wall over a few steps.
        settled = self._settle_heading(target_dir)
        if not settled.ok:
            return settled

        # Do not drive into something the ToF can already see.
        clearance = self._front_clearance_mm()
        if clearance is not None and clearance <= self.config.obstacle_stop_mm:
            return RobotCommandResult(
                False, "Blocked: obstacle {:.0f} mm ahead".format(clearance))

        self.tracker.set_target(next_cell)
        self.tracker.set_status(RobotStatus.MOVING)
        result = self.robot.move_cells(1)

        # Let the chassis and the odometry settle before the next decision.
        if self.config.step_settle_s > 0:
            time.sleep(self.config.step_settle_s)

        if not result.ok:
            # The robot's actual pose is the source of truth: if it did arrive,
            # an early drive-stop (wall reached, PID cut-off) is not a failure.
            actual, _ = self._robot_cell_dir(fresh=True)
            if actual == next_cell:
                return RobotCommandResult(True, "Arrived ({})".format(result.reason))
        return result

    def _handle_blocked(self, cell, direction_cell):
        """Records a bump.  The blocked *edge* is always a wall; whether the
        *cell* beyond it is occupied is a separate, much weaker inference.

        While auto-mapping, cells are marked FREE speculatively from long ToF
        sweeps that see past a doorway but not the partition beside it.  Calling
        such a cell an OBSTACLE on the first bump permanently blocks a square
        that is merely behind a wall, which corrupts the map and produces
        spurious "NO VALID PATH". During exploration, record the wall only.
        """
        try:
            d = dir_from_delta(direction_cell[0] - cell[0], direction_cell[1] - cell[1])
            self.map.set_wall(cell[0], cell[1], d, True, known=True)
        except ValueError:
            pass

        exploring = self.mapping_status == "ACTIVE"
        if not exploring and self.map.get(direction_cell[0], direction_cell[1]) == FREE:
            # Navigating an already-known map: a cell we had confirmed free is
            # now blocked, so something really has appeared in it.
            self.mapper.mark_obstacle(direction_cell)
            self.log("Obstacle detected at {} - map updated, replanning".format(
                direction_cell), "warn")
        else:
            self.log("Wall found at {} while exploring - map updated".format(
                direction_cell), "warn")
        self._map_dirty = True
        self._map_changed_since_step = True

    # ------------------------------------------------------------ navigation
    def _run_navigation(self):
        self.navigation_status = "ACTIVE"
        self.tracker.set_status(RobotStatus.NAVIGATING)
        self.log("Mission started: {} waypoints".format(len(self.mission_waypoints)))

        remaining = [(label, cell) for label, cell, _ in self.mission_waypoints[1:]]
        guard = 0
        try:
            while remaining and not self._should_abort():
                if not self._wait_if_paused():
                    break
                guard += 1
                if guard > self.config.automap_max_steps:
                    self.log("Navigation aborted: step limit reached", "error")
                    break

                result = self._replan_from_robot(remaining)
                if not result.ok:
                    self.navigation_status = "NO VALID PATH"
                    self.log("NO VALID PATH: {}".format(result.reason), "error")
                    break

                path = result.cells
                if len(path) < 2:
                    remaining.pop(0)
                    self._mark_waypoint_done(0)
                    continue

                self._map_changed_since_step = False
                progressed = False
                for i in range(1, len(path)):
                    if self._should_abort() or not self._wait_if_paused():
                        break
                    next_cell = path[i]
                    step = self._step_to(next_cell, expected_from=path[i - 1])
                    if not step.ok:
                        cell, _ = self._robot_cell_dir()
                        if "Blocked" in step.reason or "obstacle" in step.reason.lower():
                            self._handle_blocked(cell, next_cell)
                            break
                        if step.reason.startswith("Off path"):
                            self.log("{} - replanning".format(step.reason), "warn")
                            break
                        self.log("Step failed: {}".format(step.reason), "warn")
                        break

                    self.executed_index = i
                    progressed = True
                    actual, _ = self._robot_cell_dir()
                    state = self.tracker.get()
                    drift = deviation_from_path(path, state.map_col, state.map_row)
                    if actual != next_cell or drift > self.config.deviation_cells:
                        self.deviation_warning = True
                        self.log(
                            "Robot deviated from planned path (at {}, expected {}, "
                            "off by {:.2f} cells) - replanning".format(actual, next_cell, drift),
                            "warn",
                        )
                        break
                    self.deviation_warning = False

                    # Mark waypoints reached along the way.
                    self._check_waypoint_reached(actual, remaining)
                    if remaining and actual == remaining[0][1]:
                        remaining.pop(0)
                        break

                    if self._map_changed_since_step and not path_is_valid(
                        self.map, path[i:], allow_unknown=self.config.allow_unknown_cells
                    ):
                        self.log("Map changed - planned path invalidated, replanning", "warn")
                        break

                if not progressed and not self._should_abort():
                    # Could not move at all this round; avoid a hot loop.
                    time.sleep(0.2)

            if not remaining and not self._should_abort():
                self.navigation_status = "COMPLETE"
                self.tracker.set_status(RobotStatus.READY)
                for wp in self.mission_waypoints:
                    wp[2] = "done"
                self.log("Mission complete - goal reached")
            elif self._should_abort():
                self.navigation_status = "ABORTED"
        except Exception as exc:  # pragma: no cover - defensive
            self.navigation_status = "ERROR"
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Navigation error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            self.mission_kind = MISSION_IDLE
            self.tracker.set_target(None)

    def _mark_waypoint_done(self, index_in_remaining):
        done = 0
        for wp in self.mission_waypoints:
            if wp[2] == "done":
                done += 1
        idx = done + index_in_remaining
        if 0 <= idx < len(self.mission_waypoints):
            self.mission_waypoints[idx][2] = "done"

    def _check_waypoint_reached(self, cell, remaining):
        for wp in self.mission_waypoints:
            if wp[1] == cell and wp[2] != "done":
                wp[2] = "done"
                self.tracker.set_target(cell, wp[0])
                self.log("Reached {} at {}".format(wp[0], cell))
        for wp in self.mission_waypoints:
            if wp[2] == "pending":
                wp[2] = "active"
                break

    # ------------------------------------------------------------- auto-map
    def _run_automap(self):
        self.tracker.set_status(RobotStatus.MAPPING)
        self.mapping_status = "ACTIVE"
        self.log("Auto-mapping started (frontier exploration)")
        steps = 0
        try:
            # Seed the map with a scan from where we stand.
            self._scan_once()
            start_cell, _ = self._robot_cell_dir()
            self.visited_cells.add(start_cell)
            while not self._should_abort():
                if not self._wait_if_paused():
                    break
                steps += 1
                if steps > self.config.automap_max_steps:
                    self.log("Auto-mapping stopped: step limit", "warn")
                    break

                cell, cur_dir = self._robot_cell_dir()
                if not self.map.in_bounds(cell[0], cell[1]):
                    self.log("Robot outside map bounds - stopping", "error")
                    break

                visited = self.visited_cells if self.config.full_coverage else None
                target = self.explorer.select(cell, cur_dir, visited=visited)
                if target is None:
                    self.mapping_status = "COMPLETE"
                    self.log("Auto-mapping complete: {} cells mapped, {} driven".format(
                        self.map.stats()["known"], len(self.visited_cells)))
                    break

                label = "Frontier" if target.gain else "Coverage"
                self.tracker.set_target(target.cell, label)
                if len(target.path) < 2:
                    # Already standing on the frontier; rotate to look around.
                    self._sweep_scan()
                    if self.map.information_gain(cell[0], cell[1]) > 0:
                        self.explorer.blacklist_cell(target.cell)
                    continue

                for i in range(1, len(target.path)):
                    if self._should_abort() or not self._wait_if_paused():
                        break
                    next_cell = target.path[i]
                    step = self._step_to(next_cell, expected_from=target.path[i - 1])
                    if not step.ok:
                        if step.reason.startswith("Off path"):
                            self.log("{} - reselecting frontier".format(step.reason), "warn")
                            break
                        here, _ = self._robot_cell_dir()
                        self._handle_blocked(here, next_cell)
                        self.explorer.blacklist_cell(target.cell)
                        break
                    self._scan_once()
                    actual, _ = self._robot_cell_dir()
                    self.visited_cells.add(actual)
                    if actual != next_cell:
                        self.log("Deviation during exploration - reselecting frontier", "warn")
                        break
                    # Only an information frontier can be "already resolved".
                    # A coverage target must actually be driven to.
                    if (target.gain
                            and self.map.get(target.cell[0], target.cell[1]) == FREE
                            and self.map.information_gain(target.cell[0], target.cell[1]) == 0):
                        break

            if self.mapping_status == "ACTIVE":
                self.mapping_status = "STOPPED"
            if self.mapping_status == "COMPLETE" and self.config.plan_after_mapping:
                self._plan_after_mapping()
        except Exception as exc:  # pragma: no cover - defensive
            self.mapping_status = "ERROR"
            self.tracker.set_status(RobotStatus.ERROR)
            self.log("Auto-mapping error: {}".format(exc), "error")
        finally:
            if self.robot:
                self.robot.stop()
            if self.sim_engine is not None:
                self.sim_engine.mapping_enabled = False
            self.mission_kind = MISSION_IDLE
            self.tracker.set_target(None)
            if self.tracker.get_status() == RobotStatus.MAPPING:
                self.tracker.set_status(RobotStatus.READY)

    def _plan_after_mapping(self):
        """Solves Start -> checkpoints -> Goal on the freshly discovered map."""
        if self.map.start is None or self.map.goal is None:
            self.log("Map complete. Set Start and Goal, then press A* PATH")
            return
        result = self.plan()
        if result.ok:
            self.log("Route ready on the discovered map: {} steps, {:.2f} m".format(
                result.steps, result.distance_m))
        else:
            self.log("Map complete but no route: {}".format(result.reason), "warn")

    def coverage_summary(self):
        """(cells driven through, cells reachable) on the working map."""
        reachable = set(self.visited_cells)
        state = self.tracker.get()
        if state.valid:
            for cell in self.explorer.reachable_distances(state.cell):
                if self.map.get(cell[0], cell[1]) == FREE:
                    reachable.add(cell)
        return len(self.visited_cells), len(reachable)

    def _scan_once(self):
        """Folds the freshest sensor frame into the map (both modes)."""
        reading = None
        if self.sensor_source is not None:
            reading = self.sensor_source.read()
        if reading is None:
            reading = self._latest_reading
        if reading is None:
            return None
        update = self.mapper.integrate(reading)
        if update.changed:
            self._on_map_update(update)
        return reading

    def _sweep_scan(self):
        """Turns in place to look at all four sides of the current cell."""
        for _ in range(4):
            if self._should_abort():
                return
            self._scan_once()
            result = self.robot.turn(90.0)
            if not result.ok:
                return
            time.sleep(0.05)
        self._scan_once()

    # ====================================================================
    # status snapshots for the UI
    # ====================================================================
    def mapping_progress(self):
        stats = self.map.stats()
        return stats

    def mission_summary(self):
        """Everything the mission bar needs, computed once per frame."""
        state = self.tracker.get()
        path = self.path_result.cells if self.path_result.ok else []
        remaining_steps = 0
        if path:
            idx = min(self.executed_index, len(path) - 1)
            remaining_steps = max(0, len(path) - 1 - idx)
        cell_size = self.map.cell_size_m or self.config.cell_size_m
        return {
            "waypoints": self.mission_waypoints,
            "steps_total": self.path_result.steps if self.path_result.ok else 0,
            "steps_remaining": remaining_steps,
            "distance_remaining_m": remaining_steps * cell_size,
            "navigation": self.navigation_status,
            "mapping": self.mapping_status,
            "speed": state.velocity,
            "replans": self.replan_count,
            "deviation": self.deviation_warning,
        }

    def latest_reading(self):
        if self.sensor_source is not None:
            reading = self.sensor_source.read()
            if reading is not None:
                self._latest_reading = reading
        return self._latest_reading

    def shutdown(self):
        self.stop_mission(reason="shutdown")
        self.disconnect()
