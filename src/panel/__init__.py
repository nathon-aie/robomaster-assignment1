"""RoboMaster Mission Control Center panel package.

Modular control-panel stack layered on top of the existing Step 1-3 code:

    ui/            Pygame dashboard (control panel + live map)
    mission.py     Mission controller state machine (navigate / auto-map / safety)
    robot_iface.py Robot interface abstraction (real / mock / simulated)
    sensors.py     Sensor interface abstraction (real SensorHub / simulated raycast)
    mapper.py      Sensor readings -> occupancy grid (shared by real and simulation)
    explorer.py    Frontier-based exploration planner
    occupancy.py   Occupancy grid + edge walls + map storage
    pathfinding.py A* + checkpoint ordering
    robot_state.py Normalised RobotState, trail, tracking timeout
    geometry.py    Robot <-> map coordinate transformation
    simulation.py  Ground-truth simulation engine
"""

__all__ = [
    "geometry",
    "occupancy",
    "pathfinding",
    "robot_state",
    "sensors",
    "mapper",
    "explorer",
    "robot_iface",
    "simulation",
    "mission",
]
