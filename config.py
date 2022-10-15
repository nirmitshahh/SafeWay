"""
Configuration constants for SafeWay simulation
"""

# Simulation parameters
DEFAULT_DT = 0.1  # time step in seconds
DEFAULT_FPS = 30

# Vehicle parameters
DEFAULT_MAX_SPEED = 5.0
DEFAULT_PREFERRED_SPEED = 4.0
DEFAULT_ACCELERATION = 2.0
DEFAULT_DECELERATION = 3.0
DEFAULT_TRAJECTORY_HORIZON = 10

# V2V Communication parameters
DEFAULT_BROADCAST_RADIUS = 50.0
DEFAULT_LATENCY = 0.0
DEFAULT_PACKET_DROP_RATE = 0.0

# Conflict resolution parameters
DEFAULT_SAFETY_BUFFER = 2.0
DEFAULT_PREDICTION_HORIZON = 10

# Visualization parameters
DEFAULT_WINDOW_WIDTH = 1200
DEFAULT_WINDOW_HEIGHT = 800
DEFAULT_SCALE = 1.0

# Pathfinding parameters
WAYPOINT_THRESHOLD = 0.5
MAX_TURN_RATE = 0.15  # radians per step

