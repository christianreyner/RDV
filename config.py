# config.py

# ============================================================
# Connection
# ============================================================

SITL_CONNECTION = "udp:127.0.0.1:14550"

TRACKING_FPS = 30.0
DT = 1.0 / TRACKING_FPS


# ============================================================
# Target
# ============================================================

TARGET_NORTH_M = 20.0
TARGET_EAST_M = 5.0
TARGET_ALT_ABOVE_HOME_M = 30.0


# ============================================================
# Camera
# ============================================================

CAMERA_WIDTH_PX = 640
CAMERA_HEIGHT_PX = 480
CAMERA_HFOV_DEG = 90.0
CAMERA_VFOV_DEG = 70.0


# ============================================================
# Controller gains
# ============================================================

KPX = 3.0
KDX = 1.2
KPY = 3.0
KDY = 1.2

MAX_ACCEL_MSS = 3.0
MAX_TILT_DEG = 12.0

MAX_ROLL_RATE_DEG_S = 45.0
MAX_PITCH_RATE_DEG_S = 45.0


# ============================================================
# Thrust / launch behavior
# ============================================================

# Positive climb thrust during ground launch.
NOMINAL_CLIMB_THRUST = 0.58

# Lower thrust after initial climb.
# This is not true altitude hold; it is only a crude thrust phase change.
POST_TAKEOFF_THRUST = 0.52

POST_TAKEOFF_ALT_M = 8.0

MIN_THRUST = 0.35
MAX_THRUST = 0.75

COMPENSATE_TILT_THRUST = True


# ============================================================
# Startup
# ============================================================

PREARM_ATTITUDE_SECONDS = 1.0
INITIAL_LEVEL_SECONDS_AFTER_ARM = 1.0

LEVEL_WHEN_TARGET_NOT_VISIBLE = True
