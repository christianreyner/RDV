# config.py

# ============================================================
# Connection
# ============================================================

SITL_CONNECTION = "udp:127.0.0.1:14551"

TRACKING_FPS = 30.0
DT = 1.0 / TRACKING_FPS

# ============================================================
# Logging & Visualization
# ============================================================

#Logging
CSV_LOG_PATH = "flight_log.csv"

# Visualization
ENABLE_VISUALIZATION = False
VISUALIZATION_EVERY_N_FRAMES = 10

# ============================================================
# Target
# ============================================================

TARGET_INITIAL_NORTH_M = 50.0
TARGET_INITIAL_EAST_M = 50.0
TARGET_ALT_ABOVE_HOME_M = 100.0

# Heading convention:
# 0 deg = North, 90 deg = East, 180 deg = South, 270 deg = West
TARGET_SPEED_MPS = 10.0
TARGET_HEADING_DEG = 225


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

MAX_ACCEL_MSS = 10.0
MAX_TILT_DEG = 30.0

MAX_ROLL_RATE_DEG_S = 90.0
MAX_PITCH_RATE_DEG_S = 90.0


# ============================================================
# Thrust / launch behavior
# ============================================================

# Positive climb thrust during ground launch.
NOMINAL_CLIMB_THRUST = 0.45

# Lower thrust after initial climb.
# This is not true altitude hold; it is only a crude thrust phase change.
POST_TAKEOFF_THRUST = 0.5

POST_TAKEOFF_ALT_M = 5.0

MIN_THRUST = 0.35
MAX_THRUST = 0.98

COMPENSATE_TILT_THRUST = True


# ============================================================
# Startup
# ============================================================

PREARM_ATTITUDE_SECONDS = 1.0
INITIAL_LEVEL_SECONDS_AFTER_ARM = 1.0

LEVEL_WHEN_TARGET_NOT_VISIBLE = True
