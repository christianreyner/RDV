# config.py

# ============================================================
# Connection
# ============================================================

# SITL_CONNECTION = "udp:127.0.0.1:14551"
MAVLINK_CONNECTION = "/dev/serial0"
MAVLINK_BAUD = 115200

TRACKING_FPS = 20.0
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
# Target (WORLD CONVENTION)
# ============================================================

TARGET_INITIAL_NORTH_M = -15.0
TARGET_INITIAL_EAST_M = 0.0
TARGET_ALT_ABOVE_HOME_M = 20.0

# Heading convention:
# 0 deg = North, 90 deg = East, 180 deg = South, 270 deg = West
TARGET_SPEED_MPS = 0.0
TARGET_HEADING_DEG = 225


# ============================================================
# Camera
# ============================================================

CAMERA_WIDTH_PX = 1280
CAMERA_HEIGHT_PX = 720
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
NOMINAL_CLIMB_THRUST = 0.25

# Lower thrust after initial climb.
# This is not true altitude hold; it is only a crude thrust phase change.
POST_TAKEOFF_THRUST = 0.3

POST_TAKEOFF_ALT_M = 5.0

MIN_THRUST = 0.18
MAX_THRUST = 0.98

COMPENSATE_TILT_THRUST = True


# ============================================================
# Startup
# ============================================================

PREARM_ATTITUDE_SECONDS = 1.0
INITIAL_LEVEL_SECONDS_AFTER_ARM = 1.0

LEVEL_WHEN_TARGET_NOT_VISIBLE = True


# Live OV9281 camera. FOV above must be measured for your lens and stream mode.
USE_REAL_CAMERA = True
DRY_RUN = True                 # False enables flight after Enter confirmation.
CAMERA_COMMAND = "rpicam-vid"
CAMERA_INDEX = 0
CAMERA_WINDOW = "RDV - Target selection"
PREVIEW_DIVISOR = 2             # 2 -> 640x360; 4 -> 320x180
PREVIEW_FPS = 10
STATS_INTERVAL = 1.0
SELECT_BOX_WIDTH = 40           # Full-resolution pixels, independent of preview
SELECT_BOX_HEIGHT = 40
LOCK_CONFIRM_FRAMES = 10        # Consecutive accepted updates before Enter works
CAMERA_STALE_SECONDS = 0.75
MAX_TRACK_JUMP_PX = 100.0        # Rejected jumps cause a hold; tune for motion
MAX_BOX_AREA_RATIO = 2.5        # Maximum area change per accepted update
CAMERA_IMAGE_ROTATION_DEG = 0   # Transform normalized camera errors into controller axes
CAMERA_X_SIGN = 1               # Expected: image right = body right
CAMERA_Y_SIGN = 1               # Expected: image down = body forward (upward camera)
TELEMETRY_STALE_SECONDS = 1.0
STARTUP_TIMEOUT_SECONDS = 10.0
LIVE_CSV_LOG_PATH = "live_flight_log.csv"
