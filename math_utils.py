# math_utils.py

import math
import numpy as np


def clamp(x, lo, hi):
    return max(lo, min(hi, x))


def euler_to_rotation_body_to_ned(roll, pitch, yaw):
    """
    Rotation matrix from body frame to NED frame.

    Body:
      x forward, y right, z down

    NED:
      x north, y east, z down
    """
    cr = math.cos(roll)
    sr = math.sin(roll)
    cp = math.cos(pitch)
    sp = math.sin(pitch)
    cy = math.cos(yaw)
    sy = math.sin(yaw)

    return np.array([
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp,     cp * sr,                  cp * cr],
    ])


def quat_from_euler(roll, pitch, yaw):
    """
    Convert Euler angles to MAVLink quaternion [w, x, y, z].
    """
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return [w, x, y, z]


def slew_limit(value, previous, max_rate, dt):
    max_delta = max_rate * dt
    return clamp(value, previous - max_delta, previous + max_delta)
