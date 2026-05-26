# camera.py

import math
from dataclasses import dataclass

import numpy as np


# ============================================================
# Camera mounting transform
# ============================================================
#
# Body:
#   x forward, y right, z down
#
# Camera:
#   x image right, y image down, z optical axis
#
# For upward-looking camera:
#   camera +z -> body -z
#   camera +x -> body +y
#   camera +y -> body +x

R_BODY_CAM = np.array([
    [0.0, 1.0,  0.0],
    [1.0, 0.0,  0.0],
    [0.0, 0.0, -1.0],
])

R_CAM_BODY = R_BODY_CAM.T


@dataclass
class Camera:
    width: int
    height: int
    hfov_deg: float
    vfov_deg: float

    def __post_init__(self):
        hfov = math.radians(self.hfov_deg)
        vfov = math.radians(self.vfov_deg)

        self.cx = self.width / 2.0
        self.cy = self.height / 2.0

        self.fx = (self.width / 2.0) / math.tan(hfov / 2.0)
        self.fy = (self.height / 2.0) / math.tan(vfov / 2.0)

    def project_bearing_cam_to_pixel(self, bearing_cam):
        x, y, z = bearing_cam

        if z <= 0.0:
            return False, None, None, None, None

        ex = x / z
        ey = y / z

        u = self.cx + self.fx * ex
        v = self.cy + self.fy * ey

        visible = 0.0 <= u < self.width and 0.0 <= v < self.height

        return visible, u, v, ex, ey
