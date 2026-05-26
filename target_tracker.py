# target_tracker.py

from dataclasses import dataclass

import numpy as np

from camera import R_CAM_BODY
from math_utils import euler_to_rotation_body_to_ned


@dataclass
class TargetProjection:
    visible: bool
    u: float | None
    v: float | None
    ex: float | None
    ey: float | None
    bearing_cam: np.ndarray
    range_m: float


class TargetTracker:
    def __init__(
        self,
        camera,
        target_north_m,
        target_east_m,
        target_alt_above_home_m,
    ):
        self.camera = camera

        self.target_ned = np.array([
            target_north_m,
            target_east_m,
            -target_alt_above_home_m,
        ])

    def project_from_state(
        self,
        north_m,
        east_m,
        down_m,
        roll_rad,
        pitch_rad,
        yaw_rad,
    ):
        vehicle_ned = np.array([
            north_m,
            east_m,
            down_m,
        ])

        rel_ned = self.target_ned - vehicle_ned
        range_m = np.linalg.norm(rel_ned)

        if range_m < 1e-6:
            return None

        bearing_ned = rel_ned / range_m

        r_body_to_ned = euler_to_rotation_body_to_ned(
            roll_rad,
            pitch_rad,
            yaw_rad,
        )

        bearing_body = r_body_to_ned.T @ bearing_ned
        bearing_cam = R_CAM_BODY @ bearing_body

        visible, u, v, ex, ey = self.camera.project_bearing_cam_to_pixel(
            bearing_cam
        )

        return TargetProjection(
            visible=visible,
            u=u,
            v=v,
            ex=ex,
            ey=ey,
            bearing_cam=bearing_cam,
            range_m=range_m,
        )
