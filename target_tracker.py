# target_tracker.py

from dataclasses import dataclass
import math

import numpy as np

from camera import R_CAM_BODY
from math_utils import euler_to_rotation_body_to_ned


@dataclass
class TargetState:
    north_m: float
    east_m: float
    down_m: float


@dataclass
class TargetProjection:
    visible: bool
    u: float | None
    v: float | None
    ex: float | None
    ey: float | None
    bearing_cam: np.ndarray
    range_m: float


class MovingTarget:
    def __init__(
        self,
        initial_north_m,
        initial_east_m,
        alt_above_home_m,
        speed_mps,
        heading_deg,
    ):
        self.initial_north_m = initial_north_m
        self.initial_east_m = initial_east_m
        self.down_m = -alt_above_home_m

        self.speed_mps = speed_mps
        self.heading_rad = math.radians(heading_deg)

    def get_state(self, elapsed_s):
        north_m = (
            self.initial_north_m
            + self.speed_mps * math.cos(self.heading_rad) * elapsed_s
        )

        east_m = (
            self.initial_east_m
            + self.speed_mps * math.sin(self.heading_rad) * elapsed_s
        )

        return TargetState(
            north_m=north_m,
            east_m=east_m,
            down_m=self.down_m,
        )


class TargetTracker:
    def __init__(self, camera, moving_target):
        self.camera = camera
        self.moving_target = moving_target

    def get_target_state(self, elapsed_s):
        return self.moving_target.get_state(elapsed_s)

    def project_from_state(
        self,
        target_state,
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

        target_ned = np.array([
            target_state.north_m,
            target_state.east_m,
            target_state.down_m,
        ])

        rel_ned = target_ned - vehicle_ned
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
