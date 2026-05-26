# controller.py

import math
from dataclasses import dataclass

from math_utils import clamp, slew_limit


@dataclass
class VisualPDControllerConfig:
    kpx: float
    kdx: float
    kpy: float
    kdy: float

    max_accel_mss: float
    max_tilt_deg: float

    max_roll_rate_deg_s: float
    max_pitch_rate_deg_s: float

    nominal_climb_thrust: float
    post_takeoff_thrust: float
    post_takeoff_alt_m: float

    min_thrust: float
    max_thrust: float

    compensate_tilt_thrust: bool


class VisualPDController:
    def __init__(self, cfg: VisualPDControllerConfig, dt: float):
        self.cfg = cfg
        self.dt = dt

        self.prev_ex = 0.0
        self.prev_ey = 0.0
        self.prev_roll_cmd = 0.0
        self.prev_pitch_cmd = 0.0

        self.max_tilt = math.radians(cfg.max_tilt_deg)
        self.max_roll_rate = math.radians(cfg.max_roll_rate_deg_s)
        self.max_pitch_rate = math.radians(cfg.max_pitch_rate_deg_s)

    def get_base_thrust_and_phase(self, altitude_above_home):
        if altitude_above_home < self.cfg.post_takeoff_alt_m:
            return self.cfg.nominal_climb_thrust, "launch"
        else:
            return self.cfg.post_takeoff_thrust, "track"

    def compute_visible_target_command(self, ex, ey, altitude_above_home):
        """
        Compute roll, pitch, thrust command when target is visible.

        ex:
          normalized horizontal image error.
          positive means target is right in image.

        ey:
          normalized vertical image error.
          positive means target is down in image.
        """
        dex = (ex - self.prev_ex) / self.dt
        dey = (ey - self.prev_ey) / self.dt

        self.prev_ex = ex
        self.prev_ey = ey

        # PD in image coordinates.
        #
        # For upward camera:
        #   image right -> body right
        #   image down  -> body forward
        ax_body_cmd = self.cfg.kpy * ey + self.cfg.kdy * dey
        ay_body_cmd = self.cfg.kpx * ex + self.cfg.kdx * dex

        ax_body_cmd = clamp(
            ax_body_cmd,
            -self.cfg.max_accel_mss,
            self.cfg.max_accel_mss,
        )

        ay_body_cmd = clamp(
            ay_body_cmd,
            -self.cfg.max_accel_mss,
            self.cfg.max_accel_mss,
        )

        pitch_cmd = math.atan2(-ax_body_cmd, 9.81)
        roll_cmd = math.atan2(ay_body_cmd, 9.81)

        pitch_cmd = clamp(pitch_cmd, -self.max_tilt, self.max_tilt)
        roll_cmd = clamp(roll_cmd, -self.max_tilt, self.max_tilt)

        pitch_cmd = slew_limit(
            pitch_cmd,
            self.prev_pitch_cmd,
            self.max_pitch_rate,
            self.dt,
        )

        roll_cmd = slew_limit(
            roll_cmd,
            self.prev_roll_cmd,
            self.max_roll_rate,
            self.dt,
        )

        self.prev_pitch_cmd = pitch_cmd
        self.prev_roll_cmd = roll_cmd

        base_thrust, phase = self.get_base_thrust_and_phase(
            altitude_above_home
        )

        thrust = self._apply_thrust_compensation(
            base_thrust,
            roll_cmd,
            pitch_cmd,
        )

        return {
            "roll": roll_cmd,
            "pitch": pitch_cmd,
            "thrust": thrust,
            "phase": phase,
            "ax_body_cmd": ax_body_cmd,
            "ay_body_cmd": ay_body_cmd,
        }

    def compute_target_lost_command(self, altitude_above_home):
        """
        Level the vehicle while keeping climb/track thrust.
        """
        pitch_cmd = slew_limit(
            0.0,
            self.prev_pitch_cmd,
            self.max_pitch_rate,
            self.dt,
        )

        roll_cmd = slew_limit(
            0.0,
            self.prev_roll_cmd,
            self.max_roll_rate,
            self.dt,
        )

        self.prev_pitch_cmd = pitch_cmd
        self.prev_roll_cmd = roll_cmd

        base_thrust, phase = self.get_base_thrust_and_phase(
            altitude_above_home
        )

        thrust = self._apply_thrust_compensation(
            base_thrust,
            roll_cmd,
            pitch_cmd,
        )

        return {
            "roll": roll_cmd,
            "pitch": pitch_cmd,
            "thrust": thrust,
            "phase": phase,
        }

    def _apply_thrust_compensation(self, base_thrust, roll_cmd, pitch_cmd):
        thrust = base_thrust

        if self.cfg.compensate_tilt_thrust:
            denom = math.cos(roll_cmd) * math.cos(pitch_cmd)
            denom = max(0.3, denom)
            thrust = base_thrust / denom

        thrust = clamp(
            thrust,
            self.cfg.min_thrust,
            self.cfg.max_thrust,
        )

        return thrust
