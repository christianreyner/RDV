# app.py

import time

import config as cfg

from camera import Camera
from controller import VisualPDController, VisualPDControllerConfig
from csv_logger import CSVLogger
from logger import StatusLogger
from mavlink_client import ArduPilotClient
from target_tracker import MovingTarget, TargetTracker
from visualizer import LiveVisualizer


class VisualLaunchTrackerApp:
    def __init__(self):
        self.camera = Camera(
            width=cfg.CAMERA_WIDTH_PX,
            height=cfg.CAMERA_HEIGHT_PX,
            hfov_deg=cfg.CAMERA_HFOV_DEG,
            vfov_deg=cfg.CAMERA_VFOV_DEG,
        )

        self.moving_target = MovingTarget(
            initial_north_m=cfg.TARGET_INITIAL_NORTH_M,
            initial_east_m=cfg.TARGET_INITIAL_EAST_M,
            alt_above_home_m=cfg.TARGET_ALT_ABOVE_HOME_M,
            speed_mps=cfg.TARGET_SPEED_MPS,
            heading_deg=cfg.TARGET_HEADING_DEG,
        )
        
        self.tracker = TargetTracker(
            camera=self.camera,
            moving_target=self.moving_target,
        )


        self.controller = VisualPDController(
            cfg=VisualPDControllerConfig(
                kpx=cfg.KPX,
                kdx=cfg.KDX,
                kpy=cfg.KPY,
                kdy=cfg.KDY,

                max_accel_mss=cfg.MAX_ACCEL_MSS,
                max_tilt_deg=cfg.MAX_TILT_DEG,

                max_roll_rate_deg_s=cfg.MAX_ROLL_RATE_DEG_S,
                max_pitch_rate_deg_s=cfg.MAX_PITCH_RATE_DEG_S,

                nominal_climb_thrust=cfg.NOMINAL_CLIMB_THRUST,
                post_takeoff_thrust=cfg.POST_TAKEOFF_THRUST,
                post_takeoff_alt_m=cfg.POST_TAKEOFF_ALT_M,

                min_thrust=cfg.MIN_THRUST,
                max_thrust=cfg.MAX_THRUST,

                compensate_tilt_thrust=cfg.COMPENSATE_TILT_THRUST,
            ),
            dt=cfg.DT,
        )

        self.logger = StatusLogger()
        self.csv_logger = CSVLogger(cfg.CSV_LOG_PATH)

        self.visualizer = None
        self.visualization_frame_count = 0

        if cfg.ENABLE_VISUALIZATION:
            self.visualizer = LiveVisualizer(camera=self.camera)
            
        self.ap = ArduPilotClient(cfg.SITL_CONNECTION)

    def run(self):
        target_state_0 = self.tracker.get_target_state(elapsed_s=0.0)

        self.logger.print_startup_info(
            camera=self.camera,
            target_ned=[
                target_state_0.north_m,
                target_state_0.east_m,
                target_state_0.down_m,
            ],
        )

        self.ap.request_data_streams(rate_hz=50)
        self.ap.set_mode_guided()
        self.ap.wait_for_initial_state()

        self._prearm_attitude_hold()

        self.ap.arm()

        self._initial_level_climb()

        try:
            self._tracking_loop()

        except KeyboardInterrupt:
            self._send_stop_command()

        finally:
            self.csv_logger.close()
            
            if self.visualizer is not None:
                self.visualizer.close()
                
            print(f"CSV log saved to: {cfg.CSV_LOG_PATH}")

    def _prearm_attitude_hold(self):
        print(f"Sending pre-arm neutral attitude for {cfg.PREARM_ATTITUDE_SECONDS:.1f} s...")

        t0 = time.time()

        while time.time() - t0 < cfg.PREARM_ATTITUDE_SECONDS:
            self.ap.update_messages()

            att = self.ap.last_attitude
            yaw_hold = att.yaw if att is not None else 0.0

            self.ap.send_attitude_target(
                roll=0.0,
                pitch=0.0,
                yaw=yaw_hold,
                thrust=0.0,
            )

            time.sleep(0.02)

    def _initial_level_climb(self):
        print(f"Initial level climb for {cfg.INITIAL_LEVEL_SECONDS_AFTER_ARM:.1f} s...")

        t0 = time.time()

        while time.time() - t0 < cfg.INITIAL_LEVEL_SECONDS_AFTER_ARM:
            self.ap.update_messages()

            att = self.ap.last_attitude
            yaw_hold = att.yaw if att is not None else 0.0

            self.ap.send_attitude_target(
                roll=0.0,
                pitch=0.0,
                yaw=yaw_hold,
                thrust=cfg.NOMINAL_CLIMB_THRUST,
            )

            time.sleep(0.02)

    def _tracking_loop(self):
        next_time = time.time()
        start_time = time.time()

        print("Starting launch + visual tracking loop.")
        print("Press Ctrl+C to stop.")
        print(f"Logging CSV to: {cfg.CSV_LOG_PATH}")

        while True:
            now = time.time()
            elapsed_s = now - start_time

            if now < next_time:
                time.sleep(max(0.0, next_time - now))
                continue

            next_time += cfg.DT

            self.ap.update_messages()

            pos = self.ap.last_local_position
            att = self.ap.last_attitude

            if pos is None or att is None:
                continue

            target_state = self.tracker.get_target_state(elapsed_s)

            projection = self.tracker.project_from_state(
                target_state=target_state,
                north_m=pos.x,
                east_m=pos.y,
                down_m=pos.z,
                roll_rad=att.roll,
                pitch_rad=att.pitch,
                yaw_rad=att.yaw,
            )

            if projection is None:
                continue

            altitude_above_home = -pos.z

            if projection.visible:
                command = self.controller.compute_visible_target_command(
                    ex=projection.ex,
                    ey=projection.ey,
                    altitude_above_home=altitude_above_home,
                )
            else:
                command = self.controller.compute_target_lost_command(
                    altitude_above_home=altitude_above_home,
                )

            if projection.visible or cfg.LEVEL_WHEN_TARGET_NOT_VISIBLE:
                self.ap.send_attitude_target(
                    roll=command["roll"],
                    pitch=command["pitch"],
                    yaw=att.yaw,
                    thrust=command["thrust"],
                )

            self.logger.print_tracking_status(
                command=command,
                projection=projection,
                altitude_above_home=altitude_above_home,
            )

            self.csv_logger.log(
                time_s=elapsed_s,
                quad_pos=pos,
                target_state=target_state,
                projection=projection,
                command=command,
            )

            if self.visualizer is not None:
                self.visualization_frame_count += 1

                if (
                    self.visualization_frame_count
                    % cfg.VISUALIZATION_EVERY_N_FRAMES
                    == 0
                ):
                    self.visualizer.update(
                        quad_pos=pos,
                        target_state=target_state,
                        projection=projection,
                    )

    def _send_stop_command(self):
        print()
        print("Stopping. Sending low thrust level command...")

        for _ in range(30):
            self.ap.update_messages()

            att = self.ap.last_attitude
            yaw_hold = att.yaw if att is not None else 0.0

            self.ap.send_attitude_target(
                roll=0.0,
                pitch=0.0,
                yaw=yaw_hold,
                thrust=0.35,
            )

            time.sleep(0.05)

        print("Done.")
