"""Click -> stable CSRT lock -> Enter confirmation -> arm -> climb -> track."""
import csv
import time

import config as cfg
from camera import Camera
from controller import VisualPDController, VisualPDControllerConfig
from live_camera import LiveCameraTracker
from mavlink_client import ArduPilotClient


class LiveCameraApp:
    def __init__(self):
        self.camera = Camera(cfg.CAMERA_WIDTH_PX, cfg.CAMERA_HEIGHT_PX,
                             cfg.CAMERA_HFOV_DEG, cfg.CAMERA_VFOV_DEG)
        values = {name: getattr(cfg, name.upper())
                  for name in VisualPDControllerConfig.__dataclass_fields__}
        self.controller = VisualPDController(VisualPDControllerConfig(**values), cfg.DT)
        self.ap = None
        self.vision = None
        self.logfile = None
        self.launch_requested = False
        self.yaw_reference = None
        self.previous_observation_time = None

    def _fresh_state(self):
        now = time.monotonic()
        return (self.ap.last_attitude is not None and self.ap.last_local_position is not None and
                now-self.ap.last_attitude_time < cfg.TELEMETRY_STALE_SECONDS and
                now-self.ap.last_position_time < cfg.TELEMETRY_STALE_SECONDS and
                now-self.ap.last_heartbeat_time < 3.0)

    def _pump(self, require_lock=False):
        if self.ap is not None:
            self.ap.update_messages()
        key, new_frame = self.vision.step()
        if require_lock and not self.vision.ready():
            raise RuntimeError("Target lock lost; launch/tracking cancelled")
        return key, new_frame

    def _wait(self, predicate, label):
        deadline = time.monotonic()+cfg.STARTUP_TIMEOUT_SECONDS
        while True:
            self._pump(require_lock=True)
            if predicate():
                return
            if time.monotonic() >= deadline:
                raise RuntimeError(f"Timed out waiting for {label}")

    def run(self):
        try:
            self.vision = LiveCameraTracker(self.camera)
            print("Click target; watch the box follow it. Enter confirms selection and launch.")
            print("R resets before launch; Q/Esc/Ctrl+C stops. No flight command before Enter.")
            print(f"DRY_RUN={cfg.DRY_RUN} | Camera {cfg.TRACKING_FPS} FPS | Preview divisor {cfg.PREVIEW_DIVISOR}")
            while True:
                key, _ = self._pump()
                if key not in (10, 13):
                    continue
                if not self.vision.ready():
                    print("Selection not ready: click and wait for a stable lock.")
                    continue
                if cfg.DRY_RUN:
                    print("Selection confirmed in dry run. No MAVLink connection or arming. Q exits.")
                    # Continue observing; dry run never reaches flight startup.
                    continue
                break

            # Freeze reselection as soon as flight is confirmed.
            self.vision.flying = True
            self.ap = ArduPilotClient(cfg.MAVLINK_CONNECTION, baud=cfg.MAVLINK_BAUD,
                                      heartbeat_timeout=cfg.STARTUP_TIMEOUT_SECONDS)
            self.ap.request_data_streams(rate_hz=50)
            self._wait(self._fresh_state, "fresh position and attitude")
            if self.ap.master.motors_armed():
                raise RuntimeError("Launch sequence requires a disarmed aircraft")
            self.ap.master.mav.param_request_read_send(
                self.ap.master.target_system, self.ap.master.target_component,
                b"GUID_OPTIONS", -1)
            self._wait(lambda: "GUID_OPTIONS" in self.ap.parameters, "GUID_OPTIONS")
            if not int(self.ap.parameters["GUID_OPTIONS"]) & 8:
                raise RuntimeError("Controller requires GUID_OPTIONS bit 8 (direct thrust); parameter was not changed")
            self.ap.set_mode_guided()
            self._wait(lambda: self.ap.master.flightmode == "GUIDED", "GUIDED heartbeat")
            self.yaw_reference = self.ap.last_attitude.yaw
            self._level_phase(cfg.PREARM_ATTITUDE_SECONDS, thrust=0.0, armed=False)
            self.launch_requested = True
            self.ap.master.arducopter_arm()
            self._wait(self.ap.master.motors_armed, "arming heartbeat")
            self._level_phase(cfg.INITIAL_LEVEL_SECONDS_AFTER_ARM,
                              thrust=cfg.NOMINAL_CLIMB_THRUST, armed=True)
            self.logfile = open(cfg.LIVE_CSV_LOG_PATH, "w", newline="")
            writer = csv.writer(self.logfile)
            writer.writerow(["time_s", "u_px", "v_px", "ex", "ey", "roll_rad", "pitch_rad", "thrust", "alt_ekf_m"])
            start = time.monotonic()
            while True:
                _, new_frame = self._pump(require_lock=True)
                self._check_flight()
                if not new_frame:
                    continue
                obs = self.vision.observation
                # One controller update per new tracked frame; no reused-frame derivatives.
                if self.previous_observation_time is None:
                    self.controller.prev_ex, self.controller.prev_ey = obs.ex, obs.ey
                else:
                    self.controller.dt = max(0.001, obs.timestamp-self.previous_observation_time)
                self.previous_observation_time = obs.timestamp
                altitude = -self.ap.last_local_position.z
                cmd = self.controller.compute_visible_target_command(obs.ex, obs.ey, altitude)
                self.ap.send_attitude_target(cmd["roll"], cmd["pitch"], self.yaw_reference, cmd["thrust"])
                writer.writerow([time.monotonic()-start, obs.u, obs.v, obs.ex, obs.ey,
                                 cmd["roll"], cmd["pitch"], cmd["thrust"], altitude])
                self.logfile.flush()
        except KeyboardInterrupt:
            print("Stopped by operator.")
        except Exception as exc:
            print(f"Stopped: {exc}")
            raise
        finally:
            try:
                if self.ap is not None and self.launch_requested:
                    # A pending arm request may have succeeded even if its heartbeat was missed.
                    self.ap.update_messages()
                    if self.ap.master.flightmode == "GUIDED" and self.yaw_reference is not None:
                        self.ap.hold_current_position(self.yaw_reference)
                        print("Position hold requested once; reception/acceptance is not verified.")
                    else:
                        print("Flight mode changed; no Guided hold sent.")
            finally:
                if self.logfile is not None:
                    self.logfile.close()
                if self.vision is not None:
                    self.vision.close()
                if self.ap is not None:
                    self.ap.master.close()

    def _check_flight(self):
        if not self._fresh_state():
            raise RuntimeError("Telemetry stale")
        if self.ap.master.flightmode != "GUIDED":
            raise RuntimeError("Pilot/autopilot left GUIDED; attitude commands stopped")
        if not self.ap.master.motors_armed():
            raise RuntimeError("Aircraft disarmed; attitude commands stopped")

    def _level_phase(self, duration, thrust, armed):
        deadline = time.monotonic()+duration
        while time.monotonic() < deadline:
            _, new_frame = self._pump(require_lock=True)
            if armed:
                self._check_flight()
            elif not self._fresh_state() or self.ap.master.flightmode != "GUIDED" or self.ap.master.motors_armed():
                raise RuntimeError("Prearm state changed")
            if new_frame:
                self.ap.send_attitude_target(0.0, 0.0, self.yaw_reference, thrust)
