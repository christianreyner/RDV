# mavlink_client.py

import time

from pymavlink import mavutil

from math_utils import quat_from_euler


class ArduPilotClient:
    def __init__(self, connection_string, baud=None):
        # Start monotonic timer for MAVLink timestamps
        self.start_time = time.monotonic()
        if baud is None:
            self.master = mavutil.mavlink_connection(connection_string)
        else:
            self.master = mavutil.mavlink_connection(
                connection_string,
                baud=baud,
            )

        print("Waiting for heartbeat...")
        self.master.wait_heartbeat()
        print(
            f"Heartbeat from system={self.master.target_system}, "
            f"component={self.master.target_component}"
        )

        self.last_local_position = None
        self.last_attitude = None

    def request_data_streams(self, rate_hz=50):
        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION,
            rate_hz,
            1,
        )

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1,
            rate_hz,
            1,
        )

    def set_mode_guided(self):
        mode = "GUIDED"

        if mode not in self.master.mode_mapping():
            raise RuntimeError(f"{mode} mode not available")

        mode_id = self.master.mode_mapping()[mode]

        self.master.mav.set_mode_send(
            self.master.target_system,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            mode_id,
        )

        print("Requested GUIDED mode")

    def arm(self):
        print("Arming...")
        self.master.arducopter_arm()
        self.master.motors_armed_wait()
        print("Armed")

    def update_messages(self):
        while True:
            msg = self.master.recv_match(blocking=False)
            if msg is None:
                break

            mtype = msg.get_type()

            if mtype == "LOCAL_POSITION_NED":
                self.last_local_position = msg

            elif mtype == "ATTITUDE":
                self.last_attitude = msg

    def wait_for_initial_state(self):
        print("Waiting for LOCAL_POSITION_NED and ATTITUDE...")

        while self.last_local_position is None or self.last_attitude is None:
            self.update_messages()
            time.sleep(0.01)

        print("Got initial state")

    def send_attitude_target(self, roll, pitch, yaw, thrust):
        q = quat_from_euler(roll, pitch, yaw)

        # Milliseconds elapsed since client initialization
        time_boot_ms = int((time.monotonic() - self.start_time) * 1000) & 0xFFFFFFFF

        # Use attitude + thrust, ignore body rates.
        type_mask = 0b00000111

        self.master.mav.set_attitude_target_send(
            time_boot_ms,
            self.master.target_system,
            self.master.target_component,
            type_mask,
            q,
            0.0,
            0.0,
            0.0,
            thrust,
        )

    def hold_current_position(self, yaw):
        time_boot_ms = (
            int((time.monotonic() - self.start_time) * 1000)
            & 0xFFFFFFFF
        )

        # Use position + yaw.
        # Ignore velocity, acceleration, and yaw rate.
        type_mask = 0b100111111000  # 2552

        self.master.mav.set_position_target_local_ned_send(
            time_boot_ms,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_OFFSET_NED,
            type_mask,
            0.0, 0.0, 0.0,  # Hold current position and altitude
            0.0, 0.0, 0.0,  # Velocity: ignored
            0.0, 0.0, 0.0,  # Acceleration: ignored
            yaw,            # Fixed heading, radians
            0.0,            # Yaw rate: ignored
        )
