# mavlink_client.py

import time

from pymavlink import mavutil

from math_utils import quat_from_euler


class ArduPilotClient:
    def __init__(self, connection_string):
        self.master = mavutil.mavlink_connection(connection_string)

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

        # Use attitude + thrust, ignore body rates.
        type_mask = 0b00000111

        self.master.mav.set_attitude_target_send(
            int(time.time() * 1e6) & 0xFFFFFFFF,
            self.master.target_system,
            self.master.target_component,
            type_mask,
            q,
            0.0,
            0.0,
            0.0,
            thrust,
        )
