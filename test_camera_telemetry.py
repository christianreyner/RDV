import time
import unittest
from types import SimpleNamespace
from unittest.mock import Mock, patch

from mavlink_client import ArduPilotClient


def message(kind, system=1, component=1, **fields):
    return SimpleNamespace(get_type=lambda: kind, get_srcSystem=lambda: system,
                           get_srcComponent=lambda: component, **fields)


class CameraTelemetryTests(unittest.TestCase):
    def connect(self):
        master = Mock()
        master.recv_match.side_effect = [
            message('HEARTBEAT', 255, 190, autopilot=8, type=6),
            message('HEARTBEAT', autopilot=3, type=2),
        ]
        with patch('mavlink_client.mavutil.mavlink_connection', return_value=master):
            client = ArduPilotClient('fake')
        return client, master

    def test_gcs_heartbeat_is_not_selected(self):
        client, master = self.connect()
        self.assertEqual((master.target_system, master.target_component), (1, 1))

    def test_camera_stream_requests_need_no_local_or_global_position(self):
        client, master = self.connect()
        client.request_camera_streams(20)
        calls = master.mav.command_long_send.call_args_list
        self.assertEqual([c.args[4] for c in calls], [30, 74])
        self.assertTrue(all(c.args[5] == 50000 for c in calls))

    def test_altitude_updates_without_local_position_and_rejects_nan(self):
        client, master = self.connect()
        master.recv_match.side_effect = [
            message('VFR_HUD', 255, 190, alt=100),
            message('VFR_HUD', alt=700),
            message('VFR_HUD', alt=float('nan')), None,
        ]
        before = time.monotonic()
        client.update_messages()
        self.assertEqual(client.last_altitude, 700)
        self.assertGreaterEqual(client.last_altitude_time, before)
        self.assertIsNone(client.last_local_position)


if __name__ == '__main__':
    unittest.main()
