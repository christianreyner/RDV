import io
import sys
import tempfile
import time
import unittest
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import patch

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import config as cfg
from camera import Camera
from live_camera import LiveCameraTracker, Observation, LatestFrameReader
from live_app import LiveCameraApp


class FakeVision:
    def __init__(self, camera):
        self.calls = 0
        self.flying = False
        self.closed = False
        self.observation = Observation(650, 370, .01, .02, time.monotonic())
    def ready(self):
        return True
    def step(self):
        self.calls += 1
        self.observation.timestamp = time.monotonic()
        if self.calls == 1:
            return 13, True
        if self.calls >= 8:
            raise KeyboardInterrupt
        return 0, True
    def close(self):
        self.closed = True


class FakeAP:
    options = 8
    instances = []
    def __init__(self, *args, **kwargs):
        self.__class__.instances.append(self)
        self.armed = False
        self.commands = []
        self.holds = []
        self.parameters = {}
        self.last_attitude = SimpleNamespace(yaw=.6)
        self.last_local_position = SimpleNamespace(z=-3)
        self.master = SimpleNamespace(target_system=1, target_component=1,
            flightmode='STABILIZE', motors_armed=lambda: self.armed,
            arducopter_arm=self.arm, close=lambda: None,
            mav=SimpleNamespace(param_request_read_send=self.param))
        self.update_messages()
    def param(self, *args):
        self.parameters['GUID_OPTIONS'] = self.options
    def arm(self):
        self.armed = True
    def update_messages(self):
        self.last_attitude_time = self.last_position_time = self.last_heartbeat_time = time.monotonic()
    def request_data_streams(self, **kwargs):
        pass
    def set_mode_guided(self):
        self.master.flightmode = 'GUIDED'
    def send_attitude_target(self, *args):
        self.commands.append(args)
    def hold_current_position(self, yaw):
        self.holds.append(yaw)


class FlowTests(unittest.TestCase):
    def run_app(self, dry=False, vision=FakeVision):
        FakeAP.instances.clear()
        with tempfile.TemporaryDirectory() as td, \
             patch('live_app.LiveCameraTracker', vision), \
             patch('live_app.ArduPilotClient', FakeAP), \
             patch.object(cfg, 'DRY_RUN', dry), \
             patch.object(cfg, 'PREARM_ATTITUDE_SECONDS', 0), \
             patch.object(cfg, 'INITIAL_LEVEL_SECONDS_AFTER_ARM', 0), \
             patch.object(cfg, 'LIVE_CSV_LOG_PATH', str(Path(td)/'log.csv')):
            app = LiveCameraApp()
            app.run()
            return app

    def test_dry_run_never_connects_or_arms(self):
        app = self.run_app(dry=True)
        self.assertFalse(FakeAP.instances)
        self.assertTrue(app.vision.closed)

    def test_enter_without_lock_never_connects(self):
        class NoLock(FakeVision):
            def ready(self):
                return False
        self.run_app(vision=NoLock)
        self.assertFalse(FakeAP.instances)

    def test_confirm_arm_track_and_hold_once(self):
        app = self.run_app()
        ap = app.ap
        self.assertTrue(ap.armed)
        self.assertGreater(len(ap.commands), 0)
        self.assertEqual(ap.holds, [.6])
        self.assertTrue(all(cmd[2] == .6 for cmd in ap.commands))
        self.assertTrue(app.vision.closed)

    def test_wrong_thrust_semantics_blocks_arming(self):
        with patch.object(FakeAP, 'options', 0):
            with self.assertRaisesRegex(RuntimeError, 'direct thrust'):
                self.run_app()
        self.assertFalse(FakeAP.instances[0].armed)
        self.assertFalse(FakeAP.instances[0].commands)

    def test_camera_error_airborne_requests_hold(self):
        class Broken(FakeVision):
            def step(self):
                if FakeAP.instances and FakeAP.instances[0].armed:
                    raise RuntimeError('camera failed')
                return super().step()
        with self.assertRaisesRegex(RuntimeError, 'camera failed'):
            self.run_app(vision=Broken)
        self.assertEqual(FakeAP.instances[0].holds, [.6])

    def test_pilot_mode_change_preserved(self):
        class Pilot(FakeVision):
            def step(self):
                if FakeAP.instances and FakeAP.instances[0].armed:
                    FakeAP.instances[0].master.flightmode = 'LOITER'
                return super().step()
        with self.assertRaisesRegex(RuntimeError, 'left GUIDED'):
            self.run_app(vision=Pilot)
        self.assertFalse(FakeAP.instances[0].commands)
        self.assertFalse(FakeAP.instances[0].holds)

    def test_lock_loss_after_confirmation_prevents_arming(self):
        class Lost(FakeVision):
            def ready(self):
                return self.calls <= 1
        with self.assertRaisesRegex(RuntimeError, 'Target lock lost'):
            self.run_app(vision=Lost)
        self.assertFalse(FakeAP.instances[0].armed)

    def test_pending_reselection_invalidates_confirmation(self):
        from live_camera import Selection
        vision = LiveCameraTracker.__new__(LiveCameraTracker)
        vision.selection = Selection()
        vision.observation = Observation(10, 10, 0, 0, time.monotonic())
        vision.lock_count = cfg.LOCK_CONFIRM_FRAMES
        self.assertTrue(vision.ready())
        vision.selection.pending = (20, 30)
        self.assertFalse(vision.ready())

    def test_preview_scaling_keeps_full_resolution_selection_box(self):
        from live_camera import create_tracker
        camera = Camera(1280, 720, 90, 70)
        frame = np.random.default_rng(1).integers(0, 255, (720,1280,3), dtype=np.uint8)
        raw = cv2.cvtColor(frame, cv2.COLOR_BGR2YUV_I420).tobytes()
        class Process:
            def __init__(self, *args, **kwargs):
                self.stdout = io.BytesIO(raw)
            def poll(self): return 0
            def wait(self, **kwargs): return 0
        boxes = []
        class Tracker:
            def init(self, frame, box):
                self.box = box
                boxes.append(box)
            def update(self, frame):
                return True, self.box
        for divisor in (2, 4):
            with patch('live_camera.subprocess.Popen', Process), \
                 patch('live_camera.create_tracker', Tracker), \
                 patch.object(cfg, 'PREVIEW_DIVISOR', divisor), \
                 patch('live_camera.cv2.namedWindow'), \
                 patch('live_camera.cv2.setMouseCallback'), \
                 patch('live_camera.cv2.imshow'), \
                 patch('live_camera.cv2.destroyAllWindows'), \
                 patch('live_camera.cv2.waitKey', return_value=-1), \
                 patch('live_camera.cv2.getWindowProperty', return_value=1):
                vision = LiveCameraTracker(camera)
                vision.selection.pending = (640//divisor, 360//divisor)
                vision.step()
                self.assertAlmostEqual(vision.observation.ex, 0)
                self.assertAlmostEqual(vision.observation.ey, 0)
                vision.close()
        self.assertEqual(boxes, [(620,340,40,40), (620,340,40,40)])

    def test_reader_uses_newest_complete_frame(self):
        reader = LatestFrameReader(io.BytesIO(b'abcDEF'), 3)
        reader.thread.join(1)
        seq, raw, stamp = reader.next(0)
        self.assertEqual((seq, raw), (2, b'DEF'))
        self.assertGreater(stamp, 0)
        self.assertIsNone(reader.next(seq))

    def test_real_csrt_tracks_textured_translation(self):
        from live_camera import create_tracker
        rng = np.random.default_rng(4)
        frame = np.zeros((240,320,3), np.uint8)
        texture = rng.integers(30, 255, (40,40,3), dtype=np.uint8)
        frame[100:140,140:180] = texture
        tracker = create_tracker()
        tracker.init(frame, (140,100,40,40))
        shifted = np.zeros_like(frame)
        shifted[103:143,145:185] = texture
        success, (x,y,w,h) = tracker.update(shifted)
        self.assertTrue(success)
        self.assertAlmostEqual(x+w/2, 165, delta=4)
        self.assertAlmostEqual(y+h/2, 123, delta=4)


if __name__ == '__main__':
    unittest.main()
