"""Live camera tracker based on the supplied rpicam/CSRT reference."""
import math
import subprocess
import threading
import time
from dataclasses import dataclass

import cv2
import numpy as np
import config as cfg


def read_exact(stream, size):
    data = bytearray()
    while len(data) < size:
        chunk = stream.read(size - len(data))
        if not chunk:
            return None
        data.extend(chunk)
    return bytes(data)


class LatestFrameReader:
    """Continuously drain the pipe; retain only the newest complete raw frame."""

    def __init__(self, stream, frame_size):
        self.frame_size = frame_size
        self.stream = stream
        self.condition = threading.Condition()
        self.sequence = 0
        self.raw = None
        self.timestamp = 0.0
        self.ended = False
        self.stop = threading.Event()
        self.error = None
        self.thread = threading.Thread(target=self.run, daemon=True)
        self.thread.start()

    def run(self):
        try:
            while not self.stop.is_set():
                raw = read_exact(self.stream, self.frame_size)
                if raw is None:
                    break
                with self.condition:
                    self.raw = raw
                    self.timestamp = time.monotonic()
                    self.sequence += 1
                    self.condition.notify_all()
        except Exception as exc:
            self.error = exc
        finally:
            with self.condition:
                self.ended = True
                self.condition.notify_all()

    def next(self, previous_sequence):
        with self.condition:
            self.condition.wait_for(
                lambda: self.sequence > previous_sequence or self.ended,
                timeout=0.05,
            )
            if self.sequence > previous_sequence:
                return self.sequence, self.raw, self.timestamp
            return None


class Selection:
    """One click selects a target; mouse coordinates belong to the preview."""

    def __init__(self):
        self.pending = None

    def clear(self):
        self.pending = None

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.pending = (x, y)


def create_tracker():
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    raise RuntimeError("OpenCV lacks CSRT. Install an OpenCV contrib build.")



@dataclass
class Observation:
    u: float
    v: float
    ex: float
    ey: float
    timestamp: float


class LiveCameraTracker:
    def __init__(self, camera):
        self.camera = camera
        self.width, self.height = camera.width, camera.height
        if self.width % 32 or self.height % 2:
            raise ValueError("Raw YUV width must be divisible by 32; height must be even")
        if min(cfg.TRACKING_FPS, cfg.PREVIEW_DIVISOR, cfg.PREVIEW_FPS) <= 0:
            raise ValueError("Camera/preview rates and divisor must be positive")
        if not (10 < cfg.SELECT_BOX_WIDTH <= self.width and
                10 < cfg.SELECT_BOX_HEIGHT <= self.height):
            raise ValueError("Selection box must fit the camera frame and exceed 10 pixels")
        self.pw = max(1, round(self.width / cfg.PREVIEW_DIVISOR))
        self.ph = max(1, round(self.height / cfg.PREVIEW_DIVISOR))
        self.selection = Selection()
        self.tracker = self.bbox = self.observation = None
        self.display_frame = None
        self.sequence = 0
        self.lock_count = 0
        self.next_preview = 0.0
        self.last_frame_time = time.monotonic()
        self.flying = False
        self.work = self.count = self.skipped = 0
        self.report_time = time.monotonic()
        self.state = "Click target | Enter: confirm & launch | R: reset | Q: quit"
        cmd = [cfg.CAMERA_COMMAND, "--camera", str(cfg.CAMERA_INDEX),
               "--width", str(self.width), "--height", str(self.height),
               "--framerate", str(cfg.TRACKING_FPS), "--codec", "yuv420",
               "--nopreview", "--timeout", "0", "-o", "-"]
        self.process = subprocess.Popen(cmd, stdout=subprocess.PIPE, bufsize=0)
        self.reader = LatestFrameReader(self.process.stdout, self.width * self.height * 3 // 2)
        try:
            cv2.namedWindow(cfg.CAMERA_WINDOW, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(cfg.CAMERA_WINDOW, self.selection.callback)
        except BaseException:
            self.close()
            raise

    def reset(self):
        self.tracker = self.bbox = self.observation = None
        self.lock_count = 0
        self.selection.clear()
        self.state = "Click target | Enter: confirm & launch | R: reset | Q: quit"

    def ready(self):
        return (self.selection.pending is None and self.observation is not None and
                self.lock_count >= cfg.LOCK_CONFIRM_FRAMES and
                time.monotonic() - self.observation.timestamp < cfg.CAMERA_STALE_SECONDS)

    def step(self):
        item = self.reader.next(self.sequence)
        if item is not None:
            start = time.perf_counter()
            sequence, raw, captured = item
            self.skipped += max(0, sequence - self.sequence - 1)
            self.sequence = sequence
            self.last_frame_time = captured
            yuv = np.frombuffer(raw, dtype=np.uint8).reshape(self.height * 3 // 2, self.width)
            frame = cv2.cvtColor(yuv, cv2.COLOR_YUV2BGR_I420)
            if self.selection.pending is not None:
                if not self.flying:
                    px, py = self.selection.pending
                    cx, cy = round(px * self.width / self.pw), round(py * self.height / self.ph)
                    w, h = cfg.SELECT_BOX_WIDTH, cfg.SELECT_BOX_HEIGHT
                    box = (max(0, min(self.width-w, cx-w//2)),
                           max(0, min(self.height-h, cy-h//2)), w, h)
                    self.reset()
                    self.tracker = create_tracker()
                    result = self.tracker.init(self.display_frame if self.display_frame is not None else frame, box)
                    if result is False:
                        self.reset()
                    else:
                        self.bbox = box
                self.selection.clear()
            if self.tracker is not None:
                ok, box = self.tracker.update(frame)
                x, y, w, h = box
                valid = (ok and all(math.isfinite(v) for v in box) and
                         w > 0 and h > 0 and 0 <= x and 0 <= y and
                         x+w <= self.width and y+h <= self.height)
                if valid and self.bbox is not None:
                    ox, oy, ow, oh = self.bbox
                    shift = math.hypot(x+w/2-ox-ow/2, y+h/2-oy-oh/2)
                    ratio = w*h/(ow*oh)
                    valid = shift <= cfg.MAX_TRACK_JUMP_PX and 1/cfg.MAX_BOX_AREA_RATIO <= ratio <= cfg.MAX_BOX_AREA_RATIO
                if valid:
                    self.bbox = box
                    u, v = x+w/2, y+h/2
                    ex, ey = (u-self.camera.cx)/self.camera.fx, (v-self.camera.cy)/self.camera.fy
                    # Map mounted sensor axes into the controller's upward-camera axes.
                    angle = math.radians(cfg.CAMERA_IMAGE_ROTATION_DEG)
                    ex, ey = (math.cos(angle)*ex-math.sin(angle)*ey,
                              math.sin(angle)*ex+math.cos(angle)*ey)
                    self.observation = Observation(u, v, ex*cfg.CAMERA_X_SIGN,
                                                   ey*cfg.CAMERA_Y_SIGN, self.last_frame_time)
                    self.lock_count += 1
                    self.state = "FLYING" if self.flying else (
                        "LOCK READY | Enter: confirm & launch" if self.ready() else "LOCKING - observe target")
                else:
                    self.reset()
                    self.state = "TARGET LOST | Click to reselect"
            now = time.monotonic()
            if now >= self.next_preview:
                self.display_frame = frame.copy()
                preview = cv2.resize(frame, (self.pw, self.ph), interpolation=cv2.INTER_AREA)
                if self.bbox is not None:
                    x, y, w, h = self.bbox
                    cv2.rectangle(preview, (round(x*self.pw/self.width), round(y*self.ph/self.height)),
                                  (round((x+w)*self.pw/self.width), round((y+h)*self.ph/self.height)), (0,255,0), 2)
                cv2.drawMarker(preview, (self.pw//2, self.ph//2), (0,255,255), cv2.MARKER_CROSS, 16, 1)
                cv2.putText(preview, self.state, (8,20), cv2.FONT_HERSHEY_SIMPLEX, .45, (0,255,0), 1)
                cv2.imshow(cfg.CAMERA_WINDOW, preview)
                self.next_preview = now + 1/cfg.PREVIEW_FPS
            self.work += time.perf_counter()-start
            self.count += 1
            elapsed = now-self.report_time
            if elapsed >= cfg.STATS_INTERVAL:
                mean = self.work/self.count
                print(f"{self.state.split('|')[0].strip()} | FPS {self.count/elapsed:.1f} | Proc {mean*1000:.1f}ms | Headroom {1/mean-cfg.TRACKING_FPS:+.1f}Hz | Skip {self.skipped}")
                self.work = self.count = self.skipped = 0
                self.report_time = now
        elif self.reader.ended:
            raise RuntimeError(f"Camera stream ended: {self.reader.error or 'EOF'}")
        if time.monotonic()-self.last_frame_time > cfg.CAMERA_STALE_SECONDS:
            raise RuntimeError("Camera frame timeout")
        key = cv2.waitKey(1) & 0xff
        if key in (ord('q'), 27) or cv2.getWindowProperty(cfg.CAMERA_WINDOW, cv2.WND_PROP_VISIBLE) < 1:
            raise KeyboardInterrupt
        if key == ord('r'):
            if self.flying:
                raise KeyboardInterrupt
            self.reset()
        return key, item is not None

    def close(self):
        self.reader.stop.set()
        if self.process.poll() is None:
            self.process.terminate()
        try:
            self.process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait()
        self.reader.thread.join(timeout=2)
        self.process.stdout.close()
        cv2.destroyAllWindows()
