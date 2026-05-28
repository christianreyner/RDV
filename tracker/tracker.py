import cv2
import depthai as dai
from ultralytics import YOLO
import time

# -----------------------------
# Settings
# -----------------------------
TARGET_CLASSES = {"airplane"}   # only used for automatic re-detection
YOLO_MODEL = "yolo26n.pt"
FRAME_SIZE = (640, 400)
FPS = 30
CONF_THRESHOLD = 0.25

# Padding around selected bbox
BBOX_PAD = 0.18

# Search radius around the clicked point when detector is weak
CLICK_SEARCH_RADIUS = 80

# Minimum bbox size used if detection is very weak around the clicked point
MIN_INIT_SIZE = 24

# -----------------------------
# OpenCV tracker compatibility
# -----------------------------
def create_csrt_tracker():
    if hasattr(cv2, "legacy") and hasattr(cv2.legacy, "TrackerCSRT_create"):
        return cv2.legacy.TrackerCSRT_create()
    if hasattr(cv2, "TrackerCSRT_create"):
        return cv2.TrackerCSRT_create()
    raise RuntimeError(
        "CSRT tracker not available in this OpenCV build. "
        "Install opencv-contrib-python."
    )

# -----------------------------
# Helpers
# -----------------------------
def clamp_bbox(x, y, w, h, img_w, img_h):
    x = max(0, min(x, img_w - 1))
    y = max(0, min(y, img_h - 1))
    w = max(1, min(w, img_w - x))
    h = max(1, min(h, img_h - y))
    return x, y, w, h

def expand_bbox(bbox, pad, img_w, img_h):
    x, y, w, h = bbox
    x -= int(w * pad)
    y -= int(h * pad)
    w += int(2 * w * pad)
    h += int(2 * h * pad)
    return clamp_bbox(x, y, w, h, img_w, img_h)

def find_detections(model, frame_bgr, target_classes, conf_threshold=0.25):
    results = model(frame_bgr, verbose=False)[0]
    dets = []

    for box in results.boxes:
        cls_id = int(box.cls[0])
        label = model.names[cls_id]
        conf = float(box.conf[0])

        if label in target_classes and conf >= conf_threshold:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            dets.append({
                "bbox": (x1, y1, x2 - x1, y2 - y1),
                "label": label,
                "conf": conf
            })

    return dets

def bbox_contains_point(bbox, pt):
    x, y, w, h = bbox
    px, py = pt
    return (x <= px <= x + w) and (y <= py <= y + h)

def pick_detection_from_click(detections, click_pt):
    candidates = []
    for det in detections:
        if bbox_contains_point(det["bbox"], click_pt):
            x, y, w, h = det["bbox"]
            area = w * h
            candidates.append((area, det))

    if not candidates:
        return None

    candidates.sort(key=lambda item: item[0])
    return candidates[0][1]

def make_square_box_around_point(pt, size, img_w, img_h):
    px, py = pt
    half = size // 2
    x = px - half
    y = py - half
    return clamp_bbox(x, y, size, size, img_w, img_h)

def choose_initial_bbox_from_click(frame_bgr, model, click_pt):
    """
    Strategy:
    1) Run detector.
    2) If a detection contains the click point, use it.
    3) Otherwise, use the nearest detection within a search radius.
    4) If still nothing, create a small square centered on the click point.
    """
    detections = find_detections(model, frame_bgr, TARGET_CLASSES, CONF_THRESHOLD)

    # Best case: click falls inside a detection
    chosen = pick_detection_from_click(detections, click_pt)
    if chosen is not None:
        return chosen["bbox"], chosen["label"], chosen["conf"], "detector-hit"

    # If detector misses, choose the nearest detection center within radius
    px, py = click_pt
    nearest = None
    nearest_dist2 = None

    for det in detections:
        x, y, w, h = det["bbox"]
        cx = x + w / 2.0
        cy = y + h / 2.0
        dx = cx - px
        dy = cy - py
        dist2 = dx * dx + dy * dy

        if dist2 <= CLICK_SEARCH_RADIUS * CLICK_SEARCH_RADIUS:
            if nearest is None or dist2 < nearest_dist2:
                nearest = det
                nearest_dist2 = dist2

    if nearest is not None:
        return nearest["bbox"], nearest["label"], nearest["conf"], "detector-near"

    # Fallback: use a small square around the click point
    fb = make_square_box_around_point(
        click_pt,
        MIN_INIT_SIZE,
        frame_bgr.shape[1],
        frame_bgr.shape[0]
    )
    return fb, "manual", None, "fallback-square"

# -----------------------------
# Mouse click
# -----------------------------
click_point = None

def mouse_cb(event, x, y, flags, param):
    global click_point
    if event == cv2.EVENT_LBUTTONDOWN:
        click_point = (x, y)

# -----------------------------
# Main
# -----------------------------
model = YOLO(YOLO_MODEL)

tracker = None
tracking = False
target_label = None
target_conf = None
bbox = None
last_click_pt = None

prev_time = time.perf_counter()
fps_display = 0.0

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)

    out = cam.requestOutput(FRAME_SIZE, type=dai.ImgFrame.Type.GRAY8, fps=FPS)
    q = out.createOutputQueue()

    pipeline.start()

    cv2.namedWindow("Click object point + CSRT track")
    cv2.setMouseCallback("Click object point + CSRT track", mouse_cb)

    while True:
        msg = q.get()
        gray = msg.getCvFrame()
        display = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)

        # FPS
        now = time.perf_counter()
        dt = now - prev_time
        prev_time = now
        if dt > 0:
            fps_display = 1.0 / dt

        # User clicked -> initialize tracking from that point
        if click_point is not None:
            last_click_pt = click_point

            bbox, target_label, target_conf, init_mode = choose_initial_bbox_from_click(
                display, model, click_point
            )

            bbox = expand_bbox(bbox, BBOX_PAD, display.shape[1], display.shape[0])

            tracker = create_csrt_tracker()
            tracker.init(display, bbox)
            tracking = True

            click_point = None

        # If not tracking, wait for a click
        if not tracking:
            cv2.putText(
                display,
                "Click the object point to start tracking",
                (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.8,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                display,
                "Auto re-detect starts after tracking is lost",
                (20, 70),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (200, 200, 200),
                2,
            )

        # Track
        else:
            ok, bbox = tracker.update(display)

            if ok:
                x, y, w, h = map(int, bbox)
                cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)

                cv2.putText(
                    display,
                    f"Tracking {target_label}",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 0),
                    2,
                )

                if last_click_pt is not None:
                    cv2.circle(display, last_click_pt, 4, (0, 255, 255), -1)

            else:
                tracking = False
                tracker = None
                bbox = None
                target_label = None
                target_conf = None

                cv2.putText(
                    display,
                    "Tracking lost. Re-detecting...",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 0, 255),
                    2,
                )

                detections = find_detections(
                    model, display, TARGET_CLASSES, CONF_THRESHOLD
                )

                if detections:
                    detections.sort(key=lambda d: d["conf"], reverse=True)
                    chosen = detections[0]

                    bbox = expand_bbox(
                        chosen["bbox"],
                        BBOX_PAD,
                        display.shape[1],
                        display.shape[0]
                    )

                    tracker = create_csrt_tracker()
                    tracker.init(display, bbox)

                    tracking = True
                    target_label = chosen["label"]
                    target_conf = chosen["conf"]

                    x, y, w, h = bbox
                    cv2.rectangle(display, (x, y), (x + w, y + h), (0, 255, 0), 2)
                    cv2.putText(
                        display,
                        f"Re-locked {target_label} {target_conf:.2f}",
                        (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 0),
                        2,
                    )
                else:
                    cv2.putText(
                        display,
                        "Auto re-detect failed. Click again.",
                        (20, 70),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        (0, 255, 255),
                        2,
                    )

        cv2.putText(
            display,
            f"FPS: {fps_display:.1f}",
            (20, 110),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 255, 255),
            2,
        )

        cv2.imshow("Click object point + CSRT track", display)
        key = cv2.waitKey(1) & 0xFF

        if key == ord("q"):
            break
        elif key == ord("r"):
            tracking = False
            tracker = None
            bbox = None
            target_label = None
            target_conf = None
            click_point = None
            last_click_pt = None

cv2.destroyAllWindows()
