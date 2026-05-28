import cv2
import depthai as dai
from ultralytics import YOLO

model = YOLO("yolo26m.pt")

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)

    out = cam.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=30)
    q = out.createOutputQueue()

    pipeline.start()

    while True:
        msg = q.get()
        frame = msg.getCvFrame()

        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_GRAY2BGR)
        results = model(frame_bgr, verbose=False)[0]

        for box in results.boxes:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            conf = float(box.conf[0])

            cv2.rectangle(frame_bgr, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                frame_bgr,
                f"{label} {conf:.2f}",
                (x1, y1 - 5),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2,
            )

        cv2.imshow("detect", frame_bgr)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
