import cv2
import depthai as dai

with dai.Pipeline() as pipeline:
    cam = pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)

    out = cam.requestOutput((640, 400), type=dai.ImgFrame.Type.GRAY8, fps=30)
    q = out.createOutputQueue()

    pipeline.start()

    while True:
        msg = q.get()
        frame = msg.getCvFrame()

        cv2.imshow("camera", frame)
        if cv2.waitKey(1) == ord("q"):
            break

cv2.destroyAllWindows()
