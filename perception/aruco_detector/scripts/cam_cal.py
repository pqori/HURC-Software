#!/usr/bin/env python3
import cv2, time

cv2.namedWindow("Image Feed")
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

prev = time.time()
cal_idx = 0
frame_idx = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame_idx += 1
    if frame_idx == 30:
        cv2.imwrite(f"cal_image_{cal_idx}.jpg", frame)
        cal_idx += 1
        frame_idx = 0

    fps = 1.0 / (time.time() - prev)
    prev = time.time()
    cv2.putText(frame, f"FPS: {fps:0.2f}", (10,40),
                cv2.FONT_HERSHEY_PLAIN, 1, (0,255,0), 2)
    cv2.imshow("Image Feed", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
