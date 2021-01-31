
#!/usr/bin/env python3
# import rospy
import cv2
import numpy as np
from goprocam import GoProCamera
from goprocam import constants

gpCam = GoProCamera.GoPro()
cap = cv2.VideoCapture("udp://127.0.0.1:10000")
while True:
    ret, frame = cap.read()
    cv2.imshow("GoPro OpenCV", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
cap.release()
cv2.destroyAllWindows()
