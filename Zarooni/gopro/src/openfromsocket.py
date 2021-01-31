#!/usr/bin/env python3
# import rospy
import socket
import cv2
import numpy as np
import time

cap = cv2.VideoCapture("udp://10.5.5.9:10000", cv2.CAP_FFMPEG)
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
last_message = time.time()

while True:

    # Get an image
    ret, img = cap.read()

    # Do something with img
    cv2.imshow("My Window", img)
    cv2.waitKey(1)

cv2.destroyWindow(window_name)
cap.release()
