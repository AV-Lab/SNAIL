#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import sys


def video_callback(ros_image):
  bridge = CvBridge()
  #convert ros_image into an opencv-compatible image
  try:
    cv_image = bridge.imgmsg_to_cv2(ros_image,"rgb8")
  except CvBridgeError as e:
      print(e)
  #from now on, you can work exactly like with opencv
  cv2.imshow("Image window", cv_image)
  cv2.waitKey(1)

  
def main(args):
  rospy.init_node('video_sub', anonymous=True)
  image_sub = rospy.Subscriber("/video_stream",Image, video_callback)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)