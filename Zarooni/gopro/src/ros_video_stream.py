#!/usr/bin/env python

import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

def start_node():
	pub = rospy.Publisher('video_stream', Image, queue_size=10)
	rospy.init_node('video_publish',anonymous=True)
	rospy.loginfo('video_pub node started')
	rate=rospy.Rate(60)
	cap = cv2.VideoCapture(2)
	width = 1280
	height = 720

	# Set the video resolution to HD720
	cap.set(cv2.CAP_PROP_FRAME_WIDTH, width*2)
	cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)

	bridge = CvBridge()
	while not rospy.is_shutdown():
		retval, frame = cap.read()
		pub.publish(bridge.cv2_to_imgmsg(frame,"rgb8"))
		rate.sleep()

if __name__ == '__main__':
    try:
        start_node()
    except rospy.ROSInterruptException:
        pass
