#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import time
x_list = [0.]
z_list = [0.]
list_size= 200

def callback(msg):
	#print('f1')
	global x_list, z_list, tw
	if len(x_list)>=list_size:
		x_list.pop(0)
		z_list.pop(0)
	
	x_list.append(msg.linear.x)
	z_list.append(msg.angular.z)

	tw.linear.x = sum(x_list)/len(x_list)
	tw.angular.z = sum(z_list)/len(z_list)

	pub.publish(tw)
	
	
def function2():
	#print('f2')
	global x_list, z_list, tw
	print(x_list)
	if len(x_list)>=list_size:
		x_list.pop(0)
		z_list.pop(0)

	x_list.append(x_list[-1])
	z_list.append(z_list[-1])

	tw.linear.x = sum(x_list)/len(x_list)
	tw.angular.z = sum(z_list)/len(z_list)

	pub.publish(tw)



rospy.init_node('vel_smoother')
pub = rospy.Publisher('cmd_vel1', Twist, queue_size=10)

tw = Twist()
tw.linear.y=0
tw.linear.z=0
tw.angular.x=0
tw.angular.y=0
rospy.Time
while not rospy.is_shutdown():
	function2()
	rospy.Subscriber("cmd_vel", Twist, callback)
	time.sleep(.05)
