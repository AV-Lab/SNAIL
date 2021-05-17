#!/usr/bin/env python
import rospy
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import TwistStamped
import math

roll = pitch = yaw = 0.0
target_angle = 200
kP = 0.5

def get_rotation (msg):
    global roll, pitch, yaw
    orientation_q = msg.pose.pose.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion (orientation_list)
    print yaw

rospy.init_node('my_quaternion_to_euler')

sub = rospy.Subscriber ('/rmp440le/odom', Odometry, get_rotation)
pub = rospy.Publisher('/rmp440le/base/vel_cmd', TwistStamped, queue_size=1)
command= TwistStamped()
r = rospy.Rate(50)
while not rospy.is_shutdown():
    target_rad = target_angle * math.pi/180
    command.twist.angular.z = kP  * (target_rad - yaw)
    pub.publish(command)
    r.sleep()