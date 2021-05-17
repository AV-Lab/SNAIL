#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time,re,math

x=0.0
y=0.0
yaw=0.0


def posCallback(pose_message):
	global x
	global y, yaw
	x=pose_message.pose.pose.position.x
	y=pose_message.pose.pose.position.y
	yaw=pose_message.pose.pose.orientation.z

def move(velocity_publisher, speed, distance, is_forward):
	velocity_message=Twist()
	global x,y
	x0=x
	y0=y

	if (is_forward):
		velocity_message.linear.x = abs(speed)
	else:
		velocity_message.linear.x = -abs(speed)

	distance_moved=0.0
	loop_rate = rospy.Rate(100)

	while True:
		rospy.loginfo("segway moves forward")
		velocity_publisher.publish(velocity_message)
		loop_rate.sleep()
		distance_moved= distance_moved+abs(0.5*math.sqrt(((x-x0) **2)+((y-y0)**2)))
		print distance_moved
		if (distance_moved>distance):
			rospy.loginfo("reached")
			break

	velocity_message.linear.x=0
	velocity_publisher.publish(velocity_message)

def rotate(velocity_publisher,angular_speed_degree, relative_angle_degree,clockwise):
	global yaw
	velocity_message=Twist()
	theta0=yaw
	angular_speed=math.radians(abs(angular_speed_degree))

	if clockwise == 1:
		velocity_message.angular.z=-abs(angular_speed)
	else:
		velocity_message.angular.z=abs(angular_speed)

	loop_rate=rospy.Rate(10)
	t0=rospy.Time.now().to_sec()

	while True:
		rospy.loginfo("segway rotates")
		velocity_publisher.publish(velocity_message)

		t1=rospy.Time.now().to_sec()
		current_angle_degree=(t1-t0)*angular_speed_degree
		loop_rate.sleep()
		print ("current angle degree = ",current_angle_degree)
		print("current angular speed = ", angular_speed)

		if (current_angle_degree>relative_angle_degree):
			rospy.loginfo("reached")
			break
	velocity_message.angular.z=0
	velocity_publisher.publish(velocity_message)


def go_to_goal(velocity_publisher,x_goal,y_goal):
	global x
	global y, yaw

	loop_rate=rospy.Rate(100)
	velocity_message=Twist()

	while True:
		K_linear=0.3
		distance=abs(math.sqrt(((x_goal-x) **2)+((y_goal-y)**2)))

		linear_speed=distance*K_linear

		# if linear_speed > 8.0:
		# 	linear_speed = 8.0
		# elif linear_speed < -8.0:
		# 	linear_speed = -8.0
		# else:
		# 	linear_speed = linear_speed

		K_angular=4.0
		desired_angle_goal=math.atan2(y_goal-y,x_goal-x)
		angular_speed=(desired_angle_goal-yaw)*K_angular

		# if angular_speed > 4.4:
		# 	angular_speed = 4.4
		# elif angular_speed < -4.4:
		# 	angular_speed = -4.4
		# else:
		# 	angular_speed = angular_speed



		velocity_message.linear.x=linear_speed
		velocity_message.angular.z=angular_speed

		velocity_publisher.publish(velocity_message)

		print("x=",x,', y=',y,', distance to goal: ', distance)
		print ("linear speed = ", linear_speed, ', angular_speed =', angular_speed)

		if (distance <0.1):
			break


def desired_orientation(velocity_publisher, speed_in_degree, desired_angle_radians):
    relative_angle_radians = desired_angle_radians - yaw
    if relative_angle_radians < 0:
        clockwise = 1
    else:
        clockwise = 0
    print relative_angle_radians
    print desired_angle_radians

    rotate(velocity_publisher,speed_in_degree,math.degrees(relative_angle_radians), clockwise)
    print yaw

def spiral (velocity_publisher,wk,rk):
	vel_msg = TwistStamped()
	loop_rate = rospy.Rate(100)

	while ((x<10.5) and (y<10.5)):
		rk=rk+0.01
		vel_msg.twist.linear.x=rk
		vel_msg.twist.angular.z=wk
		velocity_publisher.publish(vel_msg)
		loop_rate.sleep()

	vel_msg.twist.linear.x =0
	vel_msg.twist.angular.z= 0
	velocity_publisher.publish(vel_msg)

def gridClean(velocity_publisher):
	desired_pose = Odometry()
	desired_pose.pose.pose.position.x = 1
	desired_pose.pose.pose.position.y = 1
	desired_pose.pose.pose.orientation.z = 0

	go_to_goal(velocity_publisher,1,1)

	desired_orientation(velocity_publisher,30,math.radians(desired_pose.pose.pose.orientation.z))

	for i in range(5):
		move(velocity_publisher,2.0,1.0,True)
		rotate(velocity_publisher,20,90,False)
		move(velocity_publisher,2.0,9.0,True)
		rotate(velocity_publisher,20,90,True)
		move(velocity_publisher,2.0,1.0,True)
		rotate(velocity_publisher,20,90,True)
		move(velocity_publisher,2.0,9.0,True)
		rotate(velocity_publisher,20,90,False)
	pass

if __name__ =='__main__':
	try:

		rospy.init_node('Segway_motion_position',anonymous=True)


		pose_subscriber = rospy.Subscriber('/segbot/general_velocity_controller/odom',Odometry, posCallback)

		velocity_publisher=rospy.Publisher('/segbot/general_velocity_controller/cmd_vel', Twist, queue_size=50)
		time.sleep(1)

		# move(velocity_publisher,2,200,False)
		# rotate(velocity_publisher,90,180,1)
		go_to_goal(velocity_publisher, input(),input())
		# desired_orientation(velocity_publisher,30,1.0)
		# spiral(velocity_publisher,0,2)
		# gridClean(velocity_publisher)

	except rospy.ROSInterruptException:
		rospy.loginfo("node terminated")