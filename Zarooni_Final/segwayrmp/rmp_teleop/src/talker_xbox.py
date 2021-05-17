#!/usr/bin/env python
# license removed for brevity
from utils import *
from system_defines import *
import time,re,math
import rospy
from sensor_msgs.msg import Joy
from rmp_msgs.msg import BoolStamped
from geometry_msgs.msg import Twist
import inputs



def _map(x,in_min,in_max,out_min,out_max):
    return int((x-in_min)*(out_max-out_min)//(in_max-in_min)+out_min)


def talker():
    #create a new publisher. we specify the topic name, then type of message then the queue size
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=50)

    rospy.init_node('velocity', anonymous=True)
    #set the loop rate
    rate = rospy.Rate(50) # 1hz
    #keep publishing until a Ctrl-C is pressed
    
    # print('Press 1 for changing modes, Press 2 for moving the segway')
    # y= int(input())
    # if y == 1:
    #     print('Please enter 1 for Standby mode or 2 for Tractor mode')
    #     x = int(input())
    #     if x == 1:
    #         msgrmp = RmpCmd()
    #         msgrmp.ID = 1281
    #         msgrmp.forward = 32
    #         msgrmp.yawrate =4
    #         pub.publish(msgrmp)
    #         print('Standby mode')
    #     elif x==2:
    #         msgrmp = RmpCmd()
    #         msgrmp.ID = 1281
    #         msgrmp.forward = 32
    #         msgrmp.yawrate =5
    #         pub.publish(msgrmp)
    #         print('Tractor mode')
    # if y==2:
    while True:
        events = inputs.get_gamepad()

        for event in events:
            print(event.code, event.state)
            if event.code=='ABS_Y':
                order = event.state 
                z = _map(order,-32768,32767,0.5,-0.5)
                print(z)
                msgrmp = Twist()
                y = (z)
                msgrmp.twist.linear.x = y
                # pub.publish(msgrmp)
                # rate.sleep()

            if event.code=='ABS_RX':
                order = event.state 
                z = _map(order,-32768,32767,1,-1)
                print(z)
                msgrmp = Twist()
                y = (z)
                msgrmp.twist.angular.z = y
                # pub.publish(msgrmp)
                # rate.sleep()

            if msgrmp>0:
                pub.publish(msgrmp)
                rate.sleep()

            # while not rospy.is_shutdown():
            #     pub.publish(msgrmp)

        #     elif event.code=='ABS_RX':
        #         order = event.state
        #         h = _map(order,-32768,32767,-4.5,4.5)
        #         b = convert_float_to_u32(h);
        #         print(h)
        #         msgrmp = RmpCmd()
        #         msgrmp.ID = 1280
        #         msgrmp.forward = 0
        #         msgrmp.yawrate = b
        #         pub.publish(msgrmp)

        #     elif event.code=='BTN_EAST':
        #         msgrmp = RmpCmd()
        #         msgrmp.ID = 1281
        #         msgrmp.forward = 32
        #         msgrmp.yawrate =5
        #         pub.publish(msgrmp)
        #         print('Tractor mode')

        #     elif event.code=='BTN_SOUTH':
        #         msgrmp = RmpCmd()
        #         msgrmp.ID = 1281
        #         msgrmp.forward = 32
        #         msgrmp.yawrate =4
        #         pub.publish(msgrmp)
        #         print('Standby mode')
        
        if event.code=='BTN_START':
            break



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass