#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>

float linx, angZ;

void filterVelocityCallback(const geometry_msgs::Twist& msg){
   //Using the callback function just for subscribing  
   //Subscribing the message and storing it in 'linx' and 'angZ'
   linx = msg.linear.x;      angZ = msg.angular.z;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "filter_velocity");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("cmd_vel",1000,&filterVelocityCallback);
  ros::Publisher pub = nh.advertise<geometry_msgs::TwistStamped>("/rmp440le/base/vel_cmd", 1000);
  ros::Rate rate(100);

  while(ros::ok()){
     geometry_msgs::TwistStamped msg;
     msg.twist.linear.x = linx;     msg.twist.angular.z = angZ;

     //It would be better to apply the conditions within the main function and use the 
     //Callback function just for subscribing
    pub.publish(msg);   //This line is for publishing. It publishes to 'turtle1/pose'
      
     rate.sleep();
     ros::spinOnce();      //Notice this
  } 
}