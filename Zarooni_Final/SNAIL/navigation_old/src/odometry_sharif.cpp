#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <iostream>
#include <geometry_msgs/TwistStamped.h>

float x_,y_,z_,w_;

void filterVelocityCallback(const geometry_msgs::Twist& msg){
   //Using the callback function just for subscribing  
   //Subscribing the message and storing it in 'linx' and 'angZ'
   x_ = msg.orienation.x;      y_ = msg.orienation.y; z_ = msg.orienation.z; w_ = msg.orienation.w;
}

int main(int argc, char **argv){
  ros::init(argc, argv, "filter_velocity");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe("imu",1000,&filterVelocityCallback);
  ros::Rate rate(100);

    static tf2_ros::TransformBroadcaster br;
    geometry_msgs::TransformStamped transformStamped;

    transformStamped.header.stamp = ros::Time::now();
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = "base_link";
    transformStamped.transform.translation.x = 0;
    transformStamped.transform.translation.y = 0;
    transformStamped.transform.translation.z = 0;

    transformStamped.transform.rotation.x = x_;
    transformStamped.transform.rotation.y = y_;
    transformStamped.transform.rotation.z = z_;
    transformStamped.transform.rotation.w = w_;
    br.sendTransform(transformStamped);
}