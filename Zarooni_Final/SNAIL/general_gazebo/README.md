## General Gazebo

This package contains the launch files and worlds to simulate the general 4 wheel skid steer robot in gazebo.

To simulate in gazebo,<br>
```roslaunch general_gazebo robot_empty_world.launch```

Parameters:
- robot_name : Name of the robot (default: husky)
- world_name : World of the gazebo to simulate (default : worlds/empty.world)

This launch file adds the robot description to the parameter server and also loads gazebo_ros control plugins namely general_joint_state_controller and general_velocity_controller

Some of the topics of interest:
- ```/robot_name/general_velocity_controller/cmd_vel``` : Input twist to the robot
- ```/robot_name/general_velocity_controller/odom``` : Wheel odometry of the robot (Change pose and twist covariance accordingly in gazebo_control config file)
- ```/robot_name/joint_states``` - Joint values of the robot
- ```/robot_name/imu``` - Imu message having information regarding orientation and linear acceleration
- ```/robot_name/navsat/fix``` - GPS information regarding latitude, longitude and altitude
- ```/robot_name/scan``` - Laser information
- ```/robot_name/realsense/color/image_raw``` - Raw image from the realsense camera
