# SNAIL

# SNAIL -> general_description -> launch -> description.launch
Launches RVIZ, you can configure which robot here, there are 3 (segway, husky, pioneer).
# SNAIL -> general_description -> config -> segbot_config.urdf.xacro
You can change the x,y,z position of the LIDAR, Camera and dimensions etc.

# segwayrmp -> rmp_base -> launch -> rmp440le.launch
Starts the segway, puts it into tractor mode; I have also included launching the RVIZ with this which you can remove.
# segwayrmp -> rmp_base -> src -> Rmp440LE.cpp
Main code file for the segway, containing all the published topics.
This contains the IP Address which is needed to connect the segway through ethernet (192.168.1.40).
You can specify the maximum velocity here (114-115)
Odometry (starts at 373).
velocity command topic can be manipulated here currently set to /cmd_vel1 (110)

# segwayrmp -> rmp_teleop -> launch -> joystick.launch
Launches the joystick control, make sure joystick is connected.
# segwayrmp -> rmp_teleop -> src -> rmp_teleop_node.cpp
Main joystick node.
You can set the velocity of rmp here (117-120)
I removed deadman switch from this code, you can uncomment the code out if you want.
velocity command topic can be manipulated here currently set to /cmd_vel which is fed to the velocity smoother (113)
# segwayrmp -> rmp_teleop -> src -> RmpXboxConverter.cpp
When you launch the teleop file, sometimes an error will kill the node depending on the type of controller (number of axes and buttons); to fix this, change the WIRELESS_AXES_SIZE and WIRELESS_BUTTONS_SIZE (47-48) accordingly (the error in the command prompt will tell you how many buttons and axes are available)

# cmd_vel_smoother -> launch -> cmd_vel_smoother.launch
remaps the velocity from /cmd_vel (joystick or navstack) to /cmd_vel1 (segway)
you can set change the parameters for the smoother here

# vehicle_navigation -> launch -> amcl_demo.launch
launches amcl localization of the vehice
change the map file to a predefined map (gmapping, rtabmap?) (line 7)
# vehicle_navigation -> launch -> move_base.launch
launches the move_base for autonomous navigation
parameters can be adusted in the config file 

# velodyne -> velodyne_pointcloud -> launch -> VLP16_points.launch
please download the file from velodyne GITHUB since it is too big ot upload here
launches the velodyne LIDAR
change the frame id to front_lidar (line 9) to account for lidar position <arg name="frame_id" default="front_lidar" /> 

# Correct Way to start autonomous navigation given a predefined map
1) roslaunch rmp_base rmp440le.launch
2) roslaunch velodyne_pointcloud VLP16_points.launch
3) roslaunch cmd_vel_smoother cmd_vel_smoother.launch
4) roslaunch vehicle_navigation amcl_demo.launch
5) roslaunch move_base.launch

# p.s do not forget to download navigation stack
