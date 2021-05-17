# SNAIL

general_description -> launch -> description.launch
# Launches RVIZ, you can configure which robot here, there are 3 (segway, husky, pioneer).
general_description -> config -> segbot_config.urdf.xacro
# You can change the x,y,z position of the LIDAR, Camera and dimensions etc.

segwayrmp -> rmp_base -> launch -> rmp440le.launch
# Starts the segway, puts it into tractor mode; I have also included launching the RVIZ with this which you can remove.
segwayrmp -> rmp_base -> src -> Rmp440LE.cpp
# Main code file for the segway, containing all the published topics.
# This contains the IP Address which is needed to connect the segway through ethernet (192.168.1.40).
# You can specify the maximum velocity here (104-105).
# Odometry (starts at 360).
# velocity command topic can be manipulated here (100)

segwayrmp -> rmp_teleop -> launch -> joystick.launch
# Launches the joystick control, make sure joystick is connected.
segwayrmp -> rmp_teleop -> src -> rmp_teleop_node.cpp
# Main joystick node.
# You can set the velocity of rmp here (156-157)
# I removed deadman switch from this code, you can uncomment the code out if you want.
# velocity command topic can be manipulated here (150)
segwayrmp -> rmp_teleop -> src -> RmpXboxConverter.cpp
# When you launch the teleop file, sometimes an error will kill the node depending on the type of controller (number of axes and buttons); to fix this, change the WIRELESS_AXES_SIZE and WIRELESS_BUTTONS_SIZE (47-48) accordingly (the error in the command prompt will tell you how many buttons and axes are available)
