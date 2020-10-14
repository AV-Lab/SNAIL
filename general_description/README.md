## General Description

This package contains a general robot description of 4 wheel skid steer robot

To visualize the robot in RViz run<br>
```roslaunch general_description description.launch```

Parameters:
- robot_name : Name of the robot (default: husky)
- enable_rviz : Set to true to visualize the model in RViz (default : false)
- load_fake_joint : GUI to set fake joint values for the joints of the robot

To launch the visualization with husky robot run : <br>
```roslaunch general_description description.launch enable_rviz:=true robot_name:=husky```

To launch the visualization with pioneer3at robot run : <br>
```roslaunch general_description description.launch enable_rviz:=true robot_name:=pioneer3at```

### Steps to add new robot model

- Add meshes of the robot in *meshes/robot_name* withe the following naming convention
  - Chasis mesh : *chasis.dae* 
    - Make sure that the origin of the mesh have no offset i.e the center of the chasis must coincide with the origin
    - All the parts involving extension needs to be added to the same mesh. If seperate link is required then we will have to edit *chasis.urdf.xacro* accordingly
    - The position of the base link frame will be at the base of the chasis
  - Wheel mesh : *left_wheel.dae* , *right_wheel.dae*
    - The mesh should not have an offset i.e the center of the wheel should have its origin.<br>
    (*Note : Blender is a good tool to set the origin of the mesh at its center*)
- Add config file in *config/robot_name_config.urdf.xacro* 
  - All the measurements are in m
  - All the position and orientation must be wrt the center of the chasis
  - Corresponding sensors can be enabled and disabled by changing the value of enable_sensor as true or false
