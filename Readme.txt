Note: Please add follwoing repositories 'include/package_kansari' before running the program, eg: <src/package_kansari/include/package_kansari> 


#﻿Commands to run the code:


To launch robot in RViz: 

ros2 launch panda_kashif demo.launch.py


To run the pick and place command:

ros2 run package_kansari pick_and_place


----------------------------------------------------------------



List of Libraries used:


<memory>: For managing dynamic memory allocation and smart pointers.


<rclcpp/rclcpp.hpp>: For ROS 2 C++ client library functionality.


<moveit/move_group_interface/move_group_interface.h>: For interfacing with MoveIt's motion planning capabilities.


<geometry_msgs/msg/pose.hpp>: For handling pose and quaternion message types in ROS 2.


<geometry_msgs/msg/quaternion.hpp>: For handling pose and quaternion message types in ROS 2.
