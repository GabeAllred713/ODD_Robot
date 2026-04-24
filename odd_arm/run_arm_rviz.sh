#!/bin/bash
source /opt/ros/humble/setup.bash
source ros_workspace/install/local_setup.bash

ros2 launch arm_robot_moveit_config demo.launch.py --noninteractive & wait
