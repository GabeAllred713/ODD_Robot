#!/bin/bash
source /opt/ros/humble/setup.bash
source ros_workspace/install/local_setup.bash

ros2 launch arm_package arm.launch.py --noninteractive & wait