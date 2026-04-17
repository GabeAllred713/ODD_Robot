#!/bin/bash

echo Launching nanoowl + cameras

source /opt/ros/humble/setup.bash
source ros_workspace/install/local_setup.bash
ros2 launch odd_arm_cv sense_launch.py --noninteractive & jetson-containers run --name odd_arm_nanoowl --ipc=host --pid=host odd_arm/nanoowl:1.0.1 bash /opt/odd_arm_cv/nanoowl.sh & wait
