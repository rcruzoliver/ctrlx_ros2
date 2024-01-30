#!/bin/bash
source $SNAP/usr/bin/setup-paths.sh
exec $ROS_BASE/opt/ros/humble/bin/ros2 launch mtc_demo initALL_demo.launch.py

