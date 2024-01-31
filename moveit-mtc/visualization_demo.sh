#!/bin/bash
source /opt/ros/humble/setup.bash
source ./install/local_setup.bash

exec ros2 launch mtc_demo visualization_demo.launch.py


