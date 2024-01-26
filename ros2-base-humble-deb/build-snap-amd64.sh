#!/bin/bash
rm -rf install/
mkdir -p install/app
rm -rf build/
rm -rf log/

source /opt/ros/humble/setup.bash
colcon build --packages-select myinterface
if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

snapcraft clean --destructive-mode
snapcraft --destructive-mode