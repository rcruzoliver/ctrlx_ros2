#!/bin/bash

rosdep install -i --from-path src --rosdistro humble -y
if [ $? -eq 0 ]
then
    echo " "
else
    exit 1
fi

rm -rf install/
mkdir -p install/app
rm -rf build/
rm -rf log/

source /opt/ros/humble/setup.bash
colcon build --packages-select cpp_pubsub

source ./install/setup.bash

echo " "
echo "Running desktop talker, publishing /test_vector3 ..."

ros2 run cpp_pubsub talker