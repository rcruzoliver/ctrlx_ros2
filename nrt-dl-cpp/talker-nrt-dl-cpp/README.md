# Simple ROS 2 Publisher in C++, with NRT reading from datalayer.

A simple ROS 2 node that publishes the topic `ros2_simple_cpp` with information read in a non-real time fashion from the datalayer node `framework/metrics/system/cpu-utilisation-percent`.


## Basis for this Project

This project is based on the official ROS 2 Tutorial: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c).


## Build
To build this snap you simply need to call:

```bash
./build-snap.sh
```