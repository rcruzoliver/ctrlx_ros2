# Simple ROS 2 Subscriber in C++, with NRT writing to datalayer.

A simple ROS 2 node which subscribes to messages of the topic `ros2_simple_cpp` and writes them to the datalayer in a newly created datalayer node under the name `ros2/listenercpp/mymessage`. For an example in which some information contained in a topic is written to an already existing datalayer node, please have a look at the [example](../../roscon24/io-manager-output-cpp/) developed for the ROSCon 24. 

## Basis for this Project

This project is based on the official ROS 2 Tutorial: [Writing a simple publisher and subscriber (C++)](https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#writing-a-simple-publisher-and-subscriber-c).


## Build
To build this snap you simply need to call:

```bash
./build-snap.sh
```