# ROS 2 Subscriber with real time connection in DataLayer
A ROS2 Humble snap for CtrlX OS containing a demo example with a ROS2 subscriber node that connects in real time a ROS topic with the DataLayer.

## Authorship
Author: Raul Cruz-Oliver

Date and place: February 2024, Switzerland

Contact: raul.cruz.oliver@gmail.com

## Decription
This project assumes that the ros2-base-humble snap is installed in your ctrlX OS. Information to build such snap can be found in [this respository](../../base-humble/).

This snap provides a ROS2 node called "rt_dl_subscriber". This node subscribes to a ROS2 topic called "/test_vector3", that is available in the same LAN. This topic contains a vector3 standard message.

The node "rt_dl_subscriber" writes the information from the topic in the Datalayer using a shared memory approach. This assures that the information is updated in a real time fashion.

## How to build the snap?
To build this snap you must have ros2-humble and ctrlx-datalayer packages installed in your system.

If this requirements are met, to build the snap simply call

```bash
./build-snap.sh
```

## How to test the snap?
Install the snap in your ctrlX OS, do not forget to allow installation from unknwon sources since this snap is not signed.

If the snap has succesfully started you will see a new node "ros2/rt/data" in the data layer. It must have been initialized with 0s.

Now connect the ctrlX CORE in the same LAN as an ubuntu machine with ros-humble installed. By calling 

```bash
./run-desktop-talker.sh
```
a talker node publishing dummy values in the topic /test_vector3 will start.

Once messages start to be published, the node in the DataLayer should start displaying the information contained in the topic. 






