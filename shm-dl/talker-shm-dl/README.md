# ROS 2 Publisher with real time connection in DataLayer
A ROS2 Humble snap for CtrlX OS containing a demo example with a ROS2 subscriber node that connects in real time a ROS topic with the DataLayer.

## Authorship
Author: Raul Cruz-Oliver

Date and place: February 2024, Switzerland

Contact: raul.cruz.oliver@gmail.com

## Decription
This project assumes that the ros2-base-humble snap is installed in your ctrlX OS. Information to build such snap can be found in [this respository](../../base-humble/). Also, the ctrlX OS in which the ROS2 application is running, must also run the PLC project included [here](../ROS2_rt.zip), this PLC program will write in the datalayer the information the node implemented in this snap expects.

This snap provides a ROS2 node called "rt_dl_publisher". This node publishes a topic called /PLC_output with information the PLC wrote in the datalayer. The node "rt_dl_publisher" reads the information from the DL using a shared memory approach. This assures that the information is updated in a real time fashion.

## How to build the snap?
To build this snap you must have ros2-humble and ctrlx-datalayer packages installed in your system.

If this requirements are met, to build the snap simply call

```bash
./build-snap.sh
```







