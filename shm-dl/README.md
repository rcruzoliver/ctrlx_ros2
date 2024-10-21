# Shared memory connection with Datalayer - collection

## Developer information Information

**Autor:** Raul Cruz Oliver (raul.cruz.oliver@gmail.com)

**Date and place:** October 2024, Buttikon, CH


## Description
This collection contains the following snaps:

### listener-shm-dl
This snap contains a ROS2 node that subscribes to a topic and writes the received information to the datalayer using a shared memory mechanims.

### talker-shm-dl
This snap contains a ROS2 node that publishes a topic with information read using shared memory from the datalayer.

### RO2_rt - PLC project
On top of the two snaps, this folder also contains a sample ctrlX PLC project (ROS2_rt.zip) to generate and read the information expected in the datalayer.

## License
Snap collection belonging to ctrlx_ros2 repository, ergo protected under license MIT.