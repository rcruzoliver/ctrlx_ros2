# SCARA state publisher - ROSCON24

A ROS 2 node (`scara_publisher`) packaged in a snap for ctrlX. 

It publishes the topic `scara_joints` with information read in the datalayer addresses `motion/axs/<??>/state/values/actual/pos`. The information in such DL address corresponds to the actual values of the SCARA robot comissioned through the motion app.

The published topic is subscribed by an application running in a external device (connected in the same network as the CORE), such application runs a Rviz visualization that displays the SCARA robot accordingly.


