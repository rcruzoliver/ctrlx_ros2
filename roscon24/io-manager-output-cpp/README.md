# IO Manager Output - ROSCON24

A ROS 2 node (`output_subscriber`) packaged in a snap for ctrlX. 

It subscribes to the topic `io_output`, published by a external device connected in the same network as the ctrlX CORE, and writes the content to the datalayer address `fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/data/XI211208/Channel_2.Value`, which directly control the physical output in the hardware.


