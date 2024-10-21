
#include <iostream>
#include <filesystem>
#include <csignal>
#include <thread>

#include <functional>
#include <memory>
#include "ctrlx_datalayer_helper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"


class OutputSubscriber : public rclcpp::Node
{

public:
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;

  comm::datalayer::DatalayerSystem datalayerSystem;

  comm::datalayer::Variant myData;
  comm::datalayer::IClient* dataLayerClient;
  
  OutputSubscriber(): Node("output_subscriber")
  { 
    // Define the subcriber
    subscription_ = this->create_subscription<std_msgs::msg::Bool>("io_output", 10, std::bind(&OutputSubscriber::topic_callback, this, _1));

    // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE
    datalayerSystem.start(false);
    dataLayerClient = datalayerSystem.factory()->createClient(DL_IPC);
  }

private:

  void topic_callback(const std_msgs::msg::Bool & msg)
  {
    myData.setValue(bool(msg.data));
    dataLayerClient->writeSync("fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/data/XI211208/Channel_2.Value", &myData);
  }

};

int main(int argc, char * argv[])
{ rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputSubscriber>());
  rclcpp::shutdown();
  return 0; 
}
