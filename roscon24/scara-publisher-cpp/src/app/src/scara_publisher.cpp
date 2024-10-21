// Copyright 2016 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
#include <chrono>
#include <functional>
#include <memory>
#include <string>


#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "sensor_msgs/msg/joint_state.hpp"


using namespace std::chrono_literals;

#include <filesystem>
#include <csignal>
#include <thread>

#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"


// Add some signal Handling so we are able to abort the program with sending sigint
static bool g_endProcess = false;

static void signalHandler(int signal)
{
  std::cout << "signal: " << signal << std::endl;
  g_endProcess = true;
  // Clean up datalayer instances so that process ends properly
  // Attention: Doesn't return if any provider or client instance is still runnning

}
//! Retrieve environment variable SNAP
//! @result The content of SNAP ales nullptr if not available
static const char* snapPath()
{
  return std::getenv("SNAP");
}

//! Test if code is runnning in snap environment
//! @result True if running snap environment
static bool isSnap()
{
  return snapPath() != nullptr;
}

//! Get Datalayer connection string
//! @param[in] ip       IP address of the ctrlX CORE: 10.0.2.2 is ctrlX COREvirtual with port forwarding
//! @param[in] user     User name
//! @param[in] password The password
//! @param[in] sslPort  The port number for SSL: 8443 if ctrlX COREvirtual with port forwarding 8443:443
//! @result Connection string
static std::string getConnectionString(
  const std::string& ip = "10.0.2.2",
  const std::string& user = "boschrexroth",
  const std::string& password = "boschrexroth",
  int sslPort = 8443)
{
  if (isSnap())
  {
    return DL_IPC;
  }

  std::string connectionString = DL_TCP + user + std::string(":") + password + std::string("@") + ip;

  if (443 == sslPort)
  {
    return connectionString;
  }

  return connectionString + std::string("?sslport=") + std::to_string(sslPort);
}

class ScaraPublisher : public rclcpp::Node
{
public:
  ScaraPublisher() : Node("scara_publisher")
  {
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("scara_joints", 10);
    joint_name_ = {"c1", "c2", "z", "c3"};
  }

  void node_publish(const std::vector<double>& joint_position_)

  {
    auto msg = std::make_unique<sensor_msgs::msg::JointState>();
    msg->name = joint_name_;
    msg->position = joint_position_;
    publisher_->publish(std::move(msg));
  }

  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr publisher_;
  size_t count_;
  std::vector<std::string> joint_name_;
};

int main(int argc, char * argv[])
{
   // Initialize ROS 2
  rclcpp::init(argc, argv);
  ScaraPublisher scara_publisher;

  std::cout << "INFO Starting ctrlX Data Layer system (without broker)" << std::endl;
  comm::datalayer::DatalayerSystem datalayerSystem;
  datalayerSystem.start(false);

  auto connectionString = getConnectionString(); // default: ctrlX CORE or ctrlX COREvirtual with Network Adpater
  std::cout << "INFO Creating ctrlX Data Layer client connection to " << connectionString << " ..." << std::endl;
  auto dataLayerClient = datalayerSystem.factory()->createClient(connectionString);

  int counter = 1;
  while (dataLayerClient->isConnected())
  {
    // std::cout << "Loop #" << counter++ << std::endl;

    auto c1_address = "motion/axs/Base/state/values/actual/pos";
    auto c2_address = "motion/axs/Middle/state/values/actual/pos";
    auto z_address = "motion/axs/ZAxis/state/values/actual/pos";
    auto c3_address = "motion/axs/Flange/state/values/actual/pos";
    
    comm::datalayer::Variant c1_value;
    comm::datalayer::Variant c2_value;
    comm::datalayer::Variant z_value;
    comm::datalayer::Variant c3_value;

    auto result_c1 = dataLayerClient->readSync(c1_address, &c1_value);
    auto result_c2 = dataLayerClient->readSync(c2_address, &c2_value);
    auto result_z = dataLayerClient->readSync(z_address, &z_value);
    auto result_c3 = dataLayerClient->readSync(c3_address, &c3_value);


    if ((result_c1 != DL_OK) || (result_c2 != DL_OK) || (result_z != DL_OK) || (result_c3 != DL_OK)) {
      std::cout <<"WARN Reading failed " << std::endl;
    } 
    else
    {
      if (c1_value.getType() == comm::datalayer::VariantType::FLOAT64)
      {
        std::vector<double> joint_pos_;
        double grad2rad = 0.01745329251;
        double mm2m = 0.001;
        joint_pos_= {grad2rad*double(c1_value), grad2rad*double(c2_value), mm2m*double(z_value), grad2rad*double(c3_value)};
        scara_publisher.node_publish(joint_pos_);
      }
      else
      {
        std::cout << "WARN Value has unexpected type" << std::endl;
      }
    }

    // std::cout << "INFO Sleeping..." << std::endl;
    sleep(0.03);
  }

  std::cout << "ERROR ctrlX Data Layer connection is broken" << std::endl;

  delete dataLayerClient;
  datalayerSystem.stop();

  return 1; // We exit because an error happend
}