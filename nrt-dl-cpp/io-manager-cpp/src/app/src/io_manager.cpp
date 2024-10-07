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
#include "std_msgs/msg/bool.hpp"

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

class InputPublisher : public rclcpp::Node
{
public:
  InputPublisher() : Node("input_publisher")
  {
    publisher_ = this->create_publisher<std_msgs::msg::Bool>("io_input", 10);
  }

  void node_publish(const bool& input)

  {
    auto msg = std::make_unique<std_msgs::msg::Bool>();
    msg->data = input;
    publisher_->publish(std::move(msg));
  }

  // rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
  size_t count_;
};

int main(int argc, char * argv[])
{
   // Initialize ROS 2
  rclcpp::init(argc, argv);
  InputPublisher input_publisher;

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

    auto input_address = "fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input/data/XI110208/Channel_2.Value";
    comm::datalayer::Variant input_value;

    auto result_input = dataLayerClient->readSync(input_address, &input_value);

    if (result_input != DL_OK) {
      std::cout <<"WARN Reading failed " << std::endl;
    } 
    else
    {
      if (input_value.getType() == comm::datalayer::VariantType::BOOL8)
      {
        bool input_;
        input_ = bool(input_value);
        input_publisher.node_publish(input_);
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