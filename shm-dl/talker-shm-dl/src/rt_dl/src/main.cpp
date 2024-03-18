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

#include <iostream>
#include <filesystem>
#include <csignal>
#include <thread>

#include <functional>
#include <memory>
#include "ctrlx_datalayer_helper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/vector3.hpp"

using std::placeholders::_1;
#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"
#include "comm/datalayer/memory_map_generated.h"



#define MEM_SIZE (24) // 24 bytes = 192 bit = 3 * float64
 // Revision should be unique for this Layout, if you need a new memory layout define a new revision, or use checksum algorithms
#define REVISION (0)

// Add some signal Handling so we are able to abort the program with sending sigint
static bool g_endProcess = false;

static void signalHandler(int signal)
{
  std::cout << "signal: " << signal << std::endl;
  g_endProcess = true;
  // Clean up datalayer instances so that process ends properly
  // Attention: Doesn't return if any provider or client instance is still runnning

}

static void closeUserMemory(comm::datalayer::DatalayerSystem* datalayer,
                            std::shared_ptr<comm::datalayer::IMemoryUser> userMemory)
{
  if (userMemory == nullptr)
  {
    return;
  }

  std::cout << "INFO Closing realtime user memory" << std::endl;

  comm::datalayer::DlResult result = datalayer->factory()->closeMemory(userMemory);
  if (comm::datalayer::STATUS_FAILED(result))
  {
    std::cout << "WARNING Closing realtime user memory failed with: " << result.toString() << std::endl;
  }
}

static void deleteOwnerMemory(comm::datalayer::DatalayerSystem* datalayer,
                              std::shared_ptr<comm::datalayer::IMemoryOwner> ownerMemory)
{
  if (ownerMemory == nullptr)
  {
    return;
  }

  std::cout << "INFO Deleting realtime owner memory" << std::endl;
  comm::datalayer::DlResult result = datalayer->factory()->deleteMemorySync(ownerMemory);
  if (comm::datalayer::STATUS_FAILED(result))
  {
    std::cout << "WARNING Closing realtime owner memory failed with: " << result.toString() << std::endl;
  }
}

// Cleanup closes the memory and stop the datalayersystem
static void cleanup(comm::datalayer::DatalayerSystem* datalayer,
                    comm::datalayer::IProvider* provider,
                    std::shared_ptr<comm::datalayer::IMemoryOwner> input,
                    std::shared_ptr<comm::datalayer::IMemoryOwner> output)
{
  deleteOwnerMemory(datalayer, input);
  deleteOwnerMemory(datalayer, output);

  if (provider != nullptr)
  {
    provider->stop();
    delete provider;
  }

  datalayer->stop();
}

class RTDLPublisher : public rclcpp::Node
{

public:
  comm::datalayer::DatalayerSystem datalayerSystem;
  std::shared_ptr<comm::datalayer::IMemoryUser> input;
  comm::datalayer::Variant tmp_memdata_;
  const comm::datalayer::MemoryMap* memMap_;
  uint32_t revision_;

  
  RTDLPublisher() : Node("rt_dl_publisher")
  { 
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("PLC_output", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&RTDLPublisher::timer_callback, this));

    // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE  
    datalayerSystem.start(false); 

    // Open the memory that will be read
    result_ = datalayerSystem.factory()->openMemory(input, "plc/app/Application/realtime_data/PLC_output");
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Opening of plc/app/Application/realtime_data/PLC_output failed with: " << result_.toString() << std::endl;
      closeUserMemory(&datalayerSystem, input);
      // return 1;
    }

    // Get the memory map
    while (true)
    {
      result_ = input->getMemoryMap(tmp_memdata_);
      std::cout << "INFO Try to get memory map of plc/app/Application/realtime_data/PLC_output: " << result_.toString() << std::endl;

      if (comm::datalayer::STATUS_SUCCEEDED(result_))
      {
        break;
      }

      std::this_thread::sleep_for(std::chrono::milliseconds(500));

    };

    memMap_ = comm::datalayer::GetMemoryMap(tmp_memdata_.getData());
    revision_ = memMap_->revision();    

    std::cout << "INFO App ready to read the datalayer" << std::endl;
  }

private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  mutable comm::datalayer::DlResult result_;
  double* x_data_;
  

  void timer_callback()
  {
    auto msg = std::make_unique<geometry_msgs::msg::Vector3>();

    uint8_t* DL_data;

    double* x_buffer_;
    x_buffer_ = (double *)malloc(sizeof(double));

    double* y_buffer_;
    y_buffer_ = (double *)malloc(sizeof(double));

    double* z_buffer_;
    z_buffer_ = (double *)malloc(sizeof(double));


    result_ = input->beginAccess(DL_data, revision_);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "WARNING Accessing input memory failed with: " << result_.toString() << std::endl;
      // continue;
    }

    memcpy(x_buffer_, DL_data, sizeof(double));
    memcpy(y_buffer_, DL_data + sizeof(double), sizeof(double));
    memcpy(z_buffer_, DL_data + 2*sizeof(double), sizeof(double));

    msg->x = *x_buffer_;
    msg->y = *y_buffer_;
    msg->z = *z_buffer_;
    publisher_->publish(*msg);

    input->endAccess();
  }

};

int main(int argc, char * argv[])
{ rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTDLPublisher>());
  rclcpp::shutdown();
  //closeUserMemory(&datalayerSystem, input);
  return 0;
}
