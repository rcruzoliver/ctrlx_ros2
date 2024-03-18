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

static comm::datalayer::Variant createMemMap(uint32_t revision)
{
  // A memory map defines the layout of memory
  // Memory Map contains:
  // {
  //    variables : [Variables]; Array of variables
  //    revision : uint = 0; Revisionnumber changes every time on variables changes
  // }
  // Variables defined like:
  // {
  //   name : string (key); Name of the variable
  //   bitoffset : uint;    Offset (in bits) of variable in memory
  //   bitsize : uint;      Size (in bits) of variable in memory
  //   type : string;       Type information
  // }
  // It defines where all variables lays in memory, the type and valid informations
  // Here is a simple layout
  flatbuffers::FlatBufferBuilder builder;
  std::vector<flatbuffers::Offset<comm::datalayer::Variable>> vecVariables;

  auto variable = comm::datalayer::CreateVariableDirect(builder,
                                                        "ROS_input",   // name of variable (has to be unique), can be divided by "/" for hierarchical structure
                                                        0, // bit offset of variable in memory
                                                        3*64,     // size of variable in bits
                                                        comm::datalayer::TYPE_DL_ARRAY_OF_FLOAT64.c_str());                                                
  vecVariables.push_back(variable);

  auto variables = builder.CreateVectorOfSortedTables(&vecVariables);
  comm::datalayer::MemoryMapBuilder memmap(builder);
  memmap.add_revision(revision);
  memmap.add_variables(variables);
  auto memmapFinished = memmap.Finish();
  builder.Finish(memmapFinished);

  comm::datalayer::Variant result;
  result.copyFlatbuffers(builder);
  return result;
}


class RTDLSubscriber : public rclcpp::Node
{

public:
  comm::datalayer::DatalayerSystem datalayerSystem;
  comm::datalayer::IProvider* provider;
  comm::datalayer::Variant myString;
  std::shared_ptr<comm::datalayer::IMemoryOwner> input;
  
  // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE

  RTDLSubscriber() : Node("rt_dl_subscriber")
  { 
    subscription_ = this->create_subscription<geometry_msgs::msg::Vector3>(
      "test_vector3", 10, 
      std::bind(&RTDLSubscriber::topic_callback, this, _1));
      
    datalayerSystem.start(false); // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE
    
    // Get provider instance
    std::cout << "INFO Getting provider instance" << std::endl;

    provider = datalayerSystem.factory()->createProvider(DL_IPC); 
    if (provider == nullptr)
    {
      std::cout << "ERROR Creating ctrlX Data Layer provider connection." << std::endl;
      cleanup(&datalayerSystem, nullptr, nullptr, nullptr);
      // return 1;
    }

    result_ = provider->start();
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Provider could not be started." << std::endl;
      cleanup(&datalayerSystem, provider, nullptr, nullptr);
      // return 1;
    }

    if (provider->isConnected() == false)
    {
      std::cout << "ERROR Provider is NOT connected." << std::endl;
      cleanup(&datalayerSystem, provider, nullptr, nullptr);
      // return 1;
    }

    // Shared memory
    std::cout << "INFO Defining shared memory" << std::endl;
    result_ = datalayerSystem.factory()->createMemorySync(input, "ros2/rt/ROS_input", provider, MEM_SIZE, comm::datalayer::MemoryType_Input);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Creation of ros2/rt/ROS_input failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    // Memory map
    std::cout << "INFO Setting memory map" << std::endl;
    comm::datalayer::Variant memMap = createMemMap(REVISION);
    result_ = input->setMemoryMap(memMap);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Setting input memMap failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    // Input ---------------------------------------
    uint8_t* DL_data = new uint8_t[MEM_SIZE];
    result_ = input->beginAccess(DL_data, REVISION);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Accessing input memory failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    std::cout << "INFO Filling memory with start value 0" << std::endl;
    memset(DL_data, 0, MEM_SIZE);

    input->endAccess();

    std::cout << "INFO App ready to receive topics" << std::endl;
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3>::SharedPtr subscription_;
  mutable comm::datalayer::DlResult result_;

  void topic_callback(const geometry_msgs::msg::Vector3 & msg) const
  {
    uint8_t* DL_data;
    // uint8_t* buffer;

    result_ = input->beginAccess(DL_data, REVISION);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "WARNING Accessing input memory failed with: " << result_.toString() << std::endl;
      // continue;
    }

    uint8_t* buffer = (uint8_t*)malloc(3*sizeof(double)); // it could be done with the "new" keyword

    double x_data = msg.x;
    double y_data = msg.y;
    double z_data = msg.z;

    memcpy(buffer, &(x_data), sizeof(double));
    memcpy(buffer + sizeof(double), &(y_data), sizeof(double));
    memcpy(buffer + 2*sizeof(double), &(z_data), sizeof(double));

    memcpy(DL_data, buffer, MEM_SIZE);

    input->endAccess();
  }

};

int main(int argc, char * argv[])
{ rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTDLSubscriber>());
  rclcpp::shutdown();
  return 0;
}
