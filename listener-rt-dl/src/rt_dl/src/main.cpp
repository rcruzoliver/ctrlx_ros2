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

using std::placeholders::_1;
#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

#define MEM_SIZE (20) // 20 bytes
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

static comm::datalayer::Variant createMemMap(size_t size, uint32_t revision)
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

  // for (uint32_t i = 0; i < (size / 4); i++)
  // {
  // char name[10];
  //  sprintf(name, "var%u", i);
  //  auto variable = comm::datalayer::CreateVariableDirect(builder,
  //                                                        name,   // name of variable (has to be unique), can be divided by "/" for hierarchical structure
  //                                                        8 * i, // bit offset of variable in memory
  //                                                        8,     // size of variable in bits
  //                                                        comm::datalayer::TYPE_DL_ARRAY_OF_STRING.c_str());
  //  vecVariables.push_back(variable);
  // }


    auto variable = comm::datalayer::CreateVariableDirect(builder,
                                                          "chatter",   // name of variable (has to be unique), can be divided by "/" for hierarchical structure
                                                          0, // bit offset of variable in memory
                                                          8 * size,     // size of variable in bits
                                                          comm::datalayer::TYPE_DL_ARRAY_OF_STRING.c_str());
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


using comm::datalayer::IProviderNode;

// Basic class Provider node interface for providing data to the system
class MyProviderNode: public IProviderNode
{
private:
  comm::datalayer::Variant m_data;

public:
  MyProviderNode(comm::datalayer::Variant data)
    : m_data(data)
  {};
  void setString(const std::string& input)
  { 
    m_data.setValue(input);

  }
  virtual ~MyProviderNode() override {};

  // Create function of an object. Function will be called whenever a object should be created.
  virtual void onCreate(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of a node. Function will be called whenever a node should be read.
  virtual void onRead(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    comm::datalayer::Variant dataRead;
    dataRead = m_data;
    callback(comm::datalayer::DlResult::DL_OK, &dataRead);
  }

  // Write function of a node. Function will be called whenever a node should be written.
  virtual void onWrite(const std::string& address, const comm::datalayer::Variant* data, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    std::cout << "INFO onWrite " <<  address << std::endl;
    
    if (data->getType() != m_data.getType())
    {
      callback(comm::datalayer::DlResult::DL_TYPE_MISMATCH, nullptr);
    }

    m_data = *data;
    callback(comm::datalayer::DlResult::DL_OK, data);
  }

  // Remove function for an object. Function will be called whenever a object should be removed.
  virtual void onRemove(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Browse function of a node. Function will be called to determine children of a node.
  virtual void onBrowse(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }

  // Read function of metadata of an object. Function will be called whenever a node should be written.
  virtual void onMetadata(const std::string& address, const comm::datalayer::IProviderNode::ResponseCallback& callback) override
  {
    // Keep this comment! Can be used as sample creating metadata programmatically.
    // callback(comm::datalayer::DlResult::DL_OK, &_metaData);

    // Take metadata from metadata.mddb
    callback(comm::datalayer::DlResult::DL_FAILED, nullptr);
  }
};


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
    subscription_ = this->create_subscription<std_msgs::msg::String>(
      "chatter", 10, 
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
    result_ = datalayerSystem.factory()->createMemorySync(input, "ros2/rt/data", provider, MEM_SIZE, comm::datalayer::MemoryType_Input);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Creation of ros2/rt/data failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    // Memory map
    std::cout << "INFO Setting memory map" << std::endl;
    comm::datalayer::Variant memMap = createMemMap(MEM_SIZE, REVISION);
    result_ = input->setMemoryMap(memMap);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Setting input memMap failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    // Input ---------------------------------------
    // uint8_t* inData = nullptr;
    // char* inWord = nullptr;
    uint8_t* inData = new uint8_t[MEM_SIZE];
    result_ = input->beginAccess(inData, REVISION);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "ERROR Accessing input memory failed with: " << result_.toString() << std::endl;
      cleanup(&datalayerSystem, provider, input, nullptr);
      // return 1;
    }

    std::cout << "INFO Filling memory with start value 0" << std::endl;
    memset(inData, 0, MEM_SIZE);

    input->endAccess();

    std::cout << "INFO App ready to receive topics" << std::endl;
  }

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  mutable comm::datalayer::DlResult result_;

  void topic_callback(const std_msgs::msg::String & msg) const
  {
    uint8_t* inData;
    uint8_t* outData;
    result_ = input->beginAccess(inData, REVISION);
    if (comm::datalayer::STATUS_FAILED(result_))
    {
      std::cout << "WARNING Accessing input memory failed with: " << result_.toString() << std::endl;
      // continue;
    }

    outData = (uint8_t*)malloc(strlen(msg.data.c_str()) + 1);

    if (outData == NULL) {
        fprintf(stderr, "Memory allocation failed\n");
        // return 1;
    }

    strcpy((char*)outData, msg.data.c_str());

    memcpy(inData, outData, MEM_SIZE);

    input->endAccess();

  }

};

int main(int argc, char * argv[])
{ rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<RTDLSubscriber>());
  rclcpp::shutdown();
  return 0;
}
