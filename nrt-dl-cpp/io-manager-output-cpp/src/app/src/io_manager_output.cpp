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
#include "std_msgs/msg/bool.hpp"

using std::placeholders::_1;
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

  void setBool(const bool& value)
  { 
    m_data.setValue(value);

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


class OutputSubscriber : public rclcpp::Node
{

public:
  comm::datalayer::DatalayerSystem datalayerSystem;
  comm::datalayer::IProvider* provider;
  comm::datalayer::Variant myData;
  MyProviderNode *pippo = new MyProviderNode(myData);
  
  // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE

  OutputSubscriber(): Node("output_subscriber")

  // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE
  
  { 
    subscription_ = this->create_subscription<std_msgs::msg::Bool>("io_output", 10, std::bind(&OutputSubscriber::topic_callback, this, _1));
    datalayerSystem.start(false);
    provider = getProvider(datalayerSystem); // ctrlX CORE (virtual)
    
    comm::datalayer::DlResult result = provider->registerNode("fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/data/XI211208/Channel_2.Value",pippo);
    if (STATUS_FAILED(result))
    {
      std::cout << "WARN Register node 'fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/data/XI211208/Channel_2.Value' failed with: " << result.toString() << std::endl;
    }
  }

private:
  void topic_callback(const std_msgs::msg::Bool & msg) const
  {
    bool output_;
    output_ = msg.data;
    pippo->setBool(bool(output_));
  }
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{ rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<OutputSubscriber>());
  rclcpp::shutdown();
  return 0;
  
  
}
