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
#include <chrono>

#include <functional>
#include <memory>
#include "ctrlx_datalayer_helper.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "myinterface/msg/datalayer_digital_io.hpp" 

using std::placeholders::_1;
#include <stdio.h>
#include <iostream>

#include "comm/datalayer/datalayer.h"
#include "comm/datalayer/datalayer_system.h"

using namespace std::chrono_literals;
// Add some signal Handling so we are able to abort the program with sending sigint
static bool g_endProcess = false;
static std::string g_inputbase="fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/input/data/XI110116/";
static std::string g_outputbase="fieldbuses/ethercat/master/instances/ethercatmaster/realtime_data/output/data/XI211116/";
constexpr unsigned int str2int(const char* str, int h = 0)
{
    return !str[h] ? 5381 : (str2int(str, h+1) * 33) ^ str[h];
}

static void signalHandler(int signal)
{
  std::cout << "signal: " << signal << std::endl;
  g_endProcess = true;
  // Clean up datalayer instances so that process ends properly
  // Attention: Doesn't return if any provider or client instance is still runnning

}

comm::datalayer::DatalayerSystem datalayerSystem;

class IoManager : public rclcpp::Node
{

public:
  
  comm::datalayer::IProvider* provider;
  comm::datalayer::IClient* client;
  comm::datalayer::Variant myString;
  MyProviderNode *statusNode = new MyProviderNode(myString);
  MyProviderNode *msgNode = new MyProviderNode(myString);
  MyProviderNode *controlNode = new MyProviderNode(myString);
  MyProviderNode *in1 = new MyProviderNode(myString);
  MyProviderNode *in2 = new MyProviderNode(myString);
  MyProviderNode *in3 = new MyProviderNode(myString);
  MyProviderNode *in4 = new MyProviderNode(myString);
  MyProviderNode *in5 = new MyProviderNode(myString);
  MyProviderNode *in6 = new MyProviderNode(myString);

  MyProviderNode *out1 = new MyProviderNode(myString);
  MyProviderNode *out2 = new MyProviderNode(myString);
  MyProviderNode *out3 = new MyProviderNode(myString);
  MyProviderNode *out4 = new MyProviderNode(myString);
  MyProviderNode *out5 = new MyProviderNode(myString);
  MyProviderNode *out6 = new MyProviderNode(myString);

  

  // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE

  IoManager()
  : Node("iomanager")

  // Starts the ctrlX Data Layer system without a new broker because one broker is already running on ctrlX CORE
  

  { 
    
    

    auto connectionString = getConnectionString();
    provider = getProvider(datalayerSystem); // ctrlX CORE (virtual)
    client=getClient(datalayerSystem);

    statusNode->setStatus(false);
    statusNode->setDynamic(false);
    controlNode->setStatus(false);
    msgNode->setString("Waiting for Start");
    
    std::cout << "INFO Register node 'ros/iomanager/Status'  " << std::endl;
    comm::datalayer::DlResult result = provider->registerNode("ros/iomanager/manager/Status",statusNode);
    comm::datalayer::DlResult result1 = provider->registerNode("ros/iomanager/manager/Control",controlNode);
    comm::datalayer::DlResult result2 = provider->registerNode("ros/iomanager/manager/DiagMessage",msgNode);
    
    in1->setDynamic(false);
    in2->setDynamic(false);
    in3->setDynamic(false);
    in4->setDynamic(false);
    in5->setDynamic(false);
    in6->setDynamic(false);
    in1->setStatus(false);
    in2->setStatus(false);
    in3->setStatus(false);
    in4->setStatus(false);
    in5->setStatus(false);
    in6->setStatus(false);

    provider->registerNode("ros/iomanager/data/in1",in1);
    provider->registerNode("ros/iomanager/data/in2",in2);
    provider->registerNode("ros/iomanager/data/in3",in3);
    provider->registerNode("ros/iomanager/data/in4",in4);
    provider->registerNode("ros/iomanager/data/in5",in5);
    provider->registerNode("ros/iomanager/data/in6",in6);


    out1->setStatus(false);
    out2->setStatus(false);
    out3->setStatus(false);
    out4->setStatus(false);
    out5->setStatus(false);
    out6->setStatus(false);

    provider->registerNode("ros/iomanager/data/out1",out1);
    provider->registerNode("ros/iomanager/data/out2",out2);
    provider->registerNode("ros/iomanager/data/out3",out3);
    provider->registerNode("ros/iomanager/data/out4",out4);
    provider->registerNode("ros/iomanager/data/out5",out5);
    provider->registerNode("ros/iomanager/data/out6",out6);


    subscription_io = this->create_subscription<myinterface::msg::DatalayerDigitalIo>(
      "ctrlx_io_interface", 10, std::bind(&IoManager::topic_callback_io, this, _1));
    subscription_set_output = this->create_subscription<myinterface::msg::DatalayerDigitalIo>(
      "ctrlx_set_output", 10, std::bind(&IoManager::topic_callback_set_output, this, _1));
    
    publisher_ = this->create_publisher<myinterface::msg::DatalayerDigitalIo>("ctrlx_io_interface", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&IoManager::timer_callback, this));

  }

private:
  void topic_callback_io(const myinterface::msg::DatalayerDigitalIo & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s' with the value %d", msg.ioname.c_str(),msg.value);
    //statusNode->setString( msg.mydatalayeraddress.c_str());
  
  }
  rclcpp::Subscription<myinterface::msg::DatalayerDigitalIo>::SharedPtr subscription_io;

    void topic_callback_set_output(const myinterface::msg::DatalayerDigitalIo & msg) const
  {
    RCLCPP_INFO(this->get_logger(), "I want to set: '%s' with the value %d", msg.ioname.c_str(),msg.value);
      switch (str2int(msg.ioname.c_str()))
    {
      case str2int("out1"):
        out1->setStatus(msg.value);
        break;
      case str2int("out2"):
        out2->setStatus(msg.value);
        break;
      case str2int("out3"):
        out3->setStatus(msg.value);
        break;
      case str2int("out4"):
        out4->setStatus(msg.value);
        break;
      case str2int("out5"):
        out5->setStatus(msg.value);
        break;
      case str2int("out6"):
        out6->setStatus(msg.value);
        break;
      case str2int("control"):
        controlNode->setStatus(msg.value);
        break;
      
    }
  }
  rclcpp::Subscription<myinterface::msg::DatalayerDigitalIo>::SharedPtr subscription_set_output;


  void sendMessage(const std::string& ioname, bool value){
    auto message = myinterface::msg::DatalayerDigitalIo();
    message.ioname = ioname;
    message.value = value;
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.ioname.c_str());
    publisher_->publish(message);
  }
  void timer_callback()
  {
    sendMessage("in1",in1->isTrue());
    sendMessage("in2",in2->isTrue());
    sendMessage("in3",in3->isTrue());
    sendMessage("in4",in4->isTrue());
    sendMessage("in5",in5->isTrue());
    sendMessage("in6",in6->isTrue());
    
    sendMessage("out1",out1->isTrue());
    sendMessage("out2",out2->isTrue());
    sendMessage("out3",out3->isTrue());
    sendMessage("out4",out4->isTrue());
    sendMessage("out5",out5->isTrue());
    sendMessage("out6",out6->isTrue());


    if (controlNode->isTrue()!=true){
      msgNode->setString("Waiting for Enable");
      std::cout << "NoEnable'  " << std::endl;
    }
    else{
    msgNode->setString("Started");
    std::cout << "Started'  " << std::endl;

    client->readSync(g_inputbase+"Channel_1.Value", in1->getDataP());
    client->readSync(g_inputbase+"Channel_2.Value", in2->getDataP());
    client->readSync(g_inputbase+"Channel_3.Value", in3->getDataP());
    client->readSync(g_inputbase+"Channel_4.Value", in4->getDataP());
    client->readSync(g_inputbase+"Channel_5.Value", in5->getDataP());
    client->readSync(g_inputbase+"Channel_6.Value", in6->getDataP());
    client->writeSync(g_outputbase+"Channel_1.Value", out1->getDataP());
    client->writeSync(g_outputbase+"Channel_2.Value", out2->getDataP());
    client->writeSync(g_outputbase+"Channel_3.Value", out3->getDataP());
    client->writeSync(g_outputbase+"Channel_4.Value", out4->getDataP());
    client->writeSync(g_outputbase+"Channel_5.Value", out5->getDataP());
    client->writeSync(g_outputbase+"Channel_6.Value", out6->getDataP());
    }


  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<myinterface::msg::DatalayerDigitalIo>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{ datalayerSystem.start(false);
  rclcpp::init(argc, argv);
  
  rclcpp::spin(std::make_shared<IoManager>());
  rclcpp::shutdown();
  return 0;
  
  
}
