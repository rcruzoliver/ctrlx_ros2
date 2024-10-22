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
#include <random>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/vector3.hpp"


using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node
{
public:
  MinimalPublisher()
  : Node("minimal_publisher"), count_(0)
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Vector3>("test_vector3", 10);
    timer_ = this->create_wall_timer(
      std::chrono::milliseconds(5), std::bind(&MinimalPublisher::timer_callback, this));
  }

private:
void timer_callback()
{

    // Initialize random number generator
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<_Float64> dis(-10.0, 10.0); // Change the range as needed
	

    auto msg = std::make_unique<geometry_msgs::msg::Vector3>();
    // msg->x = dis(gen);
    msg->x = count_;
    msg->y = dis(gen);
    msg->z = dis(gen);

    publisher_->publish(*msg);
    // RCLCPP_INFO(this->get_logger(), "Publishing: %f, %f, %f", msg->x, msg->y, msg->z);
    
    // Reset counter
    count_=count_+0.1;
    if (count_>100){
      count_=-100;
    }
}
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Vector3>::SharedPtr publisher_;
  double count_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}
