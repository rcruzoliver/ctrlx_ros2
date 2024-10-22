// Author: Raul Cruz-Oliver
// Email: raul.cruz.oliver@gmail.com
// Date, place: September 2024, Buttikon, CH

#include "nodeCollection.hpp"

// ScaraStatePublisher

ScaraStatePublisher::ScaraStatePublisher(): Node("scara_state_publisher") {
    subscription_ = this->create_subscription<sensor_msgs::msg::JointState>("scara_joints", 10, 
    std::bind(&ScaraStatePublisher::ScaraJointsCallback, this, std::placeholders::_1));
}

void ScaraStatePublisher::ScaraJointsCallback(const sensor_msgs::msg::JointState::SharedPtr msg){
    std::vector<double> jointPositions = msg->position;
	tf2_ros::TransformBroadcaster tf_broadcaster(this);
	publishTransform("base_link", "arm", 0, 0, 0.19, 0, 0, jointPositions[0], tf_broadcaster, *this); 
	publishTransform("arm", "forearm_lower", 0.22, 0.0, 0.21, 0, 0, jointPositions[1], tf_broadcaster, *this); 
	publishTransform("forearm_lower", "ballscrew", 0.24, 0, jointPositions[2], 0, 0, 0, tf_broadcaster, *this); 
	publishTransform("ballscrew", "flange", 0, 0, -0.26, 0, 0, jointPositions[3], tf_broadcaster, *this); 
}
  

// ScaraStateManager

ScaraStateManager::ScaraStateManager(): Node("scara_state_manager"), count1_(0), count2_(0),increasing1_(true), increasing2_(true){
    publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("scara_joints", 10);
    timer_ = this->create_wall_timer(
      	std::chrono::milliseconds(10), 
		std::bind(&ScaraStateManager::timer_callback, this));
	joint_name_ = {"c1", "c2", "z", "c3"};
}

void ScaraStateManager::timer_callback(){
        if (increasing1_) {count1_++; if (count1_ == 380) {increasing1_ = false;}
        } else {count1_--; if (count1_ == -659) {increasing1_ = true;}}

		if (increasing2_) {count2_++; if (count2_ == 735) {increasing2_ = false;}
        } else {count2_--; if (count2_ == -353) {increasing2_ = true;}}

		auto msg = std::make_unique<sensor_msgs::msg::JointState>();
		msg->name = joint_name_;
		msg->position = {0.001*count1_, 0.001*count2_, 0.0001*count1_, 0.0001*count1_};

      	publisher_->publish(std::move(msg));
}

