// Author: Raul Cruz-Oliver
// Email: raul.cruz.oliver@gmail.com
// Date, place: January 2023, Butikkon, CH

#pragma once

// basic C++ ROS2 utilities
#include <rclcpp/rclcpp.hpp>

// tf broadcaster utilities
#include <tf2_ros/transform_broadcaster.h>

// transform_stamped message
#include <geometry_msgs/msg/transform_stamped.hpp>

// to compute the conversion rpy to quaternion
#include <tf2/LinearMath/Quaternion.h> 

inline void publishTransform(const std::string &frame_id, const std::string &child_frame_id,
	                      double x, double y, double z, double ro, double pi, double ya, 
	                      tf2_ros::TransformBroadcaster &tf_broadcaster,
						  const rclcpp::Node &node)
	{
	    geometry_msgs::msg::TransformStamped trans_;
		tf2::Quaternion quat_;

    	quat_.setRPY(ro, pi, ya);

	    trans_.header.stamp = node.now();
	    trans_.header.frame_id = frame_id;
	    trans_.child_frame_id = child_frame_id;

	    trans_.transform.translation.x = x;
	    trans_.transform.translation.y = y;
	    trans_.transform.translation.z = z;
	    trans_.transform.rotation.x = quat_.x();
	    trans_.transform.rotation.y = quat_.y();;
	    trans_.transform.rotation.z = quat_.z();;
	    trans_.transform.rotation.w = quat_.w();;

	    tf_broadcaster.sendTransform(trans_);
	}
