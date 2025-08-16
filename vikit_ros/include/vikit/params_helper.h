/*
 * ros_params_helper.h
 *
 *  Created on: Feb 22, 2013
 *  Updated for ROS2: 2024
 *      Author: cforster
 *
 * from libpointmatcher_ros
 */

#ifndef ROS_PARAMS_HELPER_H_
#define ROS_PARAMS_HELPER_H_

#include <string>
#include <rclcpp/rclcpp.hpp>

namespace vk {

inline
bool hasParam(const std::string& name)
{
  // In ROS2, we need a node to access parameters
  // This function signature is kept for compatibility but requires a node
  RCLCPP_WARN(rclcpp::get_logger("vikit"), "hasParam requires a node context in ROS2");
  return false;
}

template<typename T>
T getParam(const std::string& name, const T& defaultValue)
{
  // In ROS2, we need a node to access parameters
  // This function signature is kept for compatibility but requires a node
  RCLCPP_WARN(rclcpp::get_logger("vikit"), "getParam requires a node context in ROS2");
  return defaultValue;
}

template<typename T>
T getParam(const std::string& name)
{
  // In ROS2, we need a node to access parameters
  // This function signature is kept for compatibility but requires a node
  RCLCPP_ERROR(rclcpp::get_logger("vikit"), "getParam requires a node context in ROS2");
  return T();
}

// ROS2 specific parameter helper functions
template<typename T>
T getParam(rclcpp::Node* node, const std::string& name, const T& defaultValue)
{
  T value;
  if(node->get_parameter(name, value))
  {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(value).c_str());
    return value;
  }
  else
    RCLCPP_WARN(node->get_logger(), "Cannot find value for parameter: %s, assigning default: %s", name.c_str(), std::to_string(defaultValue).c_str());
  return defaultValue;
}

template<typename T>
T getParam(rclcpp::Node* node, const std::string& name)
{
  T value;
  if(node->get_parameter(name, value))
  {
    RCLCPP_INFO(node->get_logger(), "Found parameter: %s, value: %s", name.c_str(), std::to_string(value).c_str());
    return value;
  }
  else
    RCLCPP_ERROR(node->get_logger(), "Cannot find value for parameter: %s", name.c_str());
  return T();
}

} // namespace vk

#endif // ROS_PARAMS_HELPER_H_
