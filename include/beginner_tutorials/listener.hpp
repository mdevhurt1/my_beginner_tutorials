/**
 * @file listener.hpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief This is a small listener node that subscribes to the "/topic" topic
 * and logs received messages.
 * @version 0.1
 * @date 2025-11-11
 * @details listener
 * See the LICENSE file for license information.
 *
 * @copyright Copyright (c) 2025 Marcus Hurt
 *
 */

#pragma once

#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MyListener : public rclcpp::Node {
 public:
  /**
   * @brief Construct a new My Listener object
   *
   */
  MyListener();

 private:
  /**
   * @brief Callback function for the subscription
   *
   * @param msg - the received message
   */
  void topic_callback(const std_msgs::msg::String& msg) const;

  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};