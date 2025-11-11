/**
 * @file listener.cpp
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

#include "beginner_tutorials/listener.hpp"

using std::placeholders::_1;

MyListener::MyListener() : Node("minimal_subscriber") {
  subscription_ = this->create_subscription<std_msgs::msg::String>(
      "topic", 10, std::bind(&MyListener::topic_callback, this, _1));
}

void MyListener::topic_callback(const std_msgs::msg::String& msg) const {
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "I heard: '" << msg.data.c_str() << "'");
}
