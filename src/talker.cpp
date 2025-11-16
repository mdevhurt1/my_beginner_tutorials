/**
 * @file talker.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief This is a small talker node that publishes messages to the "/topic"
 * topic.
 * @version 0.1
 * @date 2025-11-11
 * @details talker
 * See the LICENSE file for license information.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "beginner_tutorials/talker.hpp"

#include <string>

#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;

MyTalker::MyTalker() : Node("minimal_publisher"), count_(0) {
  publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 10);

  this->declare_parameter("publishing_flag", false);

  this->get_parameter("publishing_flag", publishing_flag_);

  RCLCPP_DEBUG_STREAM(this->get_logger(),
                      "Publishing flag is set to: " << publishing_flag_);

  service_ = this->create_service<std_srvs::srv::SetBool>(
      "set_flag", std::bind(&MyTalker::handle_set_flag_service, this,
                            std::placeholders::_1, std::placeholders::_2));

  timer_ = this->create_wall_timer(500ms,
                                   std::bind(&MyTalker::timer_callback, this));

  tf_static_broadcaster_ =
      std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

  std::vector<double> transformation = {0, 0, 1, 0, 0, 1};
  this->make_transforms(transformation);
}

void MyTalker::timer_callback() {
  if (!publishing_flag_) {
    RCLCPP_WARN_STREAM(this->get_logger(), "Publishing is disabled.");
    return;
  }

  auto message = std_msgs::msg::String();
  if (service_flag_) {
    message.data =
        "Marcus' custom service flag true " + std::to_string(count_++);
  } else {
    message.data =
        "Marcus' custom service flag false " + std::to_string(count_++);
  }
  RCLCPP_INFO_STREAM(this->get_logger(),
                     "Publishing: " << message.data.c_str());
  publisher_->publish(message);
}

void MyTalker::handle_set_flag_service(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
  service_flag_ = request->data;
  response->success = true;
  response->message = service_flag_ ? "Service started." : "Service stopped.";
  RCLCPP_INFO_STREAM(this->get_logger(), response->message.c_str());

  if (service_flag_) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Service flag set to true.");
  } else {
    RCLCPP_FATAL_STREAM(this->get_logger(), "Service flag set to false.");
  }
}

void MyTalker::make_transforms(std::vector<double> transformation) {
  geometry_msgs::msg::TransformStamped static_transform_stamped;

  static_transform_stamped.header.stamp = this->now();
  static_transform_stamped.header.frame_id = "world";
  static_transform_stamped.child_frame_id = "talk";

  static_transform_stamped.transform.translation.x = transformation[0];
  static_transform_stamped.transform.translation.y = transformation[1];
  static_transform_stamped.transform.translation.z = transformation[2];

  tf2::Quaternion quat;
  quat.setRPY(transformation[3], transformation[4], transformation[5]);
  static_transform_stamped.transform.rotation.x = quat.x();
  static_transform_stamped.transform.rotation.y = quat.y();
  static_transform_stamped.transform.rotation.z = quat.z();
  static_transform_stamped.transform.rotation.w = quat.w();

  tf_static_broadcaster_->sendTransform(static_transform_stamped);
}
