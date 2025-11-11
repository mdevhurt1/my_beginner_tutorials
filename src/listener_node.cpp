/**
 * @file listener_node.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Minimal driver for the listener node.
 * @version 0.1
 * @date 2025-11-11
 * @details listener_node
 * See the LICENSE file for license information.
 *
 * @copyright Copyright (c) 2025 Marcus Hurt
 *
 */
#include "beginner_tutorials/listener.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyListener>());
  rclcpp::shutdown();
  return 0;
}
