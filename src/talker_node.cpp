/**
 * @file talker_node.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief Minimal driver for the talker node.
 * @version 0.1
 * @date 2025-11-11
 * @details talker_node
 * See the LICENSE file for license information.
 *
 * @copyright Copyright (c) 2025
 *
 */

#include "beginner_tutorials/talker.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTalker>());
  rclcpp::shutdown();
  return 0;
}