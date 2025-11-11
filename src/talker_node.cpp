#include "beginner_tutorials/talker.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyTalker>());
  rclcpp::shutdown();
  return 0;
}