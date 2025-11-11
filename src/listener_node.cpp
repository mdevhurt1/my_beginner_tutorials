
#include "rclcpp/rclcpp.hpp"
#include "beginner_tutorials/listener.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MyListener>());
  rclcpp::shutdown();
  return 0;
}
