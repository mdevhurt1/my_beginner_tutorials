/**
 * @file integration_test_node.cpp
 * @author Marcus Hurt (mhurt@umd.edu)
 * @brief An example of an integration test node using Catch2 framework in ROS2.
 * @version 0.1
 * @date 2025-11-16
 * @details
 * See the LICENSE file for license information.
 * 
 * @copyright Copyright (c) 2025 Marcus Hurt
 * 
 */

#include <catch_ros2/catch_ros2.hpp>
#include <chrono>
#include <rclcpp/executors.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <std_msgs/msg/string.hpp>

using namespace std::chrono_literals;
using std_msgs::msg::String;

auto Logger = rclcpp::get_logger ("");

class MyTestsFixture {
public:
  MyTestsFixture () 
  {
    testerNode = rclcpp::Node::make_shared ("IntegrationTestNode");
    Logger = testerNode->get_logger();

    testerNode->declare_parameter<double> ("test_duration");

    TEST_DURATION = testerNode->get_parameter("test_duration").get_parameter_value().get<double>();
    RCLCPP_INFO_STREAM (Logger, "Got test_duration =" << TEST_DURATION);
  }

  ~MyTestsFixture ()
  {
  }

protected:
  double                  TEST_DURATION;
  rclcpp::Node::SharedPtr testerNode;
};

// Test case to check if the service server is available
TEST_CASE_METHOD (MyTestsFixture, "test service server", "[service]") {
  auto client = testerNode->create_client<std_srvs::srv::SetBool> ("set_flag");
  RCLCPP_INFO_STREAM (Logger, "set_flag client created");

  rclcpp::Time start_time    = rclcpp::Clock().now();
  bool service_found         = false;
  rclcpp::Duration duration  = 0s;
  RCLCPP_INFO_STREAM (Logger, "Performing Test...");
  auto timeout = std::chrono::milliseconds ((int) (TEST_DURATION * 1000));
  
  if (client->wait_for_service (timeout)) {
    duration = (rclcpp::Clock().now() - start_time);
    service_found = true;
  }
  
  RCLCPP_INFO_STREAM (Logger, "duration = " << duration.seconds() << " service_found=" << service_found);
  CHECK (service_found);
}
