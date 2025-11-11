
#include "beginner_tutorials/listener.hpp"

using std::placeholders::_1;

MyListener::MyListener() : Node("minimal_subscriber") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MyListener::topic_callback, this, _1));
}

void MyListener::topic_callback(const std_msgs::msg::String& msg) const  {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
}
