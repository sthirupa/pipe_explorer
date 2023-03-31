#include <cstdio>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <bits/stdc++.h>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloRealSenseSubscriber : public rclcpp::Node
{
  public:
    HelloRealSenseSubscriber(): Node("subscriber_node")
    {
      // timer_ = this->create_wall_timer(500ms, std::bind(&HelloRealSenseSubscriber::timer_callback, this));
      publisher_warn_ = this->create_publisher<std_msgs::msg::String>("/threshold_warning", 10);
      timer_warn_ = this->create_wall_timer(500ms, std::bind(&HelloRealSenseSubscriber::timer_callback, this));
      subscription_ = this->create_subscription<std_msgs::msg::String>("realsense_dist", 10, std::bind(&HelloRealSenseSubscriber::nodesubscriber_callback, this, std::placeholders::_1));
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_warn_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_warn_;
    float object_distance;
    const float threshold_distance = 30.0;
    int threshold_consec_count = 0;
    const int threshold_consec_count_MAX = 3;
    bool warning_stop = false;

    void nodesubscriber_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      object_distance = std::stof(msg->data.c_str());
      RCLCPP_INFO(this->get_logger(), "Recorded distance from Camera to Object in cm:  %s", msg->data.c_str());
      if((object_distance < threshold_distance && object_distance > 0.01) || object_distance == 0.00)
      {
        if(object_distance > 0.01)
        {
          RCLCPP_INFO(this->get_logger(), "WARNING: OBJECT DETECTED AHEAD!");
        }

        threshold_consec_count++;
        if(threshold_consec_count >= threshold_consec_count_MAX)
        {
          warning_stop = true;
        }
      }
      else
      {
        threshold_consec_count = 0;
        warning_stop = false;
      }
    }

    std::string	msg_warn_content()
    {
      return warning_stop ? "1" : "0";
    }

    void timer_callback()
    {
      auto message = std_msgs::msg::String();
      message.data = msg_warn_content();
      publisher_warn_->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloRealSenseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
