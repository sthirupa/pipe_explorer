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
      publisher_warn_ = this->create_publisher<std_msgs::msg::String>("/threshold_warning", 1);
      timer_warn_ = this->create_wall_timer(500ms, std::bind(&HelloRealSenseSubscriber::timer_callback, this));
      subscription_ = this->create_subscription<std_msgs::msg::String>("realsense_dist", 5, std::bind(&HelloRealSenseSubscriber::nodesubscriber_callback, this, std::placeholders::_1));
      yolo_subscr_ =  this->create_subscription<std_msgs::msg::String>("/yolo_signal",   2, std::bind(&HelloRealSenseSubscriber::yolo_subscr_callback, this, std::placeholders::_1));
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_warn_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_warn_;
    float object_distance;
    const float threshold_distance = 30.0;
    int threshold_consec_count = 0;
    const int threshold_consec_count_MAX = 3;
    bool warning_stop = false;
    int hysteresis_count_depth = 0;
    const int hysteresis_count_depth_MAX = 4;
    int yolo_sig = 0;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr yolo_subscr_;
    int yolo_consec_count = 0;
    const int yolo_consec_count_MAX = 3;
    int hysteresis_count_yolo = 0;
    const int hysteresis_count_yolo_MAX = 2;


    void nodesubscriber_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      object_distance = std::stof(msg->data.c_str());
      if((object_distance < threshold_distance && object_distance > 0.01) || object_distance == 0.00)
      {
        if(object_distance > 0.01)
        {
          RCLCPP_INFO(this->get_logger(), "Recorded distance from Camera to Object in cm:  %s", msg->data.c_str());
          RCLCPP_INFO(this->get_logger(), "WARNING: OBJECT DETECTED AHEAD!");
          threshold_consec_count++;
	  hysteresis_count_depth = 0;
        }

        if(threshold_consec_count >= threshold_consec_count_MAX)
        {
          warning_stop = true;
	  RCLCPP_INFO(this->get_logger(), "STOP TRIGGERED BY DEPTH");

          if(threshold_consec_count > threshold_consec_count_MAX)
          {
            threshold_consec_count = threshold_consec_count_MAX + 1;
          }
        }
      }
      else
      {
	if (warning_stop && hysteresis_count_depth < hysteresis_count_depth_MAX)
	{
	  hysteresis_count_depth++;
	}
	else
	{
          threshold_consec_count--;
          if(threshold_consec_count <= 0)
          {
            warning_stop = false;
            if(threshold_consec_count < 0)
            {
              threshold_consec_count = 0;
            }
          }
        }
      }

      if (!warning_stop) {hysteresis_count_depth=0;}
      RCLCPP_INFO(this->get_logger(), "Depth threshold count: %s", std::to_string(threshold_consec_count).c_str());
      RCLCPP_INFO(this->get_logger(),"Depth hysteresis count: %s", std::to_string(hysteresis_count_depth).c_str());
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

    void yolo_subscr_callback(const std_msgs::msg::String::SharedPtr msg)
    {
      yolo_sig = std::stoi(msg->data.c_str());
      if (yolo_sig == 1)
      {
	yolo_consec_count++;
	hysteresis_count_yolo = 0;
	
        if(yolo_consec_count >= yolo_consec_count_MAX)
        {
          warning_stop = true;
          if (yolo_consec_count > yolo_consec_count_MAX)
          {
            yolo_consec_count = yolo_consec_count_MAX + 1;
          }
        }

      }
      else
      {	
	if (warning_stop && hysteresis_count_yolo < hysteresis_count_yolo_MAX)
	{
	  hysteresis_count_yolo++;
	}
	else
	{
          yolo_consec_count--;
          if(yolo_consec_count <= 0)
          {
            warning_stop = false;
            yolo_consec_count = 0;
          }
        }
      }
    
      if (!warning_stop) {hysteresis_count_yolo=0;}
      RCLCPP_INFO(this->get_logger(), "YOLO signaled count: %s", std::to_string(yolo_consec_count).c_str());
      RCLCPP_INFO(this->get_logger(), "YOLO hysteresis count: %s", std::to_string(hysteresis_count_yolo).c_str());
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloRealSenseSubscriber>());
  rclcpp::shutdown();
  return 0;
}
