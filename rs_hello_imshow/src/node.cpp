#include <cstdio>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <bits/stdc++.h>
#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>
#include <unistd.h>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class HelloRealSense : public rclcpp::Node
{
  public:
    HelloRealSense(): Node("node")
    {
      // Adv Mode prep - potentially useful
      // auto devices = ctx.query_devices();
      // dev = devices[0];
      // serial = dev.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
      // json_filename = "preset_given_shortrange.json";

      // Adv Mode JSON Load - USELESS, does not work w L515
      // adv_mode_dev = dev.as<rs400::advanced_mode>();
      // if(!adv_mode_dev.is_enabled())
      // {
      //   adv_mode_dev.toggle_advanced_mode(true);
      // }
      // std::ifstream t(json_filename);
      // preset_json.assign((std::istreambuf_iterator<char>(t)), (std::istreambuf_iterator<char>()));
      // adv_mode_dev.load_json(preset_json);

      // Adv Mode prep, con't.
      // cfg.enable_device(serial);
      // p.start(cfg);

      // Serializable Device Setup
      p.start();
      rs2::pipeline_profile profile = p.get_active_profile();
      p.stop();
      // json_filename = "src/rs_hello_imshow/src/preset_given_shortrange.json";
      json_filename = "src/rs_hello_imshow/src/preset_custom.json";

      std::ifstream t;
      t.open(json_filename, std::ifstream::in);

      // Checking current running directory
      char *currdir = get_current_dir_name();
      std::string outsideprintmsg = currdir;
      std::string insideprintmsg = "Inside loop: " + outsideprintmsg;
      // RCLCPP_INFO(this->get_logger(), outsideprintmsg);

      // Serializable Device Open JSON file
      if (t.is_open())
      {
        // RCLCPP_INFO(this->get_logger(), insideprintmsg);
        auto serializable = profile.get_device().as<rs2::serializable_device>();

        // istream getline method - potentially useful, could not get it to work
        // std::string tread;
        // while(std::getline(t, tread))
        // {
        //   RCLCPP_INFO(this->get_logger(), tread);
        //   serializable.load_json(tread);
        // }

        // istreambuf method - get whole file at once, works
        std::string content;
        content.assign((std::istreambuf_iterator<char>(t)), (std::istreambuf_iterator<char>()));
        // RCLCPP_INFO(this->get_logger(), content);
        serializable.load_json(content);

        t.close();
      }

      // Start pipeline
      p.start();
      publisher_ = this->create_publisher<std_msgs::msg::String>("realsense_dist", 10);
      timer_ = this->create_wall_timer(500ms, std::bind(&HelloRealSense::timer_callback, this));
    }

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rs2::frameset frames;
    float width;
    float height;
    float dist_to_center;

    // JSON file loading
    rs2::context ctx;
    // rs2::device dev;
    rs2::config cfg;
    // rs400::advanced_mode adv_mode_dev;
    // std::string serial;
    std::string json_filename;
    // std::string preset_json;

    void get_distance()
    {
      frames = p.wait_for_frames();
      rs2::depth_frame depth = frames.get_depth_frame();
      width = depth.get_width();
      height = depth.get_height();
      dist_to_center = depth.get_distance(width / 2, height / 2);
    }

    void timer_callback()
    {
      this->get_distance();
      auto message = std_msgs::msg::String();
      message.data = std::to_string(dist_to_center * 100.0);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloRealSense>());
  rclcpp::shutdown();
  return 0;
}
