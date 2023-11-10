#include <cstdio>
#include <librealsense2/rs.hpp> // Include Intel RealSense Cross Platform API
#include <librealsense2/rs_advanced_mode.hpp>
#include <bits/stdc++.h>
#include <chrono>
#include <memory>
#include <fstream>
#include <iostream>
#include <string>
#include <algorithm>
#include <unistd.h>
#include <stdio.h>

#include <opencv2/opencv.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/hal/interface.h>
#include <image_transport/image_transport.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/header.hpp"
#include "cv_bridge/cv_bridge.h"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb/stb_image_write.h"
#include "../include/examples/example.hpp"
#define GLFW_DLL
#include <GLFW/glfw3.h>"

using namespace std::chrono_literals;
using namespace rs2;
using namespace cv;

const std::string rgb_topic = "/color/image_raw";
const std::string depth_topic = "/depth/image_rect_raw";

const std::string depth_window_name = "Edge of Depth Filter";
const std::string rgb_window_name = "Live Color Image";

class RealSenseCV : public rclcpp::Node
{
  public:
    RealSenseCV(): Node("camera_cv_topics_node")
    {
      // Serializable Device Setup
      p.start();
      rs2::pipeline_profile profile = p.get_active_profile();
      p.stop();
      json_filename = "src/rs_hello_imshow/src/preset_depth_circles.json";

      std::ifstream t;
      t.open(json_filename, std::ifstream::in);

      // Serializable Device Open JSON file
      if (t.is_open())
      {
        auto serializable = profile.get_device().as<rs2::serializable_device>();
        std::string content;
        content.assign((std::istreambuf_iterator<char>(t)), (std::istreambuf_iterator<char>()));
        serializable.load_json(content);
        t.close();
      }

      namedWindow(depth_window_name);
      namedWindow(rgb_window_name);

      publisher_ = this->create_publisher<std_msgs::msg::String>("realsense_dist", 5);
      // img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/realsense_image", 5); OPTION NOT USED
      // img_pub_ = image_transport::create_publisher(this, "/realsense_image", rmw_qos_profile_default);
      // timer_ = this->create_wall_timer(200ms, std::bind(&RealSenseCV::timer_callback, this)); OPTION NOT USED
      color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/color/image_raw",      10, std::bind(&RealSenseCV::show_rgb_img, this, std::placeholders::_1));
      depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/depth/image_rect_raw", 10, std::bind(&RealSenseCV::get_distance, this, std::placeholders::_1));
      
      coordinates_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/box_coordinates", 10, std::bind(&RealSenseCV::coordinates_callback, this, std::placeholders::_1));
      
      this->timer_callback();

    } //end constructor

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr coordinates_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
    float dist_to_center = 0;

    int width = 600;
    int height = 450;

    // JSON file loading
    rs2::context ctx;
    std::string json_filename;

    std_msgs::msg::String message = std_msgs::msg::String();
    std_msgs::msg::Header header;
    // sensor_msgs::msg::Image::Ptr img_msg; // img_msg PTR option
    sensor_msgs::msg::Image img_msg; // img_msg OPTION
    cv_bridge::CvImage img_cvimg;
    float* box_cords = new float[12]();

    void get_distance(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      
      Mat pub_img, gray, gaus, lapl, dst;
      // bool a = true;

      // pub_img = cv::Mat(Size(width, height), CV_8UC3, (void*)depth_fr.get_data(), Mat::AUTO_STEP);
      pub_img = cv_bridge::toCvCopy(msg, "16UC1")->image;

      // cvtColor(pub_img, gray, COLOR_BGR2GRAY);
      convertScaleAbs(pub_img, gray, 0.1);
      // GaussianBlur(pub_img, gaus, Size(3,3), 0, 0, BORDER_DEFAULT);
      GaussianBlur(gray, gaus, Size(3,3), 0, 0, BORDER_DEFAULT);
      // cvtColor(gaus, gray, COLOR_BGR2GRAY);
      // gaus.copyTo(gray);
      Laplacian(gaus, lapl, 3, 3, 3, 0, BORDER_DEFAULT);
      convertScaleAbs(lapl, dst);
      // std::cout << "loop" << std::endl;
      imshow(depth_window_name, dst);
      waitKey(3);

      // for (int r = 1; r < pub_img.rows()
    }

    void show_rgb_img(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      Mat rgb_img = cv_bridge::toCvCopy(msg, "bgr8")->image;

      int start = 0;
      while(box_cords[start] != 0 && start < 12)
	{
	  rectangle(rgb_img, Point(box_cords[start], box_cords[start+1]),
		    Point(box_cords[start+2], box_cords[start+3]), Scalar(0,255,0));
	  start += 4;
	}
      
      imshow(rgb_window_name, rgb_img);
      waitKey(3);
    }

    void coordinates_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
      auto coordinates = msg->data;
      int i = 0;
      for (auto n : coordinates)
      {
	std::cout << std::to_string(n) << ", ";
	box_cords[i] = msg->data[i];
	i++;
      }
      std::cout << std::endl;
      for (i; i < 12; i++)
      {
        box_cords[i] = 0;
      }
    }
      
    void timer_callback()
    {
      // bool a = true;
      // this->get_distance();
      message.data = std::to_string(dist_to_center * 100.0);
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      header.stamp = this->now();
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node_rscv = std::make_shared<RealSenseCV>();
  // auto color_subscription = node_rscv->create_subscription<sensor_msgs::msg::Image>(rgb_topic, 10, node_rscv->show_rgb_img);
  // auto depth_subscription = node_rscv->create_subscription<sensor_msgs::msg::Image>(depth_topic, 10, node_rscv->get_distance);
  rclcpp::spin(node_rscv);
  rclcpp::shutdown();
  return 0;
}
