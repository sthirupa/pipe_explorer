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

// 3rd party header for writing png files
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include "../include/stb/stb_image_write.h"
#include "../include/examples/example.hpp"
#define GLFW_DLL
#include <GLFW/glfw3.h>

using namespace std::chrono_literals;
using namespace rs2;
using namespace cv;

void register_glfw_callbacks(window& app, glfw_state& app_state);


class HelloRealSense : public rclcpp::Node
{
  public:
    HelloRealSense(): Node("camera_cv_node")
    {
      // Serializable Device Setup
      p.start();
      rs2::pipeline_profile profile = p.get_active_profile();
      p.stop();
      json_filename =
	// "src/rs_hello_imshow/src/preset_depth_circles.json";
	"src/rs_hello_imshow/src/preset_custom.json";

      std::ifstream t;
      t.open(json_filename, std::ifstream::in);

      // Serializable Device Open JSON file
      if (t.is_open())
      {
        auto serializable = profile.get_device().as<rs2::serializable_device>();
        // istreambuf method - get whole file at once, works
        std::string content;
        content.assign((std::istreambuf_iterator<char>(t)), (std::istreambuf_iterator<char>()));
        // RCLCPP_INFO(this->get_logger(), content);
        serializable.load_json(content);
        t.close();
      }

      // Start pipeline
      cfg.enable_stream(RS2_STREAM_DEPTH);
      cfg.enable_stream(RS2_STREAM_COLOR);
      p.start(cfg);
      // align_to_depth = &(RS2_STREAM_DEPTH);
      // align_to_color(RS2_STREAM_COLOR);
      publisher_ = this->create_publisher<std_msgs::msg::String>("realsense_dist", 5);
      // img_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/realsense_image", 5); OPTION NOT USED
      img_pub_ = image_transport::create_publisher(this, "/realsense_image", rmw_qos_profile_default);
      // timer_ = this->create_wall_timer(200ms, std::bind(&HelloRealSense::timer_callback, this)); OPTION NOT USED
      this->timer_callback();

      // for (auto i = 0; i < 30; ++i) p.wait_for_frames(); // wait for auto-exposure to stabilize
    } //end constructor

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    // rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr img_publisher_; OPTiON NOT USED
    image_transport::Publisher img_pub_;
    rs2::frameset frames;
    float width;
    float height;
    float dist_to_center = 0;

    // JSON file loading
    rs2::context ctx;
    // rs2::device dev; NOT NEEDED
    rs2::config cfg;
    std::string json_filename;
    // rs2::align *align_to_depth;
    // rs2::align *align_to_color;

    // Tutorial - Save Image to Disk + Capture
    // rs2::colorizer color_map;

    // Create a simple OpenGL window for rendering:
    std::string s = "RealSense Capture / Pointcloud";
    std::string png_file = "camera_cv_node_savedimg.png";
    std::string window_name = "Mat window";
    const char* str = s.c_str();
    float _width = 960;
    float _height = 720;
    window _app = window(_width, _height, str);

    void save_img()
    {
      // Declare two textures on the GPU, one for depth and one for color
      texture depth_image;

      frames = p.wait_for_frames();
      rs2::frame depth = frames.get_depth_frame();
      depth_image.render(depth, { 0, 0, _app.width() / 2, _app.height() });

      if (auto vf = frames.as<rs2::video_frame>()) {
        if (vf.is<rs2::depth_frame>()) {
          stbi_write_png(png_file.c_str(), vf.get_width(), vf.get_height(),
            vf.get_bytes_per_pixel(), vf.get_data(), vf.get_stride_in_bytes());
        }
      }
    }

    // Tutorial - Pointcloud

    // struct state { double yaw, pitch, last_x, last_y; bool ml; float offset_x, offset_y; texture tex; };

    glfw_state _app_state;
    // _app_state.offset_x = 0.0;
    // _app_state.offset_y = 0.0;

    rs2::pointcloud pc;
    rs2::points points;

    // rs2::colorizer color_filter;
    // rs2::threshold_filter thr_filter;
    // rs2::disparity_transform depth_to_disparity(1);

    Mat pub_img, gray, gaus, lapl, dst;
    std_msgs::msg::String message = std_msgs::msg::String();
    std_msgs::msg::Header header;
    // sensor_msgs::msg::Image::Ptr img_msg; // img_msg PTR option
    sensor_msgs::msg::Image img_msg; // img_msg OPTION
    cv_bridge::CvImage img_cvimg;
    bool a = true;

    void get_distance()
    {
      register_glfw_callbacks(_app, _app_state);
      frames = p.wait_for_frames();

      // frames = align_to_depth->process(frames);

      auto color = frames.get_color_frame();
      pc.map_to(color);

      rs2::depth_frame depth_fr = frames.get_depth_frame();
      points = pc.calculate(depth_fr);
      _app_state.tex.upload(color);

      // colorizer options
      // depth_fr = thr_filter.process(depth_fr);
      // depth_fr = depth_to_disparity.process(depth_fr);
      // depth_fr = disparity_to_depth.process(depth_fr);
      // depth_fr = color_filter.process(depth_fr);
      // thr_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.10f);
      // thr_filter.set_option(RS2_OPTION_MAX_DISTANCE, 3.0f);
      // color_filter.set_option(RS2_OPTION_HISTOGRAM_EQUALIZATION_ENABLED, 0);
      // color_filter.set_option(RS2_OPTION_COLOR_SCHEME, 9.0f);
      // color_filter.set_option(RS2_OPTION_MAX_DISTANCE, 3.0f);
      // color_filter.set_option(RS2_OPTION_MIN_DISTANCE, 0.10f);

      draw_pointcloud(_width, _height, _app_state, points);

      // IGNORE THIS
      // auto vertices = points.get_vertices();
      // auto tex_coords = points.get_texture_coordinates();
      // for (int i = 0; i < points.size(); i++) {
      //   if (vertices[i].z) {
      //     glVertex3fv(vertices[i]);
      //     glTexCoord2fv(tex_coords[i]);
      //   }
      // }

      width = depth_fr.get_width();
      height = depth_fr.get_height();

      dist_to_center = depth_fr.get_distance(width / 2, height / 2);

      std::cout << "1" << std::endl;

      pub_img = cv::Mat(Size(width, height), CV_8UC3, (void*)depth_fr.get_data(), Mat::AUTO_STEP);

      // if (a) {
      //   // img_msg = cv_bridge::CvImage(header, "bgr8", pub_img).toImageMsg(); // img_msg OPTION
      //   img_cvimg = cv_bridge::CvImage(header, "bgr8", pub_img); // img_msg PTR option
      //   img_msg = *img_cvimg.toImageMsg();
      //   std::cout << "2" << std::endl;
      //   a = !a;
      // } else {
      //   img_cvimg.image = pub_img;
      //   std::cout << "2" << std::endl;
      //   img_cvimg.toImageMsg(img_msg);
      // }

      // cvtColor(pub_img, gray, COLOR_BGR2GRAY);
      // pub_img.copyTo(gray); GaussianBlur(gray, gaus, Size(3,3), 0, 0, BORDER_DEFAULT);
      // Laplacian(gaus, lapl, CV_16S, 3, 3, 0, BORDER_DEFAULT);
      // convertScaleAbs(lapl, dst);
      // imshow(window_name, dst);
    }

    void timer_callback()
    {
      // Save to Disk
      // this->save_img();
      bool a = true;

      while(_app) {
        this->get_distance();

        message.data = std::to_string(dist_to_center * 100.0);
        RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
        publisher_->publish(message);

        // cv_bridge::CvImagePtr cv_ptr; NOT NEEDED
        header.stamp = this->now();
        std::cout << "3" << std::endl;
        // img_msg = cv_bridge::CvImage(header, "bgr8", pub_img).toImageMsg(); DONE IN get_distance()
        // img_publisher_->publish(*img_msg.get()); OPTION NOT USED

        // if (pub_img.empty()) {
        //   std::cout << "pub_img empty!" << std::endl;
        // } else {
        //   std::cout << "sensor ptr height: " << img_msg.height << std::endl; // img_msg OPTION
        //   // std::cout << "sensor ptr height: " << img_cvimg.toImageMsg()->height << std::endl; // img_msg PTR option
        // }

        // img_pub_.publish(img_msg);
        std::cout << "4" << std::endl;
      }
      _app.close();
      // glfwDestroyWindow(_app);
    }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<HelloRealSense>());
  rclcpp::shutdown();
  return 0;
}
