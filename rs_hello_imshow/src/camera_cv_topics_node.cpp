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
// #include <time.h>
#include <ctime>

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
      // img_pub_ = image_transport::create_publisher(this, "/realsense_image", rmw_qos_profile_default); // OPTION NOT USED
      // timer_ = this->create_wall_timer(200ms, std::bind(&RealSenseCV::timer_callback, this));
      color_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/color/image_raw",      10, std::bind(&RealSenseCV::show_rgb_img, this, std::placeholders::_1));
      depth_subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/depth/image_rect_raw", 10, std::bind(&RealSenseCV::get_distance, this, std::placeholders::_1));
      turn_timer_ = this->create_wall_timer(499ms, std::bind(&RealSenseCV::turn_timed_callback, this));
      time_start = std::chrono::high_resolution_clock::now();
      time_publisher_ = this->create_publisher<std_msgs::msg::String>("/turn_ongoing", 5);
      time_message.data = "NONE";
      
      coordinates_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/box_coordinates", 10, std::bind(&RealSenseCV::coordinates_callback, this, std::placeholders::_1));
      
      this->timer_callback();

    } //end constructor

    // Create a Pipeline - this serves as a top-level API for streaming and processing frames
    rs2::pipeline p;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr coordinates_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr color_subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depth_subscription_;
    float dist_to_center = 100;
    rclcpp::TimerBase::SharedPtr turn_timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr time_publisher_;

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
    int left_oncoming_count = 0;
    int right_oncoming_count = 0;
    int turn_imminent_count = 0;
    std::chrono::high_resolution_clock::time_point time_start;
    std::chrono::seconds time_curr;
    std::chrono::seconds time_start_turn;
    int time_since_turn;
    bool turning = false;
    std_msgs::msg::String time_message = std_msgs::msg::String();
    std_msgs::msg::Header time_header;

    void get_distance(const sensor_msgs::msg::Image::SharedPtr msg)
    {
      // std::cout << std::endl;
      
      Mat pub_img, gray, gaus, lapl, dst, disp_dst;

      // pub_img = cv::Mat(Size(width, height), CV_8UC3, (void*)depth_fr.get_data(), Mat::AUTO_STEP);
      pub_img = cv_bridge::toCvCopy(msg, "16UC1")->image;

      // cvtColor(pub_img, gray, COLOR_BGR2GRAY);
      convertScaleAbs(pub_img, gray, 0.1);
      // GaussianBlur(pub_img, gaus, Size(3,3), 0, 0, BORDER_DEFAULT);
      GaussianBlur(gray, gaus, Size(3,3), 0, 0, BORDER_DEFAULT);
      // cvtColor(gaus, gray, COLOR_BGR2GRAY);
      // gaus.copyTo(gray);
      Laplacian(gaus, lapl, 3, 3, 1, 0, BORDER_DEFAULT); // 3,3,3,0
      convertScaleAbs(lapl, dst);
      cvtColor(dst, disp_dst, COLOR_GRAY2BGR);

      std::vector<Vec3f> circles;
      std::cout << "started HoughCircles" << std::endl;
      HoughCircles(dst, circles, HOUGH_GRADIENT, 1, pub_img.cols/30, 140, 42, 20, 150); // 140 40 20 150
      std::cout << "# of circles detected: " << std::to_string(circles.size()) << std::endl;            

      int big_rad = 90;
      int best_row = std::round(dst.rows/2);
      int best_col = std::round(dst.cols/2);
      int best_sum = 100;
      int curr_row, curr_col, curr_rad;
      bool inner_circle_found = false;
      for (size_t i = 0; i < circles.size(); i++) {
	curr_col = circles[i][0];
	curr_row = circles[i][1];
	curr_rad = circles[i][2];
	circle(disp_dst, Point(curr_col, curr_row), curr_rad, Scalar(255,0,255), 1);

	auto dr = std::abs(best_row - curr_row);
	auto dc = std::abs(best_col - curr_col);

	if (curr_rad >= big_rad && dr+dc < best_sum) {
	  big_rad = curr_rad;
	  best_sum = dr + dc;
	  best_row = curr_row;
	  best_col = curr_col;
	}
	
        // Determine if we are close to the turn
	if (curr_rad < 89 && curr_rad > 35 && std::abs(curr_row - best_row) < 0.10*big_rad) {
	  inner_circle_found = true;
	  turn_imminent_count = 0;
	  if (curr_col < best_col - 20) {
	    left_oncoming_count++;
	  } else if (curr_col > best_col + 20) {
	    right_oncoming_count++;
	  }
	}

	std::cout << "CIRCLE No." << std::to_string(i+1) << ": (col,row,rad): " << std::to_string(curr_col)
		  << " " << std::to_string(curr_row) << " " << std::to_string(curr_rad) << std::endl;
      }
      std::cout << std::endl << "Best pipe circle (col,row,rad): " << std::to_string(best_col) << " "
		<< std::to_string(best_row) << " " << std::to_string(big_rad) << std::endl;

      if (!inner_circle_found && (left_oncoming_count > 15 || right_oncoming_count > 15)) {
	turn_imminent_count++;
      }

      const int edge_threshold = 29; // old value 45
      const int dist_threshold = 25;
      float min_distance = 5000;
      int most_suc_r = 0;
      int most_suc_c_edge = 0;
      int most_suc_c_dist = 0;
      int dist_c = -1;
      std::string success = " N";
      for (int r = 1; r < pub_img.rows - 1; r+=1) {
        for (int c = 1; c < pub_img.cols - 1; c+=1) {
          auto dist_pub = pub_img.at<uchar>(r,c);
          auto edge_pub = dst.at<uchar>(r,c);

	  /*
	  if (dist_pub < dist_threshold && edge_pub == 0) {
	    std::cout << "Z ";
	  } else if (edge_pub > edge_threshold) {
	    std::cout << "B/" << std::to_string(dist_pub) << " ";
	  } else {
	    std::cout << std::to_string(edge_pub) << "/" << std::to_string(dist_pub) << " ";
	  }
	  */

	  int point_dist_sq = (r-best_row)*(r-best_row)+(c-best_col)*(c-best_col);
	  int radius_overshoot = -5;
	  
	  /*
	  if (point_dist_sq < (big_rad+radius_overshoot)*(big_rad+radius_overshoot)) {
	    if (edge_pub > edge_threshold && dist_pub >= dist_threshold && dist_pub < dist_to_center) {
	      
	      uchar adj_edge_vals[8] = {dst.at<uchar>(r-1,c-1), dst.at<uchar>(r-1,c), dst.at<uchar>(r-1,c+1),
	                                dst.at<uchar>(r,c-1), dst.at<uchar>(r,c+1),
	                                dst.at<uchar>(r+1,c-1), dst.at<uchar>(r+1,c), dst.at<uchar>(r+1,c+1)};
	      int over_threshold = 0;
	      int under_threshold = 0;
	      for (uchar e : adj_edge_vals) {
		if (e > edge_threshold) {
		  over_threshold++;
		} else {
		  under_threshold++;
		}
	      }
	      if (over_threshold >= 2 && under_threshold >= 1) {
		dist_to_center = dist_pub;
	      }
	    }
	  } // end double if
	  */
	  
	  int big_rad_sq = (big_rad + radius_overshoot) * (big_rad + radius_overshoot);
	  if (point_dist_sq < big_rad_sq && edge_pub > edge_threshold && c <= best_col) {
	    bool edge_neighbor_check = false;
	    int edge_inbounds = 0;
	    bool z_found = false;
	    bool b_found = false;
	    uchar adj_edge_vals[8] = {dst.at<uchar>(r-1,c-1), dst.at<uchar>(r-1,c), dst.at<uchar>(r-1,c+1),
	                              dst.at<uchar>(r,c-1), dst.at<uchar>(r,c+1),
	                              dst.at<uchar>(r+1,c-1), dst.at<uchar>(r+1,c), dst.at<uchar>(r+1,c+1)};
	    for (int ii = 0; ii < 8; ii++) {
	      uchar e = adj_edge_vals[ii];
	      if (e < 2) {
		edge_inbounds++;

		if (!z_found) {
		switch (ii) {
		case 0:
		  z_found = (dst.at<uchar>(r-1,c-2) < 2) && ((dst.at<uchar>(r,c-2) < 2) || (dst.at<uchar>(r-2,c-2) < 2) || (dst.at<uchar>(r-2,c-1) < 2));
		  break;
		case 1:
		  z_found = (dst.at<uchar>(r-2,c) < 2) && ((dst.at<uchar>(r-2,c-1) < 2) || (dst.at<uchar>(r-2,c+1) < 2));
		  break;
		case 2:
		  z_found = (dst.at<uchar>(r-1,c+2) < 2) && ((dst.at<uchar>(r,c+2) < 2) || (dst.at<uchar>(r-2,c+2) < 2) || (dst.at<uchar>(r-2,c+1) < 2));
		  break;
		case 3:
		  z_found = (dst.at<uchar>(r,c-2) < 2) && ((dst.at<uchar>(r-1,c-2) < 2) || (dst.at<uchar>(r+1,c-2) < 2));
		  break;
		case 4:
		  z_found = (dst.at<uchar>(r,c+2) < 2) && ((dst.at<uchar>(r-1,c+2) < 2) || (dst.at<uchar>(r+1,c+2) < 2));
		  break;
		case 5:
		  z_found = (dst.at<uchar>(r+1,c-2) < 2) && ((dst.at<uchar>(r,c-2) < 2) || (dst.at<uchar>(r+2,c-2) < 2) || (dst.at<uchar>(r+2,c-1) < 2));
		  break;
		case 6:
		  z_found = (dst.at<uchar>(r+2,c) < 2) && ((dst.at<uchar>(r+2,c-1) < 2) || (dst.at<uchar>(r+2,c+1) < 2));
		  break;
		case 7:
		  z_found = (dst.at<uchar>(r+1,c+2) < 2) && ((dst.at<uchar>(r,c+2) < 2) || (dst.at<uchar>(r+2,c+2) < 2) || (dst.at<uchar>(r+2,c+1) < 2));
		  break;
		}}
		
	      } else if (e > edge_threshold) {
		edge_inbounds++;
		b_found = true;
	      }

	      if (edge_inbounds >= 3 && z_found && b_found) {
		edge_neighbor_check = true;
		break;
	      }
	    }
	    
	    if (edge_neighbor_check) {
	      if (most_suc_c_dist == 0) {
	        most_suc_r = r;
	        most_suc_c_edge = c;
		dist_c = dist_pub;
		success = " L1";
	      }
	      
	      auto dc_dist = best_col - c;
	      
	      // dx_start = std::round(0.6*dc_dist) - 4;
	      // if (dx_start < 0) {dx_start = 0;}
	      int dx_start = 2;
	      
	      for (int dx = dx_start; dx < std::round(1.8*dc_dist) + 4; dx++) {
	        uchar d_val = pub_img.at<uchar>(r, pub_img.cols - dx);
		if (d_val < min_distance && d_val >= dist_threshold) {
		  if (min_distance == 5000) {
		    most_suc_r = r;
		    most_suc_c_edge = c;
		    most_suc_c_dist = pub_img.cols - dx;
		    dist_c = d_val;
		    success = " L2";
		  }

		  bool dist_neighbor_check = true;
		  int dist_outofbounds = 0;
		  uchar adj_dist_vals[8] = {pub_img.at<uchar>(r-1,c-2), pub_img.at<uchar>(r-1,c), pub_img.at<uchar>(r-1,c+2),
	                                  pub_img.at<uchar>(r,c-2), pub_img.at<uchar>(r,c+2),
	                                  pub_img.at<uchar>(r+1,c-2), pub_img.at<uchar>(r+1,c), pub_img.at<uchar>(r+1,c+2)};
		  for (uchar d : adj_dist_vals) {
		    if (d >= dist_threshold && std::abs(d_val - d) > 0.25*d_val) {
		      dist_outofbounds++;
		    }
		    if (dist_outofbounds > 2) {
		      dist_neighbor_check = false;
		      break;
		    }
		  }
		
	          if (dist_neighbor_check) {
		    min_distance = (float)d_val;
		    
		    dist_c = d_val;
		    most_suc_r = r;
		    most_suc_c_edge = c;
		    most_suc_c_dist = pub_img.cols - dx;
		    success = " Y";
	          }
		  
		} // end d_val check
	      } // end for over dx from dx_start to dx_end
	    } // end if edge_neighbor_check
	  } // end if point within left half of detected circle
        } // end inner for over r,c
	// std::cout << std::endl;
      } // end double for over r,c
      
      if (min_distance < 5000) {
        dist_to_center = min_distance;
      }
      // std::cout << "MinDist= " << std::to_string(min_distance) << std::endl;
      // std::cout << "DistToCenter= " << std::to_string(dist_to_center) << std::endl;
      // std::cout << "MostSuccessful--R-Cedge-Cdist-Val-success: " << std::to_string(most_suc_r) << " " << std::to_string(most_suc_c_edge)
	    // 	<< " " << std::to_string(most_suc_c_dist) << " " << std::to_string(dist_c) << success << std::endl;

      /*
      std::vector<Vec4i> linesP;
      HoughLinesP(dst, linesP, 15, CV_PI/180, 100, 100, 4 ); // runs the actual detection
      int a_max = std::min((int)linesP.size(), 10);
      for (size_t a = 0; a < a_max; a++) {
	auto l = linesP[a];
	std::cout << "LINE No." << a+1 << " - (x1,y1,x2,y2): " << linesP[a][0] << " " << linesP[a][1] << " "
		  << linesP[a][2] << " " << linesP[a][3] << std::endl;
	line(disp_dst, Point(l[0], l[1]), Point(l[2], l[3]), Scalar(0,0,255), 2, LINE_AA);
      }
      */

      imshow(depth_window_name, disp_dst);
      waitKey(3);

    } // end function void get_distance

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
	// std::cout << std::to_string(n) << ", ";
	box_cords[i] = msg->data[i];
	i++;
      }
      // std::cout << std::endl;
      for (i; i < 12; i++)
      {
	box_cords[i] = 0;
      }
    }

    void timer_callback()
    {
      // bool a = true;
      // this->get_distance();
      if (turning) {
	message.data = std::to_string(500);
      } else {
	message.data = std::to_string(dist_to_center);
      }
      // RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
      publisher_->publish(message);
      header.stamp = this->now();
    }

    void turn_timed_callback()
    {
      std::cout << "Reached turn_timed_callback - " << left_oncoming_count << " " << right_oncoming_count << " "
		<< turn_imminent_count << std::endl;
      time_curr = std::chrono::duration_cast<std::chrono::seconds>(std::chrono::high_resolution_clock::now() - time_start);

      bool left_turn = (left_oncoming_count > 18 && left_oncoming_count >= right_oncoming_count) ? true : false;
      bool right_turn = (right_oncoming_count > 18 && right_oncoming_count > left_oncoming_count) ? true : false;
      
      if (left_turn && turn_imminent_count > 500) {
	time_message.data = "LEFT";
	right_oncoming_count = 0;
	if (!turning) {
	  time_start_turn = time_curr;
	}
	turning = true;
	time_since_turn = (time_curr - time_start_turn).count();
	std::cout << "SENDING TURN MESSAGE LEFT" << std::endl;
      } else if (right_turn && turn_imminent_count > 500) {
	time_message.data = "RIGHT";
	left_oncoming_count = 0;
        if (!turning) {
	  time_start_turn = time_curr;
        }
        turning = true;
        time_since_turn = (time_curr - time_start_turn).count();
        std::cout << "SENDING TURN MESSAGE RIGHT" << std::endl;
      } else {
        time_message.data = "NONE";
        turning = false;
      }
      time_header.stamp = this->now();
      time_publisher_->publish(time_message);

      if (time_since_turn >= 80) {
        turning = false;
        left_oncoming_count = 0;
        right_oncoming_count = 0;
	time_since_turn = 0;
      }
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
