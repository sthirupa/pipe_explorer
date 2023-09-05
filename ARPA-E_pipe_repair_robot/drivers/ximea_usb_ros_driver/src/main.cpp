#include <cerrno>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <malloc.h>
#include <unistd.h>

#include <rclcpp/rclcpp.hpp>
#include <ximea_usb_ros_driver/Node.hpp>


int main(int argc, char** argv)
{   
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto node = std::make_shared<ximea_usb_ros_driver::Node>(options);

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}