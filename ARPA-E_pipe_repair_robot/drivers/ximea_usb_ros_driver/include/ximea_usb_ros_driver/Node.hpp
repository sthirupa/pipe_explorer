#ifndef XIMEA_USB_ROS_DRIVER_NODE_HPP_
#define XIMEA_USB_ROS_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <image_transport/image_transport.hpp>
#include <ximea_usb_ros_driver/VisibilityControl.h>
#include <cv_bridge/cv_bridge.h>
#include <xiApi.h>

namespace ximea_usb_ros_driver {

class Node : public rclcpp::Node 
{
public:

    explicit Node(const rclcpp::NodeOptions & options);

protected:

    void onInit();
    void doWork();
    void reconfigureCallback();

    void updateImageBuffer();

private:

    image_transport::CameraPublisher mPublisher;
    cv::Mat mImage;
    rclcpp::TimerBase::SharedPtr mTimer;

    sensor_msgs::msg::CameraInfo::SharedPtr mCameraInfo;
    double mFrameRate;
    std::string mFrameId;
    std::string mSerialNumber;

    OnSetParametersCallbackHandle::SharedPtr mParameterSetCallback;
    
    XI_IMG mImageBufferHandle;
    HANDLE mCameraHandle = NULL;

    static constexpr char const * const mDistorationModel = "plumb_bob";
    static constexpr int const mUsbInterfaceSpeedInMbps = 2500;
};

} // namespace ximea_usb_ros_driver


#endif // XIMEA_USB_ROS_DRIVER_NODE_HPP_