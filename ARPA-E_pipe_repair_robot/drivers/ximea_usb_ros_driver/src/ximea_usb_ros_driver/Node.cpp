#include <camera_info_manager/camera_info_manager.hpp>

#include <ximea_usb_ros_driver/Node.hpp>
#include <rmw/qos_profiles.h>

namespace ximea_usb_ros_driver {

Node::Node(rclcpp::NodeOptions const & options)
: rclcpp::Node("ximea_rgb_camera", options)
, mCameraInfo{std::make_shared<sensor_msgs::msg::CameraInfo>()}
{
    auto qos = rclcpp::QoS(rclcpp::KeepLast(1), rmw_qos_profile_sensor_data);
    qos.keep_last(10);
    qos.reliable();
    qos.durability_volatile();
    mPublisher = image_transport::create_camera_publisher(this, "~/image_raw", qos.get_rmw_qos_profile());
    mFrameId = declare_parameter("frame_id", std::string("camera"));
    mFrameRate = declare_parameter("frame_rate", 30.0);
    mSerialNumber = declare_parameter("serial_number", "UPCBS2110019");

    mParameterSetCallback = add_on_set_parameters_callback(
        [this](std::vector<rclcpp::Parameter> parameters) -> rcl_interfaces::msg::SetParametersResult
        {
            RCLCPP_INFO(get_logger(), "param_change_callback");

            auto result = rcl_interfaces::msg::SetParametersResult();
            result.successful = true;

            for(auto const & parameter : parameters)
            {
                if (parameter.get_name() == "frame_id")
                {
                    mFrameId = parameter.as_string();
                    RCLCPP_INFO_STREAM(get_logger(), "Setting [frame_id] to [" << mFrameId << "]");
                }
                else if (parameter.get_name() == "frame_rate")
                {
                    mFrameRate = parameter.as_double();
                    RCLCPP_INFO_STREAM(get_logger(), "Setting [frame rate] to [" << mFrameRate << "]");
                }
                else if(parameter.get_name() == "serial_number")
                {
                    mSerialNumber = parameter.as_string();
                    RCLCPP_INFO_STREAM(get_logger(), "Setting [serial_number] to [" << mSerialNumber << "]");
                }
            }
            onInit();
            return result;
        }
    );

    onInit();
}
    
void Node::onInit()
{
    RCLCPP_INFO(get_logger(), "Initializing Driver");

    memset(&mImageBufferHandle, 0, sizeof(mImageBufferHandle));
    mImageBufferHandle.size = sizeof(mImageBufferHandle);
   
    if (mCameraHandle != NULL)
    {
       
        if(auto status = xiStopAcquisition(mCameraHandle);
            status != XI_OK || status != XI_ACQUISITION_STOPED || status != XI_ACQUISITION_STOPED)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "Could not stop camera [" << mSerialNumber << "]");
            std::runtime_error("Could not stop camera");
        }
        mCameraHandle = NULL;
    }
       
    if(auto status = xiOpenDeviceBy(XI_OPEN_BY_SN, mSerialNumber.c_str(), &mCameraHandle);
        status != XI_OK)
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Could not open camera with serial number [" << mSerialNumber << "]");
        throw std::runtime_error("Could not open camera");
    }
   
    auto setCameraParamOrThrow = [this](char const * parameter, const auto value)
    {
        auto status = XI_RETURN{};
        if constexpr (std::is_integral_v<decltype(value)> || std::is_enum_v<decltype(value)>)
            status = xiSetParamInt(mCameraHandle, parameter, value);
        else if constexpr (std::is_floating_point_v<decltype(value)>)
            status = xiSetParamFloat(mCameraHandle, parameter, static_cast<float>(value));
        else if constexpr (std::is_same_v<decltype(value), char const * const>)
            status = xiSetParamString(mCameraHandle, parameter, const_cast<char*>(value), strlen(value));
        else
        {
            static_assert(
                !std::is_same_v<decltype(value), decltype(value)>, 
                "Type must be either integer, floating point number or string literal"
            );
        }
       
        if(status != XI_OK)
        {
            RCLCPP_FATAL_STREAM(get_logger(), "Could not set xi camera parameter [" << parameter << "]" );
            std::runtime_error("Could not set xi camera parameter");
        }
    };
   
    setCameraParamOrThrow(XI_PRM_IMAGE_DATA_FORMAT, XI_RGB24);
    setCameraParamOrThrow(XI_PRM_DOWNSAMPLING, XI_DWN_2x2);
    setCameraParamOrThrow(XI_PRM_IMAGE_DATA_BIT_DEPTH, XI_BPP_8);
    setCameraParamOrThrow(XI_PRM_EXPOSURE, 30000.0f);
    setCameraParamOrThrow(XI_PRM_AUTO_WB, XI_ON);

    auto width = int{};
    if(auto status = xiGetParamInt(mCameraHandle, XI_PRM_WIDTH, &width);
        status != XI_OK)
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Could not get frame width");
        throw std::runtime_error("Could not get frame width");
    }

    auto height = int{};
    if(auto status = xiGetParamInt(mCameraHandle, XI_PRM_HEIGHT, &height);
        status != XI_OK)
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Could not get frame height");
        throw std::runtime_error("Could not get frame height");
    }
    
    if(auto status = xiStartAcquisition(mCameraHandle);
        status != XI_OK)
    {
        RCLCPP_FATAL_STREAM(get_logger(), "Could not start acquisition from camera [" << mSerialNumber << "]");
        throw std::runtime_error("Could not start acquisition from camera");
    }
    
    mCameraInfo->header.frame_id = mFrameId;
    
    mCameraInfo->width = width/2.0;
    mCameraInfo->height = height/2.0;
    mCameraInfo->distortion_model = mDistorationModel;
    mCameraInfo->d = {0.0, 0.0, 0.0, 0.0, 0.0};
    mCameraInfo->k = {
        1.0, 0.0, mCameraInfo->width/2.0,
        0.0, 1.0, mCameraInfo->height/2.0,
        0.0, 0.0, 1.0
    };
    mCameraInfo->r = {
        1.0, 0.0, 0.0,
        0.0, 1.0, 0.0,
        0.0, 0.0, 1.0
    };
    mCameraInfo->p = {
        1.0, 0.0, mCameraInfo->width/2.0, 0.0,
        0.0, 1.0, 0.0, mCameraInfo->height/2.0,
        0.0, 0.0, 1.0, 0.0
    };    

    mTimer = create_wall_timer(
        std::chrono::milliseconds(
            static_cast<size_t>(1000.0/mFrameRate)
        ),
        std::bind(&Node::doWork, this)
    );
    RCLCPP_INFO(get_logger(), "Init done");
}

void Node::doWork()
{   
    if(auto status = xiGetImage(mCameraHandle, 2000, &mImageBufferHandle);
        status != XI_OK)
    {
        auto clock = rclcpp::Clock(RCL_STEADY_TIME);
        RCLCPP_WARN_STREAM_THROTTLE(get_logger(),
            clock,
            2000,
            "Could not get image from camera [" << mSerialNumber << "]"
        );
        return;
    }

    
    auto mat = cv::Mat(
        mCameraInfo->height * 2.0,
        mCameraInfo->width * 2.0,
        CV_8UC3,
        reinterpret_cast<uchar*>(mImageBufferHandle.bp)
    );

    mCameraInfo->header.stamp = rclcpp::Clock().now();
    auto msg = cv_bridge::CvImage(
        mCameraInfo->header,
        sensor_msgs::image_encodings::TYPE_8UC3,
        mat
    ).toImageMsg();
    
    mPublisher.publish(msg, mCameraInfo);

}

} // namespace name

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ximea_usb_ros_driver::Node)
