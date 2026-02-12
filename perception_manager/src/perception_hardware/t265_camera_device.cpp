#include "perception_hardware/t265_camera_device.hpp"
#include <rclcpp/rclcpp.hpp>


namespace perception_hardware {
    void T265CameraDevice::frame_callback(const rs2::frame& f)
    {
        auto stream = f.get_profile().stream_type();
        switch (stream)
        {
            case RS2_STREAM_GYRO:
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Gyro Frame");
                break;
            case RS2_STREAM_ACCEL:
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Accel Frame");
                break;
            case RS2_STREAM_POSE:
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Pose Frame");
                break;
            case RS2_STREAM_FISHEYE:
                if(f.get_profile().stream_index() == 1)
                    RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Fisheye Frame 1");
                else 
                    RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Fisheye Frame 2");
                break;
            default:
                RCLCPP_INFO(rclcpp::get_logger("T265CameraDevice"), "Default Frame");
                break;
        }
    }
}