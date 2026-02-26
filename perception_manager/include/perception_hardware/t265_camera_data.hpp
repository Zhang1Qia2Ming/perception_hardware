#ifndef _PERCEPTION_HARDWARE_T265_CAMERA_DATA_HPP_
#define _PERCEPTION_HARDWARE_T265_CAMERA_DATA_HPP_

#include <cstdint>
#include <opencv2/opencv.hpp>

#include "mock_device/mock_camera_data.hpp"

namespace perception_hardware {

    struct alignas(64) TestT265CameraData
    {
        /* data */
        cv::Mat fisheye_left;
        cv::Mat fisheye_right;

        double pose[7];

        double linear_velocity[3];
        double angular_velocity[3];
        double linear_accel[3];

        uint64_t timestamp_nanos;
        int32_t tracker_confidence;

        char frame_id[64];
        double reserved[4];
    };

    struct alignas(64) T265CameraData
    {
        CameraDataFrame fisheye_left;  // left fisheye image - 0
        CameraDataFrame fisheye_right; // right fisheye image - 1
        IMUDataFrame imu;
        PoseDataFrame pose;

    };
    
} // namespace perception_hardware

#endif