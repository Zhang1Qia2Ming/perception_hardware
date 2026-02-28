#ifndef _PERCEPTION_HARDWARE_MOCK_CAMERA_DATA_HPP_
#define _PERCEPTION_HARDWARE_MOCK_CAMERA_DATA_HPP_

#include <opencv2/opencv.hpp>
#include <cstdint>
#include <atomic>

namespace perception_hardware {

    // struct MockCameraData {
    //     cv::Mat image;
    //     double intrinsics[4];
    //     std::string frame_id;
    //     uint64_t timestamp_nanos;
    // };

    struct alignas(64) MockCameraData {
        cv::Mat image;

        double intrinsics[4];
        std::atomic<uint64_t> timestamp_nanos{0};

        char frame_id[64];
        
        uint64_t reserved[4];
        
    };

    struct CameraDataFrame {
        std::atomic<uint64_t> timestamp_nanos{0};
        char frame_id[64];
        uint32_t height;
        uint32_t width;
        cv::Mat image;
    };

    struct IMUDataFrame {
        std::atomic<uint64_t> timestamp_nanos{0};
        char frame_id[64];
        double gyro[3];
        double accel[3];
    };

    struct PoseDataFrame {
        std::atomic<uint64_t> timestamp_nanos{0};
        char frame_id[64];
        double translation[3];
        double rotation[4];
        double velocity[3];
        double angular_velocity[3];
    };

}

#endif