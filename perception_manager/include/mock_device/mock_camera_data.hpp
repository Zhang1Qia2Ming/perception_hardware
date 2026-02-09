#ifndef _PERCEPTION_HARDWARE_MOCK_CAMERA_DATA_HPP_
#define _PERCEPTION_HARDWARE_MOCK_CAMERA_DATA_HPP_

#include <opencv2/opencv.hpp>
#include <cstdint>

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
        uint64_t timestamp_nanos;

        char frame_id[64];
        
        uint64_t reserved[4];
        
    };

}

#endif