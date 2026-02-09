#ifndef _PERCEPTION_DEVICE_FACTORY_HPP_
#define _PERCEPTION_DEVICE_FACTORY_HPP_

#include <memory>
#include <string>

#include "perception_hardware/device_base.hpp"
#include "mock_device/mock_camera_device.hpp"
#include "mock_device/mock_camera_data.hpp"

namespace perception_hardware {

class DeviceFactory {

    public:
        static std::unique_ptr<DeviceBase> create(const std::string & device_type) {
            if (device_type == "mock_camera") {
                return std::make_unique<MockCameraDevice>();
            }

            return nullptr;
        }

};

}  // namespace perception_hardware
#endif  // _PERCEPTION_DEVICE_FACTORY_HPP_