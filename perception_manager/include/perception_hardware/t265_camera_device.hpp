#include "perception_hardware/device_base.hpp"
#include "perception_hardware/t265_camera_data.hpp"

#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>

#include <thread>
#include <atomic>
#include <regex>
#include <memory>


namespace perception_hardware {

    class T265CameraDevice : public DeviceBase {
        private:
            // basic infomation
            std::string name_;
            T265CameraData data_;
            double data_ptr_address_ = 0.0;

            // RealSense core component
            rs2::device dev_;
            rs2::context ctx_;
            rs2::sensor pose_sensor_;
            // rs2::pipeline pipe_;
            // rs2::config cfg_;

            // param and state
            std::string serial_no_;
            std::string usb_port_id_;
            std::string device_type_;
            double wait_for_device_timeout_;
            double reconnect_timeout_;
            bool initial_reset_;
            std::thread query_thread_;
            std::atomic<bool> is_alive_{false};
            std::atomic<bool> is_streaming_{false};
        
        private:
            std::string parseUsbPort(std::string line)
            {
                std::string port_id;
                std::regex self_regex("(?:[^ ]+/usb[0-9]+[0-9./-]*/){0,1}([0-9.-]+)(:){0,1}[^ ]*", std::regex_constants::ECMAScript);
                std::smatch base_match;
                bool found = std::regex_match(line, base_match, self_regex);
                if (found)
                {
                    port_id = base_match[1].str();
                    if (base_match[2].str().size() == 0)    //This is libuvc string. Remove counter is exists.
                    {
                        std::regex end_regex = std::regex(".+(-[0-9]+$)", std::regex_constants::ECMAScript);
                        bool found_end = std::regex_match(port_id, base_match, end_regex);
                        if (found_end)
                        {
                            port_id = port_id.substr(0, port_id.size() - base_match[1].str().size());
                        }
                    }
                }
                return port_id;
            }

        public:
            bool on_init(const hardware_interface::ComponentInfo & info) override {
                name_ = info.name;
                auto logger = rclcpp::get_logger("T265_SensorAPI");

                try {
                    serial_no_ = info.parameters.at("serial_no");
                    usb_port_id_ = info.parameters.at("usb_port_id");
                    initial_reset_ = (info.parameters.at("initial_reset") == "true" || info.parameters.at("initial_reset") == "1");

                    is_alive_ = true;
                    is_streaming_ = false;

                    query_thread_ = std::thread([this, logger]() {
                        RCLCPP_INFO(logger, "T265 Direct Sensor thread started.");

                        while (is_alive_ && !is_streaming_) {
                            try {
                                // 1. 获取当前所有设备
                                auto list = ctx_.query_devices();
                                dev_ = rs2::device(); // 重置句柄

                                for (auto&& d : list) {
                                    std::string sn = d.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER);
                                    std::string port = parseUsbPort(d.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT));

                                    if ((serial_no_.empty() || sn == serial_no_) && (usb_port_id_.empty() || port == usb_port_id_)) {
                                        dev_ = d;
                                        break;
                                    }
                                }

                                if (!dev_) {
                                    std::this_thread::sleep_for(std::chrono::seconds(1));
                                    continue;
                                }

                                // 2. 硬件重置逻辑 (仅执行一次)
                                if (initial_reset_) {
                                    RCLCPP_INFO(logger, "Device matched. Performing Hardware Reset...");
                                    initial_reset_ = false;
                                    dev_.hardware_reset();
                                    dev_ = rs2::device();
                                    std::this_thread::sleep_for(std::chrono::seconds(5));
                                    continue;
                                }

                                // 3. 直接定位 Pose Sensor
                                auto sensors = dev_.query_sensors();
                                for (auto&& s : sensors) {
                                    if (s.is<rs2::pose_sensor>()) {
                                        pose_sensor_ = s;
                                        break;
                                    }
                                }

                                if (!pose_sensor_) {
                                    RCLCPP_ERROR(logger, "Could not find Pose Sensor on this device!");
                                    std::this_thread::sleep_for(std::chrono::seconds(2));
                                    continue;
                                }

                                // 4. 获取并配置 Pose Profile
                                auto profiles = pose_sensor_.get_stream_profiles();
                                rs2::stream_profile pose_profile;
                                for (auto& p : profiles) {
                                    if (p.stream_type() == RS2_STREAM_POSE && p.format() == RS2_FORMAT_6DOF) {
                                        pose_profile = p;
                                        break;
                                    }
                                }

                                if (!pose_profile) {
                                    RCLCPP_ERROR(logger, "6DOF Pose stream profile not found!");
                                    continue;
                                }

                                // 5. 开启传感器 (Direct Open & Start)
                                pose_sensor_.open(pose_profile);
                                pose_sensor_.start([this](rs2::frame f) {
                                    if (auto pf = f.as<rs2::pose_frame>()) {
                                        auto pose = pf.get_pose_data();

                                        // --- 坐标映射 (T265 -> ROS REP-103) ---
                                        // T265: x=right, y=up, z=backward
                                        // ROS: x=forward, y=left, z=up
                                        data_.pose[0] = -pose.translation.z; // X
                                        data_.pose[1] = -pose.translation.x; // Y
                                        data_.pose[2] =  pose.translation.y; // Z

                                        // 四元数映射
                                        data_.pose[3] =  pose.rotation.w;
                                        data_.pose[4] = -pose.rotation.z;
                                        data_.pose[5] = -pose.rotation.x;
                                        data_.pose[6] =  pose.rotation.y;

                                        data_.timestamp_nanos = static_cast<uint64_t>(pf.get_timestamp() * 1e6);
                                    }
                                });

                                is_streaming_ = true;
                                RCLCPP_INFO(logger, "T265 Pose Sensor is now Active and Streaming.");

                            } catch (const std::exception & e) {
                                RCLCPP_ERROR(logger, "Sensor API Error: %s", e.what());
                                std::this_thread::sleep_for(std::chrono::seconds(2));
                            }
                        }
                        RCLCPP_INFO(logger, "T265 Direct Sensor thread exiting.");
                    });

                } catch (const std::exception & e) {
                    RCLCPP_ERROR(logger, "on_init error: %s", e.what());
                    return false;
                }

                data_ptr_address_ = static_cast<double>(reinterpret_cast<uintptr_t>(&data_));
                return true;
            }

            void read(const rclcpp::Time & time, const rclcpp::Duration & period) override {}

            void write(const rclcpp::Time & time, const rclcpp::Duration & period) override {}

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
                return {
                    hardware_interface::StateInterface(name_, "data_ptr", &data_ptr_address_),
                };
            }

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
                return {};
            }

            std::string get_name() const override { return name_; }

            ~T265CameraDevice() override {
                is_alive_ = false;
                if(query_thread_.joinable()) {
                    query_thread_.join();
                }
                // to do: close device
                try {
                    if (is_streaming_) {
                        pose_sensor_.stop();
                        pose_sensor_.close();
                    }
                } catch (const std::exception & e) {
                    RCLCPP_ERROR(rclcpp::get_logger("T265CameraDevice"), "Stop error: %s", e.what());
                }
            }
    };


} // namespace perception_hardware