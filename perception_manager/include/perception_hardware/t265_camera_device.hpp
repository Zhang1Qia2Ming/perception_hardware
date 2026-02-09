#include "perception_hardware/device_base.hpp"

#include <rclcpp/rclcpp.hpp>
#include <librealsense2/rs.hpp>


namespace perception_hardware {

    class T265CameraDevice : public DeviceBase {
        private:
            // basic infomation
            std::string name_;
            T265_Camera_Data data_;
            double data_ptr_address_ = 0.0;

            // RealSense core component
            rs2::device device_;
            rs2::context ctx_;
            rs2::pipeline pipe_;
            rs2::config cfg_;
            std::string serial_no_;
            std::string usb_port_id_;
            std::string device_type_;
            double wait_for_device_timeout_;
            double reconnect_timeout_;
            bool initial_reset_;
            std::thread query_thread_;
            bool is_alive_{false};


        public:
            bool on_init(const hardware_interface::ComponentInfo & info) override {
                name_ = info.name;
                try {
                    serial_no_ = info.parameters.at("serial_no");
                    usb_port_id_ = info.parameters.at("usb_port_id");
                    device_type_ = info.parameters.at("device_type");
                    wait_for_device_timeout_ = std::stod(info.parameters.at("wait_for_device_timeout"));
                    reconnect_timeout_ = std::stod(info.parameters.at("reconnect_timeout"));

                    is_alive_ = true;

                    std::string rosbag_filename_ = "";

                    if(!rosbag_filename.empty()) {
                        {
                            RCLCPP_INFO("Publish from rosbag: %s", rosbag_filename_.c_str());
                            
                        }
                    }
                    else {
                        RCLCPP_INFO("Publish from realsense device: %s", serial_no_.c_str());
                        initial_reset_ = std::stoi(info.parameters.at("initial_reset"));

                        query_thread_ = std::thread([=]()
                        {

                        });

                    }
                } catch (const std::exception & e) {
                    
                }

            }

            void read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            void write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            std::string get_name() const override { return name_; }

            ~T265CameraDevice() override {
                is_alive_ = false;
                if(query_thread_.joinable()) {
                    query_thread_.join();
                }
                // to do: close device
            }
    };


} // namespace perception_hardware