#include "perception_hardware/device_base.hpp"
#include "mock_device/mock_camera_data.hpp"

#include <opencv2/opencv.hpp>

namespace perception_hardware {

    class MockCameraDevice : public DeviceBase {
        private:
            std::string name_;
            cv::VideoCapture cap_;
            MockCameraData data_;
            double data_ptr_address_ = 0.0;

            // param
            std::string video_device_;
            int image_width_;
            int image_height_;
            double framerate_;
            std::string pixel_format_;
            std::string camera_frame_id_;
            std::string output_mode_;

        public:
            bool on_init(const hardware_interface::ComponentInfo & info) override {
                name_ = info.name;
                try {
                    video_device_ = info.parameters.at("video_device");
                    image_width_ = std::stoi(info.parameters.at("image_width"));
                    image_height_ = std::stoi(info.parameters.at("image_height"));
                    framerate_ = std::stod(info.parameters.at("framerate"));
                    pixel_format_ = info.parameters.at("pixel_format");
                    camera_frame_id_ = info.parameters.at("camera_frame_id");
                    output_mode_ = info.parameters.at("output_mode");

                    data_.intrinsics[0] = std::stod(info.parameters.at("fx"));
                    data_.intrinsics[1] = std::stod(info.parameters.at("fy"));
                    data_.intrinsics[2] = std::stod(info.parameters.at("cx"));
                    data_.intrinsics[3] = std::stod(info.parameters.at("cy"));

                    cap_.open(video_device_, cv::CAP_V4L2);
                    if (!cap_.isOpened()) {
                        RCLCPP_ERROR(rclcpp::get_logger("MockCameraDevice"), "Failed to open camera: %s", video_device_.c_str());
                        rclcpp::shutdown();
                        return false;
                    }
                    
                    // 设置摄像头参数
                    if (pixel_format_ == "mjpeg") {
                        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
                    } else if (pixel_format_ == "yuyv") {
                        cap_.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('Y', 'U', 'Y', 'V'));
                    }
                    
                    cap_.set(cv::CAP_PROP_FRAME_WIDTH, image_width_);
                    cap_.set(cv::CAP_PROP_FRAME_HEIGHT, image_height_);
                    cap_.set(cv::CAP_PROP_FPS, framerate_);
                } catch (const std::out_of_range & e) {
                    RCLCPP_ERROR(rclcpp::get_logger("MockCameraDevice"), "Failed to get parameter: %s", e.what());
                    return false;
                }

                data_ptr_address_ = static_cast<double>(reinterpret_cast<uintptr_t>(&data_));
                
                // 
                RCLCPP_INFO(rclcpp::get_logger("MockCameraDevice"),
                                     "Controller Link Active! Sensor [%s] data_ptr address: 0x%lx", 
                                     name_.c_str(), data_ptr_address_);

                return true;
            }

            void read(const rclcpp::Time & time, const rclcpp::Duration & period) override {
                cv::Mat frame;
                if(cap_.read(frame) && !frame.empty()) {
                    cv::rotate(frame, data_.image, cv::ROTATE_180);
                    data_.timestamp_nanos = time.nanoseconds();
                }

                RCLCPP_INFO(rclcpp::get_logger("MockCameraDevice"),
                                     "Raw Buffer Address: %p", 
                                     (void*)(data_.image.data));

                // // 1. 如果图像还没初始化，先初始化一个黑底画布
                // if (data_.image.empty()) {
                //     data_.image = cv::Mat::zeros(480, 640, CV_8UC3);
                //     data_.frame_id = "camera_link";
                // }

                // // 2. 这里的逻辑会在每个控制周期（比如 100Hz）运行
                // // 我们画一个随时间变化的圆圈和文字
                
                // // 先清除上一帧内容（可选，不清除就是重叠效果）
                // data_.image = cv::Scalar(0, 0, 0); 

                // // 画一个跳动的圆圈
                // static int x = 0;
                // x = (x + 5) % 640;
                // cv::circle(data_.image, cv::Point(x, 240), 50, cv::Scalar(255, 0, 0), -1);

                // // 写上调试文字
                // std::string info = "Orin Zero-Copy: " + std::to_string(time.nanoseconds());
                // cv::putText(data_.image, info, cv::Point(50, 50), 
                //             cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 0), 2);

                // // 更新时间戳
                // data_.timestamp_nanos = time.nanoseconds();
            }

            void write(const rclcpp::Time & time, const rclcpp::Duration & period) override {
                // null
            }

            std::vector<hardware_interface::StateInterface> export_state_interfaces() override {
                return { hardware_interface::StateInterface(name_, "data_ptr", &data_ptr_address_) };
            }

            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override {
                return {};
            }

            std::string get_name() const override {
                return name_;
            }

            ~MockCameraDevice() override {
                if (cap_.isOpened()) {
                    cap_.release();
                }
            }

    };
}