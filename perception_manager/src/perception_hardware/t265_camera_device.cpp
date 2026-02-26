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

    void T265CameraDevice::data_callback(const rs2::frame& f){
        uint64_t ts = static_cast<uint64_t>(f.get_timestamp() * 1000000);

        // 1.handle pose frame
        if(auto pf = f.as<rs2::pose_frame>()){
            auto pose_data = pf.get_pose_data();
            data_.pose.translation[0] = pose_data.translation.x;
            data_.pose.translation[1] = pose_data.translation.y;
            data_.pose.translation[2] = pose_data.translation.z;
            data_.pose.rotation[0] = pose_data.rotation.x;
            data_.pose.rotation[1] = pose_data.rotation.y;
            data_.pose.rotation[2] = pose_data.rotation.z;
            data_.pose.velocity[0] = pose_data.velocity.x;
            data_.pose.velocity[1] = pose_data.velocity.y;
            data_.pose.velocity[2] = pose_data.velocity.z;
            data_.pose.angular_velocity[0] = pose_data.angular_velocity.x;
            data_.pose.angular_velocity[1] = pose_data.angular_velocity.y;
            data_.pose.angular_velocity[2] = pose_data.angular_velocity.z;
            data_.pose.timestamp_nanos.store(ts);
        }
        // 2.handle image frame
        else if(auto imgf = f.as<rs2::video_frame>()){
            int index = imgf.get_profile().stream_index();
            if(index == 1) {
                data_.fisheye_left.image = cv::Mat(imgf.get_height(), imgf.get_width(), CV_8UC1, (void*)imgf.get_data()).clone();
                data_.fisheye_left.timestamp_nanos.store(ts);
            }
            else if(index == 2) {
                data_.fisheye_right.image = cv::Mat(imgf.get_height(), imgf.get_width(), CV_8UC1, (void*)imgf.get_data()).clone();
                data_.fisheye_right.timestamp_nanos.store(ts);
            }
        }
        // 3.handle imu frame
        else if(auto mf = f.as<rs2::motion_frame>()){
            if(mf.get_profile().stream_type() == RS2_STREAM_GYRO && mf.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                data_.imu.gyro[0] = mf.get_motion_data().x;
                data_.imu.gyro[1] = mf.get_motion_data().y;
                data_.imu.gyro[2] = mf.get_motion_data().z;
            }
            else if(mf.get_profile().stream_type() == RS2_STREAM_ACCEL && mf.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F) {
                data_.imu.accel[0] = mf.get_motion_data().x;
                data_.imu.accel[1] = mf.get_motion_data().y;
                data_.imu.accel[2] = mf.get_motion_data().z;
            }
            data_.imu.timestamp_nanos.store(ts);
        }
    }
}
    // v 2.53.1 example

    // auto profile = pipe.start(cfg, [&](rs2::frame frame)
    // {
    //     // Cast the frame that arrived to motion frame
    //     auto motion = frame.as<rs2::motion_frame>();
    //     // If casting succeeded and the arrived frame is from gyro stream
    //     if (motion && motion.get_profile().stream_type() == RS2_STREAM_GYRO && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    //     {
    //         // Get the timestamp of the current frame
    //         double ts = motion.get_timestamp();
    //         // Get gyro measures
    //         rs2_vector gyro_data = motion.get_motion_data();
    //         // Call function that computes the angle of motion based on the retrieved measures
    //         algo.process_gyro(gyro_data, ts);
    //     }
    //     // If casting succeeded and the arrived frame is from accelerometer stream
    //     if (motion && motion.get_profile().stream_type() == RS2_STREAM_ACCEL && motion.get_profile().format() == RS2_FORMAT_MOTION_XYZ32F)
    //     {
    //         // Get accelerometer measures
    //         rs2_vector accel_data = motion.get_motion_data();
    //         // Call function that computes the angle of motion based on the retrieved measures
    //         algo.process_accel(accel_data);
    //     }
    // });