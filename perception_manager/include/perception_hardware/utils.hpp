#ifndef PERCEPTION_HARDWARE_UTILS_HPP_
#define PERCEPTION_HARDWARE_UTILS_HPP_

#include <string>
#include <vector>

#include "perception_hardware/base_types.hpp"

namespace perception_hardware {

inline std::vector<std::string> split(const std::string &str, char delimiter){
    std::vector<std::string> tokens;
    std::string token;
    std::istringstream tokenStream(str);
    while(std::getline(tokenStream, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

inline void add_pub_from_sensor_component_info(
    std::shared_ptr<BaseMember> member_ptr)
{
    for(auto& sensor_component : member_ptr->sensor_components){
        
        // [type0_topic0, type1_topic1, ...]
        size_t pos = sensor_component.find('_');
        if(pos == std::string::npos || pos == sensor_component.size()-1){
            RCLCPP_ERROR(rclcpp::get_logger("ERROR"),"Invalid sensor_component format: %s", sensor_component.c_str());
            continue;
        }       
        std::string type = sensor_component.substr(0, pos);
        std::string topic = sensor_component.substr(pos+1);

        // if(type == "image"){
        //     member_ptr->pub_raw[topic] = get_node()->create_publisher<sensor_msgs::msg::Image>(topic, 10);                    
        // }
        // else if (type == "pose")
        // {
        //     member_ptr->pub_pose[topic] = get_node()->create_publisher<geometry_msgs::msg::PoseStamped>(topic, 10);
        // }
        // else if (type == "imu")
        // {
        //     member_ptr->pub_imu[topic] = get_node()->create_publisher<sensor_msgs::msg::Imu>(topic, 10);
        // }
    }    
}


// struct BaseMember {
//         std::string name;
//         std::string device_type;
//         std::vector<std::string> sensor_components;
//         std::string interface_name;
//         std::string frame_id;
//         bool enable = false;

//         std::map<std::string, uint64_t> last_ts;
//         std::unordered_map<std::string, std::shared_ptr<TopicSync>> topic_sync_map;

//         using ImagePub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Image>::SharedPtr;
//         using ImuPub = rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::Imu>::SharedPtr;
//         using PosePub = rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::PoseStamped>::SharedPtr;
        
//         // name, publisher
//         std::map<std::string, ImagePub> pub_raw;
//         std::map<std::string, ImagePub> pub_rect;
//         std::map<std::string, ImuPub> pub_imu;
//         std::map<std::string, PosePub> pub_pose;

//         void add_topic_sync(const std::string & topic_name) {
//                 topic_sync_map[topic_name] = std::make_shared<TopicSync>();
//         }
// };

inline void print_info_in_member(
    std::shared_ptr<BaseMember> member_ptr)
{
    RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.name: %s", member_ptr->name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.device_type: %s", member_ptr->device_type.c_str());
    RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.interface_name: %s", member_ptr->interface_name.c_str());
    RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.frame_id: %s", member_ptr->frame_id.c_str());
    RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.enable: %d", member_ptr->enable);
    for(auto& sensor_component : member_ptr->sensor_components){
        RCLCPP_INFO(rclcpp::get_logger("INFO"),"Member.sensor_component: %s", sensor_component.c_str());
    }
    
}



} // namespace perception_hardware

#endif // PERCEPTION_HARDWARE_UTILS_HPP_