#include "perception_hardware/perception_system.hpp"
#include "perception_hardware/device_factory.hpp"
#include "perception_hardware/base_types.hpp"

namespace perception_hardware
{

hardware_interface::CallbackReturn PerceptionSystem::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if(
        // put info into info_
        hardware_interface::SystemInterface::on_init(info) != 
        hardware_interface::CallbackReturn::SUCCESS
    )
    {
        return hardware_interface::CallbackReturn::ERROR;
    }

    // iterate over each component of sensors
    // each sensor_info's structure is hardware_interface::ComponentInfo
    for(const hardware_interface::ComponentInfo & sensor_info : info_.sensors)
    {
        auto it = sensor_info.parameters.find("device_type");

        if(it == sensor_info.parameters.end()){
            RCLCPP_ERROR(rclcpp::get_logger("PerceptionSystem"), "Sensor %s does not have device_type parameter", sensor_info.name.c_str());
            continue;
        }

        std::string device_type = sensor_info.parameters.at("device_type");

        auto device = DeviceFactory::create(it->second);

        if(device){
            if(device->on_init(sensor_info)){
                devices_.push_back(std::move(device));
                RCLCPP_INFO(rclcpp::get_logger("PerceptionSystem"), "Sensor %s initialized", sensor_info.name.c_str());
            }
            else{
                RCLCPP_ERROR(rclcpp::get_logger("PerceptionSystem"), "Sensor %s failed to initialize", sensor_info.name.c_str());
            }
        }

    }

    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerceptionSystem::on_configure(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerceptionSystem::on_activate(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn PerceptionSystem::on_deactivate(
    const rclcpp_lifecycle::State & previous_state)
{
    return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> PerceptionSystem::export_state_interfaces()
{
    std::vector<hardware_interface::StateInterface> state_interfaces;

    for(auto & device : devices_){
        auto dev_state_interfaces = device->export_state_interfaces();
        
        for(auto & state_interface : dev_state_interfaces){
            state_interfaces.emplace_back(std::move(state_interface));
        }
    }
    
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> PerceptionSystem::export_command_interfaces()
{
    std::vector<hardware_interface::CommandInterface> command_interfaces;

    for(auto & device : devices_){
        auto dev_command_interfaces = device->export_command_interfaces();
        for(auto & command_interface : dev_command_interfaces){
            command_interfaces.emplace_back(std::move(command_interface));
        }
    }
    return command_interfaces;
}

hardware_interface::return_type PerceptionSystem::read(
    const rclcpp::Time & time, const rclcpp::Duration & period)
{
    for(auto & device : devices_){
        device->read(time, period);
    }
    
    return hardware_interface::return_type::OK;
}


} // namespace perception_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(perception_hardware::PerceptionSystem, hardware_interface::SystemInterface)
