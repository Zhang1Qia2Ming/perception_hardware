#ifndef _PERCEPTION_HARDWARE_DEVICE_BASE_HPP_
#define _PERCEPTION_HARDWARE_DEVICE_BASE_HPP_

#include <string>
#include <memory>
#include <vector>
#include <map>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"

#include "rclcpp/rclcpp.hpp"

namespace perception_hardware
{

class DeviceBase {
public:
  virtual ~DeviceBase() = default;
  
  // initial
  virtual bool on_init(const hardware_interface::ComponentInfo & info) = 0;

  virtual void read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual void write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;

  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

  virtual std::string get_name() const = 0;

};



} // namespace perception_hardware



#endif
