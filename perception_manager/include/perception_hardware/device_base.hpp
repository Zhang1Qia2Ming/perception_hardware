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


  //async function
  virtual void start() {
    if(!is_running_.exchange(true)) {
      return;
    }
    work_thread_ = std::thread(&DeviceBase::run_loop, this);
  }

  virtual void run_loop() = 0;

  virtual void stop() {
    if(!is_running_.exchange(false)) {
      return;
    }
    if(work_thread_.joinable()) {
      work_thread_.join();
    }
  }

  
  // read & write data
  virtual void read(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual void write(const rclcpp::Time & time, const rclcpp::Duration & period) = 0;

  virtual std::vector<hardware_interface::StateInterface> export_state_interfaces() = 0;

  virtual std::vector<hardware_interface::CommandInterface> export_command_interfaces() = 0;

  virtual std::string get_name() const = 0;

protected:
  std::atomic<bool> is_running_{false};
  std::thread work_thread_;

};



} // namespace perception_hardware



#endif
