#include "fake_hw_interface/fake_system.hpp"
#include <pluginlib/class_list_macros.hpp>

using namespace fake_hw_interface;

hardware_interface::CallbackReturn FakeSystem::on_init(const hardware_interface::HardwareInfo & info)
{
  // call base on_init if needed (some demos do)
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    RCLCPP_ERROR(logger_, "SystemInterface on_init failed");
    return hardware_interface::CallbackReturn::ERROR;
  }

  // store joints count and allocate arrays
  hw_joints_ = info_.joints.size();
  hw_positions_.assign(hw_joints_, 0.0);
  hw_velocities_.assign(hw_joints_, 0.0);
  hw_efforts_.assign(hw_joints_, 0.0);
  hw_commands_.assign(hw_joints_, 0.0);

  RCLCPP_INFO(logger_, "FakeSystem on_init: configured %zu joints", hw_joints_);
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeSystem::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "FakeSystem on_configure");
  // reset states
  for (size_t i = 0; i < hw_joints_; ++i) {
    hw_positions_[i] = 0.0;
    hw_velocities_[i] = 0.0;
    hw_efforts_[i] = 0.0;
    hw_commands_[i] = 0.0;
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeSystem::on_activate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "FakeSystem on_activate");
  // make command == state on activate
  for (size_t i = 0; i < hw_joints_; ++i) {
    hw_commands_[i] = hw_positions_[i];
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn FakeSystem::on_deactivate(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(logger_, "FakeSystem on_deactivate");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> FakeSystem::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < hw_joints_; ++i) {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> FakeSystem::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < hw_joints_; ++i) {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type FakeSystem::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 简单模拟：当前位置向命令值靠近（以一个固定速度或简单积分）
  for (size_t i = 0; i < hw_joints_; ++i) {
    double delta = hw_commands_[i] - hw_positions_[i];
    hw_velocities_[i] = delta * 0.5;  // 简易模型
    hw_positions_[i] += hw_velocities_[i] * (1.0 / 50.0); // 假设 50Hz 更新
    hw_efforts_[i] = 0.0;
  }
  RCLCPP_DEBUG(logger_, "FakeSystem read() done");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type FakeSystem::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // 在真实硬件这里会把 hw_commands_ 发出去（CAN/串口/Ethernet）
  for (size_t i = 0; i < hw_joints_; ++i) {
    RCLCPP_DEBUG(logger_, "Joint %zu: cmd=%.3f pos=%.3f", i, hw_commands_[i], hw_positions_[i]);
  }
  return hardware_interface::return_type::OK;
}

// 注册为 plugin
PLUGINLIB_EXPORT_CLASS(fake_hw_interface::FakeSystem, hardware_interface::SystemInterface)

