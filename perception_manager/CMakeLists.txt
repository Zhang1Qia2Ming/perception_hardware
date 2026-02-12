cmake_minimum_required(VERSION 3.16)
project(perception_hardware)

# 1. 基础依赖
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)

# 2. 传感器驱动所需依赖
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sensor_msgs REQUIRED)

set(REALSENSE2_LIBRARY "/usr/local/lib/librealsense2.so")
set(REALSENSE2_INCLUDE_DIRS "/usr/local/include")

# 3. 编译硬件接口库
# 注意：如果你的 MockCamera 或 DeviceFactory 有单独的 .cpp 文件，也要加在这里
add_library(${PROJECT_NAME} SHARED
  src/perception_hardware/perception_system.cpp
  src/perception_hardware/t265_camera_device.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${REALSENSE2_INCLUDE_DIRS}
)

# 4. 依赖注入
ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  #realsense2_camera_msgs
  pluginlib
  OpenCV
  cv_bridge
  sensor_msgs
)

target_link_libraries(${PROJECT_NAME} ${REALSENSE2_LIBRARY})

# 5. 导出插件描述文件
pluginlib_export_plugin_description_file(
  hardware_interface
  perception_hardware_plugins.xml
)

# 6. 安装规则
install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES perception_hardware_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_dependencies(hardware_interface pluginlib rclcpp)
ament_package()
