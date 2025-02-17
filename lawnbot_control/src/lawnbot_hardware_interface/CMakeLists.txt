cmake_minimum_required(VERSION 3.8)
project(lawnbot_hardware_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav_msgs REQUIRED)

# Add library
add_library(${PROJECT_NAME} SHARED src/LawnbotHardwareInterface.cpp)

# Link dependencies
ament_target_dependencies(${PROJECT_NAME} rclcpp hardware_interface pluginlib nav_msgs)

# Export plugin
pluginlib_export_plugin_description_file(hardware_interface config/lawnbot_hardware_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

ament_package()
