cmake_minimum_required(VERSION 3.8)
project(grace_firmware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(grace_msg REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(PkgConfig)
pkg_check_modules(SERIAL libserial)

ament_python_install_package(${PROJECT_NAME})

add_library(${PROJECT_NAME} SHARED 
  src/grace_interface.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE 
  include
  "${SERIAL_INCLUDE_DIRS}"
)
target_link_libraries(${PROJECT_NAME}
 "${SERIAL_LDFLAGS}")
ament_target_dependencies(${PROJECT_NAME}
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  grace_msg
)

pluginlib_export_plugin_description_file(hardware_interface grace_interface.xml)

# Kt calibration tool
add_executable(kt_calibration_node src/kt_calibration_node.cpp)
target_include_directories(kt_calibration_node PRIVATE include)
ament_target_dependencies(kt_calibration_node
  rclcpp
  std_msgs
  grace_msg
)

install(
  TARGETS ${PROJECT_NAME} kt_calibration_node
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include
  DESTINATION include
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  ${PROJECT_NAME}
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
  sensor_msgs
  std_msgs
  grace_msg
)

ament_package()
