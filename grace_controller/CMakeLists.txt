cmake_minimum_required(VERSION 3.5)
project(grace_controller)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

ament_python_install_package(${PROJECT_NAME})

install(
  DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  ${PROJECT_NAME}/pitch_monitor.py
  ${PROJECT_NAME}/hoverboard_encoder_simulator.py
  ${PROJECT_NAME}/lqr_balancing_controller.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()