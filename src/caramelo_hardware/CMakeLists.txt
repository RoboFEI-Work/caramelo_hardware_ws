cmake_minimum_required(VERSION 3.8)
project(caramelo_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pal_statistics REQUIRED)
find_package(pluginlib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(dynamixel_sdk REQUIRED)

include(CheckIncludeFileCXX)
check_include_file_cxx(lgpio.h HAVE_LGPIO_H)
find_library(LGPIO_LIB lgpio)

include_directories(include)

add_library(
  ${PROJECT_NAME}
  SHARED
  src/arm_hw_interface.cpp
  src/mobile_base_hw_interface.cpp
  src/maxon_motors_node.cpp
)

ament_target_dependencies(
  ${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pal_statistics
  pluginlib
  std_msgs
  dynamixel_sdk
)

if(HAVE_LGPIO_H AND LGPIO_LIB)
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_LGPIO=1)
  target_link_libraries(${PROJECT_NAME} ${LGPIO_LIB})
  message(STATUS "caramelo_hardware: lgpio backend enabled (Raspberry Pi 5 / RP1).")
else()
  target_compile_definitions(${PROJECT_NAME} PRIVATE CARAMELO_HAS_LGPIO=0)
  message(WARNING "caramelo_hardware: lgpio headers/libs not found (apt install liblgpio-dev). GPIO backend disabled for this build.")
endif()

pluginlib_export_plugin_description_file(hardware_interface caramelo_hw_interface.xml)

install(
  TARGETS ${PROJECT_NAME}
  DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES caramelo_hw_interface.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(
  rclcpp
  rclcpp_lifecycle
  hardware_interface
  pal_statistics
  pluginlib
  std_msgs
  dynamixel_sdk
)

ament_package()
