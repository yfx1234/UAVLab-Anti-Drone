cmake_minimum_required(VERSION 3.8)
project(falcon_intercept_camera)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
pkg_check_modules(GSTREAMER REQUIRED IMPORTED_TARGET
  gstreamer-1.0
  gstreamer-app-1.0
  gstreamer-video-1.0
)

add_executable(argus_camera_node src/argus_camera_node.cpp)
ament_target_dependencies(argus_camera_node rclcpp sensor_msgs)
target_link_libraries(argus_camera_node PkgConfig::GSTREAMER)

add_executable(usb_mjpeg_camera_node src/usb_mjpeg_camera_node.cpp)
ament_target_dependencies(usb_mjpeg_camera_node rclcpp sensor_msgs)
target_link_libraries(usb_mjpeg_camera_node PkgConfig::GSTREAMER)

install(TARGETS
  argus_camera_node
  usb_mjpeg_camera_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
