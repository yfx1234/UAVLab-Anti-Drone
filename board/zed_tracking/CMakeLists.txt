cmake_minimum_required(VERSION 3.16)
project(zed_tracking)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(zed_perception_interfaces REQUIRED)
find_package(OpenCV REQUIRED COMPONENTS core imgproc imgcodecs videoio)
find_package(CUDAToolkit REQUIRED)
find_package(OpenSSL REQUIRED)

# TensorRT(apt 安装,无 CMake 配置文件)
find_library(NVINFER_LIBRARY nvinfer REQUIRED)

set(ZED_INCLUDE_DIR "/usr/local/zed/include")
set(ZED_LIBRARY_DIR "/usr/local/zed/lib")
if(NOT EXISTS "${ZED_INCLUDE_DIR}/sl/Camera.hpp")
  message(FATAL_ERROR "ZED SDK headers not found at ${ZED_INCLUDE_DIR}")
endif()

add_executable(zed_perception_node src/zed_perception_node.cpp)
target_include_directories(zed_perception_node PRIVATE
  include
  src
  ${ZED_INCLUDE_DIR}
)
target_link_libraries(zed_perception_node
  ${NVINFER_LIBRARY}
  CUDA::cudart
  ${OpenCV_LIBS}
  ${ZED_LIBRARY_DIR}/libsl_zed.so
  ${ZED_LIBRARY_DIR}/libsl_ai.so
)
ament_target_dependencies(zed_perception_node rclcpp std_msgs)
set_target_properties(zed_perception_node PROPERTIES
  BUILD_RPATH "${ZED_LIBRARY_DIR}"
  INSTALL_RPATH "${ZED_LIBRARY_DIR}"
)

install(TARGETS zed_perception_node DESTINATION lib/${PROJECT_NAME})
add_executable(zed_perception_standalone src/zed_perception_standalone.cpp)
target_include_directories(zed_perception_standalone PRIVATE include src ${ZED_INCLUDE_DIR})
target_link_libraries(zed_perception_standalone OpenSSL::Crypto ${NVINFER_LIBRARY} CUDA::cudart ${OpenCV_LIBS}
  ${ZED_LIBRARY_DIR}/libsl_zed.so ${ZED_LIBRARY_DIR}/libsl_ai.so)
ament_target_dependencies(zed_perception_standalone rclcpp std_msgs zed_perception_interfaces)
set_target_properties(zed_perception_standalone PROPERTIES BUILD_RPATH "${ZED_LIBRARY_DIR}" INSTALL_RPATH "${ZED_LIBRARY_DIR}")
add_executable(export_svo_rgb tools/export_svo_rgb.cpp)
target_include_directories(export_svo_rgb PRIVATE include ${ZED_INCLUDE_DIR})
target_link_libraries(export_svo_rgb OpenSSL::Crypto ${OpenCV_LIBS} CUDA::cudart ${ZED_LIBRARY_DIR}/libsl_zed.so)
set_target_properties(export_svo_rgb PROPERTIES BUILD_RPATH "${ZED_LIBRARY_DIR}" INSTALL_RPATH "${ZED_LIBRARY_DIR}")
install(TARGETS zed_perception_standalone export_svo_rgb DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS tools/perception_supervisor.py tools/export_session.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/zed_track_filter_node.py
  scripts/track_hold_node.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS
  tools/mock_attitude_publisher.py
  tools/mock_vehicle_node.py
  tools/mock_perception_node.py
  tools/replay_track_check.py
  tools/sim_analyze.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  add_executable(test_perception_quality test/test_perception_quality.cpp)
  target_include_directories(test_perception_quality PRIVATE include)
  target_compile_options(test_perception_quality PRIVATE -UNDEBUG)
  add_test(NAME test_perception_quality COMMAND test_perception_quality)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_geometry test/test_geometry.cpp)
  target_include_directories(test_geometry PRIVATE include)
  ament_add_gtest(test_depth_stats test/test_depth_stats.cpp)
  target_include_directories(test_depth_stats PRIVATE include)
  ament_add_gtest(test_yolo_decode test/test_yolo_decode.cpp)
  target_include_directories(test_yolo_decode PRIVATE include)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_zed_track_filter test/test_zed_track_filter.py)
  ament_add_pytest_test(test_track_hold test/test_track_hold.py)
endif()

ament_package()
