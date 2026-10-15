cmake_minimum_required(VERSION 3.8)
project(px4_interceptor_bridge)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(anti_drone_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(interceptor_bridge_geometry src/bridge_geometry.cpp)
target_compile_features(interceptor_bridge_geometry PUBLIC cxx_std_17)
target_include_directories(interceptor_bridge_geometry PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(interceptor_bridge_geometry Eigen3::Eigen)

add_executable(interceptor_bridge_node src/interceptor_bridge_node.cpp)
target_compile_features(interceptor_bridge_node PUBLIC cxx_std_17)
target_link_libraries(interceptor_bridge_node interceptor_bridge_geometry Eigen3::Eigen)
ament_target_dependencies(interceptor_bridge_node
  anti_drone_interfaces px4_msgs rclcpp)

install(TARGETS interceptor_bridge_geometry interceptor_bridge_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  ament_add_gtest(test_bridge_geometry test/test_bridge_geometry.cpp)
  target_link_libraries(test_bridge_geometry interceptor_bridge_geometry Eigen3::Eigen)
  ament_add_gtest(test_timestamp_offset_estimator test/test_timestamp_offset_estimator.cpp)
  target_include_directories(test_timestamp_offset_estimator PRIVATE include)
endif()

ament_package()
