cmake_minimum_required(VERSION 3.8)
project(netgun_intercept)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(anti_drone_interfaces REQUIRED)
find_package(virtual_interception_interfaces REQUIRED)
find_package(px4_offboard_tracking REQUIRED)

# ---- 纯逻辑库：网枪简单模型 ----
add_library(netgun_model src/netgun_model.cpp)
target_compile_features(netgun_model PUBLIC cxx_std_17)
target_include_directories(netgun_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS netgun_model ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

# ---- 纯逻辑库：择时火控 ----
add_library(netgun_fire_control src/netgun_fire_control.cpp)
target_compile_features(netgun_fire_control PUBLIC cxx_std_17)
target_include_directories(netgun_fire_control PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
install(TARGETS netgun_fire_control ARCHIVE DESTINATION lib LIBRARY DESTINATION lib)

# ---- 打击节点 ----
add_executable(netgun_strike_node src/netgun_strike_node.cpp)
target_compile_features(netgun_strike_node PUBLIC cxx_std_17)
target_include_directories(netgun_strike_node PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>)
ament_target_dependencies(netgun_strike_node
  rclcpp px4_msgs anti_drone_interfaces virtual_interception_interfaces
  px4_offboard_tracking)
target_link_libraries(netgun_strike_node netgun_model netgun_fire_control)

# ---- 制导节点（架构 v2：三状态 + 前馈 + auto-engage） ----
add_executable(netgun_guidance_node src/netgun_guidance_node.cpp)
target_compile_features(netgun_guidance_node PUBLIC cxx_std_17)
ament_target_dependencies(netgun_guidance_node
  rclcpp geometry_msgs px4_msgs px4_offboard_tracking
  virtual_interception_interfaces std_msgs)

install(TARGETS netgun_strike_node netgun_guidance_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/run_netgun_sim.sh DESTINATION lib/${PROJECT_NAME})

ament_export_include_directories(include)
ament_export_dependencies(rclcpp px4_msgs anti_drone_interfaces
  virtual_interception_interfaces)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_netgun_model test/test_netgun_model.cpp)
  target_link_libraries(test_netgun_model netgun_model)
  ament_add_gtest(test_netgun_fire_control test/test_netgun_fire_control.cpp)
  target_link_libraries(test_netgun_fire_control netgun_fire_control)
endif()

ament_package()
