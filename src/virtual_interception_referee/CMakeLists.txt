cmake_minimum_required(VERSION 3.8)
project(virtual_interception_referee)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(virtual_interception_interfaces REQUIRED)

add_library(referee_model src/referee_model.cpp)
target_compile_features(referee_model PUBLIC cxx_std_17)
target_include_directories(referee_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(virtual_projectile_referee src/virtual_projectile_referee.cpp)
target_compile_features(virtual_projectile_referee PUBLIC cxx_std_17)
target_include_directories(virtual_projectile_referee PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  virtual_projectile_referee geometry_msgs nav_msgs px4_msgs rclcpp std_msgs
  virtual_interception_interfaces)
target_link_libraries(virtual_projectile_referee referee_model)

install(TARGETS referee_model virtual_projectile_referee
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_referee_model test/test_referee_model.cpp)
  target_include_directories(test_referee_model PRIVATE include)
  target_compile_features(test_referee_model PUBLIC cxx_std_17)
  target_link_libraries(test_referee_model referee_model)
endif()

ament_package()
