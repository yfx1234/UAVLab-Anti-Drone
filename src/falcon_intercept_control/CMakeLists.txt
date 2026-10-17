cmake_minimum_required(VERSION 3.8)
project(falcon_intercept_control)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(falcon_intercept_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(${PROJECT_NAME}_core src/control_core.cpp)
target_compile_features(${PROJECT_NAME}_core PUBLIC cxx_std_17)
target_include_directories(${PROJECT_NAME}_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(real_flight_core src/real_flight_core.cpp)
target_compile_features(real_flight_core PUBLIC cxx_std_17)
target_include_directories(real_flight_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

option(BUILD_LEGACY_DECISION_NODE
  "Build the deprecated monolithic FRD decision node (never for production)" OFF)
if(BUILD_LEGACY_DECISION_NODE)
  add_executable(intercept_decision_node src/intercept_decision_node.cpp)
  target_compile_features(intercept_decision_node PUBLIC cxx_std_17)
  target_link_libraries(intercept_decision_node ${PROJECT_NAME}_core)
  ament_target_dependencies(intercept_decision_node
    falcon_intercept_msgs geometry_msgs rclcpp std_msgs)
endif()

foreach(node search_controller_node guidance_node intercept_fsm_node safety_node)
  add_executable(${node} src/${node}.cpp)
  target_compile_features(${node} PUBLIC cxx_std_17)
  target_link_libraries(${node} real_flight_core)
  ament_target_dependencies(${node} falcon_intercept_msgs geometry_msgs rclcpp std_msgs)
endforeach()

install(TARGETS ${PROJECT_NAME}_core real_flight_core
  search_controller_node guidance_node intercept_fsm_node safety_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
if(BUILD_LEGACY_DECISION_NODE)
  install(TARGETS intercept_decision_node RUNTIME DESTINATION lib/${PROJECT_NAME})
endif()
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS tools/compare_shadow_outputs.py
  DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_control_core test/test_control_core.cpp)
  target_link_libraries(test_control_core ${PROJECT_NAME}_core)
  target_include_directories(test_control_core PRIVATE include)
  ament_add_gtest(test_real_flight_core test/test_real_flight_core.cpp)
  target_link_libraries(test_real_flight_core real_flight_core)
  target_include_directories(test_real_flight_core PRIVATE include)
  ament_add_pytest_test(test_shadow_compare test/test_shadow_compare.py)
endif()

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME}_core real_flight_core)
ament_export_dependencies(falcon_intercept_msgs geometry_msgs rclcpp std_msgs)
ament_package()
