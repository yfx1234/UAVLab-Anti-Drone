cmake_minimum_required(VERSION 3.8)
project(offboard_route_offset VERSION 3.0.0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(offboard_route_offset_node src/offboard_route_offset_node.cpp)
target_compile_features(offboard_route_offset_node PUBLIC cxx_std_17)
target_compile_definitions(
  offboard_route_offset_node
  PRIVATE OFFBOARD_ROUTE_OFFSET_VERSION="${PROJECT_VERSION}"
)
ament_target_dependencies(offboard_route_offset_node rclcpp px4_msgs std_msgs)

install(TARGETS offboard_route_offset_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch docs
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/analyze_offset_metrics.py
  scripts/collect_offset_metrics.py
  scripts/plot_velocity_path_comparison.py
  scripts/tune_route_offset.sh
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
