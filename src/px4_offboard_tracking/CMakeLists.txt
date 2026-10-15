cmake_minimum_required(VERSION 3.8)
project(px4_offboard_tracking)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(anti_drone_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(virtual_interception_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/RelativeEstimate.msg"
  "msg/RelativeKinematics.msg"
  "msg/TargetObservation.msg"
  "msg/AlignmentFeasibility.msg"
  "msg/MissionStatus.msg"
  "msg/TrackingQuality.msg"
  "msg/CaptureWindowStatus.msg"
  "msg/VirtualCaptureEvent.msg"
  "msg/SafetyStatus.msg"
  "msg/PureInterceptorStatus.msg"
  "msg/SimpleBelowTrackerStatus.msg"
  DEPENDENCIES geometry_msgs std_msgs)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

add_library(common_frame_transform src/common_frame_transform.cpp)
target_compile_features(common_frame_transform PUBLIC cxx_std_17)
target_include_directories(common_frame_transform PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(camera_model src/camera_model.cpp)
target_compile_features(camera_model PUBLIC cxx_std_17)
target_include_directories(camera_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(sample_time_sync src/sample_time_sync.cpp)
target_compile_features(sample_time_sync PUBLIC cxx_std_17)
target_include_directories(sample_time_sync PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(sample_time_sync camera_model)

add_library(target_observation_model src/target_observation_model.cpp)
target_compile_features(target_observation_model PUBLIC cxx_std_17)
target_include_directories(target_observation_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(relative_ekf src/relative_ekf.cpp)
target_compile_features(relative_ekf PUBLIC cxx_std_17)
target_include_directories(relative_ekf PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(relative_ekf Eigen3::Eigen)

add_library(estimator_evaluation src/estimator_evaluation.cpp)
target_compile_features(estimator_evaluation PUBLIC cxx_std_17)
target_include_directories(estimator_evaluation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(relative_kinematics_model src/relative_kinematics.cpp)
target_compile_features(relative_kinematics_model PUBLIC cxx_std_17)
target_include_directories(relative_kinematics_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(approach_corridor src/approach_corridor.cpp)
target_compile_features(approach_corridor PUBLIC cxx_std_17)
target_include_directories(approach_corridor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(mpc_guidance src/mpc_guidance.cpp)
target_compile_features(mpc_guidance PUBLIC cxx_std_17)
target_include_directories(mpc_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(terminal_guidance src/terminal_guidance.cpp)
target_compile_features(terminal_guidance PUBLIC cxx_std_17)
target_include_directories(terminal_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(z_axis_alignment src/z_axis_alignment.cpp)
target_compile_features(z_axis_alignment PUBLIC cxx_std_17)
target_include_directories(z_axis_alignment PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(attitude_setpoint_generator src/attitude_setpoint_generator.cpp)
target_compile_features(attitude_setpoint_generator PUBLIC cxx_std_17)
target_include_directories(attitude_setpoint_generator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(alignment_feasibility src/alignment_feasibility.cpp)
target_compile_features(alignment_feasibility PUBLIC cxx_std_17)
target_include_directories(alignment_feasibility PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(mission_state_machine src/mission_state_machine.cpp)
target_compile_features(mission_state_machine PUBLIC cxx_std_17)
target_include_directories(mission_state_machine PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(tracking_quality_gate src/tracking_quality_gate.cpp)
target_compile_features(tracking_quality_gate PUBLIC cxx_std_17)
target_include_directories(tracking_quality_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(capture_window src/capture_window.cpp)
target_compile_features(capture_window PUBLIC cxx_std_17)
target_include_directories(capture_window PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(safety_manager src/safety_manager.cpp)
target_compile_features(safety_manager PUBLIC cxx_std_17)
target_include_directories(safety_manager PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(body_rate_controller src/body_rate_controller.cpp)
target_compile_features(body_rate_controller PUBLIC cxx_std_17)
target_include_directories(body_rate_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(startup_search src/startup_search.cpp)
target_compile_features(startup_search PUBLIC cxx_std_17)
target_include_directories(startup_search PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(horizontal_speed_limiter src/horizontal_speed_limiter.cpp)
target_compile_features(horizontal_speed_limiter PUBLIC cxx_std_17)
target_include_directories(horizontal_speed_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(vertical_speed_limiter src/vertical_speed_limiter.cpp)
target_compile_features(vertical_speed_limiter PUBLIC cxx_std_17)
target_include_directories(vertical_speed_limiter PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(maneuver_feedforward src/maneuver_feedforward.cpp)
target_compile_features(maneuver_feedforward PUBLIC cxx_std_17)
target_include_directories(maneuver_feedforward PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(launch_position_planner src/launch_position_planner.cpp)
target_compile_features(launch_position_planner PUBLIC cxx_std_17)
target_include_directories(launch_position_planner PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(minimum_height_constraint src/minimum_height_constraint.cpp)
target_compile_features(minimum_height_constraint PUBLIC cxx_std_17)
target_include_directories(minimum_height_constraint PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(predictive_fov_barrier src/predictive_fov_barrier.cpp)
target_compile_features(predictive_fov_barrier PUBLIC cxx_std_17)
target_include_directories(predictive_fov_barrier PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(staged_range_policy src/staged_range_policy.cpp)
target_compile_features(staged_range_policy PUBLIC cxx_std_17)
target_include_directories(staged_range_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(unified_3d_guidance src/unified_3d_guidance.cpp)
target_compile_features(unified_3d_guidance PUBLIC cxx_std_17)
target_include_directories(unified_3d_guidance PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(capture_range_policy src/capture_range_policy.cpp)
target_compile_features(capture_range_policy PUBLIC cxx_std_17)
target_include_directories(capture_range_policy PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(pure_mission src/pure_mission.cpp)
target_compile_features(pure_mission PUBLIC cxx_std_17)
target_include_directories(pure_mission PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(predictive_shot src/predictive_shot.cpp)
target_compile_features(predictive_shot PUBLIC cxx_std_17)
target_include_directories(predictive_shot PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(shot_timing_optimizer src/shot_timing_optimizer.cpp)
target_compile_features(shot_timing_optimizer PUBLIC cxx_std_17)
target_include_directories(shot_timing_optimizer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(bearing_handover src/bearing_handover.cpp)
target_compile_features(bearing_handover PUBLIC cxx_std_17)
target_include_directories(bearing_handover PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_library(simple_below_tracker_model src/simple_below_tracker.cpp)
target_compile_features(simple_below_tracker_model PUBLIC cxx_std_17)
target_include_directories(simple_below_tracker_model PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

add_executable(offboard_vehicle src/offboard_vehicle.cpp)
target_compile_features(offboard_vehicle PUBLIC cxx_std_17)
target_include_directories(offboard_vehicle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(offboard_vehicle geometry_msgs nav_msgs px4_msgs rclcpp sensor_msgs)
target_link_libraries(offboard_vehicle approach_corridor mpc_guidance terminal_guidance
  attitude_setpoint_generator
  alignment_feasibility body_rate_controller capture_window common_frame_transform
  camera_model horizontal_speed_limiter mission_state_machine z_axis_alignment
  safety_manager startup_search tracking_quality_gate "${cpp_typesupport_target}")

add_executable(target_observation src/target_observation.cpp)
target_compile_features(target_observation PUBLIC cxx_std_17)
target_include_directories(target_observation PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(target_observation nav_msgs px4_msgs rclcpp std_msgs)
target_link_libraries(target_observation camera_model target_observation_model)
target_link_libraries(target_observation "${cpp_typesupport_target}")

add_executable(relative_estimator src/relative_estimator.cpp)
target_compile_features(relative_estimator PUBLIC cxx_std_17)
target_include_directories(relative_estimator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(relative_estimator anti_drone_interfaces px4_msgs rclcpp)
target_link_libraries(relative_estimator camera_model relative_ekf Eigen3::Eigen
  sample_time_sync "${cpp_typesupport_target}")

add_executable(simulation_truth_bridge src/simulation_truth_bridge.cpp)
target_compile_features(simulation_truth_bridge PUBLIC cxx_std_17)
target_include_directories(simulation_truth_bridge PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  simulation_truth_bridge geometry_msgs nav_msgs px4_msgs rclcpp sensor_msgs)
target_link_libraries(simulation_truth_bridge common_frame_transform)

add_executable(target_state_source src/target_state_source.cpp)
target_compile_features(target_state_source PUBLIC cxx_std_17)
target_include_directories(target_state_source PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  target_state_source geometry_msgs nav_msgs px4_msgs rclcpp sensor_msgs std_msgs)
target_link_libraries(target_state_source common_frame_transform)

add_executable(estimator_evaluator src/estimator_evaluator.cpp)
target_compile_features(estimator_evaluator PUBLIC cxx_std_17)
target_include_directories(estimator_evaluator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(estimator_evaluator nav_msgs rclcpp)
target_link_libraries(
  estimator_evaluator estimator_evaluation "${cpp_typesupport_target}")

add_executable(logical_camera_simulator src/logical_camera_simulator.cpp)
target_compile_features(logical_camera_simulator PUBLIC cxx_std_17)
target_include_directories(logical_camera_simulator PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  logical_camera_simulator anti_drone_interfaces nav_msgs px4_msgs rclcpp)
target_link_libraries(
  logical_camera_simulator camera_model sample_time_sync target_observation_model)

add_executable(pure_interceptor_controller src/pure_interceptor_controller.cpp)
target_compile_features(pure_interceptor_controller PUBLIC cxx_std_17)
target_include_directories(pure_interceptor_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  pure_interceptor_controller anti_drone_interfaces geometry_msgs px4_msgs rclcpp
  sensor_msgs std_msgs virtual_interception_interfaces)
target_link_libraries(
  pure_interceptor_controller bearing_handover capture_range_policy
  horizontal_speed_limiter vertical_speed_limiter maneuver_feedforward
  launch_position_planner minimum_height_constraint predictive_fov_barrier staged_range_policy
  predictive_shot shot_timing_optimizer
  pure_mission startup_search unified_3d_guidance common_frame_transform
  sample_time_sync
  "${cpp_typesupport_target}")

add_executable(relative_kinematics_node src/relative_kinematics_node.cpp)
target_compile_features(relative_kinematics_node PUBLIC cxx_std_17)
target_include_directories(relative_kinematics_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(relative_kinematics_node rclcpp)
target_link_libraries(relative_kinematics_node relative_kinematics_model
  "${cpp_typesupport_target}")

add_executable(simple_below_tracker src/simple_below_tracker_node.cpp)
target_compile_features(simple_below_tracker PUBLIC cxx_std_17)
target_include_directories(simple_below_tracker PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(
  simple_below_tracker anti_drone_interfaces geometry_msgs px4_msgs rclcpp
  sensor_msgs std_msgs)
target_link_libraries(
  simple_below_tracker simple_below_tracker_model common_frame_transform
  "${cpp_typesupport_target}")

install(TARGETS common_frame_transform camera_model sample_time_sync
  target_observation_model relative_ekf estimator_evaluation
  relative_kinematics_model approach_corridor mpc_guidance terminal_guidance
  z_axis_alignment attitude_setpoint_generator
  alignment_feasibility
  mission_state_machine
  tracking_quality_gate
  capture_window
  safety_manager
  body_rate_controller
  startup_search
  horizontal_speed_limiter
  vertical_speed_limiter
  launch_position_planner
  minimum_height_constraint
  predictive_fov_barrier
  staged_range_policy
  unified_3d_guidance
  capture_range_policy
  predictive_shot
  shot_timing_optimizer
  pure_mission
  bearing_handover
  simple_below_tracker_model
  offboard_vehicle target_observation relative_estimator
  simulation_truth_bridge target_state_source estimator_evaluator
  logical_camera_simulator pure_interceptor_controller
  relative_kinematics_node simple_below_tracker
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/trajectory_recorder.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME trajectory_recorder)
install(PROGRAMS scripts/analyze_predictive_shot_run.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/replay_fov_range_acceptance.py
  DESTINATION lib/${PROJECT_NAME})
install(PROGRAMS scripts/plot_trajectories.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_trajectories)
install(PROGRAMS scripts/plot_p7_acceptance.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME plot_p7_acceptance)
install(PROGRAMS scripts/realtime_attitude_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME realtime_attitude_monitor)
install(PROGRAMS scripts/mission_completion_monitor.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME mission_completion_monitor)
install(FILES scripts/trajectory_plotting.py DESTINATION lib/${PROJECT_NAME})
install(FILES scripts/p7_acceptance_plotting.py DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_realtime_attitude_monitor test/test_realtime_attitude_monitor.py)
  ament_add_pytest_test(test_p7_profiles test/test_p7_profiles.py)
  ament_add_pytest_test(
    test_pure_launch_contract test/test_pure_launch_contract.py)
  ament_add_pytest_test(
    test_p7_acceptance_plotting test/test_p7_acceptance_plotting.py)
  ament_add_gtest(test_tracking_controller test/test_tracking_controller.cpp)
  target_include_directories(test_tracking_controller PRIVATE include)
  target_compile_features(test_tracking_controller PUBLIC cxx_std_17)

  ament_add_gtest(test_target_trajectory test/test_target_trajectory.cpp)
  target_include_directories(test_target_trajectory PRIVATE include)
  target_compile_features(test_target_trajectory PUBLIC cxx_std_17)

  ament_add_gtest(
    test_minimum_height_constraint test/test_minimum_height_constraint.cpp)
  target_include_directories(test_minimum_height_constraint PRIVATE include)
  target_compile_features(test_minimum_height_constraint PUBLIC cxx_std_17)
  target_link_libraries(
    test_minimum_height_constraint minimum_height_constraint)

  ament_add_gtest(
    test_predictive_fov_barrier test/test_predictive_fov_barrier.cpp)
  target_include_directories(test_predictive_fov_barrier PRIVATE include)
  target_compile_features(test_predictive_fov_barrier PUBLIC cxx_std_17)
  target_link_libraries(test_predictive_fov_barrier predictive_fov_barrier)

  ament_add_gtest(
    test_launch_position_planner test/test_launch_position_planner.cpp)
  target_include_directories(test_launch_position_planner PRIVATE include)
  target_compile_features(test_launch_position_planner PUBLIC cxx_std_17)
  target_link_libraries(test_launch_position_planner launch_position_planner)

  ament_add_gtest(test_staged_range_policy test/test_staged_range_policy.cpp)
  target_include_directories(test_staged_range_policy PRIVATE include)
  target_compile_features(test_staged_range_policy PUBLIC cxx_std_17)
  target_link_libraries(test_staged_range_policy staged_range_policy)

  ament_add_gtest(test_common_frame_transform test/test_common_frame_transform.cpp)
  target_include_directories(test_common_frame_transform PRIVATE include)
  target_compile_features(test_common_frame_transform PUBLIC cxx_std_17)
  target_link_libraries(test_common_frame_transform common_frame_transform)

  ament_add_gtest(test_task_frame test/test_task_frame.cpp)
  target_include_directories(test_task_frame PRIVATE include)
  target_compile_features(test_task_frame PUBLIC cxx_std_17)

  ament_add_gtest(test_camera_model test/test_camera_model.cpp)
  target_include_directories(test_camera_model PRIVATE include)
  target_compile_features(test_camera_model PUBLIC cxx_std_17)
  target_link_libraries(test_camera_model camera_model)

  ament_add_gtest(test_sample_time_sync test/test_sample_time_sync.cpp)
  target_include_directories(test_sample_time_sync PRIVATE include)
  target_compile_features(test_sample_time_sync PUBLIC cxx_std_17)
  target_link_libraries(test_sample_time_sync sample_time_sync camera_model)

  ament_add_gtest(test_target_observation test/test_target_observation.cpp)
  target_include_directories(test_target_observation PRIVATE include)
  target_compile_features(test_target_observation PUBLIC cxx_std_17)
  target_link_libraries(test_target_observation target_observation_model)

  ament_add_gtest(test_relative_ekf test/test_relative_ekf.cpp)
  target_include_directories(test_relative_ekf PRIVATE include)
  target_compile_features(test_relative_ekf PUBLIC cxx_std_17)
  target_link_libraries(test_relative_ekf relative_ekf Eigen3::Eigen)

  ament_add_gtest(test_imm_scenarios test/test_imm_scenarios.cpp)
  target_include_directories(test_imm_scenarios PRIVATE include)
  target_compile_features(test_imm_scenarios PUBLIC cxx_std_17)
  target_link_libraries(
    test_imm_scenarios relative_ekf estimator_evaluation Eigen3::Eigen)

  ament_add_gtest(test_relative_kinematics test/test_relative_kinematics.cpp)
  target_include_directories(test_relative_kinematics PRIVATE include)
  target_compile_features(test_relative_kinematics PUBLIC cxx_std_17)
  target_link_libraries(test_relative_kinematics relative_kinematics_model)

  ament_add_gtest(test_approach_corridor test/test_approach_corridor.cpp)
  target_include_directories(test_approach_corridor PRIVATE include)
  target_compile_features(test_approach_corridor PUBLIC cxx_std_17)
  target_link_libraries(test_approach_corridor approach_corridor)

  ament_add_gtest(test_mpc_guidance test/test_mpc_guidance.cpp)
  target_include_directories(test_mpc_guidance PRIVATE include)
  target_compile_features(test_mpc_guidance PUBLIC cxx_std_17)
  target_link_libraries(test_mpc_guidance mpc_guidance)

  ament_add_gtest(test_terminal_guidance test/test_terminal_guidance.cpp)
  target_include_directories(test_terminal_guidance PRIVATE include)
  target_compile_features(test_terminal_guidance PUBLIC cxx_std_17)
  target_link_libraries(test_terminal_guidance terminal_guidance)

  ament_add_gtest(test_z_axis_alignment test/test_z_axis_alignment.cpp)
  target_include_directories(test_z_axis_alignment PRIVATE include)
  target_compile_features(test_z_axis_alignment PUBLIC cxx_std_17)
  target_link_libraries(test_z_axis_alignment z_axis_alignment)

  ament_add_gtest(test_attitude_setpoint_generator test/test_attitude_setpoint_generator.cpp)
  target_include_directories(test_attitude_setpoint_generator PRIVATE include)
  target_compile_features(test_attitude_setpoint_generator PUBLIC cxx_std_17)
  target_link_libraries(test_attitude_setpoint_generator attitude_setpoint_generator z_axis_alignment)

  ament_add_gtest(test_alignment_feasibility test/test_alignment_feasibility.cpp)
  target_include_directories(test_alignment_feasibility PRIVATE include)
  target_compile_features(test_alignment_feasibility PUBLIC cxx_std_17)
  target_link_libraries(test_alignment_feasibility alignment_feasibility)

  ament_add_gtest(test_mission_state_machine test/test_mission_state_machine.cpp)
  target_include_directories(test_mission_state_machine PRIVATE include)
  target_compile_features(test_mission_state_machine PUBLIC cxx_std_17)
  target_link_libraries(test_mission_state_machine mission_state_machine)

  ament_add_gtest(test_tracking_quality_gate test/test_tracking_quality_gate.cpp)
  target_include_directories(test_tracking_quality_gate PRIVATE include)
  target_compile_features(test_tracking_quality_gate PUBLIC cxx_std_17)
  target_link_libraries(test_tracking_quality_gate tracking_quality_gate)

  ament_add_gtest(test_capture_window test/test_capture_window.cpp)
  target_include_directories(test_capture_window PRIVATE include)
  target_compile_features(test_capture_window PUBLIC cxx_std_17)
  target_link_libraries(test_capture_window capture_window)

  ament_add_gtest(test_safety_manager test/test_safety_manager.cpp)
  target_include_directories(test_safety_manager PRIVATE include)
  target_compile_features(test_safety_manager PUBLIC cxx_std_17)
  target_link_libraries(test_safety_manager safety_manager)

  ament_add_gtest(test_body_rate_controller test/test_body_rate_controller.cpp)
  target_include_directories(test_body_rate_controller PRIVATE include)
  target_compile_features(test_body_rate_controller PUBLIC cxx_std_17)
  target_link_libraries(test_body_rate_controller body_rate_controller)

  ament_add_gtest(test_startup_search test/test_startup_search.cpp)
  target_include_directories(test_startup_search PRIVATE include)
  target_compile_features(test_startup_search PUBLIC cxx_std_17)
  target_link_libraries(test_startup_search startup_search)

  ament_add_gtest(test_horizontal_speed_limiter test/test_horizontal_speed_limiter.cpp)
  target_include_directories(test_horizontal_speed_limiter PRIVATE include)
  target_compile_features(test_horizontal_speed_limiter PUBLIC cxx_std_17)
  target_link_libraries(test_horizontal_speed_limiter horizontal_speed_limiter)

  ament_add_gtest(test_vertical_speed_limiter test/test_vertical_speed_limiter.cpp)
  target_include_directories(test_vertical_speed_limiter PRIVATE include)
  target_compile_features(test_vertical_speed_limiter PUBLIC cxx_std_17)
  target_link_libraries(test_vertical_speed_limiter vertical_speed_limiter)

  ament_add_gtest(test_maneuver_feedforward test/test_maneuver_feedforward.cpp)
  target_include_directories(test_maneuver_feedforward PRIVATE include)
  target_compile_features(test_maneuver_feedforward PUBLIC cxx_std_17)
  target_link_libraries(test_maneuver_feedforward maneuver_feedforward)

  ament_add_gtest(test_unified_3d_guidance test/test_unified_3d_guidance.cpp)
  target_include_directories(test_unified_3d_guidance PRIVATE include)
  target_compile_features(test_unified_3d_guidance PUBLIC cxx_std_17)
  target_link_libraries(test_unified_3d_guidance unified_3d_guidance)

  ament_add_gtest(test_capture_range_policy test/test_capture_range_policy.cpp)
  target_include_directories(test_capture_range_policy PRIVATE include)
  target_compile_features(test_capture_range_policy PUBLIC cxx_std_17)
  target_link_libraries(test_capture_range_policy capture_range_policy)

  ament_add_gtest(test_pure_mission test/test_pure_mission.cpp)
  target_include_directories(test_pure_mission PRIVATE include)
  target_compile_features(test_pure_mission PUBLIC cxx_std_17)
  target_link_libraries(test_pure_mission pure_mission)

  ament_add_gtest(test_predictive_shot test/test_predictive_shot.cpp)
  target_include_directories(test_predictive_shot PRIVATE include)
  target_compile_features(test_predictive_shot PUBLIC cxx_std_17)
  target_link_libraries(test_predictive_shot predictive_shot)

  ament_add_gtest(
    test_shot_timing_optimizer test/test_shot_timing_optimizer.cpp)
  target_include_directories(test_shot_timing_optimizer PRIVATE include)
  target_compile_features(test_shot_timing_optimizer PUBLIC cxx_std_17)
  target_link_libraries(test_shot_timing_optimizer shot_timing_optimizer)

  ament_add_gtest(test_bearing_handover test/test_bearing_handover.cpp)
  target_include_directories(test_bearing_handover PRIVATE include)
  target_compile_features(test_bearing_handover PUBLIC cxx_std_17)
  target_link_libraries(test_bearing_handover bearing_handover pure_mission)

  ament_add_gtest(test_simple_below_tracker test/test_simple_below_tracker.cpp)
  target_include_directories(test_simple_below_tracker PRIVATE include)
  target_compile_features(test_simple_below_tracker PUBLIC cxx_std_17)
  target_link_libraries(test_simple_below_tracker simple_below_tracker_model)

  ament_add_pytest_test(test_trajectory_plotting test/test_trajectory_plotting.py)
  ament_add_pytest_test(
    test_truth_isolation test/test_truth_isolation.py
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
  ament_add_pytest_test(
    test_interception_batch_runner test/test_interception_batch_runner.py)
endif()

ament_package()
