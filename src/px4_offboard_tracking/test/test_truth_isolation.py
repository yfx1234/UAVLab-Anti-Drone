from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_production_controller_has_no_simulation_truth_dependency():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    forbidden = (
        "target_truth",
        "relative_truth",
        "/evaluation/",
        "simulation_truth_bridge",
        "nav_msgs/msg/odometry",
        "virtual_interception_referee",
        "virtual_hit_result",
    )
    for token in forbidden:
        assert token not in controller
    assert "/target/fmu/out/vehicle_global_position" not in controller


def test_real_launch_starts_no_simulation_or_truth_node():
    launch = (ROOT / "launch" / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    forbidden = (
        "logical_camera_simulator",
        "simulation_truth_bridge",
        "/evaluation/",
        "target_state",
        "gazebo",
        "virtual_interception_referee",
        "virtual_projectile_referee",
    )
    for token in forbidden:
        assert token not in launch


def test_simple_below_tracker_production_path_has_no_target_truth_dependency():
    controller = (ROOT / "src" / "simple_below_tracker_node.cpp").read_text(
        encoding="utf-8"
    )
    real_launch = (
        ROOT / "launch" / "simple_below_tracker_real.launch.py"
    ).read_text(encoding="utf-8")
    forbidden = (
        "relative_truth",
        "/evaluation/",
        "simulation_truth_bridge",
        "nav_msgs/msg/odometry",
        "logical_camera_simulator",
        "virtual_interception_referee",
        "virtual_hit_result",
    )
    for source in (controller, real_launch):
        for token in forbidden:
            assert token not in source
    assert "/target/fmu/out/vehicle_global_position" not in controller

    assert "DepthObservation" in controller
    assert "out/vehicle_local_position" in controller
    assert "out/vehicle_attitude" in controller


def test_simple_real_tracker_requires_manual_entry_and_stops_after_manual_exit():
    controller = (ROOT / "src" / "simple_below_tracker_node.cpp").read_text(
        encoding="utf-8"
    )
    real_launch = (
        ROOT / "launch" / "simple_below_tracker_real.launch.py"
    ).read_text(encoding="utf-8")

    assert 'DeclareLaunchArgument("control_enabled", default_value="true")' in real_launch
    assert 'DeclareLaunchArgument("auto_engage", default_value="false")' in real_launch
    assert '"auto_disable_on_offboard_exit", default_value="true"' in real_launch
    assert '"takeoff_enabled": False' in real_launch

    timer = controller.split("void on_timer()", 1)[1].split(
        "std::string topic_prefix_", 1
    )[0]
    assert timer.index("update_manual_offboard_session();") < timer.index(
        "publish_offboard_mode();"
    )
    assert "if (heartbeat_streaming())" in timer
    assert "if (setpoint_streaming())" in timer
    assert timer.index("publish_offboard_mode();") < timer.index(
        "publish_trajectory_setpoint(tracker_result);"
    )
    assert (
        "return heartbeat_streaming() && (offboard_active() || offboard_session_seen_);"
        in controller
    )
    assert "flight_control_active()" in timer
    assert "control_enabled_ = false;" in controller
    assert "stopped_after_offboard_exit_ = true;" in controller
    assert "!offboard_session_seen_" in controller
    assert "const bool explicit_offboard_exit" in controller
    assert "offboard_session_seen_ && explicit_offboard_exit" in controller


def test_simple_sim_launch_exposes_manual_pursuer_rehearsal():
    launch = (
        ROOT / "launch" / "simple_below_tracker_sim.launch.py"
    ).read_text(encoding="utf-8")

    defaults = (
        '"target_wait_for_pursuer_takeoff_ready", default_value="false"',
        '"pursuer_control_enabled", default_value="true"',
        '"pursuer_auto_engage", default_value="true"',
        '"pursuer_takeoff_enabled", default_value="true"',
        '"pursuer_auto_disable_on_offboard_exit", default_value="false"',
    )
    for declaration in defaults:
        assert declaration in launch

    compact = "".join(launch.split())
    for name in (
        "target_wait_for_pursuer_takeoff_ready",
        "pursuer_control_enabled",
        "pursuer_auto_engage",
        "pursuer_takeoff_enabled",
        "pursuer_auto_disable_on_offboard_exit",
    ):
        assert f'LaunchConfiguration("{name}")' in compact


def test_logical_cameras_are_two_independent_publishers():
    source = (ROOT / "src" / "logical_camera_simulator.cpp").read_text(
        encoding="utf-8"
    )
    assert 'camera_output must be \'search\' or \'depth\'' in source
    assert "if (camera_output_ == LogicalCameraOutput::SEARCH)" in source

    for launch_name in (
        "simple_below_tracker_sim.launch.py",
        "pure_interceptor_sim.launch.py",
    ):
        launch = (ROOT / "launch" / launch_name).read_text(encoding="utf-8")
        assert launch.count('executable="logical_camera_simulator"') == 2
        assert 'name="logical_search_camera_simulator"' in launch
        assert 'name="logical_depth_camera_simulator"' in launch
        assert '"camera_output": "search"' in launch
        assert '"camera_output": "depth"' in launch
        assert '"search_camera_rate_hz", default_value="30.0"' in launch
        assert '"depth_camera_rate_hz", default_value="30.0"' in launch


def test_logical_camera_truth_and_attitude_share_ros_receipt_clock():
    source = (ROOT / "src" / "logical_camera_simulator.cpp").read_text(
        encoding="utf-8"
    )
    callback = source.split(
        "attitude_sub_ = create_subscription", 1
    )[1].split(
        "const auto camera_qos", 1
    )[0]
    assert "attitude_buffer_.add(receipt_time.seconds(), attitude);" in callback
    assert "px4_sample_or_receipt_time" not in callback
    assert "simulation_truth_bridge stamps synthetic truth in the ROS clock" in callback


def test_pure_interceptor_has_automatic_and_manual_entrypoints():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    real_launch = (
        ROOT / "launch" / "pure_interceptor_real.launch.py"
    ).read_text(encoding="utf-8")

    assert "update_manual_offboard_session();" in controller
    assert "begin_first_offboard_session();" in controller
    assert "capture_count_ = 0;" in controller
    assert "mission_.reset(session_start.seconds());" in controller
    assert "rejected enable after Offboard exit" in controller
    assert "armed() && offboard_active() && task_origin_locked_" in controller
    assert "if (heartbeat_streaming())" in controller
    assert "if (setpoint_streaming())" in controller
    assert (
        "return heartbeat_streaming() && (offboard_active() || offboard_session_seen_);"
        in controller
    )
    assert "if (!auto_engage_)" in controller
    assert "stopped_after_offboard_exit_ = true;" in controller
    assert "const bool explicit_offboard_exit" in controller
    assert "offboard_session_seen_ && explicit_offboard_exit" in controller
    assert 'DeclareLaunchArgument("control_enabled", default_value="true")' in real_launch
    assert 'DeclareLaunchArgument("auto_engage", default_value="false")' in real_launch
    assert 'DeclareLaunchArgument("takeoff_enabled", default_value="false")' in real_launch
    assert '"auto_disable_on_offboard_exit", default_value="true"' in real_launch

    expected = (
        "simple_below_tracker_manual_sim.launch.py",
        "pure_interceptor_manual_sim.launch.py",
        "p7_interceptor_manual_sim.launch.py",
        "pure_interceptor_manual_real.launch.py",
        "coordinate_camera_manual_real.launch.py",
        "pure_interceptor_auto_real.launch.py",
    )
    for name in expected:
        assert (ROOT / "launch" / name).is_file()

    for name in (
        "simple_below_tracker_manual_sim.launch.py",
        "pure_interceptor_manual_sim.launch.py",
        "p7_interceptor_manual_sim.launch.py",
        "pure_interceptor_manual_real.launch.py",
    ):
        source = (ROOT / "launch" / name).read_text(encoding="utf-8")
        assert '"control_enabled": "true"' in source or (
            '"pursuer_control_enabled": "true"' in source
        )


def test_full_real_entrypoint_exposes_airframe_thrust_calibration():
    real_launch = (ROOT / "launch" / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    manual_launch = (
        ROOT / "launch" / "pure_interceptor_manual_real.launch.py"
    ).read_text(encoding="utf-8")

    for parameter in ("hover_thrust", "maximum_thrust"):
        assert f'DeclareLaunchArgument("{parameter}"' in real_launch
        assert f'"{parameter}": LaunchConfiguration("{parameter}")' in real_launch
        assert f'DeclareLaunchArgument("{parameter}"' in manual_launch
        assert f'"{parameter}": LaunchConfiguration("{parameter}")' in manual_launch


def test_real_interceptor_launches_never_control_the_enemy_aircraft():
    forbidden = (
        "target_prefix",
        "target_state_topic",
        "target_system_topic",
        "target_wait_for",
        "simulation_truth_bridge",
        "logical_camera_simulator",
        "/evaluation/",
    )
    for path in (ROOT / "launch").glob("*real.launch.py"):
        source = path.read_text(encoding="utf-8")
        for token in forbidden:
            assert token not in source, f"{path.name} contains {token}"


def test_only_simulation_side_reads_evaluation_truth():
    production_files = (
        ROOT / "src" / "pure_interceptor_controller.cpp",
        ROOT / "src" / "relative_estimator.cpp",
        ROOT / "launch" / "pure_interceptor_real.launch.py",
    )
    for path in production_files:
        source = path.read_text(encoding="utf-8")
        assert "/evaluation/" not in source
        assert "relative_truth" not in source
        assert "target_truth" not in source


def test_coordinate_driven_camera_path_is_test_only_and_camera_isolated():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    real_launch = (ROOT / "launch" / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    sim_launch = (ROOT / "launch" / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    source = (ROOT / "src" / "target_state_source.cpp").read_text(
        encoding="utf-8"
    )

    for production in (controller, real_launch):
        assert "target_state_source" not in production
        assert "external_target_global_topic" not in production
    assert 'executable="target_state_source"' in sim_launch
    assert '"target_source": target_source' in sim_launch
    assert '"truth_topic": truth_topic' in sim_launch
    assert "The production estimator/controller never subscribe" in source
    assert "/target/fmu/out/vehicle_global_position" in source
    assert "TargetTrajectory" in source


def test_coordinate_camera_entrypoints_define_both_offboard_takeover_modes():
    automatic = (
        ROOT / "launch" / "coordinate_camera_auto.launch.py"
    ).read_text(encoding="utf-8")
    manual = (
        ROOT / "launch" / "coordinate_camera_manual.launch.py"
    ).read_text(encoding="utf-8")

    assert '"target_source", default_value="virtual"' in automatic
    assert '"pursuer_auto_engage", default_value="true"' in automatic
    assert '"pursuer_takeoff_enabled", default_value="true"' in automatic
    assert '"pursuer_auto_disable_on_offboard_exit", default_value="false"' in automatic
    assert '"target_source", default_value="external_px4"' in manual
    assert '"pursuer_auto_engage", default_value="false"' in manual
    assert '"pursuer_takeoff_enabled", default_value="false"' in manual
    assert (
        '"pursuer_auto_disable_on_offboard_exit", default_value="true"'
        in manual
    )
    for launch in (automatic, manual):
        assert 'executable="virtual_projectile_referee"' in launch
        assert '"/fmu/out/vehicle_local_position"' in launch
        assert '"target_speed_mps", default_value="2.0"' in launch
        assert '"target_heading_rad", default_value="3.141592653589793"' in launch
        assert '"target_radius_m", default_value="20.0"' in launch
        assert '"target_path_length_m", default_value="40.0"' in launch
        assert '"target_path_width_m", default_value="12.0"' in launch
        assert '"target_height_m", default_value="25.0"' in launch
        assert '"target_motion_ramp_s", default_value="0.0"' in launch
        assert '"first_attack_tracking_hold_s", default_value="3.0"' in launch
        assert '"terminal_alignment_tracking_hold_s", default_value="2.0"' in launch
        assert '"terminal_alignment_ramp_s", default_value="1.0"' in launch
        assert '"command_maximum_speed_mps", default_value="12.0"' in launch
        assert '"safety_maximum_speed_mps", default_value="14.0"' in launch
        assert '"attitude_gain", default_value="3.0"' in launch
        assert '"maximum_body_rate_rad_s", default_value="3.0"' in launch
        assert '"transverse_velocity_gain", default_value="1.0"' in launch
        assert '"virtual_target_initial_east_m", default_value="0.0"' in launch
        assert (
            '"virtual_target_spawn_relative_to_pursuer_on_ready",'
            in launch
        )
        assert (
            '"virtual_target_publish_before_pursuer_ready",'
            in launch
        )
        assert (
            '"virtual_target_use_pursuer_local_position",'
            in launch
        )
        assert '"truth_pursuer_position_source", default_value="local"' in launch


def test_real_virtual_target_entry_is_manual_gated_and_requires_thrust():
    launch = (
        ROOT / "launch" / "coordinate_camera_manual_real.launch.py"
    ).read_text(encoding="utf-8")
    manual = (
        ROOT / "launch" / "coordinate_camera_manual.launch.py"
    ).read_text(encoding="utf-8")
    sim = (
        ROOT / "launch" / "pure_interceptor_sim.launch.py"
    ).read_text(encoding="utf-8")
    source = (ROOT / "src" / "target_state_source.cpp").read_text(
        encoding="utf-8"
    )
    truth_bridge = (
        ROOT / "src" / "simulation_truth_bridge.cpp"
    ).read_text(encoding="utf-8")

    assert '"coordinate_camera_manual.launch.py"' in launch
    assert '"hover_thrust",\n                description=' in launch
    assert '"maximum_thrust",\n                description=' in launch
    assert '"target_source": "virtual"' in launch
    assert (
        '"virtual_target_initial_north_m", default_value="0.0"'
        in launch
    )
    assert (
        '"virtual_target_initial_east_m", default_value="0.0"'
        in launch
    )
    assert (
        '"virtual_target_initial_north_m": LaunchConfiguration('
        in launch
    )
    assert (
        '"virtual_target_initial_east_m": LaunchConfiguration('
        in launch
    )
    assert '"target_height_m": "25.0"' in launch
    assert (
        '"target_height_m": LaunchConfiguration("target_height_m")'
        not in launch
    )
    assert '"virtual_target_spawn_relative_to_pursuer_on_ready": "true"' in launch
    assert '"virtual_target_publish_before_pursuer_ready": "false"' in launch
    assert '"virtual_target_use_pursuer_local_position": "true"' in launch
    assert (
        '"virtual_target_anchor_ready_topic":\n'
        '                        "/tracking/offboard_session_started"'
    ) in launch
    assert '"target_motion_ramp_s", default_value="0.0"' in launch
    assert '"truth_pursuer_position_source": "local"' in launch
    assert '"pursuer_control_enabled": "true"' in launch
    assert '"pursuer_auto_engage": "false"' in launch
    assert '"pursuer_takeoff_enabled": "false"' in launch
    assert '"pursuer_auto_disable_on_offboard_exit": "true"' in launch
    assert '"command_maximum_speed_mps", default_value="12.0"' in launch
    assert '"safety_maximum_speed_mps", default_value="14.0"' in launch
    assert '"maximum_acceleration_mps2", default_value="1.0"' in launch
    assert '"maximum_tilt_rad", default_value="0.35"' in launch
    compact_launch = "".join(launch.split())
    compact_manual = "".join(manual.split())
    for parameter_name in (
        "known_target_direction_enabled",
        "known_target_bearing_ned_rad",
        "known_target_search_hold_s",
        "known_target_search_acceleration_mps2",
        "known_target_search_acceleration_duration_s",
        "known_target_course_enabled",
        "known_target_course_ned_rad",
        "known_target_search_velocity_mps",
        "known_target_velocity_gain",
        "known_target_course_minimum_speed_fraction",
    ):
        assert f'"{parameter_name}"' in launch
        assert f'LaunchConfiguration("{parameter_name}")' in compact_launch
        assert f'"{parameter_name}"' in manual
        assert f'LaunchConfiguration("{parameter_name}")' in compact_manual
    assert '"pursuer_hover_thrust": LaunchConfiguration(' in manual
    assert '"pursuer_maximum_thrust": LaunchConfiguration(' in manual
    assert 'DeclareLaunchArgument(\n                "pursuer_maximum_thrust"' in sim
    assert '"maximum_thrust": LaunchConfiguration(' in sim
    assert '"pursuer_local_topic": pursuer_local_topic' in sim
    assert '"virtual_use_pursuer_local_position":' in sim
    assert '"pursuer_position_source": LaunchConfiguration(' in sim
    assert "pursuer_local_position_" in source
    assert "virtual_spawn_down_m_" in source
    assert "initial_down_m" in source
    assert '"task_origin_topic", "/tracking/task_origin_ned"' in source
    assert "to_task_ned(" in source
    assert 'pursuer_position_source_ == "local"' in truth_bridge
    assert "pursuer_local_position_" in truth_bridge
    assert "to_task_ned(" in truth_bridge


def test_real_virtual_target_anchor_is_latched_on_first_offboard_entry():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    base_launch = (
        ROOT / "launch" / "pure_interceptor_sim.launch.py"
    ).read_text(encoding="utf-8")
    manual_launch = (
        ROOT / "launch" / "coordinate_camera_manual.launch.py"
    ).read_text(encoding="utf-8")
    source = (ROOT / "src" / "target_state_source.cpp").read_text(
        encoding="utf-8"
    )

    session = controller.split("void begin_first_offboard_session()", 1)[1].split(
        "void update_manual_offboard_session()", 1
    )[0]
    assert '"offboard_session_started_topic", "/tracking/offboard_session_started"' in controller
    assert "offboard_session_started_pub_->publish(session_started);" in session
    assert session.index("position_hold_ = local_position_;") < session.index(
        "offboard_session_started_pub_->publish(session_started);"
    )
    assert '"virtual_target_anchor_ready_topic"' in manual_launch
    compact_base = "".join(base_launch.split())
    assert 'LaunchConfiguration("virtual_target_anchor_ready_topic")' in compact_base
    assert "virtual_spawn_down_m_ = pursuer_task_position[2];" in source
    assert "virtual_height_above_anchor_m_" in source
    assert "virtual_first_sample_pending_ = true;" in source


def test_virtual_target_spawns_at_pursuer_gate_in_test_only_path():
    source = (ROOT / "src" / "target_state_source.cpp").read_text(
        encoding="utf-8"
    )
    sim_launch = (ROOT / "launch" / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    analyzer = (
        ROOT / "scripts" / "analyze_predictive_shot_run.py"
    ).read_text(encoding="utf-8")

    assert "capture_virtual_spawn_anchor()" in source
    assert "transform_->to_ned(pursuer_global_)" in source
    assert "virtual_publish_before_pursuer_ready_" in source
    assert "virtual_spawn_time_" in source
    assert "virtual_first_sample_pending_" in source
    assert "motion_elapsed_s = 0.0;" in source
    for parameter in (
        "virtual_target_spawn_relative_to_pursuer_on_ready",
        "virtual_target_publish_before_pursuer_ready",
    ):
        assert f'DeclareLaunchArgument(\n                "{parameter}"' in sim_launch
        assert f'LaunchConfiguration(\n                                "{parameter}"' in sim_launch
    assert 'run_parameters.get("pursuer_prefix", "/px4_1/fmu")' in analyzer


def test_complete_brakes_then_holds_before_reporting_safe_exit():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    message = (ROOT / "msg" / "PureInterceptorStatus.msg").read_text(
        encoding="utf-8"
    )

    assert "attack_complete_ = true;" in controller
    assert "publish_zero_velocity_setpoint();" in controller
    assert "output.velocity = {0.0F, 0.0F, 0.0F};" in controller
    assert "completion_position_hold_active_ = true;" in controller
    assert "safe_to_exit_offboard_ = true;" in controller
    assert "complete.data = safe_to_exit_offboard_;" in controller
    completion_block = controller.split(
        "if (mission.state == PureMissionState::COMPLETE", 1
    )[1].split("if (setpoint_streaming())", 1)[0]
    assert "flight_control_active()" in completion_block
    assert '"completion zero-velocity braking settled;' in completion_block
    for field in (
        "bool attack_complete",
        "bool completion_velocity_braking_active",
        "bool completion_position_hold_active",
        "bool safe_to_exit_offboard",
    ):
        assert field in message


def test_propeller_start_is_the_shared_task_origin_for_every_active_controller():
    pure = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    simple = (ROOT / "src" / "simple_below_tracker_node.cpp").read_text(
        encoding="utf-8"
    )
    legacy = (ROOT / "src" / "offboard_vehicle.cpp").read_text(
        encoding="utf-8"
    )
    target_source = (ROOT / "src" / "target_state_source.cpp").read_text(
        encoding="utf-8"
    )
    truth_bridge = (ROOT / "src" / "simulation_truth_bridge.cpp").read_text(
        encoding="utf-8"
    )
    analyzer = (ROOT / "scripts" / "analyze_predictive_shot_run.py").read_text(
        encoding="utf-8"
    )

    for controller in (pure, simple, legacy):
        assert 'lock_task_origin("PX4 armed transition")' in controller
        assert 'lock_task_origin("automatic arm command")' in controller
        assert '"task_origin_topic", "/tracking/task_origin_ned"' in controller
        assert "publish_task_origin()" in controller
    assert "height_above_task_origin_m(" in pure
    assert "height_above_task_origin_m(" in simple
    assert "to_task_ned(" in target_source
    assert "to_task_ned(" in truth_bridge
    assert 'records.get("/tracking/task_origin_ned", [])' in analyzer


def test_search_and_one_degree_shot_timing_are_in_active_controller_path():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    message = (ROOT / "msg" / "PureInterceptorStatus.msg").read_text(
        encoding="utf-8"
    )
    search_block = controller.split(
        "if (state == PureMissionState::SEARCH)", 1
    )[1].split(
        "if (state == PureMissionState::BEARING_TRACK", 1
    )[0]

    assert "search_pattern_.update(" in search_block
    assert "search_pattern_.reset();" in controller
    assert "shot_timing_optimizer_.update({" in controller
    assert "shot_hard_gate_passed_ && shot_timing_result_.ready" in controller
    assert "terminal_alignment_strength_ = capture_count_ > 0" in controller
    assert "fire_authorized_ =" in controller
    assert "input.attack_enabled = fire_authorized_;" in controller
    assert "input.terminal_alignment_strength = terminal_alignment_strength;" in controller
    assert "first_attack_tracking_ready" in controller
    assert "transverse_velocity_gain" in controller
    attack_block = controller.split("fire_authorized_ =", 1)[1].split(
        "const auto guidance", 1
    )[0]
    assert "attack_enable_delay_remaining_s_ <= 0.0" in attack_block
    assert "first_attack_tracking_ready" in attack_block
    assert "terminal_alignment_strength_ >= 1.0" in attack_block
    assert '"shot_aim_tolerance_rad":' in (
        ROOT / "launch" / "pure_interceptor_sim.launch.py"
    ).read_text(encoding="utf-8")
    assert "0.017453292519943295" in controller
    for field in (
        "bool search_pattern_active",
        "float64 search_pattern_progress",
        "bool shot_hard_gate_passed",
        "bool shot_timing_gate_passed",
        "float64 shot_timing_predicted_axis_error_rad",
        "bool tracking_ready",
        "float64 terminal_alignment_strength",
        "bool fire_authorized",
        "float64 attack_enable_delay_remaining_s",
        "float64 fov_approach_scale",
    ):
        assert field in message


def test_simulation_target_moves_before_pursuer_without_controller_feedback():
    launch = (ROOT / "launch" / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"target_wait_for_pursuer_intercept": False' in launch
    assert '"pursuer_controller_start_delay_s", default_value="0.0"' in launch
    assert '"require_mission_enable": True' in launch
    assert '"/simulation/target_motion_ready"' in launch
    target_node = launch.split('namespace="target"', 1)[1].split(
        'executable="simulation_truth_bridge"', 1
    )[0]
    assert "pure_interceptor_status_topic" not in target_node


def test_search_message_has_no_scale_fields():
    message = (
        ROOT.parent / "anti_drone_interfaces" / "msg" / "SearchObservation.msg"
    ).read_text(encoding="utf-8")
    fields = [
        line.strip().split()[-1]
        for line in message.splitlines()
        if line.strip() and not line.lstrip().startswith("#")
    ]
    forbidden_fields = {
        "depth",
        "depth_z_m",
        "range",
        "range_m",
        "position",
        "position_optical",
        "position_camera",
    }
    assert forbidden_fields.isdisjoint(fields)
    forbidden_tokens = ("truth", "gazebo", "outlier_injected", "simulation")
    lowered = message.lower()
    for token in forbidden_tokens:
        assert token not in lowered


def test_camera_contract_v1_has_shared_detection_fields():
    interface_root = ROOT.parent / "anti_drone_interfaces" / "msg"
    search = (interface_root / "SearchObservation.msg").read_text(encoding="utf-8")
    depth = (interface_root / "DepthObservation.msg").read_text(encoding="utf-8")
    required = (
        "uint16 INTERFACE_VERSION_V1=1",
        "uint16 interface_version",
        "std_msgs/Header header",
        "uint32 sequence",
        "uint32 track_id",
        "bool visible",
        "float32 confidence",
        "uint32 image_width",
        "uint32 image_height",
        "float64 pixel_u",
        "float64 pixel_v",
        "bool bounding_box_valid",
        "float64 bounding_box_width_px",
        "float64 bounding_box_height_px",
        "bool angles_valid",
        "float64 horizontal_angle_rad",
        "float64 vertical_angle_rad",
        "float64[4] angle_covariance",
    )
    for field in required:
        assert field in search
        assert field in depth


def test_camera_contract_contains_no_simulation_metadata():
    interface_root = ROOT.parent / "anti_drone_interfaces" / "msg"
    for name in ("SearchObservation.msg", "DepthObservation.msg"):
        message = (interface_root / name).read_text(encoding="utf-8").lower()
        for forbidden in ("truth", "gazebo", "outlier_injected", "simulation"):
            assert forbidden not in message


def test_virtual_shot_event_contains_estimates_but_no_truth_or_referee_result():
    message = (
        ROOT.parent
        / "virtual_interception_interfaces"
        / "msg"
        / "VirtualShotEvent.msg"
    ).read_text(encoding="utf-8").lower()
    assert "estimated_target_position_ned" in message
    assert "shot_direction_ned" in message
    for forbidden in ("truth", "gazebo", "virtualhitresult", "hit_result"):
        assert forbidden not in message


def test_referee_is_not_a_control_package_dependency():
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")
    package = (ROOT / "package.xml").read_text(encoding="utf-8")
    real_launch = (ROOT / "launch" / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    for source in (cmake, package, real_launch):
        assert "virtual_interception_referee" not in source


def test_depth_contract_keeps_axial_depth_and_euclidean_range_separate():
    message = (
        ROOT.parent / "anti_drone_interfaces" / "msg" / "DepthObservation.msg"
    ).read_text(encoding="utf-8")
    assert "float64 depth_z_m" in message
    assert "float64 range_m" in message
    assert "geometry_msgs/Point position_optical" in message
    assert "bool depth_valid" in message


def test_px4_messages_are_official_and_legacy_bridge_is_ignored():
    px4_message_directory = ROOT.parent / "px4_msgs" / "msg"
    forbidden = {
        "TargetInterceptorControlStatus.msg",
        "TargetInterceptorStatus.msg",
        "TargetRelativeMeasurement.msg",
        "VirtualCaptureEvent.msg",
    }
    assert forbidden.isdisjoint(path.name for path in px4_message_directory.glob("*.msg"))
    assert (ROOT.parent / "px4_interceptor_bridge" / "COLCON_IGNORE").is_file()


def test_pure_controller_active_path_has_no_mpc_or_fixed_corridor():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    sim_launch = (ROOT / "launch" / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    real_launch = (ROOT / "launch" / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    cmake = (ROOT / "CMakeLists.txt").read_text(encoding="utf-8")

    for source in (controller, sim_launch, real_launch):
        assert "mpc_guidance" not in source
        assert "approach_corridor" not in source
        assert "fixed_altitude" not in source
        assert "30.0, 0.0, 5.0" not in source

    pure_link_block = cmake.split(
        "target_link_libraries(\n  pure_interceptor_controller", 1
    )[1].split(")", 1)[0]
    assert "mpc_guidance" not in pure_link_block
    assert "approach_corridor" not in pure_link_block


def test_fov_recovery_keeps_unified_guidance_as_primary_controller():
    controller = (ROOT / "src" / "pure_interceptor_controller.cpp").read_text(
        encoding="utf-8"
    )
    recovery_block = controller.split(
        "if (state == PureMissionState::FOV_RECOVERY)", 1
    )[1].split(
        "if ((state == PureMissionState::INTERCEPT", 1
    )[0]
    assert recovery_block.index("if (guidance.valid)") < recovery_block.index(
        "else if (depth_visible())"
    )


def test_recoil_and_reattack_parameters_are_shared_by_all_pure_entrypoints():
    entrypoints = (
        "pure_interceptor_sim.launch.py",
        "pure_interceptor_manual_sim.launch.py",
        "pure_interceptor_real.launch.py",
        "pure_interceptor_manual_real.launch.py",
        "pure_interceptor_auto_real.launch.py",
        "coordinate_camera_auto.launch.py",
        "coordinate_camera_manual.launch.py",
        "coordinate_camera_manual_real.launch.py",
        "p7_interceptor_sim.launch.py",
        "p7_interceptor_manual_sim.launch.py",
    )
    for name in entrypoints:
        launch = (ROOT / "launch" / name).read_text(encoding="utf-8")
        for parameter in (
            "recoil_duration_s",
            "recoil_opening_speed_mps",
            "reattack_cooldown_s",
        ):
            assert f'"{parameter}"' in launch


def test_recoil_exit_is_not_gated_by_reattack_cooldown():
    mission = (ROOT / "src" / "pure_mission.cpp").read_text(encoding="utf-8")
    egress = mission.rsplit("case PureMissionState::EGRESS_REATTACK:", 1)[1].split(
        "case PureMissionState::COMPLETE:", 1
    )[0]
    assert "config_.recoil_duration_s" in egress
    assert "config_.reattack_cooldown_s" not in egress
    assert "PureMissionState::FOV_RECOVERY" in egress
    assert "PureMissionState::SEARCH" in egress


def test_predictive_sim_target_takeoff_height_is_thirty_metres():
    launch = (ROOT / "launch" / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    runner = (
        ROOT.parents[1] / "scripts" / "run_predictive_shot_2mps_scenario.sh"
    ).read_text(encoding="utf-8")
    assert 'DeclareLaunchArgument("target_height_m", default_value="30.0")' in launch
    assert "target_height_m:=30.0" in runner


def test_predictive_runner_exposes_signed_head_on_geometry_without_changing_defaults():
    runner = (
        ROOT.parents[1] / "scripts" / "run_predictive_shot_2mps_scenario.sh"
    ).read_text(encoding="utf-8")

    assert 'target_heading_rad="${TARGET_HEADING_RAD:-0.0}"' in runner
    assert 'default_target_initial_north_m="0.0"' in runner
    assert 'default_target_initial_east_m="0.0"' in runner
    assert 'default_pursuer_initial_east_m="-5.0"' in runner
    assert (
        'target_initial_north_m="${TARGET_INITIAL_NORTH_M:-'
        '${default_target_initial_north_m}}"' in runner
    )
    assert (
        'pursuer_initial_east_m="${PURSUER_INITIAL_EAST_M:-'
        '${default_pursuer_initial_east_m}}"' in runner
    )
    assert 'target_heading_rad:="${target_heading_rad}"' in runner
    assert (
        'TARGET_POSE="${target_initial_east_m},${target_initial_north_m},'
        '0.01,0,0,1.570796"' in runner
    )
    assert (
        'PURSUER_POSE="${pursuer_initial_east_m},${pursuer_initial_north_m},'
        '0.01,0,0,1.570796"' in runner
    )


def test_predictive_runner_head_on_mode_removes_only_the_settling_delay():
    runner = (
        ROOT.parents[1] / "scripts" / "run_predictive_shot_2mps_scenario.sh"
    ).read_text(encoding="utf-8")

    assert 'head_on_mode="${HEAD_ON_MODE:-false}"' in runner
    assert 'default_head_on_initial_distance_m="60.0"' in runner
    assert 'default_head_on_initial_distance_m="35.0"' in runner
    assert 'default_head_on_initial_distance_m="30.0"' in runner
    assert 'head_on_initial_distance_m="${HEAD_ON_INITIAL_DISTANCE_M:-' in runner
    assert (
        'known_target_direction_enabled="${KNOWN_TARGET_DIRECTION_ENABLED:-'
        '${head_on_mode}}"' in runner
    )
    assert 'default_attack_enable_delay_s="0.0"' in runner
    assert 'default_attack_enable_delay_s="10.0"' in runner
    assert (
        'attack_enable_delay_s="${ATTACK_ENABLE_DELAY_S:-'
        '${default_attack_enable_delay_s}}"' in runner
    )
    assert 'default_known_target_course_enabled="${head_on_mode}"' in runner
    assert (
        'known_target_course_enabled="${KNOWN_TARGET_COURSE_ENABLED:-'
        '${default_known_target_course_enabled}}"' in runner
    )
    assert 'default_known_target_velocity_gain="1.0"' in runner
    assert 'default_known_target_velocity_gain="0.6"' not in runner
    assert 'default_known_target_velocity_gain="0.35"' not in runner
    assert 'default_known_target_velocity_gain="0.25"' not in runner
    assert '0.875 * speed' not in runner
    assert 'default_known_target_search_velocity_mps="${target_speed_mps}"' in runner
    assert 'default_first_attack_tracking_hold_s="0.4"' in runner
    assert 'default_terminal_alignment_tracking_hold_s="0.15"' in runner
    assert 'default_terminal_alignment_ramp_s="0.25"' in runner
    assert 'default_estimate_stability_hold_s="0.3"' in runner
    assert 'default_target_motion_ramp_s="1.0"' in runner
    assert "printf 'head_on_mode=%s\\n'" in runner


def test_known_direction_search_is_operator_cue_and_defaults_off():
    controller = (
        ROOT / "src" / "pure_interceptor_controller.cpp"
    ).read_text(encoding="utf-8")
    simulation_launch = (
        ROOT / "launch" / "pure_interceptor_sim.launch.py"
    ).read_text(encoding="utf-8")
    runner = (
        ROOT.parents[1] / "scripts" / "run_predictive_shot_2mps_scenario.sh"
    ).read_text(encoding="utf-8")

    assert 'declare_parameter<bool>("known_target_direction_enabled", false)' in controller
    assert 'declare_parameter<double>("known_target_bearing_ned_rad", 0.0)' in controller
    assert 'declare_parameter<double>("known_target_search_hold_s", 6.0)' in controller
    assert (
        'declare_parameter<double>("known_target_search_acceleration_mps2", 1.5)'
        in controller
    )
    assert "known_direction_search_acceleration(search_elapsed)" in controller
    assert 'declare_parameter<bool>("known_target_course_enabled", false)' in controller
    assert 'declare_parameter<double>("known_target_course_ned_rad", 0.0)' in controller
    assert "known_course_velocity_acceleration()" in controller
    assert "course_alignment = std::clamp(" in controller
    assert "normalized_encounter_geometry = 0.5 * (course_alignment + 1.0)" in controller
    assert "known_target_course_ned_rad_ - known_target_bearing_ned_rad_" in controller
    assert (
        '"known_target_course_minimum_speed_fraction", 0.35' in controller
    )
    assert '"known_target_direction_enabled", default_value="false"' in simulation_launch
    assert 'head_on_mode="${HEAD_ON_MODE:-false}"' in runner
    assert (
        'known_target_direction_enabled="${KNOWN_TARGET_DIRECTION_ENABLED:-'
        '${head_on_mode}}"' in runner
    )
    assert 'known_target_bearing_ned_rad:="${known_target_bearing_ned_rad}"' in runner


def test_terminal_alignment_uses_valid_depth_acquire_time_without_early_fire():
    controller = (
        ROOT / "src" / "pure_interceptor_controller.cpp"
    ).read_text(encoding="utf-8")
    mission = (ROOT / "src" / "pure_mission.cpp").read_text(encoding="utf-8")

    tracking_expression = controller.split(
        "const bool continuous_first_attack_tracking =", 1
    )[1].split(";", 1)[0]
    assert "PureMissionState::DEPTH_ACQUIRE" in tracking_expression
    depth_acquire_case = mission.split(
        "case PureMissionState::DEPTH_ACQUIRE:", 1
    )[1].split("case PureMissionState::INTERCEPT:", 1)[0]
    assert "capture_event_ = true" not in depth_acquire_case


def test_unified_recorder_captures_real_session_and_completion_evidence():
    recorder = (
        ROOT.parents[1] / "scripts" / "record_unified_experiment.sh"
    ).read_text(encoding="utf-8")

    required_topics = (
        "/tracking/task_origin_ned",
        "/tracking/offboard_session_started",
        "/tracking/mission/complete",
        "/tracking/pure_interceptor/enable",
        "/tracking/simple_below/enable",
        "/tracking/pure_interceptor/status",
        "/tracking/simple_below/status",
        "/tracking/virtual_shot_event",
        "/tracking/target/state",
        "/evaluation/relative_truth",
        "/evaluation/virtual_hit_result",
        "/evaluation/virtual_solver_hit_result",
        "/evaluation/mission_success",
    )
    for topic in required_topics:
        assert topic in recorder

    required_px4_suffixes = (
        "/out/vehicle_status",
        "/out/vehicle_local_position",
        "/out/vehicle_attitude",
        "/out/vehicle_angular_velocity",
        "/out/vehicle_acceleration",
        "/out/battery_status",
        "/out/failsafe_flags",
        "/out/estimator_status_flags",
        "/out/timesync_status",
        "/out/vehicle_command_ack",
        "/in/offboard_control_mode",
        "/in/trajectory_setpoint",
        "/in/vehicle_attitude_setpoint",
    )
    for suffix in required_px4_suffixes:
        assert f'"${{pursuer_prefix}}{suffix}"' in recorder
