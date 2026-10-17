from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
LAUNCH = ROOT / "launch"

PREDICTION_PARAMETERS = (
    "target_acceleration_feedforward_gain",
    "target_acceleration_feedforward_minimum_ca_probability",
    "target_acceleration_feedforward_full_ca_probability",
    "target_acceleration_feedforward_maximum_stddev_mps2",
    "target_acceleration_feedforward_filter_time_constant_s",
    "target_acceleration_feedforward_maximum_mps2",
)

EARLY_TRACKING_PARAMETERS = (
    "attack_enable_delay_s",
    "first_attack_tracking_hold_s",
    "terminal_alignment_tracking_hold_s",
    "terminal_alignment_ramp_s",
)

ACTIVE_FULL_INTERCEPTOR_LAUNCHES = (
    "pure_interceptor_sim.launch.py",
    "pure_interceptor_real.launch.py",
    "pure_interceptor_manual_sim.launch.py",
    "pure_interceptor_manual_real.launch.py",
    "pure_interceptor_auto_real.launch.py",
    "p7_interceptor_sim.launch.py",
    "p7_interceptor_manual_sim.launch.py",
    "coordinate_camera_auto.launch.py",
    "coordinate_camera_manual.launch.py",
    "coordinate_camera_manual_real.launch.py",
)

REAL_RMW_LAUNCHES = (
    "pure_interceptor_real.launch.py",
    "simple_below_tracker_real.launch.py",
)


def test_real_launches_use_the_installed_px4_fastdds_rmw():
    for filename in REAL_RMW_LAUNCHES:
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        assert '"RMW_IMPLEMENTATION", "rmw_fastrtps_cpp"' in text
        assert "rmw_cyclonedds_cpp" not in text


def test_coordinate_real_threads_fastdds_through_the_simulation_composition():
    real = (LAUNCH / "coordinate_camera_manual_real.launch.py").read_text(
        encoding="utf-8"
    )
    manual = (LAUNCH / "coordinate_camera_manual.launch.py").read_text(
        encoding="utf-8"
    )
    simulation = (LAUNCH / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    assert '"rmw_implementation", default_value="rmw_fastrtps_cpp"' in real
    assert (
        '"rmw_implementation", default_value="rmw_cyclonedds_cpp"' in manual
    )
    assert (
        '"rmw_implementation", default_value="rmw_cyclonedds_cpp"'
        in simulation
    )
    assert '"rmw_implementation": LaunchConfiguration(' in real
    assert '"rmw_implementation": LaunchConfiguration(' in manual
    assert (
        '"RMW_IMPLEMENTATION", LaunchConfiguration("rmw_implementation")'
        in simulation
    )
    for parameter in (
        "pursuer_prefix",
        "pursuer_attitude_topic",
        "pursuer_global_topic",
        "pursuer_local_topic",
    ):
        assert f'"{parameter}"' in real
        assert f'"{parameter}"' in manual
        assert f'"{parameter}": LaunchConfiguration(' in real
        assert f'"{parameter}": LaunchConfiguration(' in manual


def test_manual_camera_referee_follows_selected_pursuer_local_topic():
    manual = (LAUNCH / "coordinate_camera_manual.launch.py").read_text(
        encoding="utf-8"
    )
    referee = manual.split('executable="virtual_projectile_referee"', 1)[1]
    assert '"pursuer_local_topic": LaunchConfiguration(' in referee
    assert '"pursuer_local_topic":\n                            "/fmu/' not in referee


def test_all_active_full_interceptor_entries_expose_prediction_contract():
    for filename in ACTIVE_FULL_INTERCEPTOR_LAUNCHES:
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        for parameter in PREDICTION_PARAMETERS:
            assert text.count(parameter) >= 2, (filename, parameter)


def test_all_active_full_interceptor_entries_expose_early_tracking_contract():
    for filename in ACTIVE_FULL_INTERCEPTOR_LAUNCHES:
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        for parameter in EARLY_TRACKING_PARAMETERS:
            assert text.count(parameter) >= 2, (filename, parameter)


def test_simulation_waits_ten_seconds_but_manual_real_starts_at_offboard_entry():
    for filename in (
        "pure_interceptor_sim.launch.py",
        "pure_interceptor_manual_sim.launch.py",
        "p7_interceptor_sim.launch.py",
        "p7_interceptor_manual_sim.launch.py",
        "coordinate_camera_auto.launch.py",
    ):
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        assert '"attack_enable_delay_s", default_value="10.0"' in text, filename
    for filename in (
        "pure_interceptor_real.launch.py",
        "pure_interceptor_manual_real.launch.py",
        "pure_interceptor_auto_real.launch.py",
        "coordinate_camera_manual.launch.py",
        "coordinate_camera_manual_real.launch.py",
    ):
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        assert '"attack_enable_delay_s", default_value="0.0"' in text, filename


def test_only_canonical_launches_construct_the_full_controller():
    constructors = []
    for path in LAUNCH.glob("*.launch.py"):
        if 'executable="pure_interceptor_controller"' in path.read_text(
            encoding="utf-8"
        ):
            constructors.append(path.name)
    assert sorted(constructors) == [
        "pure_interceptor_real.launch.py",
        "pure_interceptor_sim.launch.py",
    ]


def test_canonical_defaults_match_between_simulation_and_real_aircraft():
    simulation = (LAUNCH / "pure_interceptor_sim.launch.py").read_text(
        encoding="utf-8"
    )
    real = (LAUNCH / "pure_interceptor_real.launch.py").read_text(
        encoding="utf-8"
    )
    shared_defaults = (
        ('"maximum_acceleration_mps2", default_value="5.0"'),
        ('"command_maximum_speed_mps", default_value="12.0"'),
        ('"safety_maximum_speed_mps", default_value="14.0"'),
        ('"target_acceleration_feedforward_gain", default_value="0.2"'),
    )
    for declaration in shared_defaults:
        assert declaration in simulation
        assert declaration in real
    for launch in (simulation, real):
        assert '"handover_maximum_tilt_rad": LaunchConfiguration(' in launch
        handover = launch.split(
            '"handover_maximum_tilt_rad": LaunchConfiguration(', 1
        )[1]
        assert '"maximum_tilt_rad"' in handover[:160]


def test_real_entries_keep_first_sortie_airframe_limits():
    for filename in (
        "pure_interceptor_real.launch.py",
        "pure_interceptor_manual_real.launch.py",
        "coordinate_camera_manual_real.launch.py",
    ):
        launch = (LAUNCH / filename).read_text(encoding="utf-8")
        for parameter, expected in (
            ("maximum_tilt_rad", "0.35"),
            ("hover_thrust", "0.42"),
            ("maximum_thrust", "0.70"),
        ):
            declaration = launch.split(f'"{parameter}"', 1)[1][:500]
            assert f'default_value="{expected}"' in declaration, (
                filename,
                parameter,
            )


def test_coordinate_simulation_uses_high_speed_core_and_local_spawn_anchor():
    for filename in (
        "coordinate_camera_auto.launch.py",
        "coordinate_camera_manual.launch.py",
    ):
        text = (LAUNCH / filename).read_text(encoding="utf-8")
        for declaration in (
            '"command_maximum_speed_mps", default_value="12.0"',
            '"safety_maximum_speed_mps", default_value="14.0"',
            '"maximum_acceleration_mps2", default_value="5.0"',
            '"virtual_target_initial_east_m", default_value="0.0"',
            '"truth_pursuer_position_source", default_value="local"',
        ):
            assert declaration in text, (filename, declaration)
        assert (
            '"virtual_target_use_pursuer_local_position",\n'
            '                default_value="true"'
        ) in text
