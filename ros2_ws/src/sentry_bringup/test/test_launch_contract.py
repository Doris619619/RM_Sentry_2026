from pathlib import Path


LAUNCH = Path(__file__).resolve().parents[1] / 'launch'


def test_production_requires_explicit_hardware_and_motion_opt_in():
    text = (LAUNCH / 'production.launch.py').read_text()
    assert "DeclareLaunchArgument('enable_mcu', default_value='false'" in text
    assert "DeclareLaunchArgument('allow_motion_output', default_value='false'" in text
    assert "DeclareLaunchArgument('enable_sensor_pipeline', default_value='false'" in text
    assert "enable_mcu, \"' == 'true' and '\", allow_motion_output" in text


def test_all_bringup_serial_defaults_are_921600():
    production = (LAUNCH / 'production.launch.py').read_text()
    fixture = (LAUNCH / 'fixture.launch.py').read_text()
    assert "default_value='921600'" in production
    assert "default_value='921600'" in fixture


def test_production_declares_both_real_topology_loops():
    text = (LAUNCH / 'production.launch.py').read_text()
    assert 'Localization/control:' in text
    assert 'Decision loop:' in text
    for package in ('livox_cloudpoint_processor', 'point_lio', 'hdl_localization', 'hdl_global_localization', 'trajectory_generation',
                    'trajectory_tracking', 'decision_node'):
        assert package in text
