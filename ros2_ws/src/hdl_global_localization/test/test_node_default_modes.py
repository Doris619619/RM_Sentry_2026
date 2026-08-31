import os
import subprocess
import time
from pathlib import Path

import rclpy
from rcl_interfaces.srv import GetParameters


NODE = os.environ['HDL_GLOBAL_LOCALIZATION_NODE']
CONFIG = Path(__file__).resolve().parents[1] / 'config' / 'global_localization.yaml'


def _read_engine(extra_args):
    process = subprocess.Popen([NODE, *extra_args], stdout=subprocess.PIPE,
                               stderr=subprocess.STDOUT, text=True)
    rclpy.init()
    node = rclpy.create_node('hdl_global_localization_mode_test')
    client = node.create_client(GetParameters, '/hdl_global_localization/get_parameters')
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and not client.wait_for_service(timeout_sec=0.2):
            if process.poll() is not None:
                output = process.stdout.read() if process.stdout else ''
                raise AssertionError(f'global localization node exited early: {output}')
        assert client.service_is_ready(), 'parameter service did not become available'
        request = GetParameters.Request(names=['global_localization_engine'])
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        assert future.result() is not None
        value = future.result().values[0]
        return value.string_value
    finally:
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5.0)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=5.0)


def test_bare_node_uses_fpfh_ransac_fallback():
    assert _read_engine([]) == 'FPFH_RANSAC'


def test_standard_launch_parameter_file_uses_bbs():
    assert _read_engine(['--ros-args', '--params-file', str(CONFIG)]) == 'BBS'
