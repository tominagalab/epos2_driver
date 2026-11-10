import os
import signal
import subprocess
import sys
import tempfile
import time
import unittest
from pathlib import Path

import rclpy
from std_msgs.msg import Int32
from std_srvs.srv import SetBool
import yaml


EXECUTABLE, FAKE, CONFIG, LIBRARY_DIR = sys.argv[1:5]
sys.argv = sys.argv[:1]
os.environ['ROS_DOMAIN_ID'] = str(100 + os.getpid() % 100)
os.environ['ROS_LOCALHOST_ONLY'] = '1'


class NetworkTest(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.path = Path(self.tmp.name)
        self.trace = self.path / 'trace'
        self.output = open(self.path / 'output', 'w+', encoding='utf-8')
        with open(CONFIG, encoding='utf-8') as config:
            self.config = yaml.safe_load(config)
        motors = self.config['epos_driver_node']['ros__parameters']['motors']
        motors['node_3']['mode'] = 'position'
        motors['node_3']['namespace'] = '/robot/arm'
        motors['node_3']['position']['profile_velocity'] = 5500
        motors['node_3']['position']['max_velocity'] = 7000
        motors['node_3']['position']['acceleration'] = 9000
        self.config_path = self.path / 'config.yaml'
        self.config_path.write_text(yaml.safe_dump(self.config), encoding='utf-8')
        self.process = None
        rclpy.init()
        self.node = rclpy.create_node('epos2_test_client')

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()
        if self.process:
            self.process.send_signal(signal.SIGINT)
            try:
                self.process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.output.close()
        self.tmp.cleanup()

    def start(self, **variables):
        self.config_path.write_text(yaml.safe_dump(self.config), encoding='utf-8')
        env = os.environ.copy()
        env.update(variables)
        env['EPOS_TEST_TRACE'] = str(self.trace)
        env['LD_PRELOAD'] = FAKE
        env['LD_LIBRARY_PATH'] = LIBRARY_DIR + ':' + env.get('LD_LIBRARY_PATH', '')
        self.process = subprocess.Popen(
            [EXECUTABLE, '--ros-args', '--params-file', str(self.config_path)],
            env=env, stdout=self.output, stderr=subprocess.STDOUT,
        )

    def call(self, namespace, enabled):
        client = self.node.create_client(SetBool, namespace + '/enable')
        self.assertTrue(client.wait_for_service(timeout_sec=15), self.process_output())
        request = SetBool.Request()
        request.data = enabled
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=15)
        self.assertTrue(future.done(), self.process_output())
        result = future.result()
        self.node.destroy_client(client)
        return result

    def process_output(self):
        self.output.flush()
        return (self.path / 'output').read_text(encoding='utf-8')

    def wait_trace(self, expected):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.trace.exists() and expected in self.trace.read_text(encoding='utf-8'):
                return
            time.sleep(0.05)
        self.fail(f'Missing trace {expected}: {self.process_output()}')

    def publish(self, topic, value):
        publisher = self.node.create_publisher(Int32, topic, 10)
        deadline = time.monotonic() + 10
        while not publisher.get_subscription_count() and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreater(publisher.get_subscription_count(), 0, self.process_output())
        message = Int32()
        message.data = value
        publisher.publish(message)
        time.sleep(0.3)
        self.node.destroy_publisher(publisher)

    def test_isolated_failure_modes_profiles_and_limits(self):
        self.start(EPOS_TEST_FAIL_NODE='2')
        failed = self.call('/epos2/node_2', True)
        self.assertFalse(failed.success)
        self.assertIn('0x00001234', failed.message)
        self.assertTrue(self.call('/epos2/node_1', True).success)
        self.assertTrue(self.call('/epos2/node_4', True).success)
        self.assertTrue(self.call('/epos2/node_5', True).success)
        self.assertTrue(self.call('/robot/arm', True).success)
        self.publish('/epos2/node_1/cmd_vel_rpm', 8001)
        self.publish('/epos2/node_1/cmd_vel_rpm', -8000)
        self.publish('/robot/arm/cmd_position_counts', 12345)
        self.wait_trace('move_velocity 1 -8000 0')
        self.wait_trace('move_position 3 12345 11')
        self.assertTrue(self.call('/robot/arm', False).success)
        self.wait_trace('halt_position 3')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertNotIn('move_velocity 1 8001', trace)
        self.assertIn('velocity_profile 1 10000 10000', trace)
        self.assertIn('max_velocity 1 8000', trace)
        self.assertIn('position_profile 3 5500 9000', trace)
        self.assertIn('max_velocity 3 7000', trace)

    def test_baud_mismatch_does_not_block_usb_motor(self):
        self.start(EPOS_TEST_CAN_BAUD='500000')
        failed = self.call('/epos2/node_1', True)
        self.assertFalse(failed.success)
        self.assertIn('CAN baudrate mismatch', failed.message)
        self.assertTrue(self.call('/epos2/node_4', True).success)
        self.publish('/epos2/node_4/cmd_vel_rpm', 200)
        self.wait_trace('move_velocity 4 200 0')

    def test_failed_enable_and_disable_blocks_further_commands(self):
        self.start(EPOS_TEST_FAIL_ENABLE_NODE='1', EPOS_TEST_FAIL_DISABLE_AFTER_ENABLE='1')
        result = self.call('/epos2/node_1', True)
        self.assertFalse(result.success)
        self.assertIn('VCS_SetEnableState', result.message)
        self.assertIn('VCS_SetDisableState', result.message)
        self.assertIn('uncertain', self.call('/epos2/node_1', True).message)
        self.publish('/epos2/node_1/cmd_vel_rpm', 100)
        self.assertNotIn('move_velocity 1 100', self.trace.read_text(encoding='utf-8'))
        self.assertTrue(self.call('/epos2/node_4', True).success)

    def test_invalid_position_profile_rejected_before_device_open(self):
        motors = self.config['epos_driver_node']['ros__parameters']['motors']
        motors['node_3']['position']['profile_velocity'] = 7001
        self.config_path.write_text(yaml.safe_dump(self.config), encoding='utf-8')
        self.start()
        self.assertEqual(self.process.wait(timeout=15), 1)
        self.assertIn('profile_velocity exceeds max_velocity', self.process_output())
        self.assertFalse(self.trace.exists())

    def test_single_usb_motor_without_can(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['gateway_node_id'] = 1
        params['motor_names'] = ['node_1']
        self.start(EPOS_TEST_GATEWAY_ID='1', EPOS_TEST_CAN_BAUD='500000')
        self.assertTrue(self.call('/epos2/node_1', True).success)
        self.publish('/epos2/node_1/cmd_vel_rpm', 100)
        self.wait_trace('move_velocity 1 100 0')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertNotIn('open_subdevice', trace)
        self.assertNotIn('disable 2', trace)
        self.assertNotIn('disable 4', trace)

    def test_can_only_selection_keeps_usb_gateway_uncontrolled(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['motor_names'] = ['node_2', 'node_3']
        self.start()
        self.assertTrue(self.call('/epos2/node_2', True).success)
        self.assertTrue(self.call('/robot/arm', True).success)
        self.publish('/epos2/node_2/cmd_vel_rpm', 100)
        self.wait_trace('move_velocity 2 100 0')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertIn('open_subdevice', trace)
        self.assertNotIn('disable 1', trace)
        self.assertNotIn('disable 4', trace)
        self.assertNotIn('disable 5', trace)


if __name__ == '__main__':
    unittest.main()
