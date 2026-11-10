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
from epos2_driver.msg import MotorFeedback
import yaml


EXECUTABLE, FAKE, CONFIG, LIBRARY_DIR, TWO_MOTOR_CONFIG = sys.argv[1:6]
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

    def start(self, config_path=None, **variables):
        self.config_path.write_text(yaml.safe_dump(self.config), encoding='utf-8')
        env = os.environ.copy()
        env.update(variables)
        env['EPOS_TEST_TRACE'] = str(self.trace)
        env['LD_PRELOAD'] = FAKE
        env['LD_LIBRARY_PATH'] = LIBRARY_DIR + ':' + env.get('LD_LIBRARY_PATH', '')
        self.process = subprocess.Popen(
            [EXECUTABLE, '--ros-args', '--params-file', str(config_path or self.config_path)],
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
        failed = self.call('/epos2/right_wheel', True)
        self.assertFalse(failed.success)
        self.assertIn('0x00001234', failed.message)
        self.assertTrue(self.call('/epos2/left_wheel', True).success)
        self.assertTrue(self.call('/epos2/left_active_finger', True).success)
        self.assertTrue(self.call('/epos2/right_active_finger', True).success)
        self.assertTrue(self.call('/robot/arm', True).success)
        self.publish('/epos2/left_wheel/cmd_vel_rpm', 8001)
        self.publish('/epos2/left_wheel/cmd_vel_rpm', -8000)
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
        failed = self.call('/epos2/left_wheel', True)
        self.assertFalse(failed.success)
        self.assertIn('CAN baudrate mismatch', failed.message)
        self.assertTrue(self.call('/epos2/left_active_finger', True).success)
        self.publish('/epos2/left_active_finger/cmd_vel_rpm', 200)
        self.wait_trace('move_velocity 4 200 0')

    def test_feedback_errors_report_each_failed_motor(self):
        self.start(EPOS_TEST_CAN_BAUD='500000')
        deadline = time.monotonic() + 5
        while time.monotonic() < deadline:
            output = self.process_output()
            if ('/epos2/left_wheel: feedback:' in output and
                    '/epos2/right_wheel: feedback:' in output):
                break
            time.sleep(0.05)
        self.assertIn('/epos2/left_wheel: feedback:', self.process_output())
        self.assertIn('/epos2/right_wheel: feedback:', self.process_output())

    def test_failed_enable_and_disable_blocks_further_commands(self):
        self.start(EPOS_TEST_FAIL_ENABLE_NODE='1', EPOS_TEST_FAIL_DISABLE_AFTER_ENABLE='1')
        result = self.call('/epos2/left_wheel', True)
        self.assertFalse(result.success)
        self.assertIn('VCS_SetEnableState', result.message)
        self.assertIn('VCS_SetDisableState', result.message)
        self.assertIn('uncertain', self.call('/epos2/left_wheel', True).message)
        self.publish('/epos2/left_wheel/cmd_vel_rpm', 100)
        self.assertNotIn('move_velocity 1 100', self.trace.read_text(encoding='utf-8'))
        self.assertTrue(self.call('/epos2/left_active_finger', True).success)

    def test_invalid_position_profile_rejected_before_device_open(self):
        motors = self.config['epos_driver_node']['ros__parameters']['motors']
        motors['node_3']['position']['profile_velocity'] = 7001
        self.config_path.write_text(yaml.safe_dump(self.config), encoding='utf-8')
        self.start()
        self.assertEqual(self.process.wait(timeout=15), 1)
        self.assertIn('profile_velocity exceeds max_velocity', self.process_output())
        self.assertFalse(self.trace.exists())

    def test_invalid_namespace_rejected_before_device_open(self):
        motors = self.config['epos_driver_node']['ros__parameters']['motors']
        motors['node_4']['namespace'] = '/invalid-name'
        self.start()
        self.assertEqual(self.process.wait(timeout=15), 1)
        self.assertIn('invalid', self.process_output().lower())
        self.assertFalse(self.trace.exists())

    def test_feedback_values_and_rate_when_disabled(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['motor_names'] = ['node_4', 'node_5']
        params['feedback_rate_hz'] = 5.0
        received = {4: [], 5: []}
        subscriptions = []
        for node_id, namespace in (
            (4, '/epos2/left_active_finger'), (5, '/epos2/right_active_finger')
        ):
            subscriptions.append(self.node.create_subscription(
                MotorFeedback, namespace + '/feedback',
                lambda message, motor_id=node_id: received[motor_id].append(message), 10))
        self.start()
        deadline = time.monotonic() + 8
        while min(len(messages) for messages in received.values()) < 4 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for node_id, messages in received.items():
            self.assertGreaterEqual(len(messages), 4, self.process_output())
            for message in messages:
                self.assertEqual(message.current_ma, -100 * node_id)
                self.assertEqual(message.velocity_actual, 200 * node_id)
                self.assertEqual(message.position_counts, -300 * node_id)
                self.assertGreater(message.header.stamp.sec, 0)
            stamps = [msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 for msg in messages]
            self.assertGreater(stamps[3] - stamps[0], 0.4)
            self.assertLess(stamps[3] - stamps[0], 1.5)

    def test_default_feedback_rate_with_five_motors(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params.pop('feedback_rate_hz', None)
        received = {node_id: [] for node_id in range(1, 6)}
        namespaces = {
            1: '/epos2/left_wheel',
            2: '/epos2/right_wheel',
            3: '/robot/arm',
            4: '/epos2/left_active_finger',
            5: '/epos2/right_active_finger',
        }
        subscriptions = []
        for node_id, namespace in namespaces.items():
            subscriptions.append(self.node.create_subscription(
                MotorFeedback, namespace + '/feedback',
                lambda message, motor_id=node_id: received[motor_id].append(message), 10))
        self.start()
        deadline = time.monotonic() + 8
        while min(len(messages) for messages in received.values()) < 4 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        for messages in received.values():
            self.assertGreaterEqual(len(messages), 4, self.process_output())
            stamps = [msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9 for msg in messages]
            self.assertGreater(stamps[3] - stamps[0], 0.20)
            self.assertLess(stamps[3] - stamps[0], 0.75)

    def test_feedback_failure_isolated_and_not_published(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['motor_names'] = ['node_4', 'node_5']
        received = {4: [], 5: []}
        subscriptions = []
        for node_id, namespace in (
            (4, '/epos2/left_active_finger'), (5, '/epos2/right_active_finger')
        ):
            subscriptions.append(self.node.create_subscription(
                MotorFeedback, namespace + '/feedback',
                lambda message, motor_id=node_id: received[motor_id].append(message), 10))
        self.start(EPOS_TEST_FEEDBACK_FAIL_NODE='5', EPOS_TEST_FEEDBACK_FAIL_OPERATION='velocity')
        deadline = time.monotonic() + 4
        while len(received[4]) < 4 and time.monotonic() < deadline:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        self.assertGreaterEqual(len(received[4]), 4, self.process_output())
        self.assertEqual(received[5], [])
        self.assertIn('VCS_GetVelocityIs failed (error 0x00009876)', self.process_output())
        trace = self.trace.read_text(encoding='utf-8')
        self.assertIn('get_current 5', trace)
        self.assertNotIn('get_position 5', trace)
        self.assertTrue(self.call('/epos2/left_active_finger', True).success)
        self.assertTrue(self.call('/epos2/right_active_finger', True).success)

    def test_invalid_feedback_rate_rejected_before_device_open(self):
        self.config['epos_driver_node']['ros__parameters']['feedback_rate_hz'] = 0.0
        self.start()
        self.assertEqual(self.process.wait(timeout=15), 1)
        self.assertIn('feedback_rate_hz must be', self.process_output())
        self.assertFalse(self.trace.exists())

    def test_single_usb_motor_without_can(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['gateway_node_id'] = 1
        params['motor_names'] = ['node_1']
        self.start(EPOS_TEST_GATEWAY_ID='1', EPOS_TEST_CAN_BAUD='500000')
        self.assertTrue(self.call('/epos2/left_wheel', True).success)
        self.publish('/epos2/left_wheel/cmd_vel_rpm', 100)
        self.wait_trace('move_velocity 1 100 0')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertNotIn('open_subdevice', trace)
        self.assertNotIn('disable 2', trace)
        self.assertNotIn('disable 4', trace)

    def test_shutdown_disables_motors_before_closing_connection(self):
        self.start()
        self.assertTrue(self.call('/epos2/left_active_finger', True).success)
        self.assertTrue(self.call('/epos2/right_active_finger', True).success)
        self.process.send_signal(signal.SIGINT)
        self.assertEqual(self.process.wait(timeout=10), 0, self.process_output())
        trace = self.trace.read_text(encoding='utf-8')
        for node_id in (4, 5):
            self.assertLess(trace.index(f'halt_velocity {node_id} '), trace.index('close_subdevice'))
            self.assertLess(trace.rindex(f'disable {node_id} '), trace.index('close_subdevice'))
        self.assertLess(trace.index('close_subdevice'), trace.index('close_device'))
        self.process = None

    def test_can_only_selection_keeps_usb_gateway_uncontrolled(self):
        params = self.config['epos_driver_node']['ros__parameters']
        params['motor_names'] = ['node_2', 'node_3']
        self.start()
        self.assertTrue(self.call('/epos2/right_wheel', True).success)
        self.assertTrue(self.call('/robot/arm', True).success)
        self.publish('/epos2/right_wheel/cmd_vel_rpm', 100)
        self.wait_trace('move_velocity 2 100 0')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertIn('open_subdevice', trace)
        self.assertNotIn('disable 1', trace)
        self.assertNotIn('disable 4', trace)
        self.assertNotIn('disable 5', trace)

    def test_node4_node5_configuration(self):
        with open(CONFIG, encoding='utf-8') as config:
            full = yaml.safe_load(config)['epos_driver_node']['ros__parameters']
        with open(TWO_MOTOR_CONFIG, encoding='utf-8') as config:
            two = yaml.safe_load(config)['epos_driver_node']['ros__parameters']
        expected_namespaces = {
            'node_1': '/epos2/left_wheel',
            'node_2': '/epos2/right_wheel',
            'node_3': '/epos2/rear_wheel',
            'node_4': '/epos2/left_active_finger',
            'node_5': '/epos2/right_active_finger',
        }
        for name, namespace in expected_namespaces.items():
            self.assertEqual(full['motors'][name]['namespace'], namespace)
        self.assertEqual(two['motor_names'], ['node_4', 'node_5'])
        self.assertEqual(two['gateway_node_id'], 4)
        for name in two['motor_names']:
            self.assertEqual(two['motors'][name], full['motors'][name])
            self.assertEqual(two['motors'][name]['mode'], 'velocity')

        self.start(config_path=TWO_MOTOR_CONFIG)
        for node_id, namespace in (
            (4, '/epos2/left_active_finger'), (5, '/epos2/right_active_finger')
        ):
            self.assertTrue(self.call(namespace, True).success)
            self.publish(namespace + '/cmd_vel_rpm', 100)
            self.wait_trace(f'move_velocity {node_id} 100 0')
        trace = self.trace.read_text(encoding='utf-8')
        self.assertIn('open_subdevice', trace)
        for node_id in (1, 2, 3):
            self.assertNotIn(f'disable {node_id} ', trace)
        for node_id in (4, 5):
            self.assertIn(f'velocity_profile {node_id} 10000 10000', trace)
            self.assertIn(f'max_velocity {node_id} 8000', trace)


if __name__ == '__main__':
    unittest.main()
