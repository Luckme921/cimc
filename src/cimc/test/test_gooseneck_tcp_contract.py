"""Replay current calibration and ABB serialization entirely in memory."""
import csv
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace
import unittest

import numpy as np
import yaml
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Pose, PoseArray

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'src/cimc'))
from cimc.handeye_abb_bridge_node import (  # noqa: E402
    HandeyeAbbBridgeNode, load_opencv_matrix, quaternion_to_rotation,
)
from cimc.weld_algorithm import preview_command  # noqa: E402


class Recorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class InMemoryBridge:
    # Call the production callbacks without constructing a ROS node or socket.
    _poses_callback = HandeyeAbbBridgeNode._poses_callback
    _serialize_for_abb = HandeyeAbbBridgeNode._serialize_for_abb

    def __init__(self, calibration):
        self.require_task_armed = True
        self._armed = True
        self.require_capture_pose = True
        self.matrix_direction = 'tcp_from_camera'
        self.transform = calibration.copy()
        self.base_from_tcp = np.eye(4)
        self.base_from_tcp[:3, 3] = [0.94735, -0.079, 0.71749]
        self.base_from_tcp[:3, :3] = quaternion_to_rotation(
            -0.77967, 0.40899, -0.27723, 0.38469)
        self.send_to_abb = True
        self.max_trajectory_points = 100
        self.output_frame = 'robot_base'
        self.protocol_precision = 6
        self.pose_pub = Recorder()
        self.tx_pub = Recorder()
        self.statuses = []

    def get_clock(self):
        return SimpleNamespace(now=lambda: SimpleNamespace(to_msg=lambda: Time()))

    def get_logger(self):
        return SimpleNamespace(warn=lambda _message: None)

    def _status(self, success, message, **extra):
        self.statuses.append((success, message, extra))


class GooseneckTcpContractTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        params = yaml.safe_load((ROOT / 'src/cimc/config/handeye_bridge.yaml').read_text())[
            'handeye_abb_bridge_node']['ros__parameters']
        cls.calibration = load_opencv_matrix(params['matrix_file'], params['matrix_key'])
        if params['matrix_translation_unit'] == 'mm':
            cls.calibration[:3, 3] *= 0.001
        if params['matrix_direction'] == 'camera_from_tcp':
            cls.calibration = np.linalg.inv(cls.calibration)
        config = yaml.safe_load((ROOT / 'src/weld_seam_perception/config/gooseneck.yaml').read_text())[
            'weld_seam_node']['ros__parameters']
        cls.settings = dict(item.split('=', 1) for item in config['algorithm_overrides'])
        with tempfile.TemporaryDirectory(prefix='gooseneck_tcp_contract_') as folder:
            command = preview_command(
                ROOT, '/home/mini/scut_weld_data/pointclouds/live_20261104_160336_814.ply',
                folder, 'replay', cls.settings)
            result = subprocess.run(command, capture_output=True, text=True, timeout=30)
            if result.returncode:
                raise AssertionError(result.stderr)
            with (Path(folder) / 'replay_features.csv').open(newline='') as stream:
                cls.rows = list(csv.DictReader(stream))

    def test_default_axes_preserve_reference_sign_and_physical_z_convention(self):
        self.assertEqual(self.settings['gooseneck.tool_x_positive'], 'true')
        self.assertEqual(self.settings['gooseneck.tool_z_to_body'], 'false')
        capture_axes = self.calibration[:3, :3].T
        for row in self.rows:
            rotation = quaternion_to_rotation(*(float(row[key]) for key in ('qx', 'qy', 'qz', 'qw')))
            # Fixture-specific checks: avoid the previously introduced 180° X flip.
            # Joint-aware outward lean intentionally rotates the axes; proximity
            # to the old capture Z is no longer an acceptance requirement.
            self.assertGreater(float(rotation[:, 0] @ capture_axes[:, 0]), 0.4)
            np.testing.assert_allclose(rotation.T @ rotation, np.eye(3), atol=1e-9)
            self.assertAlmostEqual(float(np.linalg.det(rotation)), 1.0)
            body = np.array([float(row[f'world_torch_body_axis_{axis}']) for axis in 'xyz'])
            np.testing.assert_allclose(rotation[:, 2], -body, atol=1e-9)

    def test_production_transform_and_abb_frame_preserve_all_eight_poses(self):
        source = PoseArray()
        source.header.frame_id = 'camera_link'
        expected_camera = []
        for row in self.rows:
            pose = Pose()
            pose.position.x, pose.position.y, pose.position.z = [float(row[k])*0.001 for k in 'xyz']
            pose.orientation.w, pose.orientation.x, pose.orientation.y, pose.orientation.z = [
                float(row[k]) for k in ('qw', 'qx', 'qy', 'qz')]
            source.poses.append(pose)
            transform = np.eye(4)
            transform[:3, 3] = [float(row[k])*0.001 for k in 'xyz']
            transform[:3, :3] = quaternion_to_rotation(
                *(float(row[k]) for k in ('qx', 'qy', 'qz', 'qw')))
            expected_camera.append(transform)
        bridge = InMemoryBridge(self.calibration)
        capture = bridge.base_from_tcp.copy()
        bridge._poses_callback(source)
        self.assertTrue(bridge.statuses[-1][0])
        self.assertEqual(len(bridge.pose_pub.messages), 1)
        self.assertEqual(len(bridge.tx_pub.messages), 1)
        output = bridge.pose_pub.messages[0]
        wire = bridge.tx_pub.messages[0].data
        self.assertEqual(output.header.frame_id, 'robot_base')
        self.assertEqual(len(output.poses), 8)
        lines = wire.splitlines()
        self.assertEqual(len(lines), 10)
        self.assertEqual(lines[0], 'TRAJECTORY_BEGIN:8')
        self.assertEqual(lines[-1], 'TRAJECTORY_END')
        for i, camera in enumerate(expected_camera):
            expected = capture @ self.calibration @ camera
            prefix, payload = lines[i+1].split(':', 1)
            self.assertEqual(prefix, f'P{i+1}')
            values = [float(item) for item in payload.split(',')]
            self.assertEqual(len(values), 7)
            np.testing.assert_allclose(values[:3], expected[:3, 3]*1000, atol=1e-6, rtol=0)
            rotation = quaternion_to_rotation(values[4], values[5], values[6], values[3])
            np.testing.assert_allclose(rotation, expected[:3, :3], atol=2e-6, rtol=0)
        self.assertFalse(bridge._armed)
        self.assertIsNone(bridge.base_from_tcp)
        print('PASS: 8 goose poses -> current hand-eye -> ABB mm/wxyz text (in-memory only)')


if __name__ == '__main__':
    unittest.main()
