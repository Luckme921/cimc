#!/usr/bin/env python3
"""Transform camera-frame weld poses with an OpenCV hand-eye matrix."""

import json
import math
import os
import re
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, String


def quaternion_to_rotation(x, y, z, w):
    q = np.asarray([w, x, y, z], dtype=float)
    norm = np.linalg.norm(q)
    if not np.isfinite(norm) or norm < 1.0e-12:
        raise ValueError('input pose contains an invalid quaternion')
    w, x, y, z = q / norm
    return np.asarray([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)]],
        dtype=float)


def rotation_to_quaternion(rotation):
    # Stable branch-based conversion; result is ROS x,y,z,w.
    m = rotation
    trace = float(np.trace(m))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        w, x, y, z = 0.25*s, (m[2, 1]-m[1, 2])/s, \
            (m[0, 2]-m[2, 0])/s, (m[1, 0]-m[0, 1])/s
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = math.sqrt(1.0 + m[0, 0]-m[1, 1]-m[2, 2]) * 2.0
        w, x, y, z = (m[2, 1]-m[1, 2])/s, 0.25*s, \
            (m[0, 1]+m[1, 0])/s, (m[0, 2]+m[2, 0])/s
    elif m[1, 1] > m[2, 2]:
        s = math.sqrt(1.0 + m[1, 1]-m[0, 0]-m[2, 2]) * 2.0
        w, x, y, z = (m[0, 2]-m[2, 0])/s, \
            (m[0, 1]+m[1, 0])/s, 0.25*s, (m[1, 2]+m[2, 1])/s
    else:
        s = math.sqrt(1.0 + m[2, 2]-m[0, 0]-m[1, 1]) * 2.0
        w, x, y, z = (m[1, 0]-m[0, 1])/s, \
            (m[0, 2]+m[2, 0])/s, (m[1, 2]+m[2, 1])/s, 0.25*s
    q = np.asarray([x, y, z, w], dtype=float)
    return q / np.linalg.norm(q)


def load_opencv_matrix(path, key):
    text = Path(path).read_text(encoding='utf-8')
    match = re.search(
        rf'{re.escape(key)}\s*:\s*!!opencv-matrix.*?data\s*:\s*\[([^]]+)\]',
        text, flags=re.DOTALL)
    if not match:
        raise ValueError(f'cannot find OpenCV matrix key {key!r} in {path}')
    values = [float(value) for value in re.findall(
        r'[-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?', match.group(1))]
    if len(values) != 16:
        raise ValueError(f'{key} must contain 16 values, got {len(values)}')
    matrix = np.asarray(values, dtype=float).reshape(4, 4)
    if not np.all(np.isfinite(matrix)) or not np.allclose(
            matrix[3], [0, 0, 0, 1], atol=1.0e-5):
        raise ValueError('hand-eye matrix is not a finite homogeneous transform')
    # Reject a damaged/non-rigid calibration instead of silently sending bad poses.
    rotation = matrix[:3, :3]
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=5.0e-3) \
            or np.linalg.det(rotation) < 0.99:
        raise ValueError('hand-eye rotation is not a valid right-handed rotation')
    return matrix


class HandeyeAbbBridgeNode(Node):
    def __init__(self):
        super().__init__('handeye_abb_bridge_node')
        default_matrix = str(Path(get_package_share_directory('cimc')) /
                             'config' / 'handeye_result20260723.yaml')
        self.matrix_file = os.path.expanduser(self.declare_parameter(
            'matrix_file', default_matrix).value)
        self.matrix_key = self.declare_parameter(
            'matrix_key', 'handEyeMatrix').value
        self.matrix_direction = self.declare_parameter(
            'matrix_direction', 'unconfigured').value.lower()
        self.translation_unit = self.declare_parameter(
            'matrix_translation_unit', 'mm').value.lower()
        self.output_frame = self.declare_parameter(
            'output_frame_id', 'robot_base').value
        self.require_capture_pose = bool(self.declare_parameter(
            'require_capture_pose', True).value)
        self.require_task_armed = bool(self.declare_parameter(
            'require_task_armed', True).value)
        self.send_to_abb = bool(self.declare_parameter(
            'send_to_abb', False).value)
        self.protocol_precision = int(self.declare_parameter(
            'protocol_precision', 6).value)
        if self.translation_unit not in ('mm', 'm'):
            raise ValueError('matrix_translation_unit must be mm or m')
        if self.matrix_direction not in (
                'unconfigured', 'tcp_from_camera', 'camera_from_tcp'):
            raise ValueError(
                'matrix_direction must be unconfigured, tcp_from_camera, '
                'or camera_from_tcp')
        if not 0 <= self.protocol_precision <= 9:
            raise ValueError('protocol_precision must be within [0, 9]')

        self.transform = load_opencv_matrix(self.matrix_file, self.matrix_key)
        if self.translation_unit == 'mm':
            self.transform[:3, 3] *= 0.001  # PoseArray always uses metres.
        if self.matrix_direction == 'camera_from_tcp':
            self.transform = np.linalg.inv(self.transform)
        self.base_from_tcp = None
        self._armed = not self.require_task_armed

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.pose_pub = self.create_publisher(
            PoseArray, '/abb/trajectory_tcp', transient)
        self.tx_pub = self.create_publisher(String, '/abb/tx_text', QoSProfile(depth=10))
        self.status_pub = self.create_publisher(
            String, '/handeye_bridge/status', QoSProfile(depth=10))
        self.create_subscription(
            Bool, '/weld_task/trajectory_armed', self._armed_callback, transient)
        self.create_subscription(
            PoseStamped, '/weld_task/capture_tcp_pose_base',
            self._capture_pose_callback, transient)
        self.create_subscription(
            PoseArray, '/weld_seam/poses_camera_frame', self._poses_callback,
            transient)
        self.get_logger().info(
            f'Hand-eye bridge ready: file={self.matrix_file}, key={self.matrix_key}, '
            f'direction={self.matrix_direction}, output_frame={self.output_frame}, '
            f'require_task_armed={self.require_task_armed}, '
            f'send_to_abb={self.send_to_abb}')

    def _status(self, success, message, **extra):
        body = {'success': success, 'message': message}
        body.update(extra)
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(msg)

    def _armed_callback(self, msg):
        self._armed = msg.data

    def _capture_pose_callback(self, msg):
        transform = np.eye(4)
        try:
            transform[:3, :3] = quaternion_to_rotation(
                msg.pose.orientation.x, msg.pose.orientation.y,
                msg.pose.orientation.z, msg.pose.orientation.w)
            transform[:3, 3] = [msg.pose.position.x,
                                msg.pose.position.y,
                                msg.pose.position.z]
        except ValueError as exc:
            self.base_from_tcp = None
            self._status(False, f'invalid capture TCP pose: {exc}')
            return
        self.base_from_tcp = transform

    def _poses_callback(self, msg):
        if self.require_task_armed and not self._armed:
            self.get_logger().warn('Ignoring retained/unarmed camera PoseArray.')
            return
        if self.matrix_direction == 'unconfigured':
            self._status(False,
                         'matrix_direction is unconfigured; refusing transformation')
            return
        if self.require_capture_pose and self.base_from_tcp is None:
            self._status(False, 'no Base_from_TCP pose was received for this capture')
            return
        if not msg.poses:
            self._status(False, 'input PoseArray is empty')
            return
        output = PoseArray()
        output.header.stamp = self.get_clock().now().to_msg()
        output.header.frame_id = self.output_frame
        try:
            previous_q = None
            for source in msg.poses:
                input_tf = np.eye(4)
                input_tf[:3, :3] = quaternion_to_rotation(
                    source.orientation.x, source.orientation.y,
                    source.orientation.z, source.orientation.w)
                input_tf[:3, 3] = [source.position.x,
                                    source.position.y,
                                    source.position.z]
                base_from_tcp = (self.base_from_tcp if self.base_from_tcp is not None
                                 else np.eye(4))
                # Base<-Tool = Base<-TCP(capture) * TCP<-Camera * Camera<-Tool
                result = base_from_tcp @ self.transform @ input_tf
                qx, qy, qz, qw = rotation_to_quaternion(result[:3, :3])
                current_q = np.asarray([qx, qy, qz, qw])
                # q and -q encode the same rotation. Keep one hemisphere so
                # ABB interpolation never takes a false 360-degree jump.
                if previous_q is not None and np.dot(previous_q, current_q) < 0:
                    current_q = -current_q
                qx, qy, qz, qw = current_q
                previous_q = current_q
                pose = Pose()
                pose.position.x, pose.position.y, pose.position.z = result[:3, 3]
                pose.orientation.x = float(qx)
                pose.orientation.y = float(qy)
                pose.orientation.z = float(qz)
                pose.orientation.w = float(qw)
                output.poses.append(pose)
        except Exception as exc:
            self._status(False, f'pose transformation failed: {exc}')
            return

        self.pose_pub.publish(output)
        if self.send_to_abb:
            tx = String()
            tx.data = self._serialize_for_abb(output)
            self.tx_pub.publish(tx)
        self._status(True, 'trajectory transformed', count=len(output.poses),
                     frame_id=self.output_frame, sent_to_abb=self.send_to_abb)
        self._armed = not self.require_task_armed
        if self.require_capture_pose:
            self.base_from_tcp = None

    def _serialize_for_abb(self, poses):
        # Newline framing is deliberate: ABB must buffer TCP and parse complete
        # lines. Quaternion order in this wire format is qw,qx,qy,qz.
        p = self.protocol_precision
        lines = [f'TRAJECTORY_BEGIN:{len(poses.poses)}']
        for index, pose in enumerate(poses.poses, start=1):
            lines.append(
                f'P{index}:'
                f'{pose.position.x * 1000.0:.{p}f},'
                f'{pose.position.y * 1000.0:.{p}f},'
                f'{pose.position.z * 1000.0:.{p}f},'
                f'{pose.orientation.w:.{p}f},'
                f'{pose.orientation.x:.{p}f},'
                f'{pose.orientation.y:.{p}f},'
                f'{pose.orientation.z:.{p}f}')
        lines.append('TRAJECTORY_END')
        return '\n'.join(lines) + '\n'


def main(args=None):
    rclpy.init(args=args)
    node = HandeyeAbbBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
