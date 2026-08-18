#!/usr/bin/env python3
"""Coordinate one ABB START_CAPTURE request with camera and weld extraction."""

import json
import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


class WeldTaskCoordinatorNode(Node):
    def __init__(self):
        super().__init__('weld_task_coordinator_node')
        self.start_command = self.declare_parameter(
            'start_command', 'START_CAPTURE').value.strip()
        self.camera_service = self.declare_parameter(
            'camera_service', '/camera/capture').value
        self.auto_process = bool(self.declare_parameter(
            'weld_auto_process', True).value)
        self.extract_service = self.declare_parameter(
            'extract_service', '/weld_seam/extract_latest').value
        self.service_wait_timeout_s = float(self.declare_parameter(
            'service_wait_timeout_s', 3.0).value)
        self.require_capture_pose = bool(self.declare_parameter(
            'require_capture_pose', True).value)
        self.capture_pose_unit = self.declare_parameter(
            'capture_pose_unit', 'mm').value.lower()
        self.base_frame_id = self.declare_parameter(
            'base_frame_id', 'robot_base').value
        if self.capture_pose_unit not in ('mm', 'm'):
            raise ValueError('capture_pose_unit must be mm or m')

        self._lock = threading.Lock()
        self._busy = False
        self._task_id = 0

        transient = QoSProfile(depth=1)
        transient.reliability = ReliabilityPolicy.RELIABLE
        transient.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_pub = self.create_publisher(
            String, '/weld_task/status', QoSProfile(depth=10))
        self.armed_pub = self.create_publisher(
            Bool, '/weld_task/trajectory_armed', transient)
        self.capture_pose_pub = self.create_publisher(
            PoseStamped, '/weld_task/capture_tcp_pose_base', transient)
        self.create_subscription(
            String, '/abb/raw_text', self._abb_callback, QoSProfile(depth=10))
        self.create_subscription(
            String, '/weld_seam/status', self._weld_status_callback,
            QoSProfile(depth=10))
        self.create_subscription(
            String, '/handeye_bridge/status', self._bridge_status_callback,
            QoSProfile(depth=10))
        self.camera_client = self.create_client(Trigger, self.camera_service)
        self.extract_client = self.create_client(Trigger, self.extract_service)
        self._publish_armed(False)
        self._publish_status('idle', 'waiting for ABB START_CAPTURE')
        self.get_logger().info(
            f'Coordinator ready: command={self.start_command}, '
            f'camera={self.camera_service}, weld_auto_process={self.auto_process}')

    def _publish_status(self, state, message, **extra):
        body = {'state': state, 'task_id': self._task_id, 'message': message}
        body.update(extra)
        msg = String()
        msg.data = json.dumps(body, ensure_ascii=False, separators=(',', ':'))
        self.status_pub.publish(msg)

    def _publish_armed(self, value):
        msg = Bool()
        msg.data = value
        self.armed_pub.publish(msg)

    def _abb_callback(self, msg):
        # ABB TCP may contain multiple newline-delimited commands in one message.
        commands = [line.strip() for line in msg.data.replace('\r', '\n').split('\n')]
        command = next((line for line in commands if
                        line == self.start_command or
                        line.startswith(self.start_command + ':')), None)
        if command is None:
            return
        try:
            capture_pose = self._parse_capture_pose(command)
        except ValueError as exc:
            self._publish_status('fault', str(exc))
            self.get_logger().error(str(exc))
            return
        with self._lock:
            if self._busy:
                self._publish_status('busy', 'duplicate START_CAPTURE ignored')
                return
            self._busy = True
            self._task_id += 1
        if capture_pose is not None:
            self.capture_pose_pub.publish(capture_pose)
        self._publish_armed(True)
        self._publish_status('capturing', 'ABB capture request accepted')

        if not self.camera_client.wait_for_service(
                timeout_sec=self.service_wait_timeout_s):
            self._fail('camera capture service is unavailable')
            return
        future = self.camera_client.call_async(Trigger.Request())
        future.add_done_callback(self._camera_done)

    def _parse_capture_pose(self, command):
        if ':' not in command:
            if self.require_capture_pose:
                raise ValueError(
                    'START_CAPTURE rejected: expected '
                    'START_CAPTURE:x,y,z,qw,qx,qy,qz')
            return None
        fields = [field.strip() for field in command.split(':', 1)[1].split(',')]
        if len(fields) != 7:
            raise ValueError('START_CAPTURE pose must contain exactly 7 numbers')
        try:
            values = [float(field) for field in fields]
        except ValueError as exc:
            raise ValueError('START_CAPTURE pose contains a non-numeric field') from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError('START_CAPTURE pose contains NaN or infinity')
        x, y, z, qw, qx, qy, qz = values
        norm = math.sqrt(qw*qw + qx*qx + qy*qy + qz*qz)
        if norm < 1.0e-9:
            raise ValueError('START_CAPTURE quaternion has zero length')
        scale = 0.001 if self.capture_pose_unit == 'mm' else 1.0
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = self.base_frame_id
        pose.pose.position.x = x * scale
        pose.pose.position.y = y * scale
        pose.pose.position.z = z * scale
        pose.pose.orientation.w = qw / norm
        pose.pose.orientation.x = qx / norm
        pose.pose.orientation.y = qy / norm
        pose.pose.orientation.z = qz / norm
        return pose

    def _camera_done(self, future):
        try:
            response = future.result()
        except Exception as exc:  # service transport/runtime error
            self._fail(f'camera service exception: {exc}')
            return
        if response is None or not response.success:
            reason = response.message if response else 'empty response'
            self._fail(f'camera capture failed: {reason}')
            return
        self._publish_status('captured', 'PLY created', ply=response.message)
        if self.auto_process:
            # Camera node publishes /camera/pointcloud_file. weld_seam_node with
            # auto_process=true consumes it automatically, so no second call here.
            self._publish_status('extracting', 'waiting for automatic weld extraction',
                                 ply=response.message)
            return
        if not self.extract_client.wait_for_service(
                timeout_sec=self.service_wait_timeout_s):
            self._fail('manual weld extraction service is unavailable')
            return
        future = self.extract_client.call_async(Trigger.Request())
        future.add_done_callback(self._manual_extract_done)

    def _manual_extract_done(self, future):
        try:
            response = future.result()
        except Exception as exc:
            self._fail(f'weld extraction service exception: {exc}')
            return
        if response is None or not response.success:
            reason = response.message if response else 'empty response'
            self._fail(f'weld extraction failed: {reason}')

    def _weld_status_callback(self, msg):
        with self._lock:
            if not self._busy:
                return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        # weld_seam_node publishes the SDK integer status: 0 means success.
        result_code = status.get('status')
        if result_code == 0:
            self._publish_status('trajectory_ready', 'weld extraction succeeded',
                                 weld_status=status)
            # Keep the task busy and armed until the bridge confirms that it
            # consumed the matching PoseArray. This blocks overlapping jobs.
        elif isinstance(result_code, int) and result_code != 0:
            self._fail(status.get('message', 'weld extraction failed'))

    def _bridge_status_callback(self, msg):
        with self._lock:
            if not self._busy:
                return
        try:
            status = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if status.get('success') is True:
            self._publish_armed(False)
            self._publish_status('completed', 'trajectory transformed by hand-eye bridge',
                                 bridge_status=status)
            with self._lock:
                self._busy = False
        elif status.get('success') is False:
            self._fail(status.get('message', 'hand-eye bridge failed'))

    def _fail(self, reason):
        self._publish_armed(False)
        self._publish_status('fault', reason)
        with self._lock:
            self._busy = False
        self.get_logger().error(reason)


def main(args=None):
    rclpy.init(args=args)
    node = WeldTaskCoordinatorNode()
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
