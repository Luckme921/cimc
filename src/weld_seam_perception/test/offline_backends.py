#!/usr/bin/env python3
"""Isolated ROS graph: algorithm node + test subscriber only; no camera/ABB nodes."""
import csv
import json
import os
from pathlib import Path
import subprocess
import sys
import time
import yaml

os.environ['ROS_DOMAIN_ID'] = '224'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
import rclpy  # noqa: E402
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy  # noqa: E402
from geometry_msgs.msg import PoseArray  # noqa: E402
from std_msgs.msg import String  # noqa: E402

ROOT = Path(__file__).resolve().parents[3]
DATA = Path('/home/mini/scut_weld_data/pointclouds')
sys.path.insert(0, str(ROOT / 'src/cimc'))
from cimc.weld_operator_setup import active_overrides, make_highlight_ply  # noqa: E402
from cimc.weld_algorithm import preview_command  # noqa: E402


def main():
    output = ROOT / 'operator_sessions/gooseneck_ros_validation'
    output.mkdir(parents=True, exist_ok=True)
    os.environ['ROS_LOG_DIR'] = str(output / 'ros_logs')
    rclpy.init()
    node = rclpy.create_node('offline_weld_backend_test')
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE)
    states, poses = [], []
    node.create_subscription(String, '/weld_seam/status', lambda msg: states.append(json.loads(msg.data)), 10)
    node.create_subscription(PoseArray, '/weld_seam/poses_camera_frame', poses.append, qos)
    publisher = node.create_publisher(String, '/camera/pointcloud_file', qos)
    binary = ROOT / 'install/weld_seam_perception/lib/weld_seam_perception/weld_seam_node'
    try:
        for name, cloud in (
                ('gooseneck', DATA / 'live_20261104_160336_814.ply'),
                ('corrugated', DATA / 'live_20260924_095031_427.ply')):
            config = ROOT / 'src/weld_seam_perception/config' / (
                'gooseneck.yaml' if name == 'gooseneck' else 'weld_seam.yaml')
            overrides = active_overrides(config.read_text(encoding='utf-8'))
            if name == 'corrugated':
                # The field YAML ROI belongs to a different, recent exposure.
                # Use a test-only full-ROI copy for this already cropped 09/24 PLY.
                overrides['roi.enable'] = 'false'
                data = yaml.safe_load(config.read_text(encoding='utf-8'))
                data['weld_seam_node']['ros__parameters']['algorithm_overrides'] = [
                    f'{key}={value}' for key, value in overrides.items()]
                config = output / 'corrugated_test_only.yaml'
                config.write_text(yaml.safe_dump(data, allow_unicode=True), encoding='utf-8')
            states.clear()
            poses.clear()
            # Revoke the retained previous cloud before starting the next node.
            publisher.publish(String(data=''))
            log_path = output / f'{name}_node.log'
            command = [str(binary), '--ros-args', '--params-file', str(config),
                       '-p', f'output_directory:={output}', '-p', f'output_prefix:=ros_{name}']
            with log_path.open('w', encoding='utf-8') as log:
                process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic() + 20
                    while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if process.poll() is not None:
                            raise AssertionError(log_path.read_text())
                    assert publisher.get_subscription_count() > 0, 'algorithm node discovery failed'
                    publisher.publish(String(data=str(cloud)))
                    deadline = time.monotonic() + 40
                    while (not states or not poses) and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
                        if states and states[-1]['status'] != 0:
                            break
                    assert states and states[-1]['status'] == 0, (name, states, log_path.read_text())
                    assert states[-1]['algorithm'] == name
                    assert poses and len(poses[-1].poses) == states[-1]['path_points']
                    assert len(poses[-1].poses) <= 100
                    pose = poses[-1].poses[0]
                    csv_file = Path(states[-1]['csv'])
                    with csv_file.open(newline='', encoding='utf-8') as stream:
                        rows = list(csv.DictReader(stream))
                    assert abs(pose.position.x - float(rows[0]['x']) * 0.001) < 1e-10
                    assert abs(pose.orientation.w - float(rows[0]['qw'])) < 1e-10
                    assert poses[-1].header.frame_id == 'camera_link'
                    direct = preview_command(ROOT, cloud, output, f'direct_{name}', overrides)
                    result = subprocess.run(direct, capture_output=True, text=True, timeout=60)
                    assert result.returncode == 0, result.stderr
                    with (output / f'direct_{name}_features.csv').open(newline='', encoding='utf-8') as stream:
                        assert rows == list(csv.DictReader(stream)), 'online and GUI/CLI use different settings'
                    make_highlight_ply(output / f'direct_{name}_result.ply', rows[0],
                                       output / f'{name}_highlight.ply')
                    print(f'PASS: ROS {name}, {len(rows)} poses, units/quaternion/status correct; '
                          'GUI/CLI result identical and PLY highlighting supported')
                finally:
                    if process.poll() is None:
                        process.send_signal(2)
                        process.wait(timeout=10)
                    deadline = time.monotonic() + 5
                    while publisher.get_subscription_count() and time.monotonic() < deadline:
                        rclpy.spin_once(node, timeout_sec=0.1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
