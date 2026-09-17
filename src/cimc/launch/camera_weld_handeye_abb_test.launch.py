#!/usr/bin/env python3
"""Start the camera-to-ABB I/O test chain without robot motion nodes."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _load_parameters(path, node_name):
    document = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    return document.get(node_name, {}).get('ros__parameters', {})


def _validate_abb_test_configuration(context, *config_substitutions):
    config_paths = [
        Path(context.perform_substitution(value)).expanduser()
        for value in config_substitutions
    ]
    for path in config_paths:
        if not path.is_file():
            raise RuntimeError(f'ROS parameter file does not exist: {path}')

    handeye_params = _load_parameters(
        config_paths[2], 'handeye_abb_bridge_node')
    if handeye_params.get('send_to_abb') is not True:
        raise RuntimeError(
            'ABB I/O test requires handeye_bridge.yaml send_to_abb=true.')

    matrix_path = Path(str(
        handeye_params.get('matrix_file', ''))).expanduser()
    if not matrix_path.is_file():
        raise RuntimeError(f'Hand-eye matrix file does not exist: {matrix_path}')

    receiver_params = _load_parameters(
        config_paths[4], 'data_receiver_node')
    listen_host = str(receiver_params.get('listen_host', '')).strip()
    allowed_ip = str(receiver_params.get('abb_allowed_ip', '')).strip()
    listen_port = receiver_params.get('listen_port')
    if not listen_host or not allowed_ip or listen_port is None:
        raise RuntimeError(
            'data_receiver.yaml must define listen_host, listen_port and '
            'abb_allowed_ip.')
    if listen_host == allowed_ip:
        raise RuntimeError(
            'listen_host is the x86 address and must differ from abb_allowed_ip.')

    return [LogInfo(msg=[
        'ABB I/O preflight passed: x86=', listen_host, ':',
        str(listen_port), ', ABB=', allowed_ip,
        ', send_to_abb=true.'])]


def generate_launch_description():
    workspace_root = LaunchConfiguration('workspace_root')
    camera_params_file = LaunchConfiguration('camera_params_file')
    weld_params_file = LaunchConfiguration('weld_params_file')
    handeye_params_file = LaunchConfiguration('handeye_params_file')
    coordinator_params_file = LaunchConfiguration('coordinator_params_file')
    data_receiver_params_file = LaunchConfiguration('data_receiver_params_file')
    echo_status = LaunchConfiguration('echo_status')
    echo_abb_io = LaunchConfiguration('echo_abb_io')
    echo_trajectory = LaunchConfiguration('echo_trajectory')

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace_root',
            default_value='/home/mini/x86_ros2_ws',
            description='Main ROS 2 workspace containing editable src configs.'),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'chishine_camera_ros2',
                'config', 'camera.yaml'])),
        DeclareLaunchArgument(
            'weld_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'weld_seam_perception',
                'config', 'weld_seam.yaml'])),
        DeclareLaunchArgument(
            'handeye_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config',
                'handeye_bridge.yaml'])),
        DeclareLaunchArgument(
            'coordinator_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config',
                'weld_task_coordinator.yaml'])),
        DeclareLaunchArgument(
            'data_receiver_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config',
                'data_receiver.yaml'])),
        DeclareLaunchArgument(
            'echo_status', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'echo_abb_io', default_value='true',
            choices=['true', 'false']),
        DeclareLaunchArgument(
            'echo_trajectory', default_value='true',
            choices=['true', 'false']),

        LogInfo(msg='Starting camera + weld + hand-eye + ABB I/O test chain.'),
        LogInfo(msg=(
            'Safety boundary: the ABB peer may only store/print received points. '
            'No robot motion, weld-controller, weld-logic, or motor node is started.')),
        LogInfo(msg=(
            'All node parameters come from src config YAML files; this launch '
            'does not override parameter values.')),
        LogInfo(msg=(
            'Wait for every node to report ready and for ABB TCP to connect '
            'before ABB sends one newline-terminated START_CAPTURE command.')),
        OpaqueFunction(
            function=_validate_abb_test_configuration,
            args=[
                camera_params_file,
                weld_params_file,
                handeye_params_file,
                coordinator_params_file,
                data_receiver_params_file,
            ]),

        Node(
            package='chishine_camera_ros2',
            executable='chishine_camera_node',
            name='chishine_camera_node',
            parameters=[camera_params_file],
            output='screen', emulate_tty=True),
        Node(
            package='weld_seam_perception',
            executable='weld_seam_node',
            name='weld_seam_node',
            parameters=[weld_params_file],
            output='screen', emulate_tty=True),
        Node(
            package='cimc',
            executable='handeye_abb_bridge_node',
            name='handeye_abb_bridge_node',
            parameters=[handeye_params_file],
            output='screen', emulate_tty=True),
        Node(
            package='cimc',
            executable='weld_task_coordinator_node',
            name='weld_task_coordinator_node',
            parameters=[coordinator_params_file],
            output='screen', emulate_tty=True),
        Node(
            package='cimc',
            executable='data_receiver_node',
            name='data_receiver_node',
            parameters=[data_receiver_params_file],
            output='screen', emulate_tty=True),

        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/weld_task/status',
                 'std_msgs/msg/String'],
            name='weld_task_status_echo', output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/weld_seam/status',
                 'std_msgs/msg/String'],
            name='weld_seam_status_echo', output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/handeye_bridge/status',
                 'std_msgs/msg/String'],
            name='handeye_status_echo', output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/abb/raw_text',
                 'std_msgs/msg/String'],
            name='abb_raw_text_echo', output='screen',
            condition=IfCondition(echo_abb_io)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/abb/tx_text',
                 'std_msgs/msg/String'],
            name='abb_tx_text_echo', output='screen',
            condition=IfCondition(echo_abb_io)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/abb/tx_status',
                 'std_msgs/msg/String'],
            name='abb_tx_status_echo', output='screen',
            condition=IfCondition(echo_abb_io)),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '--once',
                '--qos-durability', 'volatile',
                '--qos-reliability', 'reliable',
                '/abb/trajectory_tcp', 'geometry_msgs/msg/PoseArray'],
            name='trajectory_echo_once', output='screen',
            condition=IfCondition(echo_trajectory)),
    ])
