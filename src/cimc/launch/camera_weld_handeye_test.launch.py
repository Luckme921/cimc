#!/usr/bin/env python3
"""Start the camera-to-hand-eye test chain without any robot actuator nodes."""

from pathlib import Path

import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def _validate_test_configuration(context, *config_substitutions):
    config_paths = [
        Path(context.perform_substitution(value)).expanduser()
        for value in config_substitutions
    ]
    for path in config_paths:
        if not path.is_file():
            raise RuntimeError(f'ROS parameter file does not exist: {path}')

    handeye_config = yaml.safe_load(config_paths[2].read_text(encoding='utf-8'))
    handeye_params = handeye_config.get(
        'handeye_abb_bridge_node', {}).get('ros__parameters', {})
    if handeye_params.get('send_to_abb') is not False:
        raise RuntimeError(
            'This test launch requires handeye_bridge.yaml send_to_abb=false.')

    matrix_path = Path(str(handeye_params.get('matrix_file', ''))).expanduser()
    if not matrix_path.is_file():
        raise RuntimeError(f'Hand-eye matrix file does not exist: {matrix_path}')

    return [LogInfo(msg=[
        'Configuration preflight passed: matrix_direction=',
        str(handeye_params.get('matrix_direction', '<missing>')),
        ', send_to_abb=false.'])]


def generate_launch_description():
    workspace_root = LaunchConfiguration('workspace_root')
    camera_params_file = LaunchConfiguration('camera_params_file')
    weld_params_file = LaunchConfiguration('weld_params_file')
    handeye_params_file = LaunchConfiguration('handeye_params_file')
    coordinator_params_file = LaunchConfiguration('coordinator_params_file')
    echo_status = LaunchConfiguration('echo_status')
    echo_trajectory = LaunchConfiguration('echo_trajectory')

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace_root',
            default_value='/home/mini/x86_ros2_ws',
            description='Main ROS 2 workspace containing the editable src configs.'),
        DeclareLaunchArgument(
            'camera_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'chishine_camera_ros2',
                'config', 'camera.yaml']),
            description='Chishine camera ROS parameter file.'),
        DeclareLaunchArgument(
            'weld_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'weld_seam_perception',
                'config', 'weld_seam.yaml']),
            description='Weld seam ROS parameter file.'),
        DeclareLaunchArgument(
            'handeye_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config', 'handeye_bridge.yaml']),
            description='Hand-eye bridge ROS parameter file.'),
        DeclareLaunchArgument(
            'coordinator_params_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config',
                'weld_task_coordinator.yaml']),
            description='Weld task coordinator ROS parameter file.'),
        DeclareLaunchArgument(
            'echo_status',
            default_value='true',
            choices=['true', 'false'],
            description='Print coordinator, weld, and hand-eye status topics.'),
        DeclareLaunchArgument(
            'echo_trajectory',
            default_value='true',
            choices=['true', 'false'],
            description='Print the next /abb/trajectory_tcp message in this terminal.'),

        LogInfo(msg='Starting camera + weld perception + hand-eye test chain.'),
        LogInfo(msg=[
            'Editable camera config: ', camera_params_file,
            '; weld config: ', weld_params_file,
            '; hand-eye config: ', handeye_params_file,
            '; coordinator config: ', coordinator_params_file]),
        LogInfo(msg='Safety boundary: ABB TCP, weld controller, weld logic, and motor nodes are NOT started; send_to_abb=false.'),
        LogInfo(msg='Node parameters come only from the listed YAML files; this launch does not override parameter values.'),
        LogInfo(msg='After all nodes report ready, publish exactly one START_CAPTURE:x,y,z,qw,qx,qy,qz command.'),
        OpaqueFunction(
            function=_validate_test_configuration,
            args=[
                camera_params_file,
                weld_params_file,
                handeye_params_file,
                coordinator_params_file,
            ]),

        Node(
            package='chishine_camera_ros2',
            executable='chishine_camera_node',
            name='chishine_camera_node',
            parameters=[camera_params_file],
            output='screen',
            emulate_tty=True),
        Node(
            package='weld_seam_perception',
            executable='weld_seam_node',
            name='weld_seam_node',
            parameters=[weld_params_file],
            output='screen',
            emulate_tty=True),
        Node(
            package='cimc',
            executable='handeye_abb_bridge_node',
            name='handeye_abb_bridge_node',
            parameters=[handeye_params_file],
            output='screen',
            emulate_tty=True),
        Node(
            package='cimc',
            executable='weld_task_coordinator_node',
            name='weld_task_coordinator_node',
            parameters=[coordinator_params_file],
            output='screen',
            emulate_tty=True),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '/weld_task/status',
                'std_msgs/msg/String'],
            name='weld_task_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '/weld_seam/status',
                'std_msgs/msg/String'],
            name='weld_seam_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '/handeye_bridge/status',
                'std_msgs/msg/String'],
            name='handeye_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '--once',
                '--qos-durability', 'volatile',
                '--qos-reliability', 'reliable',
                '/abb/trajectory_tcp', 'geometry_msgs/msg/PoseArray'],
            name='trajectory_echo_once',
            output='screen',
            condition=IfCondition(echo_trajectory)),
    ])
