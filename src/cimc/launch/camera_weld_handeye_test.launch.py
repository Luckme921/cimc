#!/usr/bin/env python3
"""Start the camera-to-hand-eye test chain without any robot actuator nodes."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    workspace_root = LaunchConfiguration('workspace_root')
    camera_params_file = LaunchConfiguration('camera_params_file')
    weld_params_file = LaunchConfiguration('weld_params_file')
    matrix_file = LaunchConfiguration('matrix_file')
    matrix_direction = LaunchConfiguration('matrix_direction')
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
            'matrix_file',
            default_value=PathJoinSubstitution([
                workspace_root, 'src', 'cimc', 'config',
                'handeye_result20260723.yaml']),
            description='OpenCV YAML hand-eye matrix file.'),
        DeclareLaunchArgument(
            'matrix_direction',
            default_value='tcp_from_camera',
            choices=['tcp_from_camera', 'camera_from_tcp'],
            description='Direction represented by handEyeMatrix.'),
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
            '; hand-eye matrix: ', matrix_file]),
        LogInfo(msg='Safety boundary: ABB TCP, weld controller, weld logic, and motor nodes are NOT started; send_to_abb=false.'),
        LogInfo(msg='After all nodes report ready, publish exactly one START_CAPTURE:x,y,z,qw,qx,qy,qz command.'),

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
            parameters=[{
                'matrix_file': matrix_file,
                'matrix_direction': matrix_direction,
                'matrix_translation_unit': 'mm',
                'output_frame_id': 'robot_base',
                'require_capture_pose': True,
                'require_task_armed': True,
                'send_to_abb': False,
            }],
            output='screen',
            emulate_tty=True),
        Node(
            package='cimc',
            executable='weld_task_coordinator_node',
            name='weld_task_coordinator_node',
            parameters=[{
                'weld_auto_process': True,
                'require_capture_pose': True,
                'capture_pose_unit': 'mm',
                'base_frame_id': 'robot_base',
            }],
            output='screen',
            emulate_tty=True),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/weld_task/status'],
            name='weld_task_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/weld_seam/status'],
            name='weld_seam_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=['ros2', 'topic', 'echo', '/handeye_bridge/status'],
            name='handeye_status_echo',
            output='screen',
            condition=IfCondition(echo_status)),
        ExecuteProcess(
            cmd=[
                'ros2', 'topic', 'echo', '--once',
                '--qos-durability', 'transient_local',
                '--qos-reliability', 'reliable',
                '/abb/trajectory_tcp'],
            name='trajectory_echo_once',
            output='screen',
            condition=IfCondition(echo_trajectory)),
    ])
