#!/usr/bin/env python3
"""Fixed-fixture or one-time operator-reviewed ABB perception workflow."""

from pathlib import Path
import yaml

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, EmitEvent, ExecuteProcess,
                            IncludeLaunchDescription, LogInfo, OpaqueFunction,
                            RegisterEventHandler, SetEnvironmentVariable)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node

def _validate_abb_test_configuration(context, *files):
    paths = [Path(context.perform_substitution(item)).expanduser() for item in files]
    if not all(path.is_file() for path in paths):
        raise RuntimeError('人工模式：相机/算法/手眼/协调/ABB 配置文件缺失')
    params = [yaml.safe_load(path.read_text(encoding='utf-8')) for path in paths]
    handeye = params[2]['handeye_abb_bridge_node']['ros__parameters']
    receiver = params[4]['data_receiver_node']['ros__parameters']
    coordinator = params[3]['weld_task_coordinator_node']['ros__parameters']
    if handeye.get('send_to_abb') is not True:
        raise RuntimeError('ABB 模式要求 handeye_bridge.yaml send_to_abb=true')
    # The operator may select a new calibrated matrix in the setup window.
    # Rigid-transform validation happens there before runtime nodes start.
    if receiver.get('third_party_link_enabled') is not False:
        raise RuntimeError('ABB 模式要求 third_party_link_enabled=false')
    if (receiver.get('abb_capture_command') != coordinator.get('start_command') or
            receiver.get('listen_host') == receiver.get('abb_allowed_ip')):
        raise RuntimeError('ABB 拍照命令或 IP 配置不一致')
    return [LogInfo(msg='人工模式配置预检通过；参数仍来自 src 下各 YAML。')]


def _manual_runtime(context, event, *args):
    if event.returncode != 0:
        return [LogInfo(msg='人工标定未确认；不启动相机/焊缝/手眼自动链路。'),
                EmitEvent(event=Shutdown(reason='operator setup cancelled or failed'))]
    camera, weld, handeye, coordinator, echo, status_window, workspace = args
    actions = [LogInfo(msg=(
        '人工确认已完成。预览相机已退出；现在启动相机、焊缝、手眼与协调节点。'
        '之后可连续处理多条 ABB 拍照请求。'))]
    for package, executable, config in (
            ('chishine_camera_ros2', 'chishine_camera_node', camera),
            ('weld_seam_perception', 'weld_seam_node', weld),
            ('cimc', 'handeye_abb_bridge_node', handeye),
            ('cimc', 'weld_task_coordinator_node', coordinator)):
        # The camera SDK queries LAN device metadata over HTTP. A desktop
        # proxy can make discovery report zero devices despite UDP replies.
        camera_env = ({'NO_PROXY': '*', 'no_proxy': '*'}
                      if executable == 'chishine_camera_node' else {})
        actions.append(Node(package=package, executable=executable,
                            name=executable, parameters=[config],
                            additional_env=camera_env,
                            output='screen', emulate_tty=True))
    if context.perform_substitution(status_window) == 'true':
        actions.append(ExecuteProcess(
            cmd=['python3', PathJoinSubstitution([
                workspace, 'src', 'cimc', 'cimc', 'weld_operator_status.py'])],
            name='weld_operator_status', output='screen'))
    if context.perform_substitution(echo) == 'true':
        for name, topic in (
                ('weld_task_status_echo', '/weld_task/status'),
                ('weld_seam_status_echo', '/weld_seam/status'),
                ('handeye_status_echo', '/handeye_bridge/status'),
                ('abb_raw_text_echo', '/abb/raw_text'),
                ('abb_tx_status_echo', '/abb/tx_status'),
                ('abb_task_timing_echo', '/abb/task_timing')):
            actions.append(ExecuteProcess(
                cmd=['ros2', 'topic', 'echo', topic, 'std_msgs/msg/String'],
                name=name, output='screen'))
    return actions


def _select_mode(context, *args):
    mode, workspace, camera, weld, handeye, coordinator, receiver, echo, status_window = args
    if context.perform_substitution(mode) == 'fixed':
        old_launch = Path(context.perform_substitution(workspace)) / (
            'src/cimc/launch/camera_weld_handeye_abb_test.launch.py')
        return [LogInfo(msg='固定夹具模式：沿用现有自动 launch，不改变参数来源。'),
                IncludeLaunchDescription(PythonLaunchDescriptionSource(str(old_launch)),
                    launch_arguments={
                        'workspace_root': workspace,
                        'camera_params_file': camera,
                        'weld_params_file': weld,
                        'handeye_params_file': handeye,
                        'coordinator_params_file': coordinator,
                        'data_receiver_params_file': receiver,
                        'echo_status': echo,
                        'echo_abb_io': echo,
                        'echo_trajectory': echo,
                    }.items())]
    preflight = _validate_abb_test_configuration(
        context, camera, weld, handeye, coordinator, receiver)
    setup = ExecuteProcess(
        cmd=['python3', PathJoinSubstitution([
                 workspace, 'src', 'cimc', 'cimc', 'weld_operator_setup.py']),
             workspace, weld, camera, handeye],
        name='weld_operator_setup', output='screen')
    return preflight + [
        LogInfo(msg=(
            '人工模式：先开启 ABB TCP 监听以保持通讯；确认前拒绝拍照请求。'
            '请让 ABB 程序在就绪提示后再发送 START_CAPTURE。')),
        Node(package='cimc', executable='data_receiver_node',
             name='data_receiver_node',
             parameters=[receiver, {'hold_capture_until_coordinator_ready': True}],
             output='screen', emulate_tty=True),
        RegisterEventHandler(OnProcessExit(
            target_action=setup,
            on_exit=lambda event, ctx: _manual_runtime(
                ctx, event, camera, weld, handeye, coordinator,
                echo, status_window, workspace))),
        setup,
    ]


def generate_launch_description():
    workspace = LaunchConfiguration('workspace_root')
    camera = LaunchConfiguration('camera_params_file')
    weld = LaunchConfiguration('weld_params_file')
    handeye = LaunchConfiguration('handeye_params_file')
    coordinator = LaunchConfiguration('coordinator_params_file')
    receiver = LaunchConfiguration('data_receiver_params_file')
    mode = LaunchConfiguration('workflow_mode')
    echo = LaunchConfiguration('echo_topics')
    status_window = LaunchConfiguration('show_operator_status')
    use_shared_memory = LaunchConfiguration('use_shared_memory')
    return LaunchDescription([
        DeclareLaunchArgument('workspace_root', default_value='/home/mini/x86_ros2_ws'),
        DeclareLaunchArgument('workflow_mode', default_value='manual',
                              choices=['manual', 'fixed']),
        DeclareLaunchArgument('echo_topics', default_value='false',
                              choices=['true', 'false']),
        DeclareLaunchArgument('show_operator_status', default_value='true',
                              choices=['true', 'false']),
        DeclareLaunchArgument('use_shared_memory', default_value='true',
                              choices=['true', 'false']),
        # Fast DDS DEFAULT = SHM for same-host peers, UDPv4 fallback/discovery.
        # Keep both transports: SHM-only would break remote ROS diagnostics.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_fastrtps_cpp',
                               condition=IfCondition(use_shared_memory)),
        SetEnvironmentVariable('FASTDDS_BUILTIN_TRANSPORTS', 'DEFAULT',
                               condition=IfCondition(use_shared_memory)),
        DeclareLaunchArgument('camera_params_file', default_value=PathJoinSubstitution(
            [workspace, 'src', 'chishine_camera_ros2', 'config', 'camera.yaml'])),
        DeclareLaunchArgument('weld_params_file', default_value=PathJoinSubstitution(
            [workspace, 'src', 'weld_seam_perception', 'config', 'weld_seam.yaml'])),
        DeclareLaunchArgument('handeye_params_file', default_value=PathJoinSubstitution(
            [workspace, 'src', 'cimc', 'config', 'handeye_bridge.yaml'])),
        DeclareLaunchArgument('coordinator_params_file', default_value=PathJoinSubstitution(
            [workspace, 'src', 'cimc', 'config', 'weld_task_coordinator.yaml'])),
        DeclareLaunchArgument('data_receiver_params_file', default_value=PathJoinSubstitution(
            [workspace, 'src', 'cimc', 'config', 'data_receiver.yaml'])),
        OpaqueFunction(function=_select_mode,
                       args=[mode, workspace, camera, weld, handeye,
                             coordinator, receiver, echo, status_window]),
    ])
