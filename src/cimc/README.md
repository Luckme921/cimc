# `cimc`：任务协调、手眼与 ABB TCP

这是 ROS 2 Humble 的 `ament_python` 包。当前总 launch 只使用 ABB 接收、任务协调、手眼桥接和可选只读界面；电机控制节点虽保留在包内，**不会随总 launch 启动**。完整运行入口见[工作区 README](../../README.md)和[人工标定流程](人工标定与自动运行.md)。

## 文件与当前角色

| 文件 | 作用 |
| --- | --- |
| `launch/camera_weld_operator_abb.launch.py` | 主入口：人工复核或固定夹具；默认 Fast DDS SHM+UDP、关闭多余 topic echo |
| `cimc/weld_operator_setup.py` | 启动前 ROI、相机、焊缝和手眼参数复核；最终确认才写 src YAML |
| `cimc/weld_operator_status.py` | 运行时只读状态窗，区分待发送、socket 发送与 ABB ACK |
| `cimc/data_receiver_node.py` | ABB TCP 换行分帧、收发和端到端计时 |
| `cimc/weld_task_coordinator_node.py` | 拍照请求状态机和单次轨迹授权 |
| `cimc/handeye_abb_bridge_node.py` | 相机系位姿转换、100 点上限和 ABB 文本序列化 |
| `cimc/motor_control_node.py` | 保留的串口电机控制节点；当前主链路不启动 |
| `config/*.yaml` | 运行参数；主 launch 直接指向 src 下文件 |

总 launch 人工模式先起 `data_receiver_node` 以维持 ABB TCP 连接，但确认前拒绝 `START_CAPTURE`，不缓存旧拍照位姿。确认后才启动相机、焊缝、手眼和协调节点。固定模式直接进入已有 ABB 链路，没有人工窗口。旧 `camera_weld_handeye_test.launch.py` 和 `camera_weld_handeye_abb_test.launch.py` 为保留的调试入口，不等同于主入口。

## 五个关键约定

1. ABB 发 `START_CAPTURE:x,y,z,qw,qx,qy,qz`，XYZ 为 mm；协调器发布拍照时 `Base←TCP` 和一次性授权。
2. 焊缝节点输出相机系 `PoseArray`，位置为 m；手眼桥读取 `handeye_bridge.yaml` 指定的 OpenCV 4×4 矩阵。`tcp_from_camera` 直接使用，`camera_from_tcp` 先求逆。
3. 桥接节点发布 `robot_base` 下的 `/abb/trajectory_tcp`；仅当配置允许、任务授权且点数不超过上限时，才把 mm 与 `qw,qx,qy,qz` 格式的轨迹写到 `/abb/tx_text`。
4. `/abb/tx_text` 只是待发送；`/abb/tx_status` 的 OK 表示本机 socket `sendall()` 成功；`TRAJECTORY_ACK` 表示 ABB 应用层接收确认。三者均不证明机械臂完成运动。
5. `/abb/task_timing` 从完整拍照命令到 `sendall()` 返回计时，不含 ABB 程序解析、存储、规划或执行时间。算法自身耗时见 `/weld_seam/status`。

话题、类型、QoS、坐标单位和 ABB 文本示例统一维护在[节点接口说明](../../ROS2全部节点与接口说明.md)，避免在包内重复一份可能过期的协议表。

## 配置与安全边界

| 配置 | 重点 |
| --- | --- |
| [`config/data_receiver.yaml`](config/data_receiver.yaml) | 本机 ABB 监听、来源 IP、第三方通道开关；当前 `third_party_link_enabled=false` |
| [`config/weld_task_coordinator.yaml`](config/weld_task_coordinator.yaml) | 拍照命令、TCP 单位、相机服务和任务规则 |
| [`config/handeye_bridge.yaml`](config/handeye_bridge.yaml) | 矩阵路径、键名、方向、单位、ABB 发送开关与 100 点上限 |
| [`config/handeye_result20260723.yaml`](config/handeye_result20260723.yaml) | OpenCV 矩阵数据；文件名是历史标识，不代表当前标定日期 |

手眼界面会验证矩阵是否为有限的右手刚体变换；它**不能**从矩阵数字自动识别物理安装方向、确认 TCP 定义或机器人可达。算法参数只在焊缝包的 YAML 中维护；launch 不应以字典再次覆盖。现场更换矩阵或 ROI 后，先离线复核，再低速逐点验证。

## 可选/历史功能

`data_receiver_node` 保留对 `192.168.3.5:50000` 的 JSONL v1 协议实现；当前 ABB-only YAML 禁用了连接和焊机反馈订阅。启用它会把第三方命令映射到 `/weld/control`、`/weld/set_param_real` 和 `/cimc/motor_speed`，需独立安全验收；详见[第三方协议](../../第三方焊接通信协议.md)。

`motor_control_node` 订阅 `/cimc/motor_speed` (`std_msgs/Float32`，单位 r/s)，通过串口控制电机，带重连逻辑。不要仅为验证 ABB 焊缝路径而启动它或发布转速。旧固定点工艺逻辑不在当前总 launch 中。

## 构建与软件测试

```bash
cd /home/mini/x86_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select cimc --base-paths src
source install/setup.bash
python3 -m unittest discover -v -s src/cimc/test -p 'test_*.py'
```

单测只覆盖软件解析、计时、配置处理和状态模型，不会连接设备，也不能替代 ABB 现场验收。修改 `src/cimc/config/*.yaml` 后重启相关节点即可；修改 Python/launch 后重新构建 `cimc`，避免 `install/` 仍指向旧版本。
