# cimc 包说明：ABB 数据与旋转焊枪电机

这是 `ament_python` 包，包含 ABB 数据、任务协调、手眼变换和旋转焊枪电机节点。旧 DOE 测试节点、测试模板和缓存已经移除。

## 文件作用

| 文件 | 作用 |
|---|---|
| `cimc/motor_control_node.py` | 订阅转速，使用串口协议控制偏心旋转焊枪电机；支持拔插检测和定时重连 |
| `cimc/data_receiver_node.py` | TCP 接收 ABB，同时与 192.168.3.5 按 JSONL v1 双向通信并直接映射焊机/电机话题 |
| `cimc/weld_task_coordinator_node.py` | 协调一次拍照、自动提取和手眼变换任务 |
| `cimc/handeye_abb_bridge_node.py` | 把相机系焊接位姿转换到拍照时 TCP 所在基坐标系 |
| `config/handeye_bridge.yaml` | 手眼矩阵路径、方向、单位、输出坐标系和发送安全开关 |
| `config/weld_task_coordinator.yaml` | 拍照、自动提取、捕获位姿单位和基坐标系参数 |
| `config/data_receiver.yaml` | ABB/第三方网络、协议输入限制、一元模式占位值和话题参数 |
| `launch/camera_weld_handeye_test.launch.py` | 启动当前安全测试链，并打印状态与下一条基坐标轨迹 |
| `setup.py` | 安装 ROS 2 console scripts、配置和 launch |
| `setup.cfg` | 把可执行入口安装到 `lib/cimc` |
| `package.xml` | 声明 `rclpy/std_msgs/geometry_msgs/python3-serial` 运行依赖 |

## motor_control_node

接口：

- 订阅 `/cimc/motor_speed`，类型 `std_msgs/msg/Float32`，含义为目标转速 r/s；
- `0.0` 发送 `OFFOFF`；
- 从停止到正转速先发送 `ONONON`，等待 0.1 s，再发送 `V_r/s:x.x`；
- 运行中修改正转速时只发送新的 `V_r/s:x.x`；
- 串口错误会关闭句柄，由定时器重新连接。

可调 ROS 参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `serial_port` | FTDI 固定 by-id 路径 | 推荐使用 by-id，避免 `/dev/ttyUSBx` 拔插后变化 |
| `baud_rate` | `115200` | 电机控制器波特率 |
| `reconnect_interval_s` | `3.0` | 串口健康检查/重连周期 |
| `speed_topic` | `/cimc/motor_speed` | 转速订阅话题 |

运行：

```bash
ros2 run cimc motor_control_node
ros2 topic pub --once /cimc/motor_speed std_msgs/msg/Float32 "{data: 3.0}"
ros2 topic pub --once /cimc/motor_speed std_msgs/msg/Float32 "{data: 0.0}"
```

## 新增自动拍照与手眼轨迹节点

本包现在还安装两个节点：

- `weld_task_coordinator_node`：从 `/abb/raw_text` 接收独立一行 `START_CAPTURE`，调用 `/camera/capture`，等待焊缝算法结果，并对本次轨迹发布一次性授权。
- `handeye_abb_bridge_node`：读取 OpenCV YAML 里的 `handEyeMatrix`，对 `/weld_seam/poses_camera_frame` 的位置和四元数作刚体变换，发布 `/abb/trajectory_tcp`，再把换算为 mm 的 ASCII 轨迹发往 `/abb/tx_text`。

`data_receiver_node` 新增订阅 `/abb/tx_text`，并使用 ABB 已经建立的 TCP 连接发送；`/abb/tx_status` 只表示 socket `sendall()` 结果，不等于 ABB RAPID 已解析或已执行。

完整参数、矩阵方向、ABB 协议和测试步骤见 [任务协调与手眼ABB桥接节点说明.md](./任务协调与手眼ABB桥接节点说明.md)。

检查设备权限：

```bash
ls -l /dev/serial/by-id/
groups
sudo usermod -aG dialout "$USER"
```

加入 `dialout` 后需重新登录。不要用长期 `sudo ros2 run` 代替正确权限。

## data_receiver_node

网络流向：

```text
ABB 192.168.125.1 -> 本机 192.168.125.2:45000 -> ROS 话题
                                           \-> JSONL abb_rx ----\
/weld/feedback_raw -> JSONL weld_feedback -----------------------> 192.168.3.5:50000
/weld/control、/weld/set_param_real、/cimc/motor_speed <--- JSONL command/setpoints
```

接口：

- 发布 `/abb/raw_text`，`std_msgs/msg/String`：ABB 原始 ASCII 文本；
- 发布 `/abb/weld_point`，`geometry_msgs/msg/Point`：解析 `P...:x,y,z,...` 的前三个坐标；
- 订阅 `/weld/feedback_raw`，`std_msgs/msg/UInt8MultiArray`：把每个 6 字节 TPDO1 解析并封装为 `weld_feedback` JSONL 帧；
- 发布 `/weld/control` 和 `/weld/set_param_real`：把第三方动作/电流请求直接交给现有焊机驱动；
- 发布 `/cimc/motor_speed`：把第三方旋转速度直接交给现有电机节点；
- 发布 `/third_party/status`：记录第三方请求在协议层是否接受。

可调 ROS 参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `listen_host` | `192.168.125.2` | 本机绑定地址，必须实际配置在某网卡上 |
| `listen_port` | `45000` | ABB 连接的 TCP 监听端口 |
| `abb_allowed_ip` | `192.168.125.1` | 只允许该 ABB 来源 IP |
| `forward_ip` | `192.168.3.5` | 第三方控制设备地址 |
| `forward_port` | `50000` | 第三方 TCP 服务端口 |
| `forward_queue_size` | `500` | 非阻塞转发队列容量 |
| `weld_feedback_topic` | `/weld/feedback_raw` | 焊机原始反馈帧话题 |
| `third_party_command_enabled` | `true` | 是否把合法第三方请求发布到底层控制话题 |
| `third_party_max_line_bytes` | `4096` | 单条 JSONL 最大字节数 |
| `unary_voltage_placeholder_v` | `20.0` | 一元模式下底层数组第二项占位值 |
| `min_current_a/max_current_a` | `1.0/350.0` | 网络电流软限制 |
| `min_rotation_speed_rps/max_rotation_speed_rps` | `0.0/6.0` | 网络转速软限制；当前上限取历史工艺值 |
| `stop_on_third_party_disconnect` | `true` | 起焊后第三方断线时停焊并停电机 |

ABB 接收线程不等待第三方发送成功；第三方断线时后台线程重连，所以不会阻塞 ABB 接收。重连时会丢弃断线期间的旧实时帧，避免第三方把历史反馈误认为当前状态。队列满时丢弃新反馈，不让内存无限增长。

运行和改 IP：

```bash
ros2 run cimc data_receiver_node --ros-args \
  --params-file ~/x86_ros2_ws/src/cimc/config/data_receiver.yaml
```

固定协议见仓库根目录的 [第三方焊接通信协议.md](../../第三方焊接通信协议.md)。`data_receiver_node` 已处理 TCP 半包/粘包、版本、递增序号、字段类型、数值范围、ACK 和起焊后断线停机。ACK 只表示 ROS 话题已经发布，不表示真实设备执行成功。

部署前检查：

```bash
ip -br addr
ss -lntp | grep 45000
ros2 topic echo /abb/raw_text
ros2 topic echo /abb/weld_point
```

若 `bind` 报错，通常是 `listen_host` 并不属于本机，而不是 ROS 2 问题。

## 构建

```bash
source /opt/ros/humble/setup.bash
cd ~/x86_ros2_ws
colcon build --symlink-install --packages-select cimc
source install/setup.bash
```

当前阶段的相机+焊缝+手眼测试可使用：

```bash
ros2 launch cimc camera_weld_handeye_test.launch.py
```

该 launch 不启动 `data_receiver_node`、`weld_controller_node`、`weld_logic_node` 或 `motor_control_node`。它只指定主工作区 `src` 内的四个 ROS 参数 YAML，不在 launch 里覆盖参数值。手眼配置必须显式保持 `send_to_abb: false`，否则测试 launch 会在启动节点前拒绝运行。
