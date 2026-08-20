# cimc 包说明：ABB 数据与旋转焊枪电机

这是 `ament_python` 包，只保留用户最新版本的 `data_receiver_node.py`、`motor_control_node.py` 和 Python 包初始化文件。旧 DOE 测试节点、launch、测试模板和缓存已经移除。

## 文件作用

| 文件 | 作用 |
|---|---|
| `cimc/motor_control_node.py` | 订阅转速，使用串口协议控制偏心旋转焊枪电机；支持拔插检测和定时重连 |
| `cimc/data_receiver_node.py` | TCP 接收 ABB 文本/坐标，同时异步转发 ABB 原始流和焊机 6 字节反馈到另一工控机 |
| `setup.py` | 安装两个 ROS 2 console script，不再注册 DOE 节点 |
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
                                           \-> 异步队列 -> 192.168.125.5:50000
/weld/status 中严格匹配到的 RX 6 字节反馈 -----------/
```

接口：

- 发布 `/abb/raw_text`，`std_msgs/msg/String`：ABB 原始 ASCII 文本；
- 发布 `/abb/weld_point`，`geometry_msgs/msg/Point`：解析 `P...:x,y,z,...` 的前三个坐标；
- 订阅 `/weld/status`，`std_msgs/msg/String`：只匹配 `[<- RX 接收 ... 原始帧: xx xx xx xx xx xx]`，避免误转发 TX 帧。

可调 ROS 参数：

| 参数 | 默认值 | 含义 |
|---|---|---|
| `listen_host` | `192.168.125.2` | 本机绑定地址，必须实际配置在某网卡上 |
| `listen_port` | `45000` | ABB 连接的 TCP 监听端口 |
| `abb_allowed_ip` | `192.168.125.1` | 只允许该 ABB 来源 IP |
| `forward_ip` | `192.168.125.5` | 第三方工控机地址 |
| `forward_port` | `50000` | 第三方 TCP 服务端口 |
| `forward_queue_size` | `500` | 非阻塞转发队列容量 |

接收线程不等待第三方转发成功；转发断线时后台线程重连，所以第三方工控机故障不会阻塞 ABB 接收。队列满时当前代码按实时优先策略丢弃新数据，不让控制路径无限积压。

运行和改 IP：

```bash
ros2 run cimc data_receiver_node --ros-args \
  -p listen_host:=192.168.125.2 \
  -p abb_allowed_ip:=192.168.125.1 \
  -p forward_ip:=192.168.125.5
```

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

本包没有 launch；两个节点必须分别启动和验收。
