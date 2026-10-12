# ROS 2 节点与接口：当前 ABB 焊缝链路

本表按当前 `camera_weld_operator_abb.launch.py` 和源码整理。**存在于仓库中的节点，不等于总 launch 会启动它。**参数实际值以 `src/*/config/*.yaml` 为准；本表只解释职责和协议，不复制易过期的现场数值。启动顺序见[测试手册](实际启动与分节点测试手册.md)。

## 节点范围

| 节点 | 包 | 总 launch | 职责 |
| --- | --- | --- | --- |
| `data_receiver_node` | `cimc` | 人工/固定模式 | 监听 ABB TCP，换行分帧，收拍照指令和 ACK；发送轨迹文本并计时 |
| `chishine_camera_node` | `chishine_camera_ros2` | 人工确认后/固定模式 | 厂商 SDK 连接相机，软件触发，写 PLY 并发布文件路径 |
| `weld_seam_node` | `weld_seam_perception` | 人工确认后/固定模式 | 进程内调用 SDK；输出相机系路径、CSV、可视化和状态 |
| `weld_task_coordinator_node` | `cimc` | 人工确认后/固定模式 | 单次请求状态机，保存拍照时 TCP，触发相机并授权轨迹 |
| `handeye_abb_bridge_node` | `cimc` | 人工确认后/固定模式 | 相机系 → 机器人基坐标系；点数检查及 ABB 文本序列化 |
| `weld_operator_status.py` | `cimc` 源码脚本，非 ROS 节点 | 仅人工模式确认后，默认开启 | 只读状态窗；关闭界面不会停止核心节点 |

`weld_controller_node`、`motor_control_node`、历史 `weld_logic_node`、192.168.3.5 TCP 通道不在当前总 launch 的执行链中。`data_receiver_node` 虽保留第三方协议代码，但当前 `data_receiver.yaml` 关闭 `third_party_link_enabled`。旧单节点或历史包的细节分别见各包 README 和[第三方协议](第三方焊接通信协议.md)。

## 一次请求的数据流

1. ABB 通过 TCP 向本机监听端口发送一行 `START_CAPTURE:x,y,z,qw,qx,qy,qz`；`data_receiver_node` 发布 `/abb/raw_text`。XYZ 是拍照时机器人基坐标系下 TCP 的 mm，四元数为 ABB 的 `q1..q4`（对应 `qw,qx,qy,qz`）。
2. 协调器验证请求、发布 `/weld_task/capture_tcp_pose_base` 与一次性 `/weld_task/trajectory_armed`，调用 `/camera/capture`。相机完整写入 PLY 后发布 `/camera/pointcloud_file`（仅绝对路径）。
3. 焊缝节点按 YAML 中 `algorithm_overrides` 运行 SDK，发布 `/weld_seam/poses_camera_frame`、`/weld_seam/status`，并写 CSV/result PLY。失败不会发布伪轨迹。
4. 手眼桥使用 `Base←TCP_at_capture × TCP←Camera × Camera←Tool` 生成 `/abb/trajectory_tcp`。矩阵若以 `camera_from_tcp` 存储，节点先求逆；`send_to_abb=true` 且授权/点数有效时再发布 `/abb/tx_text`。
5. ABB 接收节点通过现有 socket `sendall()` 写出轨迹，发布 `/abb/tx_status` 和 `/abb/task_timing`。ABB 回传 `TRAJECTORY_ACK` 时出现在 `/abb/raw_text`。ACK 不是运动完成信号。

桥接节点不读取 ABB `robconf`、工件坐标或机器人关节状态，不做逆运动学、碰撞或奇异点检查。目标姿态符合坐标变换并不保证 ABB `MoveL` 可达。

## 主要话题与服务

| 名称 | 类型 | 发布/提供者 → 使用者 | 意义 |
| --- | --- | --- | --- |
| `/abb/raw_text` | `std_msgs/String` | 接收节点 → 协调器/状态窗 | ABB 完整文本行，含 START_CAPTURE、ACK、错误 |
| `/abb/connection_status` | `std_msgs/String` JSON | 接收节点 → 状态窗 | listening/connected/disconnected 等 |
| `/camera/capture` | `std_srvs/Trigger` | 相机 ← 协调器 | 一次软件触发，不应在正常任务中重复人工调用 |
| `/camera/pointcloud_file` | `std_msgs/String` | 相机 → 焊缝节点 | 已写完 PLY 的绝对路径 |
| `/weld_seam/extract_latest` | `std_srvs/Trigger` | 焊缝节点 | `auto_process=false` 时处理最后一个路径 |
| `/weld_seam/poses_camera_frame` | `geometry_msgs/PoseArray` | 焊缝节点 → 手眼桥 | 相机系位姿，位置单位 m |
| `/weld_seam/result_csv`、`/weld_seam/result_visualization` | `std_msgs/String` | 焊缝节点 | 结果文件路径 |
| `/weld_seam/status` | `std_msgs/String` JSON | 焊缝节点 → 协调器/状态窗 | 成败、点数、耗时和结果路径 |
| `/weld_task/status` | `std_msgs/String` JSON | 协调器 → 接收节点/状态窗 | task_id、capturing/extracting/completed/fault 等 |
| `/weld_task/trajectory_armed` | `std_msgs/Bool` | 协调器 → 手眼桥 | 本次轨迹授权；不是物理安全许可 |
| `/weld_task/capture_tcp_pose_base` | `geometry_msgs/PoseStamped` | 协调器 → 手眼桥 | 拍照时 Base←TCP，位置单位 m |
| `/handeye_bridge/status` | `std_msgs/String` JSON | 手眼桥 → 协调器/状态窗 | 变换结果、输出点数 |
| `/abb/trajectory_tcp` | `geometry_msgs/PoseArray` | 手眼桥 | `robot_base` 下轨迹，位置单位 m |
| `/abb/tx_text` | `std_msgs/String` | 手眼桥 → 接收节点 | 准备发送的多行轨迹文本，**不是送达证明** |
| `/abb/tx_status` | `std_msgs/String` | 接收节点 → 状态窗 | 本机 socket 发送结果，**不是 ABB 执行证明** |
| `/abb/task_timing` | `std_msgs/String` JSON | 接收节点 → 状态窗 | START_CAPTURE 完整接收至 sendall 返回的 ms |

焊缝 CSV/PLY 原始位置单位 mm；`PoseArray` 均用 m。ABB 文本发送时再转回 mm。ROS `orientation` 字段以 `x,y,z,w` 表示四元数；CSV/ABB 文本按 `qw,qx,qy,qz` 排列。输出坐标系和手眼方向必须与示教器当前 tool/wobj 定义实际一致，不能只看 `frame_id` 字符串。

## QoS 与 DDS 边界

- `/camera/pointcloud_file` 与 `/weld_seam/poses_camera_frame` 的发布/订阅均为 reliable、transient-local、depth 1，便于晚加入节点得到最近一次结果；**离线重放必须独占测试域**，避免把旧路径当成本次数据。
- 协调器的任务状态、拍照 TCP 位姿、一次性授权，以及 ABB 连接状态也使用 reliable、transient-local、depth 1，避免启动发现时丢失就绪状态。
- ABB 指令、发送文本、阶段性状态等默认 reliable、volatile，常用 depth 10。控制链没有为了“省资源”改为 best-effort；改 QoS 前须核对发布者和订阅者兼容性。
- 总 launch 默认 Fast DDS `DEFAULT` 传输：同机优先 SHM，保留 UDPv4 发现/回退。相机厂商 SDK 和 ABB socket 不走 ROS DDS；大点云通过磁盘 PLY 传递，SHM 不等于点云零拷贝。
- 状态窗默认开启，多个 `ros2 topic echo` 默认关闭；需要详细调试时使用 `echo_topics:=true`，结束后恢复默认以减少额外 DDS 参与者。

## ABB 文本边界

输入示例：

```text
START_CAPTURE:947.35,-79.0,717.49,0.38469,-0.77967,0.40899,-0.27723\r\n
```

输出按 ABB 所需逐行格式：

```text
TRAJECTORY_BEGIN:2
P1:923.690003,117.447487,684.017467,0.131353,-0.615537,0.684630,-0.367617
P2:943.954166,117.826506,664.288783,0.131353,-0.615537,0.684630,-0.367617
TRAJECTORY_END
```

实际点数可变，不能假定恒为 2、6、8 或 29；桥接节点有 `max_trajectory_points`（当前按 ABB 100 点上限配置）。接收节点处理 TCP 半包/粘包和 `\n`/`\r\n`；ABB 程序应在应用层完成点数、序号和姿态检查后回 ACK，不应把 socket 可写当成轨迹安全确认。

## 代码与配置入口

| 主题 | 入口 |
| --- | --- |
| ABB 收发和计时 | [`src/cimc/cimc/data_receiver_node.py`](src/cimc/cimc/data_receiver_node.py) · [`data_receiver.yaml`](src/cimc/config/data_receiver.yaml) |
| 协调和手眼 | [`weld_task_coordinator_node.py`](src/cimc/cimc/weld_task_coordinator_node.py) · [`handeye_abb_bridge_node.py`](src/cimc/cimc/handeye_abb_bridge_node.py) |
| 相机与焊缝 ROS 适配 | [`chishine_camera_node.cpp`](src/chishine_camera_ros2/src/chishine_camera_node.cpp) · [`weld_seam_node.cpp`](src/weld_seam_perception/src/weld_seam_node.cpp) |
| 算法配置和 SDK | [`weld_seam.yaml`](src/weld_seam_perception/config/weld_seam.yaml) · [算法源码说明](x86_weld_seam_test/README.md) |
