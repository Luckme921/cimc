# ROS 2 全部节点与接口说明

## 1. 当前节点总览

本文以当前 x86_ros2_ws/src 源码为准。系统现有 8 个业务节点：

| 包 | 节点 | 当前职责 |
|---|---|---|
| cimc | data_receiver_node | ABB TCP收发、坐标解析、第三方数据转发 |
| cimc | motor_control_node | 串口控制偏心旋转焊枪电机 |
| cimc | weld_task_coordinator_node | 接收拍照命令、调用相机、管理一次感知任务 |
| cimc | handeye_abb_bridge_node | 手眼变换、生成ABB基坐标轨迹和发送文本 |
| chishine_camera_ros2 | chishine_camera_node | 发现/连接相机、软件触发、保存PLY |
| weld_seam_perception | weld_seam_node | 调用焊缝SDK并发布CSV、PLY和PoseArray |
| weld_controller | weld_controller_node | USB-CAN控制焊机、解析焊机反馈 |
| weld_controller | weld_logic_node | 根据ABB点序号切换焊接工艺 |

整体链路：

~~~text
ABB
 -> data_receiver_node
 -> weld_task_coordinator_node
 -> chishine_camera_node
 -> weld_seam_node
 -> handeye_abb_bridge_node
 -> data_receiver_node
 -> ABB

ABB点序号
 -> weld_logic_node
 -> weld_controller_node / motor_control_node
 -> 焊机 / 旋转电机
~~~

---

## 2. data_receiver_node

### 2.1 当前功能

- 在 x86 工控机上建立 TCP 服务端，默认监听 192.168.125.2:45000。
- 只接受默认 ABB 地址 192.168.125.1 的连接。
- 接收 ABB 原始 ASCII 文本并发布到 ROS 2。
- 解析 P...:x,y,z,... 格式，发布前三个坐标。
- 接收 /abb/tx_text，通过同一条 ABB TCP 连接发送回机器人。
- 把 ABB 原始字节和焊机 RX 六字节反馈异步转发到第三方工控机。

### 2.2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| listen_host | 192.168.125.2 | 本机ABB通信网卡地址 |
| listen_port | 45000 | TCP监听端口 |
| abb_allowed_ip | 192.168.125.1 | 允许连接的ABB地址 |
| forward_ip | 192.168.125.5 | 第三方工控机地址 |
| forward_port | 50000 | 第三方TCP端口 |
| forward_queue_size | 500 | 第三方转发队列容量 |

### 2.3 话题

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 发布 | /abb/raw_text | std_msgs/msg/String | ABB原始文本 |
| 发布 | /abb/weld_point | geometry_msgs/msg/Point | 从 P... 文本提取的XYZ |
| 订阅 | /abb/tx_text | std_msgs/msg/String | 等待发给ABB的ASCII文本 |
| 发布 | /abb/tx_status | std_msgs/msg/String | OK或ERROR发送状态 |
| 订阅 | /weld/status | std_msgs/msg/String | 提取焊机RX六字节帧供第三方转发 |

### 2.4 使用示例

~~~bash
ros2 run cimc data_receiver_node
~~~

临时改参数：

~~~bash
ros2 run cimc data_receiver_node --ros-args \
  -p listen_host:=192.168.125.2 \
  -p abb_allowed_ip:=192.168.125.1 \
  -p forward_ip:=192.168.125.5
~~~

模拟待发文本：

~~~bash
ros2 topic pub --once /abb/tx_text std_msgs/msg/String \
  "{data: 'TEST_TO_ABB\n'}"
~~~

### 2.5 当前限制

- OK 只表示 socket.sendall() 成功，不代表 ABB 已解析或执行。
- ABB 未连接时，当前待发消息会丢弃，避免重连后误发旧轨迹。
- TCP 可能拆包和粘包；当前没有完整的跨 recv() 半包缓存。
- 第三方通道混合转发 ABB 原始数据和焊机反馈，需要双方约定协议。

---

## 3. motor_control_node

### 3.1 当前功能

- 通过 FTDI 串口控制偏心旋转焊枪电机。
- 支持启动、正转速度修改和停止。
- 定时检测串口是否掉线，掉线后自动重连。

### 3.2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| serial_port | 固定FTDI by-id路径 | 电机串口 |
| baud_rate | 115200 | 波特率 |
| reconnect_interval_s | 3.0 | 检查和重连周期 |
| speed_topic | /cimc/motor_speed | 转速话题 |

### 3.3 订阅

| 名称 | 类型 | 含义 |
|---|---|---|
| /cimc/motor_speed | std_msgs/msg/Float32 | 目标速度，单位 r/s |

### 3.4 串口指令

- 速度为 0：发送 OFFOFF。
- 从停止变为正速度：发送 ONONON，等待 0.1 秒，再发送 V_r/s:x.x。
- 运行中改变正速度：只发送新的 V_r/s:x.x。

### 3.5 示例

~~~bash
ros2 run cimc motor_control_node
ros2 topic pub --once /cimc/motor_speed std_msgs/msg/Float32 "{data: 3.0}"
ros2 topic pub --once /cimc/motor_speed std_msgs/msg/Float32 "{data: 0.0}"
~~~

### 3.6 当前限制

- 负速度没有实现反转逻辑。
- 没有独立的电机反馈或应答话题。
- 串口断线期间收到的速度指令会丢弃。

---

## 4. chishine_camera_node

### 4.1 当前功能

- 通过厂家SDK发现网络和USB相机。
- 按相机序列号选择设备；序列号为空时选第一台。
- 启动 Z16 深度流，可选 RGB8。
- 设置深度范围、增益、曝光、帧时间和软件触发。
- 每次服务调用抓取一帧，重建点云并保存PLY。

### 4.2 主要参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| camera_serial | 空 | 指定相机序列号 |
| enable_network_discovery | true | 启用网络发现 |
| enable_usb_discovery | true | 启用USB发现 |
| discovery_timeout_ms | 3000 | 单次发现超时 |
| discovery_attempts | 10 | 发现尝试次数 |
| discovery_retry_interval_ms | 1000 | 重试间隔 |
| output_directory | 空 | 默认保存到 SCUT_WELD_DATA_ROOT/pointclouds |
| file_prefix | capture | 文件前缀 |
| capture_timeout_ms | 5000 | 取帧超时 |
| depth_width/height/fps | 0/0/0 | 0表示不限制匹配流 |
| rgb_width/height/fps | 0/0/0 | RGB流条件 |
| enable_rgb | false | 默认只输出XYZ，更快 |
| binary_ply | true | 保存二进制PLY |
| depth_min_mm | 460 | 最小深度 |
| depth_max_mm | 520 | 最大深度 |
| depth_gain | 1.0 | 增益 |
| depth_exposure | 8000.0 | 曝光 |
| depth_frame_time | 10000.0 | 帧时间 |

### 4.3 服务和话题

| 类别 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 服务 | /camera/capture | std_srvs/srv/Trigger | 拍照一次，响应message为PLY绝对路径 |
| 发布 | /camera/pointcloud_file | std_msgs/msg/String | 发布PLY绝对路径，Transient Local |

### 4.4 示例

~~~bash
ros2 run chishine_camera_ros2 chishine_camera_node --ros-args \
  --params-file ~/x86_ros2_ws/src/chishine_camera_ros2/config/camera.yaml

ros2 service call /camera/capture std_srvs/srv/Trigger "{}"
~~~

### 4.5 当前限制

- 相机和主机IP不通过ROS参数设置，必须先配置Linux网卡和相机网络。
- 启动时找不到相机会抛出异常并退出。
- 使用互斥锁，同一时间只执行一次拍照。

---

## 5. weld_seam_node

### 5.1 当前功能

- 接收PLY文件路径，不在DDS中传输整个点云。
- 直接链接 libweld_seam_sdk.so，不启动外部CLI。
- 生成特征点CSV、可视化PLY、特征点PLY和ROS PoseArray。
- 支持收到路径自动处理，也支持服务手动重算。

### 5.2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| input_ply_topic | /camera/pointcloud_file | PLY路径输入 |
| output_directory | 空 | 默认 SCUT_WELD_DATA_ROOT/weld_results |
| output_prefix | weld_seam | 输出文件前缀 |
| config_file | 空 | 空表示使用SDK编译默认值 |
| frame_id | camera_link | PoseArray坐标系标签 |
| position_scale_to_ros | 0.001 | CSV毫米转换为ROS米 |
| auto_process | true | 收到PLY路径后立即运行 |
| algorithm_overrides | YAML配置 | 覆盖SDK参数 |

当前YAML覆盖：

~~~text
roi.enable=false
normal.mode=auto
orientation.tool_positive_z_points_from_tcp_to_body=true
~~~

### 5.3 接口

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 订阅 | /camera/pointcloud_file | String | PLY绝对路径，Transient Local |
| 服务 | /weld_seam/extract_latest | Trigger | 重算最近收到的PLY |
| 发布 | /weld_seam/result_csv | String | CSV路径，Transient Local |
| 发布 | /weld_seam/result_visualization | String | 可视化PLY路径，Transient Local |
| 发布 | /weld_seam/poses_camera_frame | PoseArray | 相机坐标轨迹，位置单位m |
| 发布 | /weld_seam/status | String JSON | 状态、点数和耗时 |

状态JSON主要字段：

~~~text
status, message, input_ply, csv, seam_points, path_points,
normal_ms, primary_ms, feature_ms, total_ms
~~~

status=0 表示成功。

### 5.4 示例

~~~bash
ros2 run weld_seam_perception weld_seam_node --ros-args \
  --params-file ~/x86_ros2_ws/src/weld_seam_perception/config/weld_seam.yaml

ros2 topic pub --once --qos-durability transient_local \
  /camera/pointcloud_file std_msgs/msg/String \
  "{data: '$SCUT_WELD_DATA_ROOT/pointclouds/20260529101148-1.ply'}"

ros2 service call /weld_seam/extract_latest std_srvs/srv/Trigger "{}"
~~~

### 5.5 当前限制

- 同时只处理一个点云，并发请求会被拒绝。
- PoseArray只有位置和姿态，没有CSV中的 feature_type、weld_enabled 等工艺字段。
- frame_id 只是标签，必须保证实际PLY确实属于该相机坐标系。

---

## 6. weld_task_coordinator_node

### 6.1 当前功能

- 从 ABB 文本中识别拍照指令。
- 读取拍照瞬间的 Base_from_TCP 位置和姿态。
- 调用相机拍照服务。
- 防止上一任务未完成时重复开始新任务。
- 监视焊缝提取和手眼变换结果。
- 通过一次性 armed 信号防止桥接节点处理旧的DDS保留轨迹。

### 6.2 ABB拍照格式

~~~text
START_CAPTURE:x_mm,y_mm,z_mm,qw,qx,qy,qz
~~~

示例：

~~~text
START_CAPTURE:1200.0,100.0,800.0,1.0,0.0,0.0,0.0
~~~

它表示拍照瞬间 ABB 基坐标系下的 TCP 位姿。四元数会自动归一化。

### 6.3 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| start_command | START_CAPTURE | ABB命令前缀 |
| camera_service | /camera/capture | 相机服务 |
| weld_auto_process | true | 假定焊缝节点自动处理 |
| extract_service | /weld_seam/extract_latest | 手动提取服务 |
| service_wait_timeout_s | 3.0 | 等待服务时间 |
| require_capture_pose | true | 必须携带拍照TCP位姿 |
| capture_pose_unit | mm | ABB输入XYZ单位 |
| base_frame_id | robot_base | 基坐标系标签 |

### 6.4 接口

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 订阅 | /abb/raw_text | String | ABB拍照命令 |
| 订阅 | /weld_seam/status | String JSON | 判断算法结果 |
| 订阅 | /handeye_bridge/status | String JSON | 判断变换结果 |
| 客户端 | /camera/capture | Trigger | 请求拍照 |
| 客户端 | /weld_seam/extract_latest | Trigger | 非自动模式下请求提取 |
| 发布 | /weld_task/status | String JSON | 任务状态和task_id |
| 发布 | /weld_task/trajectory_armed | Bool | 授权下一条轨迹 |
| 发布 | /weld_task/capture_tcp_pose_base | PoseStamped | Base_from_TCP，XYZ单位m |

状态大致为：

~~~text
idle -> capturing -> captured -> extracting
     -> trajectory_ready -> completed
~~~

异常为 fault，重复命令提示 busy。

### 6.5 示例

~~~bash
ros2 run cimc weld_task_coordinator_node

ros2 topic pub --once /abb/raw_text std_msgs/msg/String \
  "{data: 'START_CAPTURE:1200.0,100.0,800.0,1.0,0.0,0.0,0.0'}"
~~~

### 6.6 当前限制

- weld_auto_process 不会自动修改 weld_seam_node 的 auto_process，两者必须手动保持一致。
- 暂无算法和手眼阶段的总超时，某个节点无响应时任务可能长期busy。
- task_id 没有写入PoseArray，目前主要靠armed防止旧轨迹。

---

## 7. handeye_abb_bridge_node

### 7.1 当前功能

- 读取OpenCV YAML中的 handEyeMatrix。
- 校验矩阵是否是有效齐次刚体变换。
- 保存拍照瞬间的 Base_from_TCP。
- 将相机坐标轨迹转换到ABB基坐标系。
- 统一相邻四元数符号，避免 q 和 -q 导致错误的长路径插补。
- 生成 ABB ASCII 轨迹文本。

变换关系：

~~~text
Base_from_Tool =
Base_from_TCP_at_capture * TCP_from_Camera * Camera_from_Tool
~~~

### 7.2 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| matrix_file | cimc安装目录中的YAML | 手眼矩阵文件 |
| matrix_key | handEyeMatrix | OpenCV矩阵键 |
| matrix_direction | unconfigured | tcp_from_camera或camera_from_tcp |
| matrix_translation_unit | mm | 矩阵平移单位 |
| output_frame_id | robot_base | 输出坐标系 |
| require_capture_pose | true | 必须有拍照TCP位姿 |
| require_task_armed | true | 必须有任务授权 |
| send_to_abb | false | 默认禁止真实发送 |
| protocol_precision | 6 | 数字小数位数 |

matrix_direction=unconfigured 时会拒绝变换，防止未知方向下误发机器人。

### 7.3 接口

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 订阅 | /weld_task/trajectory_armed | Bool | 一次性任务授权 |
| 订阅 | /weld_task/capture_tcp_pose_base | PoseStamped | Base_from_TCP |
| 订阅 | /weld_seam/poses_camera_frame | PoseArray | 相机坐标轨迹 |
| 发布 | /abb/trajectory_tcp | PoseArray | ABB基坐标轨迹，XYZ单位m |
| 发布 | /abb/tx_text | String | ABB ASCII轨迹 |
| 发布 | /handeye_bridge/status | String JSON | 变换和发送请求状态 |

### 7.4 ABB文本格式

~~~text
TRAJECTORY_BEGIN:N
P1:x_mm,y_mm,z_mm,qw,qx,qy,qz
P2:x_mm,y_mm,z_mm,qw,qx,qy,qz
...
TRAJECTORY_END
~~~

### 7.5 安全测试

~~~bash
ros2 run cimc handeye_abb_bridge_node --ros-args \
  -p matrix_direction:=tcp_from_camera \
  -p require_task_armed:=false \
  -p require_capture_pose:=false \
  -p send_to_abb:=false
~~~

### 7.6 当前限制

- 手眼矩阵方向尚未确认，默认不变换、不发送。
- 没有机器人工作空间、关节限位、碰撞或焊枪干涉检查。
- 会处理PoseArray全部点，但PoseArray没有焊接使能和特征类型。
- 没有ABB接收应答，不能证明机器人已经完整接收轨迹。

---

## 8. weld_controller_node

### 8.1 当前功能

- 使用 libcontrolcan.so 控制 USB-CAN2。
- 固定使用通道0、125 kbps、焊机节点ID 0x02。
- 发送CANopen NMT启动命令。
- 启动后每20 ms发送一次RPDO1。
- 每10 ms轮询一次TPDO1反馈。
- 每500 ms发布一次焊机仪表盘文本。

### 8.2 话题

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 订阅 | /weld/control | std_msgs/msg/String | 焊机动作命令 |
| 订阅 | /weld/set_param_real | Float32MultiArray | [电流A, 目标电压V] |
| 发布 | /weld/status | String | TX/RX帧、状态、故障和反馈值 |

### 8.3 控制命令

| 字符串 | 作用 |
|---|---|
| start_system | 发送NMT并启动CAN收发线程 |
| start_welding | 启动焊接组合命令 |
| stop_welding | 停止焊接 |
| start_gas | 开启气检/保护气 |
| wire_forward | 正向送丝 |
| wire_backward | 反向送丝 |
| fault_reset | 故障复位脉冲200 ms |
| use_builtin_curve | 内置曲线，压强固定0 |
| use_calc_curve | 根据电流和目标电压计算压强 |

### 8.4 示例

~~~bash
ros2 run weld_controller weld_controller_node
ros2 topic pub --once /weld/control std_msgs/msg/String "{data: 'start_system'}"
ros2 topic pub --once /weld/set_param_real std_msgs/msg/Float32MultiArray \
  "{data: [250.0, 30.0]}"
~~~

### 8.5 当前限制

- CAN类型、通道、波特率和节点ID是编译期常量，不是ROS参数。
- 部分厂家API失败时没有充分报错，节点仍可能继续运行。
- 默认使用内置曲线，此时目标电压不会用于压强计算。
- 电流最大限制350 A，但负值和目标电压仍需加强校验。

---

## 9. weld_logic_node

### 9.1 当前功能

- 根据ABB到达点的序号切换焊接电流、电压和旋弧速度。
- 监听焊机反馈，等待起弧成功后进入正式工艺。
- 支持手动接管、软件急停和参数覆盖。
- 启动3秒后自动请求焊机start_system。
- 焊机状态丢失超过3秒时尝试重新激活。

### 9.2 接口

| 方向 | 名称 | 类型 | 作用 |
|---|---|---|---|
| 订阅 | /abb/raw_text | String | START_GAS或数字点序号 |
| 订阅 | /weld/status | String | 焊机心跳和起弧状态 |
| 订阅 | /cimc/override_cmd | String | 模式和手动动作 |
| 订阅 | /cimc/override_param | Float32MultiArray | [电流, 电压, 转速] |
| 发布 | /weld/control | String | 给焊机底层的控制命令 |
| 发布 | /weld/set_param_real | Float32MultiArray | [电流, 电压] |
| 发布 | /cimc/motor_speed | Float32 | 旋弧速度 |

### 9.3 现有固定点流程

- 第一条数字作为本次ABB序号基准，内部归一化为0。
- 点0：安全过渡点。
- 点1：240 A、29.6 V、6 r/s引弧。
- 起弧成功后等待200 ms，切入正式工艺。
- 点2～7：按代码内固定工艺表切换底边、斜边、顶边参数。
- 点7后1秒切入245 A、29.8 V、6 r/s收弧参数。
- 点8：停止焊接和电机。
- 点9：任务结束并清除序号基准。

### 9.4 手动命令

| 命令 | 作用 |
|---|---|
| MODE_MANUAL | 手动接管，屏蔽ABB自动逻辑 |
| MODE_AUTO | 恢复自动并重置序号 |
| ESTOP | 软件停止焊接和电机 |
| USE_BUILTIN | 使用焊机内置曲线 |
| USE_CALC | 使用动态压强计算 |
| 其他焊机命令 | 仅在手动模式直接转发到 /weld/control |

示例：

~~~bash
ros2 topic pub --once /cimc/override_cmd std_msgs/msg/String \
  "{data: 'MODE_MANUAL'}"
ros2 topic pub --once /cimc/override_cmd std_msgs/msg/String \
  "{data: 'stop_welding'}"
ros2 topic pub --once /cimc/override_param std_msgs/msg/Float32MultiArray \
  "{data: [250.0, 30.0, 6.0]}"
~~~

### 9.5 当前重要限制

- 仍按固定9个物理点设计，与焊缝算法可变点数不匹配，暂时不应加入真实自动焊接。
- 工艺参数和延时是C++常量，修改后需重新编译。
- override_param 当前在自动模式也能覆盖参数。
- ESTOP只是软件停止，不能代替硬件急停。
- 焊机节点状态文字是“起弧[✅]”，逻辑节点查找的是“起弧成功[✅]”，两者不一致，可能导致状态一直停在 WAIT_ARC_SUCCESS。

---

## 10. 当前建议

现阶段可以分别测试：

- data_receiver_node
- motor_control_node
- chishine_camera_node
- weld_seam_node
- handeye_abb_bridge_node（保持 send_to_abb=false）
- weld_controller_node（先不实际起弧）

真实自动焊接前还需要完成：

1. 确认手眼矩阵方向。
2. 确认ABB发送的是拍照瞬间 Base_from_TCP。
3. 确认ABB和ROS四元数顺序、坐标系和工具方向。
4. 为ABB轨迹增加接收应答。
5. 优化weld_logic_node，使其支持可变点数和工艺元数据。
6. 增加工作空间、碰撞、干涉和姿态跳变安全检查。

---

## 11. 常用检查命令

每个新终端先执行：

~~~bash
source /opt/ros/humble/setup.bash
source ~/x86_ros2_ws/install/setup.bash
~~~

检查系统：

~~~bash
ros2 node list
ros2 topic list | sort
ros2 service list | sort
ros2 pkg executables cimc
ros2 pkg executables weld_controller
ros2 node info /weld_seam_node
ros2 param list /weld_seam_node
ros2 param get /weld_seam_node auto_process
~~~

