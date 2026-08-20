# 任务协调与手眼 ABB 桥接节点

## 1. 新增的完整数据链

```text
ABB --TCP--> data_receiver_node --/abb/raw_text--> weld_task_coordinator_node
                                                       |
                                                       | /camera/capture 服务
                                                       v
                                             chishine_camera_node
                                                       |
                                             /camera/pointcloud_file
                                                       v
                                                weld_seam_node
                                                       |
                                      /weld_seam/poses_camera_frame
                                                       v
                                           handeye_abb_bridge_node
                                                       |
                                      /abb/trajectory_tcp + /abb/tx_text
                                                       v
                                             data_receiver_node
                                                       |
                                                  同一条 TCP
                                                       v
                                                      ABB
```

## 2. 为什么不让新节点直接连 ABB

ABB 是 TCP 客户端，`data_receiver_node` 是 `192.168.125.2:45000` 服务端。ABB 已经和该节点建立了一条全双工 TCP 连接。另一个节点无法再次绑定同一 IP/端口，也无法直接取得另一进程里的 socket。

因此桥接节点发布 `/abb/tx_text`，由 `data_receiver_node` 通过原来的 socket 发送。新增 `/abb/tx_status` 报告成功字节数或断线/队列错误。断线时数据会丢弃，不在下次连接时补发旧轨迹。

## 3. weld_task_coordinator_node

订阅：

- `/abb/raw_text` (`std_msgs/msg/String`)：匹配一行 `START_CAPTURE:x,y,z,qw,qx,qy,qz`；XYZ 默认为 mm，位姿表示拍照瞬间的 `Base_from_TCP`。默认拒绝不带位姿的 `START_CAPTURE`。
- `/weld_seam/status` (`std_msgs/msg/String`)：读取 JSON 里的 SDK `status`，`0` 为成功。

服务客户端：

- `/camera/capture` (`std_srvs/srv/Trigger`)：请求一次拍照。
- `/weld_seam/extract_latest` (`std_srvs/srv/Trigger`)：仅在 `weld_auto_process=false` 时调用。

发布：

- `/weld_task/status` (`String` JSON)：`idle/capturing/captured/extracting/trajectory_ready/fault/busy`。
- `/weld_task/trajectory_armed` (`Bool`, Transient Local)：一次 `START_CAPTURE` 只允许桥接下一条 PoseArray，防止节点启动后转发 DDS 保留的旧轨迹。
- `/weld_task/capture_tcp_pose_base` (`geometry_msgs/msg/PoseStamped`)：拍照瞬间的 `Base_from_TCP`，ROS 内部XYZ为m。

`weld_seam_node` 目前代码和 `config/weld_seam.yaml` 中的 `auto_process` 默认都是 `true`。相机成功后会发布 PLY 路径，焊缝节点自动处理；协调节点不再重复调用提取服务。

## 4. handeye_abb_bridge_node

默认读取安装后的：

```text
<cimc share>/config/handeye_result20260723.yaml
```

当前矩阵为：

```text
 0.389464  0.917831 -0.076840  -43.021843 mm
-0.842378  0.321223 -0.432685  -40.029476 mm
-0.372449  0.233244  0.898265 -331.731110 mm
 0         0         0           1
```

因为标定文件没有记录方向，`matrix_direction` 默认为 `unconfigured`，节点会拒绝转换和发送。确认为 `TCP_from_camera` 后设：

```bash
-p matrix_direction:=tcp_from_camera
```

完整变换为：

```text
T_base_tool = T_base_tcp_at_capture * T_tcp_camera * T_camera_tool
```

PoseArray 位置单位是 m，而 YAML 平移是 mm，节点会自动把矩阵平移乘 `0.001`。如果标定软件实际输出 `Camera_from_TCP`，运行时必须设：

```bash
-p matrix_direction:=camera_from_tcp
```

节点同时转换位置和姿态，并对相邻四元数统一符号，避免 `q/-q` 导致 ABB 误认为长路径旋转。

发布：

- `/abb/trajectory_tcp` (`geometry_msgs/msg/PoseArray`)：ABB Base 坐标系下的变换后轨迹，位置仍为 m。
- `/abb/tx_text` (`std_msgs/msg/String`)：发往 ABB 的 ASCII 协议。
- `/handeye_bridge/status` (`String` JSON)：矩阵/轨迹变换及发送状态。

ABB 文本协议为：

```text
TRAJECTORY_BEGIN:N
P1:x_mm,y_mm,z_mm,qw,qx,qy,qz
...
PN:x_mm,y_mm,z_mm,qw,qx,qy,qz
TRAJECTORY_END
```

TCP 可能粘包或拆包，ABB RAPID 必须按 `\n` 累积并解析完整行，不能假设一次 `SocketReceive` 就是一个点。

## 5. 重要坐标系限制

现在 ABB 必须在 `START_CAPTURE` 同一行提供拍照时的 `Base_from_TCP`：

```text
T_base_tool = T_base_tcp_at_capture * T_tcp_camera * T_camera_tool
```

在没有确认标定矩阵方向和 ABB 目标坐标系前，必须保持 `send_to_abb=false`做离线验证；这也是代码默认值。

## 6. 构建

```bash
cd ~/x86_ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cimc
source install/setup.bash
```

## 7. 安全离线测试（不发 ABB）

```bash
ros2 run cimc handeye_abb_bridge_node --ros-args \
  -p require_task_armed:=false \
  -p require_capture_pose:=false \
  -p matrix_direction:=tcp_from_camera \
  -p send_to_abb:=false
```

然后运行焊缝节点并发布测试 PLY，检查：

```bash
ros2 topic echo /abb/trajectory_tcp \
  geometry_msgs/msg/PoseArray \
  --qos-durability transient_local
ros2 topic echo /handeye_bridge/status
```

## 8. 实际链路启动

分别启动：

```bash
ros2 run cimc data_receiver_node
ros2 run chishine_camera_ros2 chishine_camera_node --ros-args \
  --params-file ~/x86_ros2_ws/src/chishine_camera_ros2/config/camera.yaml
ros2 run weld_seam_perception weld_seam_node --ros-args \
  --params-file ~/x86_ros2_ws/src/weld_seam_perception/config/weld_seam.yaml
ros2 run cimc weld_task_coordinator_node
ros2 run cimc handeye_abb_bridge_node --ros-args \
  -p matrix_direction:=tcp_from_camera \
  -p send_to_abb:=false
```

先保持 `send_to_abb=false`，用 `/abb/trajectory_tcp` 和可视化完成方向、单位和原点验证，再与 ABB RAPID 协议联调。
