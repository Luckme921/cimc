# weld_seam_perception 包说明

该包是焊缝 SDK 的轻量 ROS 2 适配层。算法不复制到本包，也不通过 shell 启动 CLI；节点直接链接工作区内 `x86_weld_seam_test/install` 的 `libweld_seam_sdk.so`，接收 PLY 路径并发布结构化结果。当前 CMake 要求 SDK 至少 2.4.10；启动日志中的版本才是实际加载的运行库版本。工作区源码版本与已安装库可能不同；修改算法源码后仍须重新编译、安装 SDK 并重建本包。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/weld_seam_node.cpp` | 输入路径、互斥调用 SDK、发布文件路径/JSON/PoseArray |
| `config/weld_seam.yaml` | ROS 参数和算法 `key=value` 覆盖 |
| `CMakeLists.txt` | 从工作区 SDK 安装目录查找 `weld_seam_sdk >= 2.4.10` 并链接 `weld_seam::sdk` |
| `package.xml` | rclcpp、String、Trigger、PoseArray 依赖 |

## ROS 2 接口

订阅：

- 默认 `/camera/pointcloud_file`，`std_msgs/msg/String`，Transient Local；消息内容必须是 PLY 文件路径。程序会去除首尾空白，并在 Linux 展开开头的 `~/`。

服务：

- `/weld_seam/extract_latest`，`std_srvs/srv/Trigger`：处理最近收到的 PLY。适合 `auto_process=false`。

发布：

| 话题 | 类型 | 内容 |
|---|---|---|
| `/weld_seam/result_csv` | `std_msgs/msg/String` | 成功结果 CSV 绝对路径 |
| `/weld_seam/result_visualization` | `std_msgs/msg/String` | 红色焊缝+粉红十字可视化 PLY 路径 |
| `/weld_seam/status` | `std_msgs/msg/String` | JSON：状态、输入、点数和分阶段耗时 |
| `/weld_seam/poses_camera_frame` | `geometry_msgs/msg/PoseArray` | 相机坐标系位姿；位置已由 mm 转 m |

CSV 和 PoseArray 发布者为 Transient Local，后启动的调试订阅者能收到最近结果。状态为普通 QoS，表示每次运行事件。

## ROS 参数

| 参数 | 默认值 | 含义 |
|---|---|---|
| `input_ply_topic` | `/camera/pointcloud_file` | PLY 路径话题 |
| `output_directory` | 空 | 默认 `~/scut_weld_data/weld_results` |
| `output_prefix` | `weld_seam` | 实际前缀还会加输入 PLY stem |
| `config_file` | 空 | 可指向独立 SDK 的 `default.conf` |
| `frame_id` | `camera_link` | PoseArray 坐标系标签 |
| `position_scale_to_ros` | `0.001` | CSV mm 转 ROS m |
| `auto_process` | `true` | 收到 PLY 后是否立即处理 |
| `algorithm_overrides` | 见 YAML | 传给 SDK 的重复 `key=value` 参数 |

算法参数优先级：SDK 编译默认 `< config_file < algorithm_overrides`。YAML 当前明确设置：

```yaml
algorithm_overrides:
  - "roi.enable=true"
  - "normal.mode=auto"
  - "orientation.tool_positive_z_points_from_tcp_to_body=false"
  - "orientation.tool_x_reference=workpiece_x"
  - "orientation.tool_x_points_along_positive_workpiece_x=false"
  - "path.mode=rounded_features"
```

`tool_x_reference=workpiece_x` 时，算法保持 Tool Z 不变，并将工件焊接前进轴投影到 Tool Z 的法平面作为 Tool X，以减少局部角点造成的绕枪轴滚转。现场 TCP +X 与焊接前进方向相反时，只需把 `tool_x_points_along_positive_workpiece_x` 改为 `false`；需要复现 SDK 2.2.0 行为时可将参考模式改为 `corner_bisector`。

ROI、四类位置偏置、四类姿态等均可继续添加，不用重新编译节点。例如：

```yaml
  - "roi.enable=true"
  - "roi.max_y=300"
  - "offset.protruding_left.x=2.5"
```

完整键名及含义见 `x86_weld_seam_test/config/default.conf` 和该工程 README。

路径模式只需在同一个 YAML 中切换，launch 不覆盖该参数：

```yaml
# 旧版四类单拐点模式（SDK通用默认，当前ROS YAML未启用）
- "path.mode=feature_points"

# 稀疏圆角模式：现场姿态坐标系，首末单点、内部圆角5点、确认角之间1个中点
- "path.mode=rounded_features"

# 曲率轮廓备用模式；ROS话题、CSV列、手眼和ABB协议均不变
- "path.mode=adaptive_contour"
```

三行只能启用一行。当前 YAML 使用 `rounded_features`，圆角目标弧长间距为 3 mm；首末已确认角各 1 点，内部完整圆角默认 5 点，确认角之间增加 1 个中点。若裁剪边界落在一段有充分实测支撑的直线上，还会增加实测边界焊接点及必要中点，而不是因为红缝超出角点便拒绝整条轨迹；CSV 以 `measured_boundary_seam_endpoint` 标记。可靠实测点优先；小空洞拟合点以 CSV 的 `point_source=modeled_small_hole_from_adjacent_lines` 和结果 PLY 的亮绿色十字星标记，不能当作实测点。内部四类跳类、目标顺序回退、跨度过大的空洞仍拒绝向手眼或 ABB 发布。三种模式都受含安全点在内的 100 点上限保护；ROS 仍按列名读取 `x,y,z,qw,qx,qy,qz`，`raw_*` 和 `point_source` 诊断列不改变手眼或 ABB 协议。

ROI 是相机坐标下的额外裁剪，即使输入 PLY 已裁剪也会再次生效。**当前 YAML 已启用一次现场拍照位对应的固定 X/Y 边界；Z 仍为全范围**。这些数字不是通用默认 ROI，换拍照位或工件摆放后须重新复核。可先离线裁剪 PLY 并检查结果，或在人工标定窗口中更新对应边界；仅裁剪某个历史 PLY 不会自动改变下一次 ROS 相机产生的文件。全范围输入若仍包含视野外缘的多余红缝，首末覆盖保护可能拒绝发布轨迹；这不是 ROS 节点启动失败，不应只靠放大外推阈值消除。每次改 ROI 后先核对角点顺序和结果 PLY。精确当前值始终以 [`config/weld_seam.yaml`](config/weld_seam.yaml) 为准。

## 构建

先从**本工作区副本**构建并安装 SDK；独立仓库 `~/x86_weld_seam_test` 和旧目录 `~/scut_weld_sdk_install` 不是本包当前的 CMake 搜索目标：

```bash
cd /home/mini/x86_ros2_ws/x86_weld_seam_test
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
cmake --install build --prefix /home/mini/x86_ros2_ws/x86_weld_seam_test/install
```

再构建本包：

```bash
source /opt/ros/humble/setup.bash
cd /home/mini/x86_ros2_ws
colcon build --packages-select weld_seam_perception --base-paths src \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

运行与离线 PLY 测试：

```bash
ros2 run weld_seam_perception weld_seam_node --ros-args \
  --params-file ~/x86_ros2_ws/src/weld_seam_perception/config/weld_seam.yaml

ros2 topic pub --once --qos-durability transient_local \
  --qos-reliability reliable /camera/pointcloud_file std_msgs/msg/String \
  "{data: '/absolute/sample.ply'}"
```

## 并发和故障行为

SDK 当前按一次一份大点云同步运行。节点用 `try_lock` 防止第二个请求与正在运行的提取重叠；并发请求会明确拒绝，不会争用输出文件或造成内存峰值翻倍。失败时只发布状态，不发布伪 CSV/PoseArray。

## 坐标和手眼变换边界

- CSV：位置为相机/PLY 坐标系的 mm，四元数同属相机坐标系；
- PoseArray：位置换算为 m，`frame_id=camera_link`；
- 本节点未应用手眼矩阵，也未直接向 ABB 发送目标；
- 后续由 `handeye_abb_bridge_node` 根据拍照时 `Base←TCP` 与配置的 `TCP←Camera` 执行变换，再次验证工具 TCP/枪尖方向；本节点只发布相机系结果。

这样可让“视觉识别”“标定变换”“机器人通信”三类错误分别定位，不把某台机器人的标定常数绑定进通用焊缝 SDK。
