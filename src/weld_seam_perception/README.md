# weld_seam_perception 包说明

该包是焊缝 SDK 2.2 的轻量 ROS 2 适配层。算法不复制到本包，也不通过 shell 启动 CLI；节点直接链接 `libweld_seam_sdk.so`，接收 PLY 路径并发布结构化结果。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/weld_seam_node.cpp` | 输入路径、互斥调用 SDK、发布文件路径/JSON/PoseArray |
| `config/weld_seam.yaml` | ROS 参数和算法 `key=value` 覆盖 |
| `CMakeLists.txt` | 要求 `weld_seam_sdk >= 2.2` 并链接 `weld_seam::sdk` |
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
  - "roi.enable=false"
  - "normal.mode=auto"
  - "orientation.tool_positive_z_points_from_tcp_to_body=true"
```

ROI、四类位置偏置、四类姿态等均可继续添加，不用重新编译节点。例如：

```yaml
  - "roi.enable=true"
  - "roi.max_y=300"
  - "offset.protruding_left.x=2.5"
```

完整键名及含义见 `x86_weld_seam_test/config/default.conf` 和该工程 README。

## 构建

先安装 SDK：

```bash
cd ~/x86_weld_seam_test
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
cmake --install build --prefix "$HOME/scut_weld_sdk_install"
```

再构建本包：

```bash
export CMAKE_PREFIX_PATH="$HOME/scut_weld_sdk_install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$HOME/scut_weld_sdk_install/lib:$LD_LIBRARY_PATH"
source /opt/ros/humble/setup.bash
cd ~/x86_ros2_ws
colcon build --symlink-install --packages-select weld_seam_perception \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

运行与离线 PLY 测试：

```bash
ros2 run weld_seam_perception weld_seam_node --ros-args \
  --params-file ~/x86_ros2_ws/src/weld_seam_perception/config/weld_seam.yaml

ros2 topic pub --once /camera/pointcloud_file std_msgs/msg/String \
  "{data: '/absolute/sample.ply'}"
```

## 并发和故障行为

SDK 当前按一次一份大点云同步运行。节点用 `try_lock` 防止第二个请求与正在运行的提取重叠；并发请求会明确拒绝，不会争用输出文件或造成内存峰值翻倍。失败时只发布状态，不发布伪 CSV/PoseArray。

## 坐标和手眼变换边界

- CSV：位置为相机/PLY 坐标系的 mm，四元数同属相机坐标系；
- PoseArray：位置换算为 m，`frame_id=camera_link`；
- 本节点未应用手眼矩阵，也未直接向 ABB 发送目标；
- 后续应由独立节点把 `T_base_camera * T_camera_tcp`（具体乘法链取决于标定方式）转换到 ABB 所需坐标系，并再次验证工具 TCP/枪尖方向。

这样可让“视觉识别”“标定变换”“机器人通信”三类错误分别定位，不把某台机器人的标定常数绑定进通用焊缝 SDK。
