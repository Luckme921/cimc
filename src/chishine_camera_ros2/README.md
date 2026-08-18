# chishine_camera_ros2 包说明

该包把 Chishine3D SDK 3.2.52 的设备发现、连接、软件触发、Z16 深度重建和 PLY 写盘封装为一个 ROS 2 节点。它不会连续无条件采集；只有收到 `/camera/capture` 服务请求才抓取一帧。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/chishine_camera_node.cpp` | 相机生命周期、软件触发、点云重建、文件发布 |
| `config/camera.yaml` | 相机序列号、发现重试、流、曝光、深度范围和输出参数 |
| `CMakeLists.txt` | 查找 x86_64/aarch64 原厂库、设置 RPATH、把动态库安装到节点旁 |
| `THIRD_PARTY_NOTICES.md` | 原厂授权条件，随包安装 |

## ROS 2 接口

- 服务 `/camera/capture`，`std_srvs/srv/Trigger`：触发一帧；成功时 `response.message` 是 PLY 绝对路径；
- 发布 `/camera/pointcloud_file`，`std_msgs/msg/String`，Transient Local：PLY 完整写盘并校验非空后发布绝对路径。

Transient Local 让稍后启动的焊缝节点也能收到最近一次成功采集路径。

## 参数

| 参数 | 默认值 | 含义 |
|---|---:|---|
| `camera_serial` | 空 | 空表示第一台；多相机按序列号选择 |
| `enable_network_discovery` | `true` | 启用 SDK 网络发现两层开关 |
| `enable_usb_discovery` | `true` | 启用 SDK UVC 发现 |
| `discovery_timeout_ms` | `3000` | 每次枚举超时 |
| `discovery_attempts` | `10` | 启动阶段重复枚举次数 |
| `discovery_retry_interval_ms` | `1000` | 枚举重试间隔 |
| `output_directory` | 空 | 默认 `~/scut_weld_data/pointclouds` |
| `file_prefix` | `capture` | PLY 文件名前缀，后接毫秒时间戳 |
| `capture_timeout_ms` | `5000` | 软件触发后的取帧超时 |
| `depth_width/height/fps` | `0` | `0` 表示接受第一种匹配 Z16 流 |
| `rgb_width/height/fps` | `0` | 只在 `enable_rgb=true` 时使用 |
| `enable_rgb` | `false` | 焊缝算法只需 XYZ；默认关闭以减少等待和裁剪 |
| `binary_ply` | `true` | 二进制 PLY 更快更小 |
| `depth_min_mm/max_mm` | `460/520` | 深度算法工作范围 |
| `depth_gain` | `1.0` | 深度增益 |
| `depth_exposure` | `8000` | 深度曝光 |
| `depth_frame_time` | `10000` | 深度帧时间 |

参数文件路径：`config/camera.yaml`。

## IP 与序列号

SDK 不接收 `camera_ip`/`host_ip` 参数。Ubuntu 网卡和相机自身必须先处于可路由网络，节点调用 `queryCameras` 发现设备，然后用 `CameraInfo` 连接。`camera_serial` 是发现后的选择条件。

网络问题先用同级 `x86_chishine_camera_test` 的 `--list --usb false` 排查。独立程序都枚举不到时，不应继续修改 ROS 2 话题或 QoS。

## 构建

三个工程保持同级时可自动找到 SDK；也建议显式设置：

```bash
export CHISHINE_3D_CAMERA_SDK_ROOT="$HOME/x86_chishine_camera_test/vendor_sdk"
source /opt/ros/humble/setup.bash
cd ~/x86_ros2_ws
colcon build --symlink-install --packages-select chishine_camera_ros2 \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
```

运行：

```bash
ros2 run chishine_camera_ros2 chishine_camera_node --ros-args \
  --params-file ~/x86_ros2_ws/src/chishine_camera_ros2/config/camera.yaml
ros2 service call /camera/capture std_srvs/srv/Trigger "{}"
ros2 topic echo --once /camera/pointcloud_file
```

节点启动时即连接并启动深度流，退出时停止流并断开。捕获服务用互斥锁保护相机，两个并发请求不会同时操作厂商句柄。

## 输出与异常策略

可选属性（增益、曝光、深度范围）若相机型号不支持，节点告警并继续使用相机当前值。下列错误会令启动/请求失败：无相机、指定序列号不存在、连接失败、无 Z16 流、无内参、不能启用软件触发、取帧失败、重建 0 点或 PLY 写盘失败。

节点只在文件存在且大小非零后发布路径，因此焊缝节点不会收到一个尚未写完的文件。
