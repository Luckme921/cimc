# Chishine3D 相机实时点云定位工具

这个独立工程用于安装相机、调整拍照位置和确认工件是否完整进入3D视野。它直接调用 Chishine3D SDK 3.2.52 的连续深度流，在内存中将 Z16 深度图重建为 XYZ 点云，并通过 PCL 实时显示。

它不会修改已经验证通过的 `chishine_camera_ros2` 节点，也不会自动调用焊缝提取算法。正式工作仍然采用 `/camera/capture` 软件触发单帧拍照；本工具仅用于现场定位和成像调试。

## 1. 工作流程

```text
发现并连接相机
  → 选择 Z16 深度流
  → 设置 TRIGGER_MODE_OFF 连续输出
  → 循环获取深度帧
  → 使用相机内参和 depthScale 重建 XYZ
  → 内存中更新三维窗口
  → 按 S 时才保存当前完整点云为 PLY
```

实时显示不会每帧执行“写PLY再读PLY”，因此磁盘开销和延迟明显更低。窗口为了流畅性最多显示指定数量的抽样点；按 `S` 保存的仍然是当前帧的完整有效点云。

## 2. 目录说明

| 文件 | 作用 |
|---|---|
| `src/chishine_live_viewer.cpp` | 相机连接、连续取帧、XYZ重建、PCL实时显示和快照保存 |
| `CMakeLists.txt` | 查找 SDK、PCL 并编译 `chishine_live_viewer` |
| `README.md` | 中文部署、运行、按键和故障说明 |

默认复用同级目录：

```text
~/x86_chishine_camera_test/vendor_sdk
```

也可以用 `-DCHISHINE_SDK_ROOT=...` 指定其他SDK路径。

## 3. 使用前必须停止ROS相机节点

同一时刻只允许一个进程连接相机。运行实时预览前，在原来启动 `chishine_camera_node` 的终端按 `Ctrl+C`。

检查是否仍有相机节点：

```bash
ros2 node list | grep chishine_camera
```

如果没有输出，才启动实时预览。不要同时运行实时查看器和ROS相机节点。

## 4. 安装依赖

```bash
sudo apt update
sudo apt install -y build-essential cmake libpcl-dev
```

厂商库应已经按相机测试阶段安装：

```bash
ls -lh /usr/local/lib/lib3DCamera.so
ldconfig -p | grep lib3DCamera
```

## 5. 编译

假设三个工程位于：

```text
~/x86_chishine_camera_test
~/x86_chishine_live_viewer
~/x86_ros2_ws
```

执行：

```bash
cd ~/x86_chishine_live_viewer

cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DCHISHINE_SDK_ROOT="$HOME/x86_chishine_camera_test/vendor_sdk"

cmake --build build -j"$(nproc)"
```

检查动态库：

```bash
ldd ./build/chishine_live_viewer | grep -E '3DCamera|not found'
```

不得出现 `not found`，`lib3DCamera.so` 最好指向：

```text
/usr/local/lib/lib3DCamera.so
```

## 6. 启动实时预览

先尝试普通用户运行，因为已经验证ROS相机节点可以由普通用户连接：

```bash
cd ~/x86_chishine_live_viewer

./build/chishine_live_viewer \
  --network true \
  --usb false \
  --save-dir /home/mini/scut_weld_data/pointclouds
```

如果普通用户显示 `devices=0`，使用之前已经验证成功的临时 `CAP_NET_RAW` 启动方法。这个方法最终仍以 `mini` 用户显示窗口，不会产生root所有者的PLY：

```bash
cd ~/x86_chishine_live_viewer

sudo --preserve-env=DISPLAY,XAUTHORITY,WAYLAND_DISPLAY,XDG_RUNTIME_DIR \
  setpriv \
  --reuid=mini \
  --regid=mini \
  --init-groups \
  --inh-caps=+net_raw \
  --ambient-caps=+net_raw \
  ./build/chishine_live_viewer \
    --network true \
    --usb false \
    --save-dir /home/mini/scut_weld_data/pointclouds
```

如 `sudo` 提示不允许保留某个不存在的显示变量，可先检查：

```bash
echo "$DISPLAY"
echo "$WAYLAND_DISPLAY"
echo "$XDG_RUNTIME_DIR"
```

本机Ubuntu桌面一般直接普通用户运行即可。

## 7. 窗口操作

| 操作 | 功能 |
|---|---|
| 鼠标左键拖动 | 旋转点云 |
| 鼠标滚轮 | 缩放 |
| 鼠标中键拖动 | 平移 |
| `S` | 保存当前完整点云到 `--save-dir` |
| `Q` 或 `Esc` | 退出程序 |
| `Ctrl+C` | 从终端退出 |

窗口左上角显示：帧编号、当前完整点数、显示点数和刷新频率。

## 8. 推荐运行参数

默认选择相机提供的第一路 Z16，当前设备一般是 `960x600`。安装定位阶段建议先用默认值：

```bash
./build/chishine_live_viewer \
  --network true \
  --usb false \
  --point-size 2 \
  --max-display-points 300000 \
  --save-dir /home/mini/scut_weld_data/pointclouds
```

需要观察更高分辨率时：

```bash
./build/chishine_live_viewer \
  --network true \
  --usb false \
  --width 1920 \
  --height 1200 \
  --max-display-points 400000 \
  --save-dir /home/mini/scut_weld_data/pointclouds
```

高分辨率重建和显示开销更大，不一定更适合定位。实时窗口抽样只影响显示速度，不影响按 `S` 保存的完整PLY。

## 9. 成像参数默认策略

本工具默认不覆盖相机当前的增益、曝光、帧时间和深度范围，只切换为连续输出模式。这符合“相机已经调好参数，程序只负责实时观察”的使用方式。

仅在需要实验时才传入：

```bash
./build/chishine_live_viewer \
  --network true \
  --usb false \
  --depth-min-mm 300 \
  --depth-max-mm 1500 \
  --frame-time 20000 \
  --exposure 15000 \
  --gain 1.0
```

如果曝光大于当前帧时间，应同时把 `--frame-time` 设置得更大。

## 10. 拍照定位建议

1. 先用哑光白纸板确认相机能够产生大面积连续点云。
2. 再放入波纹板，调整相机，使水平底面、L形侧面和完整波纹周期都进入视野。
3. 查看目标区域是否存在大面积空洞、过曝或遮挡。
4. 位置满意后按 `S` 保存测试PLY。
5. 退出实时预览，再启动ROS相机节点。
6. 调用 `/camera/capture` 保存正式PLY，并交给焊缝提取节点。

实时预览模式和ROS正式软件触发模式可能在曝光时序上略有差异，因此最终仍需用 `/camera/capture` 保存一帧进行焊缝算法验证。

## 11. 常见问题

### `devices=0`

- 确认ROS相机节点已经停止；
- 确认 `ping 192.168.3.99` 正常；
- 确认路由走 `enp87s0`；
- 使用上面的 `setpriv + CAP_NET_RAW` 命令。

### 窗口打开但没有点

- 相机是否朝向有效工作距离内的物体；
- 镜头或投影窗口是否有保护盖；
- 先用哑光白纸板测试；
- 暂时不要限定过窄的深度范围。

### 刷新较慢

降低显示点数：

```bash
--max-display-points 150000
```

或者保持 `960x600`，不要直接切换到 `1920x1200`。

### 保存的PLY在哪里

按 `S` 后终端会打印绝对路径，文件名形式为：

```text
live_20260819_153012_123.ply
```

### 退出后如何恢复正式工作

先退出查看器，然后重新启动ROS相机节点：

```bash
ros2 run chishine_camera_ros2 chishine_camera_node --ros-args \
  --params-file ~/x86_ros2_ws/src/chishine_camera_ros2/config/camera.yaml
```

