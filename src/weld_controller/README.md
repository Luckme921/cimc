# weld_controller 包说明：USB-CAN 焊机与工艺逻辑

本包保留两个用户最新 C++ 节点。`weld_controller_node` 是实际 CAN 驱动，`weld_logic_node` 是结合 ABB 区域信息控制焊机和旋转电机的上层逻辑。前期只运行前者；后者虽然编译安装，但默认不启动。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/weld_controller_node.cpp` | 调用 `controlcan.h/libcontrolcan.so`，发送焊机 CAN 帧、轮询状态、发布诊断 |
| `src/weld_logic_node.cpp` | 解析 `/abb/raw_text` 的区域/状态，发布焊机工艺与电机转速，并支持人工覆盖 |
| `CMakeLists.txt` | 严格查找 controlcan 头文件/库并链接 `Threads::Threads` |
| `package.xml` | ROS 2 C++ 和标准消息依赖 |

## controlcan 安装与查找

动态库必须是 Linux x86_64 版本：

```bash
file libcontrolcan.so
ldd libcontrolcan.so
```

推荐系统安装：

```bash
sudo install -m 0644 controlcan.h /usr/local/include/controlcan.h
sudo install -m 0755 libcontrolcan.so /usr/local/lib/libcontrolcan.so
sudo ldconfig
```

或使用独立目录：

```text
/opt/controlcan/include/controlcan.h
/opt/controlcan/lib/libcontrolcan.so
```

```bash
export CONTROLCAN_ROOT=/opt/controlcan
```

CMake 不再假设包内存在一个实际为空的 `include/`，也不会模糊链接任意同名库。找不到时会在配置阶段打印头文件/库结果并失败。

## weld_controller_node 接口

- 订阅 `/weld/control`，`std_msgs/msg/String`：系统、送气、送丝、起弧、停弧等命令；
- 订阅 `/weld/set_param_real`，`std_msgs/msg/Float32MultiArray`：焊机实时参数；
- 发布 `/weld/status`，`std_msgs/msg/String`：发送/接收原始帧、焊机反馈与掉线状态。

源码使用相对名 `weld/control`，在根命名空间运行时解析为 `/weld/control`。如果以后把节点放进 ROS namespace，话题也会随 namespace 改变；生产集成时需要明确 remap。

运行：

```bash
ros2 run weld_controller weld_controller_node
ros2 topic echo /weld/status
ros2 topic pub --once /weld/control std_msgs/msg/String "{data: 'start_system'}"
ros2 topic pub --once /weld/set_param_real std_msgs/msg/Float32MultiArray \
  "{data: [230.0, 22.0]}"
```

焊机与 CAN 盒属于实际执行设备。任何点火/送丝命令都应在设备安全、人员撤离和急停有效的条件下测试。

## weld_logic_node 接口（先不运行）

订阅：

- `/abb/raw_text`：ABB 状态和区域编号；
- `/weld/status`：焊机在线/起弧状态；
- `/cimc/override_cmd`：自动/手动/急停/参数模式覆盖；
- `/cimc/override_param`：人工电流、电压、电机转速等参数。

发布：

- `/weld/control`：焊机动作；
- `/weld/set_param_real`：焊机工艺参数；
- `/cimc/motor_speed`：偏心旋转焊枪转速。

它启动 3 秒后会发送 `start_system`，并带焊机状态看门狗，因此“只是运行看看”也可能产生实际控制输出。必须在 ABB 区域语义、工艺表、CAN 驱动和电机节点分别验收后再启动。

## 构建与运行库检查

```bash
source /opt/ros/humble/setup.bash
cd ~/x86_ros2_ws
colcon build --symlink-install --packages-select weld_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ldd install/weld_controller/lib/weld_controller/weld_controller_node
```

`ldd` 不应出现 `libcontrolcan.so => not found`。本包不提供 launch，避免误启动 `weld_logic_node`。
