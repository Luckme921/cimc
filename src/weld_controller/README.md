# weld_controller 包说明：USB-CAN 焊机驱动

`weld_controller_node` 是本包唯一构建和安装的节点。旧 `weld_logic_node.cpp` 仅保留用于历史对比；它按 ABB 点号自行决定工艺，当前架构不编译或运行它。

`192.168.3.5` 的 JSONL v1 解析和底层话题映射由 `cimc/data_receiver_node.py` 完成，不在本包再增加中间焊接大脑。协议见仓库根目录的 `第三方焊接通信协议.md`。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/weld_controller_node.cpp` | 调用 `controlcan.h/libcontrolcan.so`，发送焊机 CAN 帧、轮询状态、发布诊断 |
| `src/weld_logic_node.cpp` | 历史固定点号工艺大脑；仅保留源码，不编译、不安装 |
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
- 发布 `/weld/feedback_raw`，`std_msgs/msg/UInt8MultiArray`：每个真实 TPDO1 的原始 6 字节反馈帧。

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

## weld_logic_node.cpp（历史源码，不编译）

旧源码订阅 ABB 点号并用 C++ 固定表决定电流、电压和转速，而且节点启动后会自动请求 `start_system`。它与“192.168.3.5 决定工艺、x86 只执行”的新职责冲突，因此已从 CMake 构建和安装目标移除，不能再通过 `ros2 run` 启动。

## 构建与运行库检查

```bash
source /opt/ros/humble/setup.bash
cd ~/x86_ros2_ws
colcon build --symlink-install --packages-select weld_controller \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source install/setup.bash
ldd install/weld_controller/lib/weld_controller/weld_controller_node
```

`ldd` 不应出现 `libcontrolcan.so => not found`。本包不提供 launch，安装环境只应列出 `weld_controller_node`。
