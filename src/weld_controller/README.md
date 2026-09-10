# weld_controller 包说明：USB-CAN 焊机与远程执行接口

`weld_controller_node` 是实际 CAN 驱动；`weld_remote_interface_node` 把第三方设备的高级意图转换成现有焊机和旋转电机话题。旧 `weld_logic_node` 仍保留用于历史对比和回退，但它按 ABB 点号自行决定工艺，当前架构不再运行它。

当前 `192.168.3.5` 的正式入站报文格式尚未确定，因此网络接收层还没有接入。现阶段先通过 `/weld/remote/*` 话题模拟第三方指令；正式协议确定后只需把网络报文映射到同一组 ROS 输入，不改底层焊机驱动。

## 文件作用

| 文件 | 作用 |
|---|---|
| `src/weld_controller_node.cpp` | 调用 `controlcan.h/libcontrolcan.so`，发送焊机 CAN 帧、轮询状态、发布诊断 |
| `src/weld_remote_interface_node.cpp` | 接收第三方高级指令和 `[电流, 旋转速度]`，校验后映射到已有底层话题 |
| `config/weld_remote_interface.yaml` | dry-run 开关、指令顺序、参数范围和话题配置 |
| `src/weld_logic_node.cpp` | 历史固定点号工艺大脑；保留源码但当前不运行 |
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

## weld_remote_interface_node（当前替代工艺大脑）

该节点启动时不会自动送气、起弧或启动电机。当前配置默认：

```yaml
output_enabled: false
require_gas_before_weld: true
```

`output_enabled=false` 是 dry-run：指令会被校验、更新内部模拟状态并发布 `/weld/remote/status`，但不会发布到底层焊机和电机话题。

订阅：

- `/weld/remote/command`，`String`：高级动作指令；
- `/weld/remote/setpoints`，`Float32MultiArray`：严格为 `[current_A, rotation_speed_rps]`。

发布：

- `/weld/control`：焊机动作；
- `/weld/set_param_real`：`[current_A, voltage_placeholder_V]`；
- `/cimc/motor_speed`：偏心旋转焊枪转速。
- `/weld/remote/status`：指令是否接受、是否真正转发、dry-run 和内部状态。

状态中的 `forwarded=true` 只表示已发布到 ROS 底层话题，不表示 CAN 焊机、电机或物理工艺已经成功执行；真实结果必须结合 `/weld/status`、`/weld/feedback_raw` 和设备状态判断。

高级命令：

| 命令 | 行为 |
|---|---|
| `GAS_ON` / `START_GAS` | 依次请求一元内置曲线、CAN 启动和送气 |
| `WELD_START` / `START_WELDING` | 请求起焊；默认必须先收到 `GAS_ON` |
| `WELD_STOP` / `STOP_WELDING` / `STOP_ALL` | 停焊并把旋转速度置零 |
| `FAULT_RESET` | 请求焊机故障复位 |

默认一元模式下，第三方只实时给电流和旋转速度。兼容底层两元素接口所填的 `unary_voltage_placeholder_v` 不参与内置曲线的 CAN 压强计算。

安全 dry-run 模拟：

```bash
ros2 run weld_controller weld_remote_interface_node --ros-args \
  --params-file ~/x86_ros2_ws/src/weld_controller/config/weld_remote_interface.yaml

ros2 topic echo /weld/remote/status
ros2 topic pub --once /weld/remote/command std_msgs/msg/String "{data: 'GAS_ON'}"
ros2 topic pub --once /weld/remote/setpoints std_msgs/msg/Float32MultiArray \
  "{data: [250.0, 6.0]}"
ros2 topic pub --once /weld/remote/command std_msgs/msg/String "{data: 'WELD_START'}"
ros2 topic pub --once /weld/remote/command std_msgs/msg/String "{data: 'WELD_STOP'}"
```

只有完成焊机、气路、电机、硬件急停及软件顺序验证后，才能把配置中的 `output_enabled` 改为 `true`。软件 `WELD_STOP` 不能代替硬件急停。

## weld_logic_node（历史节点，不运行）

旧节点订阅 ABB 点号并用 C++ 固定表决定电流、电压和转速，而且启动后会自动请求 `start_system`。它与“192.168.3.5 决定工艺、x86 只执行”的新职责冲突，因此没有加入任何 launch，也不应和 `weld_remote_interface_node` 同时运行。

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
