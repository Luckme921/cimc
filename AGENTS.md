# Codex ROS2 工控机开发规则

## 1. 主开发工作区

本项目唯一的主开发工作区是：

/home/mini/x86_ros2_ws

主要代码位于：

/home/mini/x86_ros2_ws/src

默认情况下，源码修改、配置修改、编译和软件测试都应限制在：

/home/mini/x86_ros2_ws

内部。

不要主动修改工作区之外的文件。

---

## 2. ROS2 环境

系统：

Ubuntu 22.04
ROS2 Humble

基础 ROS2 环境：

source /opt/ros/humble/setup.bash

工作区：

/home/mini/x86_ros2_ws

编译前优先从干净的 ROS2 Humble 环境开始。

不要因为历史 install 目录存在，就默认旧的 overlay 一定正确。

修改代码之后，优先只编译受影响 package：

colcon build --packages-select <package_name>

只有有充分理由时才进行整个 workspace 构建。

不要无意义删除：

build/
install/
log/

如果认为必须清理构建目录，先说明原因。

---

## 3. 外部参考目录

允许读取 /home/mini 下所有名称以 x86 或 scut 开头的目录。

当前已知包括：

/home/mini/x86_chishine_camera_test
/home/mini/x86_chishine_live_viewer
/home/mini/x86_weld_seam_test
/home/mini/x86_ros2_ws_copy
/home/mini/scut_weld_sdk_install
/home/mini/scut_weld_data

这些目录属于：

- 历史测试代码
- SDK 示例
- 独立验证程序
- 旧版本实现
- 数据或运行参考

允许：

- 阅读
- grep / rg 搜索
- 对比源码
- 查找 SDK API 用法
- 查找相机初始化方式
- 查找算法调用方式
- 查找历史实现
- 用于判断当前 ROS2 实现是否正确

默认禁止：

- 修改
- 删除
- 移动
- 重命名
- 覆盖
- 格式化这些目录里的文件

只有用户明确要求修改某个外部目录时才可以修改。

---

## 4. Bash 环境

允许读取：

/home/mini/.bashrc

主要用于检查：

- ROS2 环境
- PATH
- LD_LIBRARY_PATH
- 相机 SDK 环境变量
- 焊缝 SDK 环境变量
- source setup.bash
- 第三方库环境配置

默认禁止修改：

/home/mini/.bashrc

如果认为 .bashrc 必须修改：

1. 先指出当前问题。
2. 给出建议修改内容。
3. 说明可能产生的影响。
4. 等待用户明确要求后再修改。

---

## 5. 真实工业硬件安全规则

这是连接真实工业设备的 ROS2 工程。

除非用户明确要求，否则禁止主动执行可能导致物理设备动作的命令。

特别禁止自行执行：

- ABB 机器人运动
- ABB 自动轨迹执行
- 机器人 RAPID 程序启动
- 自动发送运动轨迹
- 焊机起弧
- 焊接工艺启动
- 电机运动
- 自动运动控制
- 激光器危险输出
- 任何可能导致机械运动的 ROS2 service/action/topic 命令

对于以下操作也要谨慎：

- ros2 launch
- ros2 run
- ros2 service call
- ros2 action send_goal
- ros2 topic pub
- 厂商 SDK 示例程序
- 硬件测试程序

在无法确定是否会产生真实硬件动作时：

不要执行。

先说明命令是什么、会影响什么设备，再等待用户决定。

---

## 6. 默认允许的软件操作

可以主动执行：

- pwd
- ls
- find
- tree
- rg
- grep
- cat
- sed 的只读查看操作
- git status
- git diff
- git log
- ROS2 package 静态分析
- CMakeLists.txt 分析
- package.xml 分析
- Python/C++ 源码分析
- 编译
- lint
- 静态检查
- 不连接真实执行器的单元测试
- 对离线 CSV/PLY/图片等数据进行处理
- 查看 ROS2 interface 定义

编译本身如果可能执行自定义危险脚本，需要先检查相关 build script。

---

## 7. 修改代码前

修改任何代码前：

1. 阅读相关 package。
2. 阅读相关 CMakeLists.txt / package.xml。
3. 搜索当前实现。
4. 搜索 /home/mini 下 x86* 和 scut* 参考目录。
5. 判断是否已有成熟测试实现。
6. 明确问题根因。
7. 尽量选择最小修改方案。

不要为了“优化”而进行无关的大范围重构。

---

## 8. 修改代码后

每次修改后：

1. 列出修改文件。
2. 查看 git diff（如果存在 Git）。
3. 检查明显编译问题。
4. 优先编译受影响 package。
5. 报告编译结果。
6. 区分：
   - 静态验证通过
   - 编译通过
   - 软件测试通过
   - 硬件测试通过
7. 修改代码后在x86_ros2_ws下执行上传git的指令，这样可以保证每次的修改都被git记录，失败代码回退也方便，git add . ,git commit -m “时间：月+日+时+分+合理的修改内容解释” ，git push ,这三个命令就可以上传到云端git了

如果没有真实硬件验证，必须明确写：

“尚未进行真实硬件验证。”

不能把“编译通过”等同于“真实系统已经验证”。

---

## 9. 当前工程开发原则

当前重点是：

- ROS2 节点架构
- 相机采集
- Chishine 相机 SDK
- 焊缝识别 SDK
- 焊缝数据
- 坐标变换
- 手眼标定
- TCP 位姿
- ABB 通信
- 后续机器人焊接流程

开发时优先利用现有稳定测试程序，而不是重新实现已经验证过的 SDK 调用逻辑。

特别是涉及：

- 相机枚举
- 相机初始化
- 软件触发
- 图像获取
- SDK 初始化
- 焊缝识别
- 数据格式解析

时，优先对比：

/home/mini/x86_chishine_camera_test
/home/mini/x86_chishine_live_viewer
/home/mini/x86_weld_seam_test
/home/mini/scut_weld_sdk_install

---

## 10. 核心原则

安全优先。

先理解，再修改。

先最小修改，再编译。

先离线验证，再连接硬件。

任何涉及真实机器人运动、焊接、电机或其他危险执行器的步骤，都必须由用户明确决定。
