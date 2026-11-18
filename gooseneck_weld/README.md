# 鹅颈槽焊缝 SDK 1.0.2

Ubuntu 22.04 / ROS 2 Humble 的独立 C++ SDK，与波纹板 SDK 分开安装。来自用户在 Windows 验证的移植包，提取核心保留原来的几何逻辑。原始压缩包存于 [upstream](upstream/gooseneck_weld_portable.zip)，SHA256：`a0c5b9b7b4df92ece23932858db069263927120d4a213e04cc90c508be37b601`。桌面原文件未修改。

## 代码与运行入口

| 文件 | 用途 |
| --- | --- |
| [weld_extract.cpp](weld_extract.cpp) | 三平面、分槽、局部底板高度与红色焊缝筛选 |
| [gooseneck_trajectory.hpp](gooseneck_trajectory.hpp) | 每槽六个实测焊点、两个安全点、工具框架与逐点补偿 |
| [sdk.cpp](sdk.cpp) | PLY/ROI、参数校验、选择槽、CSV/PLY规范化与计时 |
| [SDK头文件](include/gooseneck_sdk/gooseneck_sdk.hpp) | `gooseneck_sdk::run()`，复用当前RunOptions/RunResult传输契约 |
| [ROS YAML](../src/weld_seam_perception/config/gooseneck.yaml) | 完整可调参数，含八点的平移与旋转 |
| [回归测试](tests/regression.py) | 从原始包编译基线，比较真实点云与工具轴、补偿、距离 |
| [离线测试脚本](../scripts/offline_weld.py) | 直接读取ROS YAML，使用与操作界面相同的参数分流 |

当前ROS节点根据 `algorithm_overrides` 中的 `pipeline.algorithm` 分流：省略或 `corrugated` 是原波纹板，`gooseneck` 是本算法。波纹板源码、默认配置和手眼/ABB协议没有替换。节点进程内调用对应共享库；Python便利接口和人工界面使用同一SDK的CLI。

## 与波纹板的区别

| 项目 | 波纹板 | 鹅颈槽 |
| --- | --- | --- |
| 参考结构 | L底板、波纹侧面与周期四类角 | 两平行立面和底板，识别一个或多个U槽 |
| 红色焊缝 | 当前稳定SDK的平面/法向/轮廓流程 | XYZ几何，分层侧边接触线，局部底面中值及L圆角高度补偿 |
| 轨迹 | 三种path.mode，四类角/圆角与直线中点 | 每槽固定8点，竖缝、底部四个转角及两个安全点 |
| 默认姿态 | 当前配置的旧角点继承策略 | 共享参考姿态 + 可调左右侧板外倾；关闭整体修正时复现原包 |
| 平移补偿 | `offset.*` 沿工件坐标轴 | `gooseneck.*.dx/dy/dz` 沿相机XYZ |
| ROS输出 | mm CSV、相机PoseArray(m)、wxyz列 | 相同，继续复用手眼与ABB节点 |

鹅颈槽参考工件坐标以背板/底面交线为原点线，X为参考横轴（朝相机X正方向），Z为底板朝上的法向，Y由右手关系求出。该坐标只用于诊断；机器人使用CSV的相机 `x,y,z,qw,qx,qy,qz`。它与波纹板的工件原点和轴正负定义不相同。

## 点序、姿态与补偿

| 序号 | 角色 | 来源 |
| --- | --- | --- |
| P1 | safe_start | 从起焊点沿开放侧退让 |
| P2 | weld_start | 左端竖缝实测点 |
| P3–P6 | corner_1…corner_4 | 依次沿U底部的四个实测红色焊缝点 |
| P7 | weld_end | 右端竖缝实测点 |
| P8 | safe_end | 从末焊点沿开放侧退让 |

焊点从本次筛选的红色样本中取近邻，有支撑距离检查；它们不是凭空生成的理想交点。安全点为计算点。位置补偿后目标会离开原始测量位置；CSV保存 `raw_*` 和实际偏移以便核对。原包将P7的weld_enabled写为0；集成CSV按“是否为焊点”统一为P2–P7=1。这不是起弧控制指令，ABB仍按既有程序决定动作。

`gooseneck.tool_x_positive=true` 是1.0.1起的默认值，保留原包工具X。按当前手眼矩阵，历史样本的该轴与拍照TCP +X点积约+0.941；此前1.0.0默认反向时约−0.941，会引入不必要的近反向姿态，因此已纠正。`false` 仍可主动反向工具X/Y、保持Z不变（绕工具Z旋转180°），切换不改变零补偿点位。不能从两个算法里“workpiece X”这个名称直接推断实际TCP一致；物理相机安装、TCP和标定沿用原系统，首次联调仍需核对实际目标姿态。

`gooseneck.tool_z_to_body=false` 表示 +Z 从枪体指向TCP；true相反。整体修正和逐点角补偿均为0时，所有点使用原包的同一共享框架；每点roll、pitch、yaw依次绕当前工具局部X、Y、Z旋转，单位度。改变工具X符号也会改变局部轴角补偿的物理含义，应先确定轴向再调角。

### 侧板避让姿态（1.0.2）

原包的8点共享角平分线姿态会使枪轴近似平行于左右侧板。单槽160336的已观测点云中，P2/P3从TCP往枪体方向15–80mm的轴线距左侧板点云仅约0.6–0.8mm；这只是线与可见点云的距离，不是完整枪体与工件的真实间隙，但不能忽略。

- `gooseneck.side_lean_deg`：0–20°，SDK默认0；**ROS鹅颈槽配置当前选10°作为待实机验收候选**。P1–P4向槽左外侧、P5–P8向槽右外侧倾斜，左右镜像。安全点和相邻焊点同侧倾角；横向直线P4→P5有约2倍外倾角的姿态插值。
- `gooseneck.body_tilt_deg`：−30–30°，默认0。在原约45°参考姿态上修正；正值朝底板朝上法向抬枪、远离底板但更靠近前立面；负值相反。**它不能替代左右侧板外倾**。
- 修正顺序：原共享框架 → 整体抬枪 → 左右侧板外倾 → 逐点roll/pitch/yaw。取点、ROI、点位、安全点XYZ、点序、手眼和ABB协议均不变。
- Z向侧面倾斜时，X不可能既完全固定又与Z正交。实现取原X参考在新Z垂直平面内的最近投影，保留正反方向；10°外倾时X也约偏转10°，**不是180°翻转**。逐点角补偿会进一步改变轴向。
- 设两项均0即可复现1.0.1的位姿，不必回退识别算法。SDK不传参数仍为0/0，原包基线回归继续有效。

白线始终是TCP→枪体；当`tool_z_to_body=false`时与工具+Z相反。45°只是普通角接的常见工艺参考，不是防撞依据；喷嘴半径、焊丝伸出、枪颈形状、机器人逆解与完整工件都未建模，不能把轴线间距当作枪体净空。[Miller角接焊枪定位说明](https://www.millerwelds.com/en-US/resources/knowledge-hub/mig-welding/how-to/understanding-the-basics-of-mig-welding-for-mild-steel)

两端安全点的位置先继承邻接焊点的XYZ补偿，再叠加本点补偿；退让方向仍按原包从竖缝开放侧计算。安全点自身的姿态角补偿独立，不自动复制邻接焊点的角补偿。平移上限±100mm、角度±180°是输入校验范围，不是机械臂可达范围。

## 长度、退距、槽号和ROI

- `gooseneck.vertical_length_mm=40`：两端竖缝从局部底面起的高度上限。实际末端吸附实测点，可能略小于设定值；不是整条轨迹总长。
- `gooseneck.clearance_mm=30`：首末安全点退距mm，默认30。
- `gooseneck.groove_index=1`：按参考横轴位置从小到大选择一个槽，一次输出完整8点。双槽可选1或2；不把跨槽移动误当成连续焊接。
- `roi.*`：沿用相机XYZ/mm裁剪。独立鹅颈槽YAML默认全范围；已裁剪数据可直接处理。全幅数据应包含槽、底板和两平行参考立面。底板或立面支持不足时，原算法即不能可靠提取，不能仅删除几何检查。
- `gooseneck.<角色>.dx/dy/dz/roll/pitch/yaw`：8角色×6字段，默认均0，见YAML完整示例。

SDK支持ASCII/二进制PLY，由PCL读取XYZ，法向非必需。原始压缩包仅支持严格的二进制XYZ+法向字段；本集成只规范化输入输出，几何提取仍依据XYZ。SDK内部在独立临时目录生成，成功后发布结果，原始点云文件不被覆盖。

`config_file` 可指向本SDK格式的key=value文件；不要把波纹板.conf传给鹅颈槽。一般保持空串，直接使用ROS YAML的algorithm_overrides。参数在节点启动时读取，改完需要重启对应节点。

## 编译、安装与离线测试

在工作区根目录：

```bash
cmake -S gooseneck_weld -B build/gooseneck_sdk -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX=/home/mini/x86_ros2_ws/gooseneck_weld/install
cmake --build build/gooseneck_sdk -j2
cmake --install build/gooseneck_sdk
source /opt/ros/humble/setup.bash
colcon build --packages-select weld_seam_perception cimc --base-paths src
```

完全离线，不启动ROS：

```bash
python3 scripts/offline_weld.py \
  /home/mini/scut_weld_data/pointclouds/live_20261104_160336_814.ply \
  operator_sessions/gooseneck_manual single \
  --yaml src/weld_seam_perception/config/gooseneck.yaml
```

同一命令加 `--set gooseneck.tool_x_positive=false`、更换前缀为 `reversed_x` 可对比反向X。补偿示例：`--set gooseneck.corner_2.dx=1.5 --set gooseneck.corner_2.pitch=5`。双槽用 `live_20261104_160017_391.ply`，加 `--set gooseneck.groove_index=2`。这些临时覆盖不修改正式YAML。

输出 `<前缀>_features.csv`、`_result.ply`、`_feature_points.ply`、`_settings.conf`。settings记录实际ROI/补偿/方向和检测到的槽数。CSV包含槽号、角色、原始和补偿位置、四元数、工具X及物理枪体轴。result.ply保留分割颜色和红色焊缝，只叠加所选槽的轨迹；黄绿色箭头为工具+X、白线为物理枪体方向。

1.0.2 CSV追加`body_tilt_deg`、`side_lean_deg`和`body_angle_to_base_deg`，最后一个字段按最终逐点补偿后的物理枪体轴与底板平面计算夹角（正值朝底板上方）。`pose_source=outward_side_lean_frame`表示启用侧板外倾，`tilted_transverse_frame`表示仅整体抬枪；界面选中行可查看角度。

## 界面和launch

原命令继续默认波纹板。人工界面在算法页选 `gooseneck`，进入“鹅颈槽参数与逐点补偿”，预览并确认后才写入本次传入的配置文件。选择历史PLY和预览不会保存正式参数。此前“未在YAML中启用的参数编辑后不写入”的默认值比较错误已修复，写入后回读校验并打印配置路径。

希望独立保存两种工件参数时，在现场原launch后加：

```bash
ros2 launch cimc camera_weld_operator_abb.launch.py \
  weld_params_file:=/home/mini/x86_ros2_ws/src/weld_seam_perception/config/gooseneck.yaml
```

固定模式再加 `workflow_mode:=fixed`；旧无界面的ABB launch也支持weld_params_file。两套完整launch不能同时运行。本次没有替换默认launch，也没有改手眼矩阵或ABB文本协议。

## 本次离线验收

`python3 gooseneck_weld/tests/regression.py` 从原始zip编译基线，比较四份真实点云（两单槽、两双槽），每个槽的XYZ和原X姿态一致；另验证X/Y翻转、Z保持、四元数单位长度和右手性、逐点补偿、竖缝长度、安全退距、确定性和无效槽号拒绝。两份不满足参考平面条件的样本在原包和新SDK中均拒绝，未把它们计为成功。

1.0.2另对四份点云的每个槽验证±15°抬枪、10°左右外倾和组合修正：所有XYZ不变，左右枪体分量方向正确，工具X取最近投影，改变X/Z符号不改变物理避让方向，0修正仍与原包相同。这里只验证数值和接口，不证明实际无碰撞。

1.0.0结果在 `operator_sessions/gooseneck_integration/`，其single默认X为旧的反向候选；1.0.1当前TCP约定下的结果在 `operator_sessions/gooseneck_same_tcp_validation/`，请优先查看该目录。`test/offline_backends.py` 位于ROS适配包下，使用隔离ROS域224，仅运行算法节点和测试订阅器，核对话题和离线CLI结果一致；不会运行相机、手眼、ABB接收或执行器节点。`src/cimc/test/test_gooseneck_tcp_contract.py` 另使用生产桥接回调、当前手眼和模拟拍照位姿在内存中验证8点到ABB毫米/wxyz文本，全部发布器为内存记录器，不创建ROS节点或socket。

编译、软件回归与离线ROS接口验证不能验证实际TCP定义、ABB逆解或碰撞。尚未进行真实硬件验证。
