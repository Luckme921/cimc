# x86 焊缝提取算法与 SDK 2.4.4

本目录是 Ubuntu 22.04 x86_64 上的独立算法工程。它同时生成：

- `weld_seam_extractor`：直接输入 PLY、输出 CSV/PLY 的测试程序；
- `libweld_seam_sdk.so`：供 ROS 2 节点或其他 C++ 程序进程内调用的共享库；
- 可安装的头文件和 CMake package，外部工程可使用 `find_package(weld_seam_sdk)`。

2.4.4 延续 2.4.3 的四类工具姿态、工件偏置和首末边界保护，并修正五点圆角的贪心取点：前一个实测点不能占据下一个目标的 X 区域，同组目标沿期望 Y 方向不回退。可靠红点优先，只有两侧支撑充分且宽度受 `path.max_bridge_gap` 限制的小空洞才用连续切向模型补点。CSV 的 `point_source=modeled_small_hole_from_adjacent_lines` 和结果 PLY 的亮绿色十字星明确标记非实测点；点序回退/过近会整条拒绝。`path.rounded_visualization_only=true` 只写诊断 PLY、返回失败、不写可发送 ABB 的 CSV。

2.4.2 收紧了 `rounded_features` 的首末安全边界：如果首末已确认角之外还有超过 `feature.max_corner_extrapolation` 的红色焊缝，则拒绝发布可能漏焊的部分轨迹。CSV 同时保存偏置前 `raw_*` 和最终机器人目标，便于区分识别误差与工艺偏置。

## 1. 文件说明

| 路径 | 作用 |
|---|---|
| `main.cpp` | 完整焊缝/拐点/姿态算法；既是 CLI 源码也是 SDK 实现 |
| `CMakeLists.txt` | 构建 CLI、共享库、安装包和 CMake 导出配置 |
| `include/weld_seam_sdk/weld_seam_sdk.hpp` | 稳定的 C++ SDK 公共接口 |
| `config/default.conf` | 全部可运行时修改的算法参数及当前生产默认值 |
| `参数手册.md` | 全部运行参数的逐项中文含义、坐标方向和调参顺序 |
| `cmake/weld_seam_sdkConfig.cmake.in` | 供安装后的 `find_package` 使用 |
| `build/` | 本机编译目录，可删除后重新生成 |
| `output/` | 建议保存独立测试结果，不参与编译 |

## 2. 总处理流程

```mermaid
flowchart TD
    A["输入 PLY：XYZ，法向量可有可无"] --> B["单次读取 PCLPointCloud2\n检查 XYZ 与 nx/ny/nz"]
    B --> C["ROI：可选，先裁掉无关视野"]
    C --> D["0.5 mm VoxelGrid 降采样"]
    D --> E{"normal.mode"}
    E -->|"auto 且输入法向完整有效"| F["复用、单位化并朝向相机光心"]
    E -->|"auto 缺失/低质量"| G["NormalEstimationOMP，K 邻域重算"]
    E -->|"recompute"| G
    E -->|"reuse 但质量不足"| H["明确报错，不输出伪结果"]
    F --> I["RANSAC 搜索主平面并全分辨率精修"]
    G --> I
    I --> J["构建工件局部右手坐标系"]
    J --> K["稳定版红色焊缝点提取"]
    K --> L{"path.mode"}
    L -->|"feature_points"| M["四类周期拓扑、拐点与安全弦"]
    L -->|"rounded_features"| R["现场姿态 + 首末单点 + 内部圆角5点 + 角间中点"]
    L -->|"adaptive_contour"| P["中值轮廓、离群抑制、曲率自适应采样"]
    M --> N["分类偏置、姿态与安全过渡点"]
    R --> N
    P --> N
    N --> O["CSV + 精确点 PLY + 粉红十字可视化 PLY"]
```

### 输入和输出数据

1. 输入是相机坐标系中的 PLY。坐标单位必须与现有算法一致，当前样本为 mm。
2. ROI 和几何检测都在 PLY 坐标系中进行；算法内部再根据 L 形底面、侧面建立工件局部坐标系。
3. CSV 的 `x,y,z` 仍是 PLY/相机坐标系下的 mm，四元数也表达同一相机坐标系下的工具姿态。
4. 后续手眼矩阵负责把整个位姿从相机坐标系变换到机器人基座或其他目标坐标系。本 SDK 不擅自应用手眼矩阵。

## 3. 法向量策略（2.2 新增）

`normal.mode` 有三种值：

| 值 | 实际行为 | 使用建议 |
|---|---|---|
| `auto` | 默认。输入 PLY 含高质量法向量就复用，否则自动回退重算 | 生产环境推荐 |
| `recompute` | 忽略 PLY 法向量，体素后按 K 邻域重新估计 | 对比旧稳定版、怀疑相机法向量时使用 |
| `reuse` | 强制使用输入法向量；字段缺失或有效率不足就失败 | 相机法向量质量已被严格验证时使用 |

相机 PLY 文件头常写 `nx/ny/nz`，PCL 标准字段是 `normal_x/normal_y/normal_z`。程序会在内存中归一化字段名，不复制点数据。输入法向量满足以下条件才进入复用快路径：

1. 三个分量同时存在；
2. XYZ 有限的点中，法向量有限且模长非零的比例不低于 `normal.reuse_min_valid_ratio`，默认 `0.995`；
3. 经过 VoxelGrid 后再次检查有效率；
4. 每个法向量重新单位化，并统一朝向 PLY 原点（通常是相机光心），避免左右腰符号因相机法向方向习惯而翻转。

`auto` 的设计目标是：有法向量时节省最耗时的法线估计，没有法向量、字段不完整或质量不可靠时自动保持旧版本稳定行为。

已对当前根目录 4 个原厂 PLY 做二进制字段统计：有效法向比例约为 `0.99957–0.99987`，有效法向模长均值约为 `1.0`，均高于默认 `0.995`，因此会进入复用快路径。新相机/新固件仍应观察终端实际日志，不应只根据文件扩展名判断。

终端会打印：

```text
Normals after VoxelGrid: source=reused_from_input, valid=.../...
```

或：

```text
Normals after VoxelGrid: source=recomputed, valid=.../..., K=20
```

计时现在互不重叠：

- `Normal processing time`：体素后的法向整理或重算耗时；PLY 字段读取/前置有效率检查计入总时间；
- `Primary seam extraction time (excluding normals)`：ROI、体素、平面和红色焊缝提取，不含法向阶段；
- `Secondary feature extraction time`：折线拐点、姿态和输出路径构造；
- `Total time including PLY I/O`：包含读取、全部算法和文件写出。

## 4. Ubuntu 22.04 编译

安装依赖：

```bash
sudo apt update
sudo apt install -y build-essential cmake libpcl-dev libeigen3-dev
```

普通 Release 编译：

```bash
cd ~/x86_weld_seam_test
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j"$(nproc)"
```

如果可执行文件只在当前这台 13 代酷睿工控机使用，可增加 CPU 原生优化：

```bash
cmake -S . -B build \
  -DCMAKE_BUILD_TYPE=Release \
  -DWELD_SEAM_NATIVE_OPTIMIZATION=ON
cmake --build build -j"$(nproc)"
```

`-march=native` 可能更快，但生成的二进制不保证能在较老 CPU 上运行。要交付给其他 x86_64 主机时保持该选项为 `OFF`。

## 5. 独立程序测试

```bash
./build/weld_seam_extractor \
  /absolute/input.ply \
  ./output \
  sample01 \
  --config ./config/default.conf \
  --set normal.mode=auto
```

参数优先级为：源码默认值 `<` 配置文件 `<` 命令行重复出现的 `--set key=value`。

例如临时打开 ROI，无需重新编译：

```bash
./build/weld_seam_extractor input.ply output test \
  --config config/default.conf \
  --set roi.enable=true \
  --set roi.min_y=-90 \
  --set roi.max_y=-30 \
  --set offset.protruding_left.x=2.5
```

使用同一历史 PLY 对比两种路径模式：

```bash
# 默认四类拐点路径
./build/weld_seam_extractor input.ply output corners \
  --config config/default.conf \
  --set path.mode=feature_points

# 备用连续轮廓路径；完整轨迹（含两个安全点）不会超过100点
./build/weld_seam_extractor input.ply output contour \
  --config config/default.conf \
  --set path.mode=adaptive_contour

# 推荐的稀疏圆角路径。N个确认角的默认点数为：
# 2个安全点 + 2个首末单点 + (N-2)*5个内部圆角点 + (N-1)个角间中点。
# 例如N=4时为17点。位置优先来自实测红点；有支撑的小空洞补点会标明来源。
./build/weld_seam_extractor input.ply output rounded \
  --config config/default.conf \
  --set path.mode=rounded_features

# 只看全范围 PLY 的候选结果：预期返回非零，只写 *_result.ply；
# 无 CSV/精确点 PLY，绝不可把诊断结果发给 ABB。
./build/weld_seam_extractor input.ply output inspect_only \
  --set path.mode=rounded_features \
  --set path.rounded_visualization_only=true
```

只测试法向策略：

```bash
# 自动复用或回退
./build/weld_seam_extractor input.ply output auto --set normal.mode=auto

# 强制重算，用于结果/耗时对照
./build/weld_seam_extractor input.ply output recompute --set normal.mode=recompute

# 强制复用，输入无有效法向时应明确失败
./build/weld_seam_extractor input.ply output reuse --set normal.mode=reuse
```

## 6. 输出文件

以输出前缀 `sample01` 为例：

| 文件 | 内容 |
|---|---|
| `sample01_features.csv` | 机器人路径点、点类型、XYZ、四元数、局部坐标和诊断字段 |
| `sample01_feature_points.ply` | 精确输出坐标点集合 |
| `sample01_result.ply` | 原点云着色、红色焊缝、粉红十字星和姿态方向辅助线 |

粉红色标记由多条线构成，但 CSV 输出坐标是十字中心，不是标记簇中任意一点。

`feature_points` 只输出四类焊接拐点：`PROTRUDING_LEFT`、`PROTRUDING_RIGHT`、`RECESSED_LEFT`、`RECESSED_RIGHT`。一般的不完整视野边缘不会制造拐点；周期模型只负责提出首末边界候选。候选附近须存在与预期类型一致的两条有效实测直线，且转折两侧均有足够分箱支持；证据不足就忽略候选。`rounded_features` 还会拒绝首末明显未覆盖的部分轨迹，要求重新选取拍照视野/ROI。

`adaptive_contour` 不要求四类拐点检测完整。其焊接行在 CSV 中标记为 `feature_type=adaptive_contour_point`、`point_source=robust_profile_adaptive_sampling`，姿态源为 `local_contour_tangent`。三种模式都在首尾增加安全过渡点；ROS 节点仍只读取统一的 `x,y,z,qw,qx,qy,qz`，因此手眼与 ABB 协议不变。

`rounded_features` 先运行与 `feature_points` 相同的四类周期角点检测，再到红色焊缝原始点集合中选择位置。检测链首末角固定为中心单点，内部有双侧支持的完整圆角才展开多个点；直线中点只生成在两个已确认角之间。小空洞仅在两侧直线和稳健轮廓支撑下拟合，不会把距离目标数毫米的错误实测点吸附进路径；拟合点在 CSV 与 PLY 明确标记。超过 `path.max_bridge_gap` 的未知段、内部四类跳类、点序回退或首末未覆盖均安全失败。相邻两角之间的同一物理直线只做一次鲁棒拟合，直线中点和相邻圆角边界共用该斜率以保持姿态连续。`raw_x/raw_y/raw_z` 和 `raw_workpiece_*` 保存偏置前坐标，原有 `x/y/z` 和 `workpiece_*` 保存工艺偏置后的最终目标。工具 X、工具 Z、四类 work/lead 角和正负方向仍保持现场成功版本的同一姿态函数。

旧 `feature_points` 每个物理拐角只输出一个离散点，左右腰切换时相邻 CSV 四元数出现 30–45° 变化属于该模式的几何定义。优先使用 `rounded_features` 在保持旧工具坐标系的前提下把圆角转姿分摊到 5 个点；`adaptive_contour` 仍保留 20 mm 弧长姿态平滑与 6° 切线步长限制，适合需要更密连续轮廓的离线对照。

## 7. 主要可调参数

完整、可直接运行的列表以 `config/default.conf` 为准。常用分组如下：

- `roi.*`：输入 PLY 坐标系下的长方体裁剪范围；`roi.enable=false` 表示全视野。
- `normal.*`：输入法向复用/重算策略、有效率阈值、K 邻域和线程数。
- `primary.*`：底面/侧面筛选、粗平面搜索、红色焊缝带宽、法向阈值和欧式聚类范围。
- `feature.*`：轮廓分箱、线段法向投票、孔洞桥接、线段长度、重复角点合并、带实测证据的边界补角、安全弦和安全过渡距离。
- `path.*`：输出模式、直线/曲率点距、轮廓与姿态平滑、姿态步长限制、空洞安全上限、`rounded_features` 首末单点/拓扑校验、统一工艺角和最多100点限制。
- `orientation.*`：四类拐点的工作角/前进角、工具正 Z 轴定义，以及工具 X 使用工件轴或旧角平分线。
- `orientation.start_transition/end_transition.*_offset_deg`：首末安全点相对相邻焊点的独立姿态增量；默认0保持平滑继承。
- `offset.start_transition.*`、`offset.end_transition.*`：两个安全过渡点的位置微调。
- `offset.protruding_left/right.*`、`offset.recessed_left/right.*`：四类真实拐点沿工件局部 XYZ 的位置微调。
- `offset.straight_protruding_flat/left_waist/recessed_flat/right_waist.*`：稀疏圆角模式四类直线中点的独立偏移。
- `offset.adaptive_contour.*`：连续轮廓全部焊接点的统一工件 XYZ 偏置。
- `visualization.*`：粉红十字半径、枪体方向线长度，只影响校验显示。

ROI 的数值是 PLY/相机坐标系，不是算法内部工件局部坐标系。例如要删除相机 Y 大于 300 mm 的点：

```text
roi.enable=true
roi.min_y=-inf
roi.max_y=300
```

## 8. 安装共享 SDK，供 ROS 2 使用

推荐安装在用户目录，避免污染系统：

```bash
cmake --install build --prefix "$HOME/scut_weld_sdk_install"
export CMAKE_PREFIX_PATH="$HOME/scut_weld_sdk_install:$CMAKE_PREFIX_PATH"
export LD_LIBRARY_PATH="$HOME/scut_weld_sdk_install/lib:$LD_LIBRARY_PATH"
```

其他 CMake 工程的用法：

```cmake
find_package(weld_seam_sdk 2.3 CONFIG REQUIRED)
target_link_libraries(your_target PRIVATE weld_seam::sdk)
```

C++ 调用入口：

```cpp
weld_seam_sdk::RunOptions options;
options.input_ply = "/data/part.ply";
options.output_directory = "/data/result";
options.output_prefix = "part01";
options.config_file = "/data/default.conf";
options.parameter_overrides = {
    "normal.mode=auto", "roi.enable=false", "path.mode=feature_points"};
const weld_seam_sdk::RunResult result = weld_seam_sdk::run(options);
```

## 9. 性能与稳定性建议

1. 必须使用 `Release`，Debug 下 PCL 法向量和 RANSAC 会显著变慢。
2. 优先使用准确 ROI 减少体素和 RANSAC 输入点数；第一次部署保持全视野验证，再逐步收紧。
3. 先用同一 PLY 对比 `auto` 与 `recompute` 的红色焊缝、四类拐点和 CSV。结果一致后保留 `auto`。
4. `normal.reuse_min_valid_ratio` 不建议为追求速度大幅降低；少量零向量可能通过体素传播，进而影响平面法向聚类。
5. 外部程序不要同时并发调用同一个输出前缀，否则输出文件会互相覆盖。ROS 节点已经用互斥锁拒绝并发提取。

## 10. 常见问题

- `PCL not found`：安装 `libpcl-dev`，删除 `build/CMakeCache.txt` 后重新配置。
- 运行时找不到 `libweld_seam_sdk.so.2`：设置 `LD_LIBRARY_PATH`，或安装到 `/usr/local` 后执行 `sudo ldconfig`。
- `normal.mode=reuse requires...`：输入 PLY 无三分量法向或有效率不足；改回 `auto` 或 `recompute`。
- ROI 后为 0 点：ROI 是相机坐标系，检查正负方向和单位。
- 新相机姿态导致左右类别整体反转：先确认输入法向和 `normal.mode` 日志，再检查 `feature.protruding_is_larger_local_y`；不要先随意交换四组位置/姿态参数。
- 机器人在 XYZ 正确时仍出现大幅绕焊枪轴旋转：优先使用 `orientation.tool_x_reference=workpiece_x`，并按真实 TCP 的 X 轴方向选择 `orientation.tool_x_points_along_positive_workpiece_x=true/false`。Tool X 会投影到 Tool Z 的法平面，以保证输出始终是正交右手坐标系。
