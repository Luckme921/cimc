#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <algorithm>
#include <fstream>
#include <iomanip>
#include <string>
#include <map>
#include <set>
#include <sstream>
#include <filesystem>
#include <cctype>
#include <utility>
#include <chrono>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/register_point_struct.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/search/kdtree.h>
#include <pcl/common/centroid.h>
#include <pcl/features/normal_3d_omp.h>
#include <Eigen/Dense>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <weld_seam_sdk/weld_seam_sdk.hpp>

// 只接收 PLY 中的 XYZ + normal_x/y/z，不要求 curvature。
// 这样对“相机输出了法向但没输出曲率”的 PLY 不会产生误导性警告。
struct EIGEN_ALIGN16 InputPointWithNormal {
    PCL_ADD_POINT4D;
    PCL_ADD_NORMAL4D;
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

POINT_CLOUD_REGISTER_POINT_STRUCT(InputPointWithNormal,
    (float, x, x)
    (float, y, y)
    (float, z, z)
    (float, normal_x, normal_x)
    (float, normal_y, normal_y)
    (float, normal_z, normal_z))

typedef pcl::PointNormal PointInT;
typedef pcl::PointXYZRGB PointOutT;

struct PlaneInfo {
    Eigen::Vector3f normal;
    float d;
    int point_count;
};

static std::vector<PlaneInfo> extractDominantPlanes(
    const pcl::PointCloud<PointInT>::ConstPtr& cloud,
    int maximum_plane_count,
    int minimum_inliers,
    float distance_threshold)
{
    std::vector<PlaneInfo> planes;
    if (!cloud || cloud->empty()) return planes;

    pcl::PointCloud<PointInT>::Ptr remaining(new pcl::PointCloud<PointInT>(*cloud));
    pcl::SACSegmentation<PointInT> segmentation;
    segmentation.setOptimizeCoefficients(true);
    segmentation.setModelType(pcl::SACMODEL_PLANE);
    segmentation.setMethodType(pcl::SAC_RANSAC);
    segmentation.setMaxIterations(1000);
    segmentation.setDistanceThreshold(distance_threshold);

    for (int i = 0; i < maximum_plane_count; ++i) {
        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients coefficients;
        segmentation.setInputCloud(remaining);
        segmentation.segment(*inliers, coefficients);
        if (static_cast<int>(inliers->indices.size()) < minimum_inliers ||
            coefficients.values.size() < 4) {
            break;
        }

        Eigen::Vector3f normal(
            coefficients.values[0], coefficients.values[1], coefficients.values[2]);
        const float normal_length = normal.norm();
        if (!std::isfinite(normal_length) || normal_length < 1e-6f) break;
        PlaneInfo plane;
        plane.normal = normal / normal_length;
        plane.d = coefficients.values[3] / normal_length;
        plane.point_count = static_cast<int>(inliers->indices.size());
        planes.push_back(plane);

        pcl::ExtractIndices<PointInT> extract;
        extract.setInputCloud(remaining);
        extract.setIndices(inliers);
        extract.setNegative(true);
        pcl::PointCloud<PointInT>::Ptr next(new pcl::PointCloud<PointInT>);
        extract.filter(*next);
        remaining = next;
        if (static_cast<int>(remaining->size()) < minimum_inliers) break;
    }
    return planes;
}

static bool refinePlaneOnFullResolutionCloud(
    const pcl::PointCloud<PointInT>::ConstPtr& cloud,
    Eigen::Vector3f& normal,
    float& d,
    float support_distance,
    int minimum_support_points)
{
    if (!cloud || cloud->empty() || !normal.allFinite()) return false;

    Eigen::Vector3d sum = Eigen::Vector3d::Zero();
    int support_count = 0;
    for (size_t i = 0; i < cloud->size(); ++i) {
        const PointInT& point = cloud->points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        const Eigen::Vector3f position(point.x, point.y, point.z);
        if (std::abs(normal.dot(position) + d) <= support_distance) {
            sum += position.cast<double>();
            ++support_count;
        }
    }
    if (support_count < minimum_support_points) return false;

    const Eigen::Vector3d centroid = sum / static_cast<double>(support_count);
    Eigen::Matrix3d covariance = Eigen::Matrix3d::Zero();
    for (size_t i = 0; i < cloud->size(); ++i) {
        const PointInT& point = cloud->points[i];
        if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
            !std::isfinite(point.z)) continue;
        const Eigen::Vector3f position_f(point.x, point.y, point.z);
        if (std::abs(normal.dot(position_f) + d) > support_distance) continue;
        const Eigen::Vector3d delta = position_f.cast<double>() - centroid;
        covariance += delta * delta.transpose();
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> solver(covariance);
    if (solver.info() != Eigen::Success) return false;
    Eigen::Vector3f refined_normal = solver.eigenvectors().col(0).cast<float>().normalized();
    if (!refined_normal.allFinite()) return false;
    if (refined_normal.dot(normal) < 0.0f) refined_normal = -refined_normal;
    normal = refined_normal;
    d = -normal.dot(centroid.cast<float>());
    return std::isfinite(d);
}

// ============================================================================
// 二次提取：从已经确认的红色焊缝点中提取有序折线角点。
// 这里不参与、也不改变上游焊缝提取，只消费 is_final_seam 的结果。
// ============================================================================
enum SeamSegmentType {
    SEGMENT_UNKNOWN = -1,
    SEGMENT_FLAT = 0,
    SEGMENT_DIAGONAL_POSITIVE = 1,
    SEGMENT_DIAGONAL_NEGATIVE = 2
};

struct FeatureExtractionParams {
    // 沿波纹延伸方向压缩红点时的分箱宽度，通常略大于体素尺寸。
    float profile_bin_width = 0.8f;

    // 与现有可视化法向分类一致：平底段、左右腰、圆角过渡段。
    float flat_normal_dot_min = 0.80f;
    float diagonal_normal_dot_min = 0.12f;
    float bin_label_vote_ratio = 0.45f;

    // 太短的标签游程视为法向噪声；真实直线段至少需要这些分箱。
    int min_segment_bins = 3;
    float min_segment_span = 1.2f;

    // 用二维轮廓自身的局部斜率修正法向标签，避免法向翻转造成漏段。
    int local_fit_half_window_bins = 8;
    float local_fit_radius = 10.0f;
    float local_line_max_median_residual = 0.65f;
    float flat_profile_slope_max = 0.22f; // 约 12.4 度

    // 工件先验：上下底约 110mm 且不超过 130mm，腰约 45mm。
    // 只作为防止跨周期误连接的宽松上限，不强制完整段必须达到 110/45mm。
    float flat_segment_max_length = 135.0f;
    float waist_segment_max_length = 70.0f;

    // 允许跨越拍照空洞连接相邻直线。70mm 可覆盖一条约 45mm 的腰部缺测，
    // 但配合角度、交点位置及段长约束，不会直接跨过 110mm 平底造假点。
    float max_hole_bridge = 70.0f;
    float max_corner_extrapolation = 18.0f;
    float min_corner_angle_deg = 10.0f;

    // 真实相邻角沿约 45mm 的腰分布；18mm 内的同类型角视为同一圆角被重复检测。
    float duplicate_corner_merge_distance = 18.0f;

    // 仅在内部计算中取角点两侧约5mm位置构造安全弦；最终每个角只输出一个点。
    float corner_fit_support_distance = 5.0f;
    float recessed_chord_search_radius = 7.0f;

    // 首/末安全过渡点与相邻真实焊接点保持相同的工件 X；只在工件 YZ 平面避让。
    // +Y 指向 L 侧板/开放侧，+Z 指向离开蓝色底板的上方，默认各避让 20mm。
    float safe_transition_offset_y = 20.0f;
    float safe_transition_offset_z = 20.0f;

    // 凹角在此邻域内寻找离理论交点最近的真实圆弧红点。
    float recessed_arc_search_radius = 10.0f;

    // 在局部邻域内使用 z_local 的低分位靠近蓝色底面；不直接取最小值，
    // 避免单个离群点把机械臂坐标拉到底面以下。
    float bottom_z_quantile = 0.10f;
    float low_z_xy_tolerance = 1.5f;

    // y_local=0 位于 L 侧板，y_local 越大越远离 L 侧板。按本工件的物理定义，
    // 朝 L 侧板靠前的一层是凸出层，所以较小 y_local 才是凸角。
    bool protruding_is_larger_local_y = false;

    // 粉红色三维十字星的单侧臂长；十字交点就是最终输出坐标。
    float visualization_marker_radius = 8.0f;
};

struct AdaptiveContourParams {
    // feature_points 保留 2.2.2 的四类拐点路径；adaptive_contour 沿已经
    // 提取出的红色焊缝轮廓自适应离散，供曲率或制造误差较大的工件使用。
    std::string mode = "feature_points";

    // 直线段稀疏、转角/曲率段密集。实际点距还会在 max_points 约束下
    // 等比例放大，绝不会通过截断轨迹来满足 ABB 点数上限。
    float straight_spacing = 12.0f;
    float corner_spacing = 4.0f;
    float corner_influence_radius = 15.0f;
    float curvature_window = 10.0f;
    float curvature_threshold_deg = 8.0f;

    // 路径位置仍跟随真实轮廓；姿态切线按弧长做对称平滑，并限制相邻采样点
    // 的切线转角，避免局部毛刺或近竖直腰段把焊枪四元数变成单点突变。
    float orientation_smoothing_radius = 20.0f;
    float max_orientation_step_deg = 6.0f;

    // profile bin 本身已使用中值压缩；这里再用小窗口中值识别点焊飞溅等
    // 局部离群点。连续空洞不超过 max_bridge_gap 时线性跨越，超过则拒绝输出。
    int smoothing_half_window_bins = 8;
    float outlier_max_distance = 3.0f;
    float max_bridge_gap = 15.0f;

    // 包含首末两个安全过渡点。ABB 当前单条轨迹最多保存 100 点。
    int max_points = 100;

    // 连续轮廓模式使用统一工艺角，局部开放方向由轮廓切线实时计算。
    float work_angle_deg = 45.0f;
    float lead_angle_deg = 0.0f;
};

enum TorchPoseGroup {
    TORCH_PROTRUDING_LEFT = 0,
    TORCH_PROTRUDING_RIGHT = 1,
    TORCH_RECESSED_LEFT = 2,
    TORCH_RECESSED_RIGHT = 3
};

struct TorchOrientationParams {
    // work_angle：焊枪“TCP 指向枪体”的中心轴与蓝色底板平面的夹角。
    // 45°为底板与波纹板开放侧法向的默认角平分姿态；角度越大，枪体越竖直。
    float protruding_left_work_angle_deg = 45.0f;
    float protruding_right_work_angle_deg = 45.0f;
    float recessed_left_work_angle_deg = 45.0f;
    float recessed_right_work_angle_deg = 45.0f;

    // lead_angle：焊枪在底板平面内沿焊接前进方向的前倾/后倾角。
    // 正值朝 CSV 顺序前倾，负值反向，0°表示不前倾。建议先保持0°。
    float protruding_left_lead_angle_deg = 0.0f;
    float protruding_right_lead_angle_deg = 0.0f;
    float recessed_left_lead_angle_deg = 0.0f;
    float recessed_right_lead_angle_deg = 0.0f;

    // CSV 同时输出 PLY 世界姿态 qw... 和工件姿态 workpiece_qw...，顺序均为 w,x,y,z。
    // true：工具 +Z 轴定义为 TCP -> 枪体；false：工具 +Z 轴定义为枪体 -> TCP。
    // 必须按照实际机器人 TCP 工具坐标系修改，否则姿态可能相反。
    bool tool_positive_z_points_from_tcp_to_body = true;

    // workpiece_x：工具 +X 以工件 X（CSV 前进方向）为参考，先投影到垂直于
    // 工具 +Z 的平面，避免局部角点改变引起不必要的绕枪轴滚转。
    // corner_bisector：保留 2.2.0 及更早版本的局部角平分线行为，便于回退。
    std::string tool_x_reference = "workpiece_x";
    // 仅在 tool_x_reference=workpiece_x 时生效；false 使用工件 -X。
    bool tool_x_points_along_positive_workpiece_x = true;

    // 在 weld_seam_result.ply 中用白色短线显示 TCP -> 枪体方向；0 表示不显示。
    float visualization_body_axis_length_mm = 25.0f;
};

struct WeldPoseData {
    // PLY/world 坐标系表达的工具姿态。
    float qw;
    float qx;
    float qy;
    float qz;

    // 右手工件坐标系表达的同一姿态：
    // X=N_tangent（CSV前进），Y=-N_side（朝L侧板/开放侧），Z=N_bottom（向上）。
    float workpiece_qw;
    float workpiece_qx;
    float workpiece_qy;
    float workpiece_qz;

    float torch_body_axis_x;
    float torch_body_axis_y;
    float torch_body_axis_z;
    float tool_x_axis_x;
    float tool_x_axis_y;
    float tool_x_axis_z;
    float work_angle_deg;
    float lead_angle_deg;
    TorchPoseGroup group;
    bool inherited_from_nearest_corner;
    bool default_without_corner;
};

struct WorkpieceOffset3f {
    // 三个分量始终沿工件坐标轴，而不是沿相机/PLY固定轴。
    float x = 0.0f; // 沿 N_tangent：CSV 焊缝排序方向
    float y = 0.0f; // 沿 -N_side：从波纹板指向 L 侧板/开放侧
    float z = 0.0f; // 沿 N_bottom：离开蓝色底板向上
};

struct FeaturePositionOffsetParams {
    WorkpieceOffset3f start_transition;
    WorkpieceOffset3f end_transition;
    WorkpieceOffset3f protruding_left;
    WorkpieceOffset3f protruding_right;
    WorkpieceOffset3f recessed_left;
    WorkpieceOffset3f recessed_right;
    // adaptive_contour 模式的全部焊接采样点使用同一组连续偏置，避免在
    // 四类拐点边界发生毫米级阶跃；安全过渡点在此基础上再加首/末微调。
    WorkpieceOffset3f adaptive_contour;
};

struct SeamProfileSample {
    Eigen::Vector3f world;
    float local_x;
    float local_y;
    float local_z;
    SeamSegmentType type;
};

struct SeamProfileBin {
    float local_x;
    float local_y;
    float local_z;
    SeamSegmentType type;
};

struct RobustLine2D {
    // local_y = slope * local_x + intercept
    float slope;
    float intercept;
    bool valid;
};

struct SeamLineRun {
    SeamSegmentType type;
    std::vector<int> bin_indices;
    float x_first;
    float x_last;
    RobustLine2D line;
    bool valid;
};

// 沿 local_x 正方向，一个完整波纹周期中的四类真实转角严格按此顺序出现：
// 右腰->低层平段、低层平段->左腰、左腰->高层平段、高层平段->右腰。
// 数值顺序本身用于后续周期拓扑筛选，请勿随意调整。
enum CornerTopologyState {
    CORNER_RIGHT_WAIST_TO_FLAT = 0,
    CORNER_FLAT_TO_LEFT_WAIST = 1,
    CORNER_LEFT_WAIST_TO_FLAT = 2,
    CORNER_FLAT_TO_RIGHT_WAIST = 3,
    CORNER_TOPOLOGY_UNKNOWN = -1
};

struct WeldFeaturePoint {
    Eigen::Vector3f world;
    Eigen::Vector3f local;
    bool is_transition_point;
    bool is_start_transition;
    bool protruding;
    bool measured_on_arc;
    float distance_to_ideal;
    int merged_detection_count;
    float left_line_slope;
    float right_line_slope;
    Eigen::Vector3f ideal_local;
    SeamSegmentType left_segment;
    SeamSegmentType right_segment;
    CornerTopologyState topology_state;
    float detection_score;
    bool topology_inferred;
    bool is_contour_sample = false;
};

static float medianOf(std::vector<float> values)
{
    if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float result = values[middle];
    if (values.size() % 2 == 0) {
        std::nth_element(values.begin(), values.begin() + middle - 1, values.end());
        result = 0.5f * (result + values[middle - 1]);
    }
    return result;
}

static float quantileOf(std::vector<float> values, float q)
{
    if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
    std::sort(values.begin(), values.end());
    q = std::max(0.0f, std::min(1.0f, q));
    const float position = q * static_cast<float>(values.size() - 1);
    const size_t lower = static_cast<size_t>(std::floor(position));
    const size_t upper = static_cast<size_t>(std::ceil(position));
    const float alpha = position - static_cast<float>(lower);
    return values[lower] * (1.0f - alpha) + values[upper] * alpha;
}

static float fastQuantileOf(std::vector<float> values, float q)
{
    if (values.empty()) return std::numeric_limits<float>::quiet_NaN();
    q = std::max(0.0f, std::min(1.0f, q));
    const size_t index = static_cast<size_t>(
        q * static_cast<float>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + index, values.end());
    return values[index];
}

static SeamSegmentType classifySeamNormal(
    const PointInT& point,
    const Eigen::Vector3f& n_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    Eigen::Vector3f normal(point.normal_x, point.normal_y, point.normal_z);
    if (!normal.allFinite() || normal.squaredNorm() < 1e-8f) return SEGMENT_UNKNOWN;
    normal.normalize();

    if (std::abs(n_side.dot(normal)) >= params.flat_normal_dot_min) {
        return SEGMENT_FLAT;
    }

    const float tangent_dot = n_tangent.dot(normal);
    if (tangent_dot >= params.diagonal_normal_dot_min) {
        return SEGMENT_DIAGONAL_POSITIVE;
    }
    if (tangent_dot <= -params.diagonal_normal_dot_min) {
        return SEGMENT_DIAGONAL_NEGATIVE;
    }
    return SEGMENT_UNKNOWN; // 圆角及法向不稳定区，不拿来拟合直线。
}

static std::vector<SeamProfileSample> collectSeamProfileSamples(
    const pcl::PointCloud<PointInT>::ConstPtr& cloud,
    const std::vector<bool>& is_final_seam,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    std::vector<SeamProfileSample> samples;
    samples.reserve(cloud->size());

    for (size_t i = 0; i < cloud->size(); ++i) {
        if (i >= is_final_seam.size() || !is_final_seam[i]) continue;
        const PointInT& point = cloud->points[i];
        Eigen::Vector3f world(point.x, point.y, point.z);
        if (!world.allFinite()) continue;

        SeamProfileSample sample;
        sample.world = world;
        sample.local_x = n_tangent.dot(world);
        sample.local_y = n_side.dot(world) + d_side;
        sample.local_z = n_bottom.dot(world) + d_bottom;
        sample.type = classifySeamNormal(point, n_side, n_tangent, params);
        samples.push_back(sample);
    }
    return samples;
}

static std::vector<SeamProfileBin> buildProfileBins(
    const std::vector<SeamProfileSample>& samples,
    const FeatureExtractionParams& params)
{
    std::vector<SeamProfileBin> result;
    if (samples.empty() || params.profile_bin_width <= 0.0f) return result;

    float min_x = samples.front().local_x;
    float max_x = samples.front().local_x;
    for (size_t i = 1; i < samples.size(); ++i) {
        min_x = std::min(min_x, samples[i].local_x);
        max_x = std::max(max_x, samples[i].local_x);
    }

    const int bin_count = std::max(
        1, static_cast<int>(std::floor((max_x - min_x) / params.profile_bin_width)) + 1);
    std::vector<std::vector<int> > members(static_cast<size_t>(bin_count));

    for (size_t i = 0; i < samples.size(); ++i) {
        int bin_index = static_cast<int>(
            std::floor((samples[i].local_x - min_x) / params.profile_bin_width));
        bin_index = std::max(0, std::min(bin_count - 1, bin_index));
        members[static_cast<size_t>(bin_index)].push_back(static_cast<int>(i));
    }

    for (int bin_index = 0; bin_index < bin_count; ++bin_index) {
        const std::vector<int>& indices = members[static_cast<size_t>(bin_index)];
        if (indices.empty()) continue; // 保留孔洞，不做强行插值。

        std::vector<float> xs;
        std::vector<float> ys;
        std::vector<float> zs;
        int votes[3] = {0, 0, 0};
        xs.reserve(indices.size());
        ys.reserve(indices.size());
        zs.reserve(indices.size());

        for (size_t j = 0; j < indices.size(); ++j) {
            const SeamProfileSample& sample = samples[static_cast<size_t>(indices[j])];
            xs.push_back(sample.local_x);
            ys.push_back(sample.local_y);
            zs.push_back(sample.local_z);
            if (sample.type >= SEGMENT_FLAT && sample.type <= SEGMENT_DIAGONAL_NEGATIVE) {
                votes[static_cast<int>(sample.type)]++;
            }
        }

        int best_label = 0;
        if (votes[1] > votes[best_label]) best_label = 1;
        if (votes[2] > votes[best_label]) best_label = 2;
        const float vote_ratio = static_cast<float>(votes[best_label]) /
            static_cast<float>(indices.size());

        SeamProfileBin bin;
        bin.local_x = medianOf(xs);
        bin.local_y = medianOf(ys);
        bin.local_z = medianOf(zs);
        bin.type = vote_ratio >= params.bin_label_vote_ratio
            ? static_cast<SeamSegmentType>(best_label)
            : SEGMENT_UNKNOWN;
        result.push_back(bin);
    }
    return result;
}

static void removeShortLabelNoise(
    std::vector<SeamProfileBin>& bins,
    const FeatureExtractionParams& params)
{
    // 仅在“短错误段夹在同类长段中间”时改标签，避免抹掉真实的短平底。
    for (int iteration = 0; iteration < 3; ++iteration) {
        std::vector<int> known;
        for (size_t i = 0; i < bins.size(); ++i) {
            if (bins[i].type != SEGMENT_UNKNOWN) known.push_back(static_cast<int>(i));
        }
        if (known.size() < 3) return;

        struct LabelRun {
            int first_known;
            int last_known;
            SeamSegmentType type;
        };
        std::vector<LabelRun> runs;
        int run_start = 0;
        for (size_t i = 1; i <= known.size(); ++i) {
            if (i == known.size() ||
                bins[static_cast<size_t>(known[i])].type !=
                    bins[static_cast<size_t>(known[static_cast<size_t>(run_start)])].type) {
                LabelRun run;
                run.first_known = run_start;
                run.last_known = static_cast<int>(i) - 1;
                run.type = bins[static_cast<size_t>(known[static_cast<size_t>(run_start)])].type;
                runs.push_back(run);
                run_start = static_cast<int>(i);
            }
        }

        bool changed = false;
        for (size_t i = 1; i + 1 < runs.size(); ++i) {
            const LabelRun& run = runs[i];
            const int count = run.last_known - run.first_known + 1;
            const float span =
                bins[static_cast<size_t>(known[static_cast<size_t>(run.last_known)])].local_x -
                bins[static_cast<size_t>(known[static_cast<size_t>(run.first_known)])].local_x;
            if (runs[i - 1].type == runs[i + 1].type &&
                (count < params.min_segment_bins || span < params.min_segment_span)) {
                for (int k = run.first_known; k <= run.last_known; ++k) {
                    bins[static_cast<size_t>(known[static_cast<size_t>(k)])].type = runs[i - 1].type;
                }
                changed = true;
            }
        }
        if (!changed) return;
    }
}

static RobustLine2D robustFitLine(
    const std::vector<SeamProfileBin>& bins,
    const std::vector<int>& indices)
{
    RobustLine2D line;
    line.slope = 0.0f;
    line.intercept = 0.0f;
    line.valid = false;
    if (indices.size() < 2) return line;

    std::vector<double> weights(indices.size(), 1.0);
    double slope = 0.0;
    double intercept = 0.0;

    for (int iteration = 0; iteration < 6; ++iteration) {
        double sum_w = 0.0;
        double mean_x = 0.0;
        double mean_y = 0.0;
        for (size_t i = 0; i < indices.size(); ++i) {
            const SeamProfileBin& bin = bins[static_cast<size_t>(indices[i])];
            sum_w += weights[i];
            mean_x += weights[i] * static_cast<double>(bin.local_x);
            mean_y += weights[i] * static_cast<double>(bin.local_y);
        }
        if (sum_w <= 0.0) return line;
        mean_x /= sum_w;
        mean_y /= sum_w;

        double covariance = 0.0;
        double variance_x = 0.0;
        for (size_t i = 0; i < indices.size(); ++i) {
            const SeamProfileBin& bin = bins[static_cast<size_t>(indices[i])];
            const double dx = static_cast<double>(bin.local_x) - mean_x;
            covariance += weights[i] * dx * (static_cast<double>(bin.local_y) - mean_y);
            variance_x += weights[i] * dx * dx;
        }
        if (variance_x < 1e-8) return line;
        slope = covariance / variance_x;
        intercept = mean_y - slope * mean_x;

        std::vector<float> absolute_residuals;
        absolute_residuals.reserve(indices.size());
        for (size_t i = 0; i < indices.size(); ++i) {
            const SeamProfileBin& bin = bins[static_cast<size_t>(indices[i])];
            absolute_residuals.push_back(static_cast<float>(std::abs(
                static_cast<double>(bin.local_y) -
                (slope * static_cast<double>(bin.local_x) + intercept))));
        }
        const float mad = medianOf(absolute_residuals);
        const double huber_limit = std::max(0.15, 1.5 * 1.4826 * static_cast<double>(mad));
        for (size_t i = 0; i < absolute_residuals.size(); ++i) {
            const double residual = static_cast<double>(absolute_residuals[i]);
            weights[i] = residual <= huber_limit ? 1.0 : huber_limit / residual;
        }
    }

    line.slope = static_cast<float>(slope);
    line.intercept = static_cast<float>(intercept);
    line.valid = std::isfinite(line.slope) && std::isfinite(line.intercept);
    return line;
}

static SeamSegmentType classifyProfileSlope(
    float slope,
    const FeatureExtractionParams& params)
{
    if (std::abs(slope) <= params.flat_profile_slope_max) return SEGMENT_FLAT;
    return slope > 0.0f ? SEGMENT_DIAGONAL_POSITIVE : SEGMENT_DIAGONAL_NEGATIVE;
}

static void refineBinLabelsFromLocalGeometry(
    std::vector<SeamProfileBin>& bins,
    const FeatureExtractionParams& params)
{
    if (bins.size() < static_cast<size_t>(params.min_segment_bins)) return;
    std::vector<SeamSegmentType> refined_labels(bins.size(), SEGMENT_UNKNOWN);

    for (size_t i = 0; i < bins.size(); ++i) {
        const int first = std::max(0,
            static_cast<int>(i) - params.local_fit_half_window_bins);
        const int last = std::min(static_cast<int>(bins.size()) - 1,
            static_cast<int>(i) + params.local_fit_half_window_bins);
        std::vector<int> local_indices;
        for (int j = first; j <= last; ++j) {
            if (std::abs(bins[static_cast<size_t>(j)].local_x - bins[i].local_x) <=
                params.local_fit_radius) {
                local_indices.push_back(j);
            }
        }
        if (local_indices.size() < static_cast<size_t>(params.min_segment_bins + 1)) continue;

        const RobustLine2D local_line = robustFitLine(bins, local_indices);
        if (!local_line.valid) continue;

        std::vector<float> residuals;
        residuals.reserve(local_indices.size());
        for (size_t j = 0; j < local_indices.size(); ++j) {
            const SeamProfileBin& bin = bins[static_cast<size_t>(local_indices[j])];
            residuals.push_back(std::abs(
                bin.local_y - (local_line.slope * bin.local_x + local_line.intercept)));
        }
        if (medianOf(residuals) <= params.local_line_max_median_residual) {
            refined_labels[i] = classifyProfileSlope(local_line.slope, params);
        }
    }

    // 局部几何拟合成功时优先用斜率标签；拐角或大孔洞处拟合失败时，
    // 保留原法向标签，避免把真实直线端部全部丢掉。
    for (size_t i = 0; i < bins.size(); ++i) {
        if (refined_labels[i] != SEGMENT_UNKNOWN) bins[i].type = refined_labels[i];
    }
}

static std::vector<SeamLineRun> buildLineRuns(
    const std::vector<SeamProfileBin>& bins,
    const FeatureExtractionParams& params)
{
    std::vector<SeamLineRun> runs;
    std::vector<int> known;
    for (size_t i = 0; i < bins.size(); ++i) {
        if (bins[i].type != SEGMENT_UNKNOWN) known.push_back(static_cast<int>(i));
    }
    if (known.empty()) return runs;

    int run_start = 0;
    for (size_t i = 1; i <= known.size(); ++i) {
        if (i == known.size() ||
            bins[static_cast<size_t>(known[i])].type !=
                bins[static_cast<size_t>(known[static_cast<size_t>(run_start)])].type) {
            SeamLineRun run;
            run.type = bins[static_cast<size_t>(known[static_cast<size_t>(run_start)])].type;
            for (int k = run_start; k < static_cast<int>(i); ++k) {
                run.bin_indices.push_back(known[static_cast<size_t>(k)]);
            }
            run.x_first = bins[static_cast<size_t>(run.bin_indices.front())].local_x;
            run.x_last = bins[static_cast<size_t>(run.bin_indices.back())].local_x;
            const float span = run.x_last - run.x_first;
            run.valid = static_cast<int>(run.bin_indices.size()) >= params.min_segment_bins &&
                span >= params.min_segment_span;
            run.line = run.valid ? robustFitLine(bins, run.bin_indices) : RobustLine2D{0, 0, false};
            run.valid = run.valid && run.line.valid;
            if (run.valid) {
                // 最终段类型由实际二维斜率决定，法向只用于前期分段提示。
                run.type = classifyProfileSlope(run.line.slope, params);
                const float delta_y = run.line.slope * (run.x_last - run.x_first);
                const float observed_length = std::sqrt(
                    (run.x_last - run.x_first) * (run.x_last - run.x_first) +
                    delta_y * delta_y);
                const float maximum_length = run.type == SEGMENT_FLAT
                    ? params.flat_segment_max_length
                    : params.waist_segment_max_length;
                if (observed_length > maximum_length) run.valid = false;
            }
            runs.push_back(run);
            run_start = static_cast<int>(i);
        }
    }
    return runs;
}

static float lineRunObservedLength(const SeamLineRun& run)
{
    const float dx = run.x_last - run.x_first;
    const float dy = run.line.slope * dx;
    return std::sqrt(dx * dx + dy * dy);
}

static CornerTopologyState classifyCornerTopology(
    SeamSegmentType left,
    SeamSegmentType right)
{
    if (left == SEGMENT_DIAGONAL_NEGATIVE && right == SEGMENT_FLAT) {
        return CORNER_RIGHT_WAIST_TO_FLAT;
    }
    if (left == SEGMENT_FLAT && right == SEGMENT_DIAGONAL_POSITIVE) {
        return CORNER_FLAT_TO_LEFT_WAIST;
    }
    if (left == SEGMENT_DIAGONAL_POSITIVE && right == SEGMENT_FLAT) {
        return CORNER_LEFT_WAIST_TO_FLAT;
    }
    if (left == SEGMENT_FLAT && right == SEGMENT_DIAGONAL_NEGATIVE) {
        return CORNER_FLAT_TO_RIGHT_WAIST;
    }
    return CORNER_TOPOLOGY_UNKNOWN;
}

static bool topologyStateIsProtruding(CornerTopologyState state)
{
    return state == CORNER_RIGHT_WAIST_TO_FLAT ||
        state == CORNER_FLAT_TO_LEFT_WAIST;
}

static float maximumSegmentLengthAfterCorner(
    CornerTopologyState state,
    const FeatureExtractionParams& params)
{
    // 右腰->平段和左腰->平段之后都是一条上下底；另两类之后都是腰段。
    return state == CORNER_RIGHT_WAIST_TO_FLAT ||
        state == CORNER_LEFT_WAIST_TO_FLAT
        ? params.flat_segment_max_length
        : params.waist_segment_max_length;
}

static float estimateDepthLevelSplit(
    const std::vector<SeamLineRun>& runs,
    const std::vector<SeamProfileBin>& bins)
{
    std::vector<float> flat_levels;
    for (size_t i = 0; i < runs.size(); ++i) {
        if (!runs[i].valid || runs[i].type != SEGMENT_FLAT) continue;
        const float center_x = 0.5f * (runs[i].x_first + runs[i].x_last);
        flat_levels.push_back(runs[i].line.slope * center_x + runs[i].line.intercept);
    }

    if (flat_levels.size() >= 2) {
        float low = *std::min_element(flat_levels.begin(), flat_levels.end());
        float high = *std::max_element(flat_levels.begin(), flat_levels.end());
        for (int iteration = 0; iteration < 10; ++iteration) {
            double sum_low = 0.0, sum_high = 0.0;
            int count_low = 0, count_high = 0;
            for (size_t i = 0; i < flat_levels.size(); ++i) {
                if (std::abs(flat_levels[i] - low) <= std::abs(flat_levels[i] - high)) {
                    sum_low += flat_levels[i];
                    ++count_low;
                } else {
                    sum_high += flat_levels[i];
                    ++count_high;
                }
            }
            if (count_low > 0) low = static_cast<float>(sum_low / count_low);
            if (count_high > 0) high = static_cast<float>(sum_high / count_high);
        }
        if (std::abs(high - low) > 0.5f) return 0.5f * (low + high);
    }

    std::vector<float> all_y;
    all_y.reserve(bins.size());
    for (size_t i = 0; i < bins.size(); ++i) all_y.push_back(bins[i].local_y);
    return 0.5f * (quantileOf(all_y, 0.20f) + quantileOf(all_y, 0.80f));
}

static float estimateCornerZ(
    const std::vector<SeamProfileSample>& samples,
    float ideal_x,
    float ideal_y,
    float search_radius,
    float bottom_quantile)
{
    std::vector<float> nearby_z;
    const float radius_squared = search_radius * search_radius;
    for (size_t i = 0; i < samples.size(); ++i) {
        const float dx = samples[i].local_x - ideal_x;
        const float dy = samples[i].local_y - ideal_y;
        if (dx * dx + dy * dy <= radius_squared) nearby_z.push_back(samples[i].local_z);
    }
    if (!nearby_z.empty()) return quantileOf(nearby_z, bottom_quantile);

    // 大孔洞时至少利用相邻直线段端部的高度，不凭空固定为某个 z。
    std::vector<std::pair<float, float> > distance_and_z;
    distance_and_z.reserve(samples.size());
    for (size_t i = 0; i < samples.size(); ++i) {
        distance_and_z.push_back(std::make_pair(
            std::abs(samples[i].local_x - ideal_x), samples[i].local_z));
    }
    std::sort(distance_and_z.begin(), distance_and_z.end());
    const size_t count = std::min<size_t>(20, distance_and_z.size());
    for (size_t i = 0; i < count; ++i) nearby_z.push_back(distance_and_z[i].second);
    return nearby_z.empty() ? 0.0f : quantileOf(nearby_z, bottom_quantile);
}

static int selectMeasuredPointNearBottom(
    const std::vector<SeamProfileSample>& samples,
    float target_x,
    float target_y,
    float search_radius,
    const FeatureExtractionParams& params)
{
    if (samples.empty()) return -1;

    float nearest_distance_squared = std::numeric_limits<float>::max();
    for (size_t i = 0; i < samples.size(); ++i) {
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        nearest_distance_squared = std::min(nearest_distance_squared, dx * dx + dy * dy);
    }

    // 只在最接近目标轮廓位置的一条窄带内向底面寻找，防止为了降低 z
    // 跑到相邻的另一条直线或另一个圆角上。
    const float maximum_distance = std::min(
        search_radius, std::sqrt(nearest_distance_squared) + params.low_z_xy_tolerance);
    const float maximum_distance_squared = maximum_distance * maximum_distance;
    std::vector<float> candidate_z;
    candidate_z.reserve(64);
    for (size_t i = 0; i < samples.size(); ++i) {
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        if (dx * dx + dy * dy <= maximum_distance_squared) {
            candidate_z.push_back(samples[i].local_z);
        }
    }
    if (candidate_z.empty()) return -1;

    const float robust_low_z = quantileOf(candidate_z, params.bottom_z_quantile);
    int best_sample = -1;
    float best_score = std::numeric_limits<float>::max();
    for (size_t i = 0; i < samples.size(); ++i) {
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        const float distance_squared = dx * dx + dy * dy;
        if (distance_squared > maximum_distance_squared) continue;
        // z 偏差占主导，二维距离只用于同高度候选的稳定决胜。
        const float score = std::abs(samples[i].local_z - robust_low_z) +
            0.02f * std::sqrt(distance_squared);
        if (score < best_score) {
            best_score = score;
            best_sample = static_cast<int>(i);
        }
    }
    return best_sample;
}

static int selectSafeRecessedPointNearBottom(
    const std::vector<SeamProfileSample>& samples,
    float target_x,
    float target_y,
    float search_radius,
    float lower_x,
    float upper_x,
    float ideal_y,
    const FeatureExtractionParams& params)
{
    // 先施加凹角的几何安全约束，再在安全候选中选择靠近底面的真实红点。
    // 这样不会因“最低点恰好在背面”而丢掉同邻域内仍然有效的圆弧点。
    const float backside_tolerance = 0.5f;
    const auto is_safe = [&](const SeamProfileSample& sample) {
        if (sample.local_x < lower_x || sample.local_x > upper_x) return false;
        return params.protruding_is_larger_local_y
            ? sample.local_y >= ideal_y - backside_tolerance
            : sample.local_y <= ideal_y + backside_tolerance;
    };

    float nearest_distance_squared = std::numeric_limits<float>::max();
    bool found_safe_candidate = false;
    for (size_t i = 0; i < samples.size(); ++i) {
        if (!is_safe(samples[i])) continue;
        found_safe_candidate = true;
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        nearest_distance_squared = std::min(nearest_distance_squared, dx * dx + dy * dy);
    }
    if (!found_safe_candidate) return -1;

    const float maximum_distance = std::min(
        search_radius, std::sqrt(nearest_distance_squared) + params.low_z_xy_tolerance);
    const float maximum_distance_squared = maximum_distance * maximum_distance;
    std::vector<float> candidate_z;
    candidate_z.reserve(64);
    for (size_t i = 0; i < samples.size(); ++i) {
        if (!is_safe(samples[i])) continue;
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        if (dx * dx + dy * dy <= maximum_distance_squared) {
            candidate_z.push_back(samples[i].local_z);
        }
    }
    if (candidate_z.empty()) return -1;

    const float robust_low_z = quantileOf(candidate_z, params.bottom_z_quantile);
    int best_sample = -1;
    float best_score = std::numeric_limits<float>::max();
    for (size_t i = 0; i < samples.size(); ++i) {
        if (!is_safe(samples[i])) continue;
        const float dx = samples[i].local_x - target_x;
        const float dy = samples[i].local_y - target_y;
        const float distance_squared = dx * dx + dy * dy;
        if (distance_squared > maximum_distance_squared) continue;
        const float score = std::abs(samples[i].local_z - robust_low_z) +
            0.02f * std::sqrt(distance_squared);
        if (score < best_score) {
            best_score = score;
            best_sample = static_cast<int>(i);
        }
    }
    return best_sample;
}

static Eigen::Vector3f localToWorld(
    float local_x,
    float local_y,
    float local_z,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent)
{
    return local_x * n_tangent +
        (local_y - d_side) * n_side +
        (local_z - d_bottom) * n_bottom;
}

static WeldFeaturePoint makeSafeTransitionPoint(
    const WeldFeaturePoint& anchor_feature,
    bool is_start,
    float transition_offset_y,
    float transition_offset_z,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent)
{
    WeldFeaturePoint transition = anchor_feature;
    transition.is_transition_point = true;
    transition.is_start_transition = is_start;
    transition.measured_on_arc = false;
    transition.distance_to_ideal = 0.0f;
    transition.merged_detection_count = 1;

    // 过渡点不再沿轮廓前后外推：工件 X 与真实焊接点严格相同，只向开放侧和
    // 底板上方退出。这样不会因为视野首尾恰好落在圆角附近而跨到另一段轮廓。
    float adjacent_slope = is_start
        ? anchor_feature.left_line_slope : anchor_feature.right_line_slope;
    if (!std::isfinite(adjacent_slope)) adjacent_slope = 0.0f;
    transition.local = anchor_feature.local;
    // 内部 local_y 与工件 +Y 相反，所以 +Y 避让需要减 local_y。
    transition.local.y() -= transition_offset_y;
    transition.local.z() += transition_offset_z;
    transition.world = localToWorld(
        transition.local.x(), transition.local.y(), transition.local.z(),
        n_bottom, d_bottom, n_side, d_side, n_tangent);
    transition.ideal_local = transition.local;

    // 过渡点只关联一条安全直线，两侧斜率统一，保留诊断语义。
    transition.left_line_slope = adjacent_slope;
    transition.right_line_slope = adjacent_slope;
    const SeamSegmentType adjacent_segment = is_start
        ? anchor_feature.left_segment : anchor_feature.right_segment;
    transition.left_segment = adjacent_segment;
    transition.right_segment = adjacent_segment;
    return transition;
}

static std::vector<WeldFeaturePoint> mergeNearbyCornerDetections(
    std::vector<WeldFeaturePoint> corners,
    const FeatureExtractionParams& params,
    const Eigen::Vector3f& n_bottom)
{
    (void)n_bottom;
    if (corners.empty()) return corners;
    std::sort(corners.begin(), corners.end(),
        [](const WeldFeaturePoint& a, const WeldFeaturePoint& b) {
            if (a.topology_state != b.topology_state) {
                return a.topology_state < b.topology_state;
            }
            return a.ideal_local.x() < b.ideal_local.x();
        });

    std::vector<WeldFeaturePoint> merged_corners;
    const float merge_distance_squared =
        params.duplicate_corner_merge_distance * params.duplicate_corner_merge_distance;

    size_t group_begin = 0;
    while (group_begin < corners.size()) {
        size_t group_end = group_begin + 1;
        while (group_end < corners.size()) {
            const Eigen::Vector2f delta = corners[group_end].ideal_local.head<2>() -
                corners[group_begin].ideal_local.head<2>();
            if (corners[group_end].topology_state != corners[group_begin].topology_state ||
                delta.squaredNorm() > merge_distance_squared) {
                break;
            }
            ++group_end;
        }

        size_t best = group_begin;
        int detection_count = 0;
        for (size_t i = group_begin; i < group_end; ++i) {
            detection_count += std::max(1, corners[i].merged_detection_count);
            if (corners[i].detection_score > corners[best].detection_score) best = i;
        }

        // 同一个物理角可能由多组跨短噪声段的直线配对触发。保留支撑最完整的
        // 那一组，而不是把不同斜率逐项平均；后者会制造不存在的工具姿态。
        WeldFeaturePoint merged = corners[best];
        merged.merged_detection_count = detection_count;
        if (detection_count > 1) {
            std::cout << "Merged " << detection_count << " nearby topology-consistent "
                << (merged.protruding ? "protruding" : "recessed")
                << " detections; kept best-supported feature at local_x="
                << merged.ideal_local.x() << std::endl;
        }
        merged_corners.push_back(merged);
        group_begin = group_end;
    }

    std::sort(merged_corners.begin(), merged_corners.end(),
        [](const WeldFeaturePoint& a, const WeldFeaturePoint& b) {
            return a.ideal_local.x() < b.ideal_local.x();
        });
    return merged_corners;
}

static std::vector<WeldFeaturePoint> selectTopologyConsistentCornerChain(
    const std::vector<WeldFeaturePoint>& corners,
    const FeatureExtractionParams& params)
{
    if (corners.size() < 2) return corners;

    // 动态规划选择 local_x 单调且遵循四类周期顺序的候选链。允许缺失一至两个
    // 状态继续连接，但施加惩罚；相同状态不能在没有完整周期的情况下连续出现。
    const float base_reward = 100.0f;
    const float skipped_state_penalty = 60.0f;
    std::vector<float> best_score(corners.size(), 0.0f);
    std::vector<int> predecessor(corners.size(), -1);

    for (size_t i = 0; i < corners.size(); ++i) {
        best_score[i] = base_reward +
            std::max(0.0f, std::min(50.0f, corners[i].detection_score));
        for (size_t j = 0; j < i; ++j) {
            const int left_state = static_cast<int>(corners[j].topology_state);
            const int right_state = static_cast<int>(corners[i].topology_state);
            if (left_state < 0 || right_state < 0) continue;
            const int forward_steps = (right_state - left_state + 4) % 4;
            if (forward_steps == 0) continue;

            float maximum_distance = 0.0f;
            for (int step = 0; step < forward_steps; ++step) {
                const CornerTopologyState state = static_cast<CornerTopologyState>(
                    (left_state + step) % 4);
                maximum_distance += maximumSegmentLengthAfterCorner(state, params);
            }
            const float actual_distance =
                (corners[i].ideal_local.head<2>() -
                 corners[j].ideal_local.head<2>()).norm();
            const float minimum_distance = params.duplicate_corner_merge_distance *
                static_cast<float>(forward_steps);
            if (actual_distance < minimum_distance) {
                continue;
            }
            if (actual_distance > maximum_distance) continue;

            const float candidate_score = best_score[j] + base_reward +
                std::max(0.0f, std::min(50.0f, corners[i].detection_score)) -
                skipped_state_penalty * static_cast<float>(forward_steps - 1);
            if (candidate_score > best_score[i]) {
                best_score[i] = candidate_score;
                predecessor[i] = static_cast<int>(j);
            }
        }
    }

    size_t best_end = 0;
    for (size_t i = 1; i < corners.size(); ++i) {
        if (best_score[i] > best_score[best_end]) best_end = i;
    }

    std::vector<WeldFeaturePoint> selected_reverse;
    for (int index = static_cast<int>(best_end); index >= 0;
         index = predecessor[static_cast<size_t>(index)]) {
        selected_reverse.push_back(corners[static_cast<size_t>(index)]);
        if (predecessor[static_cast<size_t>(index)] < 0) break;
    }
    std::reverse(selected_reverse.begin(), selected_reverse.end());

    if (selected_reverse.size() != corners.size()) {
        std::cout << "Topology stabilization kept " << selected_reverse.size()
            << '/' << corners.size()
            << " merged corner candidates in the four-state corrugation cycle."
            << std::endl;
    }
    return selected_reverse;
}

static float flatSlopeFromFeature(const WeldFeaturePoint& feature)
{
    if (feature.left_segment == SEGMENT_FLAT) return feature.left_line_slope;
    if (feature.right_segment == SEGMENT_FLAT) return feature.right_line_slope;
    return 0.0f;
}

static WeldFeaturePoint makePeriodicBoundaryCompletion(
    const WeldFeaturePoint& reference_corner,
    const WeldFeaturePoint& reference_cycle_anchor,
    const WeldFeaturePoint& boundary_cycle_anchor,
    const WeldFeaturePoint* boundary_hint,
    float predicted_x,
    const std::vector<SeamProfileSample>& samples,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    WeldFeaturePoint completed = reference_corner;
    completed.is_transition_point = false;
    completed.is_start_transition = false;
    completed.protruding = topologyStateIsProtruding(completed.topology_state);
    completed.measured_on_arc = false;
    completed.distance_to_ideal = 0.0f;
    completed.merged_detection_count = 1;
    completed.detection_score = 0.0f;
    completed.topology_inferred = true;

    float ideal_x = predicted_x;
    float ideal_y = reference_corner.ideal_local.y() +
        (boundary_cycle_anchor.ideal_local.y() -
         reference_cycle_anchor.ideal_local.y());
    if (boundary_hint != nullptr) {
        ideal_x = boundary_hint->ideal_local.x();
        ideal_y = boundary_hint->ideal_local.y();
        const float boundary_flat_slope = flatSlopeFromFeature(*boundary_hint);
        if (completed.topology_state == CORNER_RIGHT_WAIST_TO_FLAT ||
            completed.topology_state == CORNER_LEFT_WAIST_TO_FLAT) {
            completed.right_line_slope = boundary_flat_slope;
        } else {
            completed.left_line_slope = boundary_flat_slope;
        }
    }

    const float ideal_z = estimateCornerZ(
        samples, ideal_x, ideal_y, params.recessed_arc_search_radius,
        params.bottom_z_quantile);
    completed.local = Eigen::Vector3f(ideal_x, ideal_y, ideal_z);
    completed.ideal_local = completed.local;
    completed.world = localToWorld(
        ideal_x, ideal_y, ideal_z,
        n_bottom, d_bottom, n_side, d_side, n_tangent);
    return completed;
}

static void completePeriodicBoundaryCorners(
    std::vector<WeldFeaturePoint>& corners,
    const std::vector<WeldFeaturePoint>& structural_hints,
    const std::vector<SeamProfileSample>& samples,
    const std::vector<SeamProfileBin>& bins,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    // 至少五个角意味着当前帧已经包含一整周期，并且同类角至少重复一次。
    // 只有这种帧内证据充分的情况才补边界；绝不依据固定节距向视野外造点。
    if (corners.size() < 5 || bins.empty()) return;

    const float boundary_tolerance = 1.0f;

    // 末端：用上一个同类角到其后继角的实际间距，预测当前同类角的后继。
    {
        const size_t last = corners.size() - 1;
        const CornerTopologyState anchor_state = corners[last].topology_state;
        int previous_anchor = -1;
        for (int i = static_cast<int>(last) - 1; i >= 0; --i) {
            if (corners[static_cast<size_t>(i)].topology_state == anchor_state) {
                previous_anchor = i;
                break;
            }
        }
        const CornerTopologyState expected = static_cast<CornerTopologyState>(
            (static_cast<int>(anchor_state) + 1) % 4);
        int reference_next = -1;
        if (previous_anchor >= 0) {
            for (size_t i = static_cast<size_t>(previous_anchor + 1); i < last; ++i) {
                if (corners[i].topology_state == expected) {
                    reference_next = static_cast<int>(i);
                    break;
                }
            }
        }

        if (previous_anchor >= 0 && reference_next >= 0) {
            const float predicted_x = corners[last].ideal_local.x() +
                corners[static_cast<size_t>(reference_next)].ideal_local.x() -
                corners[static_cast<size_t>(previous_anchor)].ideal_local.x();
            const float profile_end_x = bins.back().local_x;
            if (predicted_x > corners[last].ideal_local.x() &&
                predicted_x <= profile_end_x + boundary_tolerance &&
                profile_end_x - predicted_x <= params.max_corner_extrapolation) {
                const WeldFeaturePoint* hint = nullptr;
                float best_hint_distance = params.max_corner_extrapolation;
                for (size_t i = 0; i < structural_hints.size(); ++i) {
                    const WeldFeaturePoint& candidate = structural_hints[i];
                    if (candidate.ideal_local.x() <= corners[last].ideal_local.x() ||
                        candidate.protruding != topologyStateIsProtruding(expected) ||
                        candidate.detection_score <
                            params.duplicate_corner_merge_distance) {
                        continue;
                    }
                    const float distance =
                        std::abs(candidate.ideal_local.x() - predicted_x);
                    if (distance <= best_hint_distance) {
                        best_hint_distance = distance;
                        hint = &candidate;
                    }
                }
                WeldFeaturePoint completed = makePeriodicBoundaryCompletion(
                    corners[static_cast<size_t>(reference_next)],
                    corners[static_cast<size_t>(previous_anchor)], corners[last], hint,
                    predicted_x, samples,
                    n_bottom, d_bottom, n_side, d_side, n_tangent, params);
                corners.push_back(completed);
                std::cout << "Completed one end-boundary corner from in-frame period "
                    << "topology at local_x=" << completed.ideal_local.x()
                    << (hint != nullptr ? " using a boundary turn hint." : ".")
                    << std::endl;
            }
        }
    }

    // 起点与末端完全对称：利用后一个同类角及其前驱反推缺失前驱。
    if (corners.size() < 5) return;
    {
        const CornerTopologyState anchor_state = corners.front().topology_state;
        int next_anchor = -1;
        for (size_t i = 1; i < corners.size(); ++i) {
            if (corners[i].topology_state == anchor_state) {
                next_anchor = static_cast<int>(i);
                break;
            }
        }
        const CornerTopologyState expected = static_cast<CornerTopologyState>(
            (static_cast<int>(anchor_state) + 3) % 4);
        int reference_previous = -1;
        if (next_anchor > 0) {
            for (int i = next_anchor - 1; i > 0; --i) {
                if (corners[static_cast<size_t>(i)].topology_state == expected) {
                    reference_previous = i;
                    break;
                }
            }
        }

        if (next_anchor > 0 && reference_previous > 0) {
            const float predicted_x = corners.front().ideal_local.x() -
                (corners[static_cast<size_t>(next_anchor)].ideal_local.x() -
                 corners[static_cast<size_t>(reference_previous)].ideal_local.x());
            const float profile_start_x = bins.front().local_x;
            if (predicted_x < corners.front().ideal_local.x() &&
                predicted_x >= profile_start_x - boundary_tolerance &&
                predicted_x - profile_start_x <= params.max_corner_extrapolation) {
                const WeldFeaturePoint* hint = nullptr;
                float best_hint_distance = params.max_corner_extrapolation;
                for (size_t i = 0; i < structural_hints.size(); ++i) {
                    const WeldFeaturePoint& candidate = structural_hints[i];
                    if (candidate.ideal_local.x() >= corners.front().ideal_local.x() ||
                        candidate.protruding != topologyStateIsProtruding(expected) ||
                        candidate.detection_score <
                            params.duplicate_corner_merge_distance) {
                        continue;
                    }
                    const float distance =
                        std::abs(candidate.ideal_local.x() - predicted_x);
                    if (distance <= best_hint_distance) {
                        best_hint_distance = distance;
                        hint = &candidate;
                    }
                }
                WeldFeaturePoint completed = makePeriodicBoundaryCompletion(
                    corners[static_cast<size_t>(reference_previous)],
                    corners[static_cast<size_t>(next_anchor)], corners.front(), hint,
                    predicted_x, samples,
                    n_bottom, d_bottom, n_side, d_side, n_tangent, params);
                corners.insert(corners.begin(), completed);
                std::cout << "Completed one start-boundary corner from in-frame period "
                    << "topology at local_x=" << completed.ideal_local.x()
                    << (hint != nullptr ? " using a boundary turn hint." : ".")
                    << std::endl;
            }
        }
    }
}

static std::vector<WeldFeaturePoint> buildSingleSafeCornerPoints(
    const std::vector<WeldFeaturePoint>& corners,
    const std::vector<SeamProfileSample>& samples,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    std::vector<WeldFeaturePoint> safe_corners;
    safe_corners.reserve(corners.size());
    const float support_distance = params.corner_fit_support_distance;

    for (size_t i = 0; i < corners.size(); ++i) {
        const WeldFeaturePoint& corner = corners[i];
        const Eigen::Vector3f ideal = corner.ideal_local;

        Eigen::Vector2f left_direction(1.0f, corner.left_line_slope);
        Eigen::Vector2f right_direction(1.0f, corner.right_line_slope);
        if (!left_direction.allFinite() || !right_direction.allFinite() ||
            left_direction.norm() < 1e-6f || right_direction.norm() < 1e-6f) {
            continue;
        }
        left_direction.normalize();
        right_direction.normalize();

        const Eigen::Vector2f ideal_xy = ideal.head<2>();
        const Eigen::Vector2f left_xy =
            ideal_xy - support_distance * left_direction;
        const Eigen::Vector2f right_xy =
            ideal_xy + support_distance * right_direction;

        WeldFeaturePoint center = corner;
        if (center.protruding) {
            // 凸角中心保留两条直线的外侧理论交点。
            center.local = ideal;
            center.world = localToWorld(
                ideal.x(), ideal.y(), ideal.z(),
                n_bottom, d_bottom, n_side, d_side, n_tangent);
            center.measured_on_arc = false;
            center.distance_to_ideal = 0.0f;
        } else {
            // 凹角不再以可能位于背面的理论顶点为搜索目标，而以两侧支撑点弦中点
            // 为目标寻找真实红点。若没有安全实测点，弦中点本身是保守回退点。
            const Eigen::Vector2f chord_midpoint = 0.5f * (left_xy + right_xy);
            const float lower_x = std::min(left_xy.x(), right_xy.x()) - 1.0f;
            const float upper_x = std::max(left_xy.x(), right_xy.x()) + 1.0f;
            const int measured_index = selectSafeRecessedPointNearBottom(
                samples, chord_midpoint.x(), chord_midpoint.y(),
                params.recessed_chord_search_radius,
                lower_x, upper_x, ideal.y(), params);

            bool accepted_measured_point = false;
            if (measured_index >= 0) {
                const SeamProfileSample& measured =
                    samples[static_cast<size_t>(measured_index)];
                center.world = measured.world;
                center.local = Eigen::Vector3f(
                    measured.local_x, measured.local_y, measured.local_z);
                center.measured_on_arc = true;
                accepted_measured_point = true;
            }

            if (!accepted_measured_point) {
                center.local = Eigen::Vector3f(
                    chord_midpoint.x(), chord_midpoint.y(),
                    estimateCornerZ(samples, chord_midpoint.x(), chord_midpoint.y(),
                        params.recessed_chord_search_radius, params.bottom_z_quantile));
                center.world = localToWorld(
                    center.local.x(), center.local.y(), center.local.z(),
                    n_bottom, d_bottom, n_side, d_side, n_tangent);
                center.measured_on_arc = false;
            }
            center.distance_to_ideal =
                (center.local.head<2>() - ideal_xy).norm();
        }
        safe_corners.push_back(center);
    }

    std::sort(safe_corners.begin(), safe_corners.end(),
        [](const WeldFeaturePoint& a, const WeldFeaturePoint& b) {
            return a.local.x() < b.local.x();
        });
    return safe_corners;
}

static std::vector<WeldFeaturePoint> extractOrderedWeldFeatures(
    const pcl::PointCloud<PointInT>::ConstPtr& cloud,
    const std::vector<bool>& is_final_seam,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& params)
{
    std::vector<WeldFeaturePoint> features;
    std::vector<SeamProfileSample> samples = collectSeamProfileSamples(
        cloud, is_final_seam, n_bottom, d_bottom, n_side, d_side, n_tangent, params);
    if (samples.size() < 2) {
        std::cerr << "Feature extraction skipped: too few red seam points ("
            << samples.size() << ")." << std::endl;
        return features;
    }

    std::vector<SeamProfileBin> bins = buildProfileBins(samples, params);
    if (bins.empty()) {
        std::cerr << "Feature extraction skipped: no valid profile bins." << std::endl;
        return features;
    }
    if (bins.size() < 6) {
        std::cerr << "Feature extraction skipped: too few profile bins ("
            << bins.size() << "); no complete four-type corner is exported."
            << std::endl;
        return features;
    }
    refineBinLabelsFromLocalGeometry(bins, params);
    removeShortLabelNoise(bins, params);
    const std::vector<SeamLineRun> runs = buildLineRuns(bins, params);
    int valid_run_count = 0;
    for (size_t i = 0; i < runs.size(); ++i) {
        if (runs[i].valid) ++valid_run_count;
    }
    std::cout << "Secondary profile: red_points=" << samples.size()
        << ", bins=" << bins.size()
        << ", valid_line_runs=" << valid_run_count << "/" << runs.size()
        << std::endl;
    const float depth_split = estimateDepthLevelSplit(runs, bins);
    std::cout << "Corner-layer mapping: "
        << (params.protruding_is_larger_local_y
            ? "larger local_y = PROTRUDING"
            : "smaller local_y (toward L side) = PROTRUDING")
        << ", split_y=" << depth_split << " mm" << std::endl;
    const float min_corner_angle =
        params.min_corner_angle_deg * static_cast<float>(3.14159265358979323846 / 180.0);

    // 只消费有效直线段，但不再限定必须是相邻的两个有效 run。真实平段/腰段之间
    // 偶尔会插入刚好达到最小阈值的短假斜线；在孔洞上限内枚举后续 run，再由
    // 四类物理转移、支撑质量和整条周期拓扑共同消歧。
    std::vector<int> valid_run_indices;
    for (size_t i = 0; i < runs.size(); ++i) {
        if (runs[i].valid) valid_run_indices.push_back(static_cast<int>(i));
    }
    std::vector<WeldFeaturePoint> structural_hints;

    for (size_t valid_i = 0; valid_i + 1 < valid_run_indices.size(); ++valid_i) {
        const SeamLineRun& left =
            runs[static_cast<size_t>(valid_run_indices[valid_i])];
        for (size_t right_i = valid_i + 1;
             right_i < valid_run_indices.size(); ++right_i) {
            const SeamLineRun& right =
                runs[static_cast<size_t>(valid_run_indices[right_i])];
            const float observed_gap = right.x_first - left.x_last;
            if (observed_gap > params.max_hole_bridge) break;
            if (observed_gap < 0.0f) continue;

            const CornerTopologyState topology_state =
                classifyCornerTopology(left.type, right.type);
            if (topology_state == CORNER_TOPOLOGY_UNKNOWN) continue;

            const float angle_left = std::atan(left.line.slope);
            const float angle_right = std::atan(right.line.slope);
            float turn_angle = std::abs(angle_right - angle_left);
            if (turn_angle > static_cast<float>(3.14159265358979323846 / 2.0)) {
                turn_angle = static_cast<float>(3.14159265358979323846) - turn_angle;
            }
            if (turn_angle < min_corner_angle) continue;

            const float denominator = left.line.slope - right.line.slope;
            if (std::abs(denominator) < 1e-5f) continue;
            const float ideal_x =
                (right.line.intercept - left.line.intercept) / denominator;
            const float ideal_y = left.line.slope * ideal_x + left.line.intercept;

            // 非边界补全候选必须有实测轮廓覆盖；不在这里向视野外造点。
            if (ideal_x < bins.front().local_x || ideal_x > bins.back().local_x) continue;
            if (ideal_x < left.x_last - params.max_corner_extrapolation ||
                ideal_x > right.x_first + params.max_corner_extrapolation) {
                continue;
            }

            float flat_y = ideal_y;
            if (left.type == SEGMENT_FLAT) {
                flat_y = left.line.slope * ideal_x + left.line.intercept;
            } else {
                flat_y = right.line.slope * ideal_x + right.line.intercept;
            }
            const bool protruding = params.protruding_is_larger_local_y
                ? flat_y >= depth_split
                : flat_y <= depth_split;

            float skipped_support_length = 0.0f;
            for (size_t skipped = valid_i + 1; skipped < right_i; ++skipped) {
                skipped_support_length += lineRunObservedLength(
                    runs[static_cast<size_t>(valid_run_indices[skipped])]);
            }
            const float left_support = lineRunObservedLength(left);
            const float right_support = lineRunObservedLength(right);
            const float left_extrapolation =
                std::max(0.0f, ideal_x - left.x_last);
            const float right_extrapolation =
                std::max(0.0f, right.x_first - ideal_x);

            WeldFeaturePoint feature;
            feature.is_transition_point = false;
            feature.is_start_transition = false;
            feature.protruding = protruding;
            feature.measured_on_arc = false;
            feature.distance_to_ideal = 0.0f;
            feature.merged_detection_count = 1;
            feature.left_line_slope = left.line.slope;
            feature.right_line_slope = right.line.slope;
            feature.left_segment = left.type;
            feature.right_segment = right.type;
            feature.topology_state = topology_state;
            feature.detection_score =
                std::min(left_support, right_support) +
                0.05f * std::max(left_support, right_support) -
                0.25f * skipped_support_length -
                0.10f * observed_gap -
                0.50f * (left_extrapolation + right_extrapolation);
            feature.topology_inferred = false;

            const float ideal_z = estimateCornerZ(
                samples, ideal_x, ideal_y, params.recessed_arc_search_radius,
                params.bottom_z_quantile);
            feature.local = Eigen::Vector3f(ideal_x, ideal_y, ideal_z);
            feature.ideal_local = feature.local;
            feature.world = localToWorld(
                ideal_x, ideal_y, ideal_z,
                n_bottom, d_bottom, n_side, d_side, n_tangent);

            if (protruding == topologyStateIsProtruding(topology_state)) {
                features.push_back(feature);
            } else {
                // 错层的转向不能作为真实角，但其位置可作为首末周期补全的边界提示。
                structural_hints.push_back(feature);
            }
        }
    }

    // 合并同类近邻检测，再用四状态周期约束剔除平段内部的短假转折。
    const std::vector<WeldFeaturePoint> unique_features =
        mergeNearbyCornerDetections(features, params, n_bottom);
    std::vector<WeldFeaturePoint> stabilized_features =
        selectTopologyConsistentCornerChain(unique_features, params);
    completePeriodicBoundaryCorners(
        stabilized_features, structural_hints, samples, bins,
        n_bottom, d_bottom, n_side, d_side, n_tangent, params);
    const std::vector<WeldFeaturePoint> safe_corner_points =
        buildSingleSafeCornerPoints(
            stabilized_features, samples,
            n_bottom, d_bottom, n_side, d_side, n_tangent,
            params);
    std::cout << "Built " << safe_corner_points.size()
        << " single safe corner points using +/-"
        << params.corner_fit_support_distance << " mm fit supports." << std::endl;
    if (safe_corner_points.empty()) {
        std::cerr << "No complete protruding/recessed corner found; no transition "
                     "point is generated." << std::endl;
        return safe_corner_points;
    }

    // 最终焊接特征只允许四类真实凹凸角。拍摄视野的原始首尾红点不再输出，
    // 从而消除边界恰好落在圆角时“边界点+真实角点”过近的问题。
    // 在首/末真实角的相同工件 X 处，沿开放侧 +Y、底板上方 +Z 各生成安全过渡点。
    const WeldFeaturePoint start_transition = makeSafeTransitionPoint(
        safe_corner_points.front(), true,
        params.safe_transition_offset_y, params.safe_transition_offset_z,
        n_bottom, d_bottom, n_side, d_side, n_tangent);
    const WeldFeaturePoint end_transition = makeSafeTransitionPoint(
        safe_corner_points.back(), false,
        params.safe_transition_offset_y, params.safe_transition_offset_z,
        n_bottom, d_bottom, n_side, d_side, n_tangent);

    std::vector<WeldFeaturePoint> ordered_features;
    ordered_features.reserve(safe_corner_points.size() + 2);
    ordered_features.push_back(start_transition);
    for (size_t i = 0; i < safe_corner_points.size(); ++i) {
        ordered_features.push_back(safe_corner_points[i]);
    }
    ordered_features.push_back(end_transition);
    return ordered_features;
}

struct AdaptiveProfilePoint {
    Eigen::Vector3f local;
    float arc_length;
    float corner_distance;
};

static std::vector<SeamProfileBin> filterAdaptiveProfile(
    const std::vector<SeamProfileBin>& input,
    const AdaptiveContourParams& params,
    float& largest_gap)
{
    std::vector<SeamProfileBin> filtered = input;
    largest_gap = 0.0f;
    for (size_t i = 1; i < input.size(); ++i) {
        largest_gap = std::max(
            largest_gap, input[i].local_x - input[i - 1].local_x);
    }

    // 只替换明显偏离邻域中值的单点/短飞溅，不对正常轮廓做大窗口平滑，
    // 因而不会把真实圆角或板材曲率压平成理论梯形。
    for (size_t i = 0; i < input.size(); ++i) {
        const int first = std::max(
            0, static_cast<int>(i) - params.smoothing_half_window_bins);
        const int last = std::min(
            static_cast<int>(input.size()) - 1,
            static_cast<int>(i) + params.smoothing_half_window_bins);
        std::vector<float> ys;
        std::vector<float> zs;
        ys.reserve(static_cast<size_t>(last - first + 1));
        zs.reserve(static_cast<size_t>(last - first + 1));
        for (int j = first; j <= last; ++j) {
            // 不跨越不可接受的大孔洞借用另一侧数据。
            if (std::abs(input[static_cast<size_t>(j)].local_x - input[i].local_x) >
                params.max_bridge_gap) {
                continue;
            }
            ys.push_back(input[static_cast<size_t>(j)].local_y);
            zs.push_back(input[static_cast<size_t>(j)].local_z);
        }
        if (ys.empty()) continue;
        const float median_y = medianOf(ys);
        const float median_z = medianOf(zs);
        const float dy = input[i].local_y - median_y;
        const float dz = input[i].local_z - median_z;
        if (std::sqrt(dy * dy + dz * dz) > params.outlier_max_distance) {
            filtered[i].local_y = median_y;
            filtered[i].local_z = median_z;
        }
    }

    // 第二级小窗口中值用于消除红色焊缝带厚度造成的逐 bin 抖动。窗口只有
    // 数毫米，保留真实圆角/板材曲率，同时让后续位姿切线连续可执行。
    std::vector<SeamProfileBin> smoothed = filtered;
    for (size_t i = 0; i < filtered.size(); ++i) {
        const int first = std::max(
            0, static_cast<int>(i) - params.smoothing_half_window_bins);
        const int last = std::min(
            static_cast<int>(filtered.size()) - 1,
            static_cast<int>(i) + params.smoothing_half_window_bins);
        std::vector<float> ys;
        std::vector<float> zs;
        for (int j = first; j <= last; ++j) {
            if (std::abs(filtered[static_cast<size_t>(j)].local_x -
                         filtered[i].local_x) > params.max_bridge_gap) {
                continue;
            }
            ys.push_back(filtered[static_cast<size_t>(j)].local_y);
            zs.push_back(filtered[static_cast<size_t>(j)].local_z);
        }
        if (!ys.empty()) {
            smoothed[i].local_y = medianOf(ys);
            smoothed[i].local_z = medianOf(zs);
        }
    }
    return smoothed;
}

static float adaptiveTurnAngleDeg(
    const std::vector<AdaptiveProfilePoint>& profile,
    size_t index,
    float window)
{
    if (index == 0 || index + 1 >= profile.size()) return 0.0f;
    size_t left = index;
    while (left > 0 &&
           profile[index].arc_length - profile[left].arc_length < window) {
        --left;
    }
    size_t right = index;
    while (right + 1 < profile.size() &&
           profile[right].arc_length - profile[index].arc_length < window) {
        ++right;
    }
    if (left == index || right == index) return 0.0f;

    Eigen::Vector2f incoming =
        profile[index].local.head<2>() - profile[left].local.head<2>();
    Eigen::Vector2f outgoing =
        profile[right].local.head<2>() - profile[index].local.head<2>();
    if (!incoming.allFinite() || !outgoing.allFinite() ||
        incoming.norm() < 1e-5f || outgoing.norm() < 1e-5f) {
        return 0.0f;
    }
    incoming.normalize();
    outgoing.normalize();
    const float dot = std::max(-1.0f, std::min(1.0f, incoming.dot(outgoing)));
    return std::acos(dot) * static_cast<float>(180.0 / 3.14159265358979323846);
}

static float interpolateProfileScalar(
    const std::vector<AdaptiveProfilePoint>& profile,
    float arc,
    bool corner_distance)
{
    if (profile.empty()) return 0.0f;
    if (arc <= 0.0f) {
        return corner_distance ? profile.front().corner_distance : 0.0f;
    }
    if (arc >= profile.back().arc_length) {
        return corner_distance ? profile.back().corner_distance : 0.0f;
    }
    const auto upper = std::upper_bound(
        profile.begin(), profile.end(), arc,
        [](float value, const AdaptiveProfilePoint& point) {
            return value < point.arc_length;
        });
    const size_t right = static_cast<size_t>(upper - profile.begin());
    const size_t left = right - 1;
    const float span = profile[right].arc_length - profile[left].arc_length;
    const float alpha = span > 1e-6f
        ? (arc - profile[left].arc_length) / span : 0.0f;
    if (corner_distance) {
        return (1.0f - alpha) * profile[left].corner_distance +
            alpha * profile[right].corner_distance;
    }
    return alpha;
}

static Eigen::Vector3f interpolateAdaptiveLocal(
    const std::vector<AdaptiveProfilePoint>& profile,
    float arc,
    float slope_window,
    float& local_slope)
{
    if (arc <= 0.0f) arc = 0.0f;
    if (arc >= profile.back().arc_length) arc = profile.back().arc_length;
    const auto upper = std::upper_bound(
        profile.begin(), profile.end(), arc,
        [](float value, const AdaptiveProfilePoint& point) {
            return value < point.arc_length;
        });
    size_t right = static_cast<size_t>(upper - profile.begin());
    if (right == 0) right = 1;
    if (right >= profile.size()) right = profile.size() - 1;
    const size_t left = right - 1;
    const float span = profile[right].arc_length - profile[left].arc_length;
    const float alpha = span > 1e-6f
        ? (arc - profile[left].arc_length) / span : 0.0f;
    const Eigen::Vector3f local =
        (1.0f - alpha) * profile[left].local + alpha * profile[right].local;
    // 姿态切线不能由相邻 0.8mm bin 决定。也不能用 Y 对 X 的普通回归：
    // 波纹腰段接近局部竖直时 X 方差很小，回归斜率会被毫米级噪声放大。
    // 改为固定弧长窗口两端的有向弦，稳定覆盖水平段、斜腰和近竖直腰段。
    size_t fit_left = left;
    while (fit_left > 0 &&
           arc - profile[fit_left].arc_length < slope_window) {
        --fit_left;
    }
    size_t fit_right = right;
    while (fit_right + 1 < profile.size() &&
           profile[fit_right].arc_length - arc < slope_window) {
        ++fit_right;
    }
    const Eigen::Vector2f chord =
        profile[fit_right].local.head<2>() -
        profile[fit_left].local.head<2>();
    if (chord.allFinite() && std::abs(chord.x()) > 1e-6f) {
        local_slope = chord.y() / chord.x();
    } else {
        // profile 按 local_x 递增，真正完全竖直只会来自数值退化。用有限大
        // 斜率保留腰段方向，后续统一在角度域平滑而不是直接平均此数值。
        local_slope = chord.y() >= 0.0f ? 1e6f : -1e6f;
    }
    return local;
}

static std::vector<float> smoothAdaptiveTangentSlopes(
    const std::vector<float>& sample_arcs,
    const std::vector<float>& raw_slopes,
    float smoothing_radius,
    float max_step_deg,
    float& raw_max_step_deg,
    float& smoothed_max_step_deg)
{
    const float radians_to_degrees =
        static_cast<float>(180.0 / 3.14159265358979323846);
    const float degrees_to_radians = 1.0f / radians_to_degrees;
    std::vector<float> raw_angles(raw_slopes.size(), 0.0f);
    for (size_t i = 0; i < raw_slopes.size(); ++i) {
        raw_angles[i] = std::atan(raw_slopes[i]);
    }

    raw_max_step_deg = 0.0f;
    for (size_t i = 1; i < raw_angles.size(); ++i) {
        raw_max_step_deg = std::max(
            raw_max_step_deg,
            std::abs(raw_angles[i] - raw_angles[i - 1]) * radians_to_degrees);
    }

    // 三角权重中心窗口没有单向相位滞后。平均 atan(slope) 而非 slope，
    // 使近竖直腰段不会因极大数值获得不成比例的权重。
    std::vector<float> centered(raw_angles.size(), 0.0f);
    for (size_t i = 0; i < raw_angles.size(); ++i) {
        double weighted_angle = 0.0;
        double weight_sum = 0.0;
        for (size_t j = 0; j < raw_angles.size(); ++j) {
            const float distance = std::abs(sample_arcs[j] - sample_arcs[i]);
            if (distance > smoothing_radius) continue;
            const float weight = smoothing_radius > 1e-6f
                ? std::max(0.0f, 1.0f - distance / smoothing_radius)
                : (i == j ? 1.0f : 0.0f);
            weighted_angle += static_cast<double>(weight) * raw_angles[j];
            weight_sum += weight;
        }
        centered[i] = weight_sum > 1e-9
            ? static_cast<float>(weighted_angle / weight_sum) : raw_angles[i];
    }

    // 分别从首、末端构造满足最大角步长的序列，再取平均。两个满足限幅
    // 的实数角序列取平均后仍满足同一限幅，同时避免单向滤波在末端积累滞后。
    const float max_step = max_step_deg * degrees_to_radians;
    std::vector<float> forward = centered;
    for (size_t i = 1; i < forward.size(); ++i) {
        const float delta = forward[i] - forward[i - 1];
        forward[i] = forward[i - 1] +
            std::max(-max_step, std::min(max_step, delta));
    }
    std::vector<float> backward = centered;
    for (size_t i = backward.size(); i-- > 1;) {
        const size_t previous = i - 1;
        const float delta = backward[previous] - backward[i];
        backward[previous] = backward[i] +
            std::max(-max_step, std::min(max_step, delta));
    }

    std::vector<float> slopes(raw_slopes.size(), 0.0f);
    smoothed_max_step_deg = 0.0f;
    float previous_angle = 0.0f;
    for (size_t i = 0; i < slopes.size(); ++i) {
        const float angle = 0.5f * (forward[i] + backward[i]);
        const float safe_angle = std::max(
            -89.0f * degrees_to_radians,
            std::min(89.0f * degrees_to_radians, angle));
        slopes[i] = std::tan(safe_angle);
        if (i > 0) {
            smoothed_max_step_deg = std::max(
                smoothed_max_step_deg,
                std::abs(safe_angle - previous_angle) * radians_to_degrees);
        }
        previous_angle = safe_angle;
    }
    return slopes;
}

static std::vector<float> makeAdaptiveSampleArcs(
    const std::vector<AdaptiveProfilePoint>& profile,
    const AdaptiveContourParams& params,
    float spacing_scale)
{
    std::vector<float> arcs;
    const float total = profile.back().arc_length;
    arcs.push_back(0.0f);
    float current = 0.0f;
    while (current < total - 1e-4f) {
        const float distance_to_corner = interpolateProfileScalar(
            profile, current, true);
        const float blend = params.corner_influence_radius > 1e-6f
            ? std::max(0.0f, std::min(
                1.0f, distance_to_corner / params.corner_influence_radius))
            : 1.0f;
        const float base_spacing = params.corner_spacing +
            blend * (params.straight_spacing - params.corner_spacing);
        const float spacing = std::max(0.25f, spacing_scale * base_spacing);
        const float next = std::min(total, current + spacing);
        if (next <= current + 1e-5f) break;
        if (next >= total - 1e-4f &&
            total - current < 0.75f * spacing_scale * params.corner_spacing &&
            arcs.size() > 1) {
            // 不在路径末端留下 1~2mm 的几乎重复点；用真实末端替换上一个
            // 采样点，保持轮廓完整且不增加机器人无意义的极短 MoveL。
            arcs.back() = total;
            current = total;
            break;
        }
        arcs.push_back(next);
        current = next;
    }
    if (arcs.back() < total - 1e-4f) arcs.push_back(total);
    return arcs;
}

static std::vector<WeldFeaturePoint> extractAdaptiveContourPath(
    const pcl::PointCloud<PointInT>::ConstPtr& cloud,
    const std::vector<bool>& is_final_seam,
    const Eigen::Vector3f& n_bottom,
    float d_bottom,
    const Eigen::Vector3f& n_side,
    float d_side,
    const Eigen::Vector3f& n_tangent,
    const FeatureExtractionParams& feature_params,
    const AdaptiveContourParams& params,
    std::string& error)
{
    std::vector<WeldFeaturePoint> result;
    const std::vector<SeamProfileSample> samples = collectSeamProfileSamples(
        cloud, is_final_seam, n_bottom, d_bottom, n_side, d_side,
        n_tangent, feature_params);
    if (samples.size() < 2) {
        error = "adaptive contour: too few red seam points";
        return result;
    }
    std::vector<SeamProfileBin> bins = buildProfileBins(samples, feature_params);
    if (bins.size() < 3) {
        error = "adaptive contour: too few robust profile bins";
        return result;
    }

    float largest_gap = 0.0f;
    bins = filterAdaptiveProfile(bins, params, largest_gap);
    if (largest_gap > params.max_bridge_gap) {
        std::ostringstream stream;
        stream << "adaptive contour: profile gap " << largest_gap
               << " mm exceeds path.max_bridge_gap="
               << params.max_bridge_gap << " mm";
        error = stream.str();
        return result;
    }

    std::vector<AdaptiveProfilePoint> profile;
    profile.reserve(bins.size());
    for (size_t i = 0; i < bins.size(); ++i) {
        AdaptiveProfilePoint point;
        point.local = Eigen::Vector3f(
            bins[i].local_x, bins[i].local_y, bins[i].local_z);
        point.arc_length = 0.0f;
        point.corner_distance = std::numeric_limits<float>::infinity();
        if (!profile.empty()) {
            // 焊缝位于底板平面附近，密度控制只按轮廓 XY 弧长计算，避免法向
            // 或点焊造成的微小 Z 抖动虚增路径长度。
            point.arc_length = profile.back().arc_length +
                (point.local.head<2>() - profile.back().local.head<2>()).norm();
        }
        profile.push_back(point);
    }
    if (profile.back().arc_length < 1.0f) {
        error = "adaptive contour: usable contour is shorter than 1 mm";
        return result;
    }

    size_t curvature_bin_count = 0;
    for (size_t i = 0; i < profile.size(); ++i) {
        if (adaptiveTurnAngleDeg(profile, i, params.curvature_window) >=
            params.curvature_threshold_deg) {
            profile[i].corner_distance = 0.0f;
            ++curvature_bin_count;
        }
    }
    // 没有明显转角时全程按直线间距；否则两次扫描得到每个 profile bin
    // 到最近曲率区的真实弧长距离。
    if (curvature_bin_count > 0) {
        float distance = std::numeric_limits<float>::infinity();
        for (size_t i = 0; i < profile.size(); ++i) {
            if (profile[i].corner_distance == 0.0f) distance = 0.0f;
            else if (i > 0 && std::isfinite(distance)) {
                distance += profile[i].arc_length - profile[i - 1].arc_length;
            }
            profile[i].corner_distance = std::min(
                profile[i].corner_distance, distance);
        }
        distance = std::numeric_limits<float>::infinity();
        for (size_t i = profile.size(); i-- > 0;) {
            if (profile[i].corner_distance == 0.0f) distance = 0.0f;
            else if (i + 1 < profile.size() && std::isfinite(distance)) {
                distance += profile[i + 1].arc_length - profile[i].arc_length;
            }
            profile[i].corner_distance = std::min(
                profile[i].corner_distance, distance);
        }
    } else {
        for (AdaptiveProfilePoint& point : profile) {
            point.corner_distance = params.corner_influence_radius;
        }
    }

    const size_t weld_budget = static_cast<size_t>(params.max_points - 2);
    float spacing_scale = 1.0f;
    std::vector<float> sample_arcs = makeAdaptiveSampleArcs(
        profile, params, spacing_scale);
    for (int iteration = 0;
         sample_arcs.size() > weld_budget && iteration < 32; ++iteration) {
        const float ratio = static_cast<float>(sample_arcs.size()) /
            static_cast<float>(weld_budget);
        spacing_scale *= std::max(1.05f, ratio * 1.01f);
        sample_arcs = makeAdaptiveSampleArcs(profile, params, spacing_scale);
    }
    if (sample_arcs.size() > weld_budget) {
        error = "adaptive contour: cannot satisfy path.max_points without truncation";
        return result;
    }

    std::vector<float> all_y;
    all_y.reserve(profile.size());
    for (const AdaptiveProfilePoint& point : profile) {
        all_y.push_back(point.local.y());
    }
    const float depth_split = medianOf(all_y);

    std::vector<Eigen::Vector3f> sampled_locals;
    std::vector<float> raw_slopes;
    sampled_locals.reserve(sample_arcs.size());
    raw_slopes.reserve(sample_arcs.size());
    for (float arc : sample_arcs) {
        float raw_slope = 0.0f;
        sampled_locals.push_back(interpolateAdaptiveLocal(
            profile, arc, params.curvature_window, raw_slope));
        raw_slopes.push_back(raw_slope);
    }
    float raw_max_tangent_step_deg = 0.0f;
    float smoothed_max_tangent_step_deg = 0.0f;
    const std::vector<float> pose_slopes = smoothAdaptiveTangentSlopes(
        sample_arcs, raw_slopes,
        params.orientation_smoothing_radius,
        params.max_orientation_step_deg,
        raw_max_tangent_step_deg,
        smoothed_max_tangent_step_deg);

    std::vector<WeldFeaturePoint> weld_points;
    weld_points.reserve(sample_arcs.size());
    for (size_t sample_index = 0;
         sample_index < sample_arcs.size(); ++sample_index) {
        const float arc = sample_arcs[sample_index];
        const float slope = pose_slopes[sample_index];
        WeldFeaturePoint feature = {};
        feature.local = sampled_locals[sample_index];
        feature.world = localToWorld(
            feature.local.x(), feature.local.y(), feature.local.z(),
            n_bottom, d_bottom, n_side, d_side, n_tangent);
        feature.is_transition_point = false;
        feature.is_start_transition = false;
        feature.protruding = feature_params.protruding_is_larger_local_y
            ? feature.local.y() >= depth_split : feature.local.y() <= depth_split;
        feature.measured_on_arc = true;
        feature.distance_to_ideal = 0.0f;
        feature.merged_detection_count = 1;
        feature.left_line_slope = slope;
        feature.right_line_slope = slope;
        feature.ideal_local = feature.local;
        feature.left_segment = classifyProfileSlope(slope, feature_params);
        feature.right_segment = feature.left_segment;
        feature.topology_state = CORNER_TOPOLOGY_UNKNOWN;
        feature.detection_score = interpolateProfileScalar(profile, arc, true);
        feature.topology_inferred = false;
        feature.is_contour_sample = true;
        weld_points.push_back(feature);
    }
    if (weld_points.size() < 2) {
        error = "adaptive contour: fewer than two weld samples were generated";
        return result;
    }

    result.reserve(weld_points.size() + 2);
    result.push_back(makeSafeTransitionPoint(
        weld_points.front(), true,
        feature_params.safe_transition_offset_y,
        feature_params.safe_transition_offset_z,
        n_bottom, d_bottom, n_side, d_side, n_tangent));
    result.insert(result.end(), weld_points.begin(), weld_points.end());
    result.push_back(makeSafeTransitionPoint(
        weld_points.back(), false,
        feature_params.safe_transition_offset_y,
        feature_params.safe_transition_offset_z,
        n_bottom, d_bottom, n_side, d_side, n_tangent));

    std::cout << "Adaptive contour profile: red_points=" << samples.size()
        << ", bins=" << bins.size()
        << ", length=" << profile.back().arc_length << " mm"
        << ", curvature_bins=" << curvature_bin_count
        << ", largest_gap=" << largest_gap << " mm"
        << ", spacing_scale=" << spacing_scale
        << ", tangent_step_raw_max=" << raw_max_tangent_step_deg << " deg"
        << ", tangent_step_smoothed_max=" << smoothed_max_tangent_step_deg << " deg"
        << ", weld_samples=" << weld_points.size()
        << ", total_path_points=" << result.size() << std::endl;
    return result;
}

static const char* segmentTypeName(SeamSegmentType type)
{
    if (type == SEGMENT_FLAT) return "flat";
    if (type == SEGMENT_DIAGONAL_POSITIVE) return "left_waist";
    if (type == SEGMENT_DIAGONAL_NEGATIVE) return "right_waist";
    return "unknown";
}

static const char* torchPoseGroupName(TorchPoseGroup group)
{
    if (group == TORCH_PROTRUDING_LEFT) return "protruding_left";
    if (group == TORCH_PROTRUDING_RIGHT) return "protruding_right";
    if (group == TORCH_RECESSED_LEFT) return "recessed_left";
    return "recessed_right";
}

static bool validateTorchOrientationParams(const TorchOrientationParams& params)
{
    const float work_angles[4] = {
        params.protruding_left_work_angle_deg,
        params.protruding_right_work_angle_deg,
        params.recessed_left_work_angle_deg,
        params.recessed_right_work_angle_deg
    };
    const float lead_angles[4] = {
        params.protruding_left_lead_angle_deg,
        params.protruding_right_lead_angle_deg,
        params.recessed_left_lead_angle_deg,
        params.recessed_right_lead_angle_deg
    };
    for (int i = 0; i < 4; ++i) {
        // 0°会使枪体贴平底板，90°完全竖直；均不作为本算法的有效调节范围端点。
        if (!std::isfinite(work_angles[i]) ||
            work_angles[i] <= 0.0f || work_angles[i] >= 90.0f) {
            return false;
        }
        // 避免工具轴几乎与行进方向重合而导致工具横滚方向退化。
        if (!std::isfinite(lead_angles[i]) || std::abs(lead_angles[i]) >= 85.0f) {
            return false;
        }
    }
    if (params.tool_x_reference != "workpiece_x" &&
        params.tool_x_reference != "corner_bisector") {
        return false;
    }
    return std::isfinite(params.visualization_body_axis_length_mm) &&
        params.visualization_body_axis_length_mm >= 0.0f;
}

static TorchPoseGroup classifyTorchPoseGroup(const WeldFeaturePoint& feature)
{
    const bool has_left_waist =
        feature.left_segment == SEGMENT_DIAGONAL_POSITIVE ||
        feature.right_segment == SEGMENT_DIAGONAL_POSITIVE;
    const bool has_right_waist =
        feature.left_segment == SEGMENT_DIAGONAL_NEGATIVE ||
        feature.right_segment == SEGMENT_DIAGONAL_NEGATIVE;

    bool uses_left_waist = false;
    if (has_left_waist != has_right_waist) {
        uses_left_waist = has_left_waist;
    } else {
        // 极端孔洞可能让两条腰直接相邻，或标签暂时未知；此时使用拟合斜率
        // 绝对值更大的一侧决定左右组，保证分类仍然确定且可复现。
        const float representative_slope =
            std::abs(feature.left_line_slope) >= std::abs(feature.right_line_slope)
            ? feature.left_line_slope : feature.right_line_slope;
        uses_left_waist = representative_slope >= 0.0f;
    }

    if (feature.protruding) {
        return uses_left_waist ? TORCH_PROTRUDING_LEFT : TORCH_PROTRUDING_RIGHT;
    }
    return uses_left_waist ? TORCH_RECESSED_LEFT : TORCH_RECESSED_RIGHT;
}

static void getTorchGroupAngles(
    TorchPoseGroup group,
    const TorchOrientationParams& params,
    float& work_angle_deg,
    float& lead_angle_deg)
{
    if (group == TORCH_PROTRUDING_LEFT) {
        work_angle_deg = params.protruding_left_work_angle_deg;
        lead_angle_deg = params.protruding_left_lead_angle_deg;
    } else if (group == TORCH_PROTRUDING_RIGHT) {
        work_angle_deg = params.protruding_right_work_angle_deg;
        lead_angle_deg = params.protruding_right_lead_angle_deg;
    } else if (group == TORCH_RECESSED_LEFT) {
        work_angle_deg = params.recessed_left_work_angle_deg;
        lead_angle_deg = params.recessed_left_lead_angle_deg;
    } else {
        work_angle_deg = params.recessed_right_work_angle_deg;
        lead_angle_deg = params.recessed_right_lead_angle_deg;
    }
}

static WeldPoseData makeTorchPoseForCorner(
    const WeldFeaturePoint& feature,
    const Eigen::Vector3f& n_bottom,
    const Eigen::Vector3f& n_side,
    const Eigen::Vector3f& n_tangent,
    const TorchOrientationParams& params)
{
    const TorchPoseGroup group = classifyTorchPoseGroup(feature);
    float work_angle_deg = 45.0f;
    float lead_angle_deg = 0.0f;
    getTorchGroupAngles(group, params, work_angle_deg, lead_angle_deg);

    // 每条二维轮廓直线 y=m*x+b 在底板平面内的切向为 X+mY；
    // mX-Y 与它垂直且恒指向较小 local_y，即 L 侧板所在的开放空间。
    Eigen::Vector3f left_travel =
        n_tangent + feature.left_line_slope * n_side;
    Eigen::Vector3f right_travel =
        n_tangent + feature.right_line_slope * n_side;
    if (!left_travel.allFinite() || left_travel.norm() < 1e-6f) left_travel = n_tangent;
    if (!right_travel.allFinite() || right_travel.norm() < 1e-6f) right_travel = n_tangent;
    left_travel.normalize();
    right_travel.normalize();

    Eigen::Vector3f left_clearance =
        feature.left_line_slope * n_tangent - n_side;
    Eigen::Vector3f right_clearance =
        feature.right_line_slope * n_tangent - n_side;
    if (!left_clearance.allFinite() || left_clearance.norm() < 1e-6f) {
        left_clearance = -n_side;
    }
    if (!right_clearance.allFinite() || right_clearance.norm() < 1e-6f) {
        right_clearance = -n_side;
    }
    left_clearance.normalize();
    right_clearance.normalize();

    // 相邻两段切向和开放侧法向分别取角平分线；这使枪体在凹角处倾向离开
    // 波纹板两侧，而不是朝某一侧板面钻入。
    Eigen::Vector3f travel = left_travel + right_travel;
    if (!travel.allFinite() || travel.norm() < 1e-6f) travel = n_tangent;
    travel.normalize();
    Eigen::Vector3f clearance = left_clearance + right_clearance;
    if (!clearance.allFinite() || clearance.norm() < 1e-6f) clearance = -n_side;
    clearance.normalize();

    // 数值正交化：clearance 指向开放侧，travel 指向 CSV 的下一点方向。
    travel -= clearance * clearance.dot(travel);
    if (!travel.allFinite() || travel.norm() < 1e-6f) {
        travel = n_bottom.cross(clearance);
        if (travel.dot(n_tangent) < 0.0f) travel = -travel;
    }
    travel.normalize();

    const float degrees_to_radians =
        static_cast<float>(3.14159265358979323846 / 180.0);
    const float work_angle = work_angle_deg * degrees_to_radians;
    const float lead_angle = lead_angle_deg * degrees_to_radians;
    Eigen::Vector3f horizontal_direction =
        std::cos(lead_angle) * clearance + std::sin(lead_angle) * travel;
    horizontal_direction.normalize();

    // 始终记录 TCP -> 枪体的物理中心轴，便于在 CSV 中直接检查避让方向。
    Eigen::Vector3f torch_body_axis =
        std::cos(work_angle) * horizontal_direction +
        std::sin(work_angle) * n_bottom;
    torch_body_axis.normalize();

    Eigen::Vector3f tool_z = params.tool_positive_z_points_from_tcp_to_body
        ? torch_body_axis : -torch_body_axis;
    tool_z.normalize();
    // Tool Z 保持由工作角、前倾角和开放侧避让方向决定。Tool X 默认以固定的
    // 工件 +/-X 为参考；因为旋转矩阵必须正交，先把参考方向投影到 Tool Z 的
    // 法平面。仅当参考方向恰好与 Tool Z 垂直时，最终 Tool X 才与工件 X 完全重合。
    Eigen::Vector3f tool_x_reference = travel;
    if (params.tool_x_reference == "workpiece_x") {
        tool_x_reference = params.tool_x_points_along_positive_workpiece_x
            ? n_tangent : -n_tangent;
    }
    Eigen::Vector3f tool_x =
        tool_x_reference - tool_z * tool_z.dot(tool_x_reference);
    if (!tool_x.allFinite() || tool_x.norm() < 1e-6f) {
        // 参考 X 与 Tool Z 极端共线时退回旧切向，避免生成无效姿态。
        tool_x = travel - tool_z * tool_z.dot(travel);
    }
    if (!tool_x.allFinite() || tool_x.norm() < 1e-6f) {
        tool_x = clearance - tool_z * tool_z.dot(clearance);
    }
    if (!tool_x.allFinite() || tool_x.norm() < 1e-6f) {
        tool_x = tool_z.unitOrthogonal();
    }
    tool_x.normalize();
    Eigen::Vector3f tool_y = tool_z.cross(tool_x);
    if (!tool_y.allFinite() || tool_y.norm() < 1e-6f) {
        tool_y = tool_z.unitOrthogonal();
    }
    tool_y.normalize();
    tool_x = tool_y.cross(tool_z).normalized();
    tool_y = tool_z.cross(tool_x).normalized();

    // 旋转矩阵的三列是在 PLY 世界坐标系中表示的工具 +X/+Y/+Z 轴。
    Eigen::Matrix3f world_from_tool;
    world_from_tool.col(0) = tool_x;
    world_from_tool.col(1) = tool_y;
    world_from_tool.col(2) = tool_z;
    Eigen::Quaternionf quaternion(world_from_tool);
    quaternion.normalize();

    // 额外生成相对于工件的姿态。这里必须使用右手基：X=焊缝前进、Y=朝L侧板、Z=向上。
    // 内部轮廓 local_y 为朝波纹内部正方向，因此工件 +Y 正好是 -N_side。
    Eigen::Matrix3f world_from_workpiece;
    world_from_workpiece.col(0) = n_tangent;
    world_from_workpiece.col(1) = -n_side;
    world_from_workpiece.col(2) = n_bottom;
    Eigen::Quaternionf world_from_workpiece_quaternion(world_from_workpiece);
    world_from_workpiece_quaternion.normalize();
    Eigen::Quaternionf workpiece_from_tool =
        world_from_workpiece_quaternion.conjugate() * quaternion;
    workpiece_from_tool.normalize();

    WeldPoseData pose = {};
    pose.qw = quaternion.w();
    pose.qx = quaternion.x();
    pose.qy = quaternion.y();
    pose.qz = quaternion.z();
    pose.workpiece_qw = workpiece_from_tool.w();
    pose.workpiece_qx = workpiece_from_tool.x();
    pose.workpiece_qy = workpiece_from_tool.y();
    pose.workpiece_qz = workpiece_from_tool.z();
    pose.torch_body_axis_x = torch_body_axis.x();
    pose.torch_body_axis_y = torch_body_axis.y();
    pose.torch_body_axis_z = torch_body_axis.z();
    pose.tool_x_axis_x = tool_x.x();
    pose.tool_x_axis_y = tool_x.y();
    pose.tool_x_axis_z = tool_x.z();
    pose.work_angle_deg = work_angle_deg;
    pose.lead_angle_deg = lead_angle_deg;
    pose.group = group;
    pose.inherited_from_nearest_corner = false;
    pose.default_without_corner = false;
    return pose;
}

static std::vector<WeldPoseData> buildOrderedWeldPoses(
    const std::vector<WeldFeaturePoint>& features,
    const Eigen::Vector3f& n_bottom,
    const Eigen::Vector3f& n_side,
    const Eigen::Vector3f& n_tangent,
    const TorchOrientationParams& params)
{
    std::vector<WeldPoseData> poses(features.size());
    std::vector<bool> assigned(features.size(), false);
    std::vector<size_t> corner_indices;
    corner_indices.reserve(features.size());

    for (size_t i = 0; i < features.size(); ++i) {
        if (features[i].is_transition_point) continue;
        poses[i] = makeTorchPoseForCorner(
            features[i], n_bottom, n_side, n_tangent, params);
        assigned[i] = true;
        corner_indices.push_back(i);
    }

    for (size_t i = 0; i < features.size(); ++i) {
        if (assigned[i]) continue;
        if (!corner_indices.empty()) {
            size_t nearest_corner = corner_indices.front();
            if (features[i].is_transition_point) {
                // 安全点由首/末焊接点生成，必须继承该相邻点姿态。按三维距离
                // 搜索可能在波纹折返处误选另一条近邻边，造成无意义姿态跳变。
                nearest_corner = features[i].is_start_transition
                    ? corner_indices.front() : corner_indices.back();
            } else {
                float nearest_distance_squared = std::numeric_limits<float>::max();
                for (size_t j = 0; j < corner_indices.size(); ++j) {
                    const size_t candidate = corner_indices[j];
                    const Eigen::Vector2f delta =
                        features[candidate].local.head<2>() - features[i].local.head<2>();
                    if (delta.squaredNorm() < nearest_distance_squared) {
                        nearest_distance_squared = delta.squaredNorm();
                        nearest_corner = candidate;
                    }
                }
            }
            poses[i] = poses[nearest_corner];
            poses[i].inherited_from_nearest_corner = true;
        } else {
            // 防御性回退：若上游异常地只给出过渡点，则使用开放侧默认45°姿态。
            WeldFeaturePoint fallback = features[i];
            fallback.protruding = true;
            fallback.left_line_slope = 0.0f;
            fallback.right_line_slope = 0.0f;
            fallback.left_segment = SEGMENT_FLAT;
            fallback.right_segment = SEGMENT_DIAGONAL_POSITIVE;
            poses[i] = makeTorchPoseForCorner(
                fallback, n_bottom, n_side, n_tangent, params);
            poses[i].default_without_corner = true;
        }
        assigned[i] = true;
    }

    // q 与 -q 表示同一个旋转。强制相邻四元数位于同一半球，避免控制器按数值
    // 读取 CSV 时出现无意义的符号翻转；真实平滑插补仍由机器人控制器完成。
    for (size_t i = 1; i < poses.size(); ++i) {
        const float quaternion_dot =
            poses[i - 1].qw * poses[i].qw + poses[i - 1].qx * poses[i].qx +
            poses[i - 1].qy * poses[i].qy + poses[i - 1].qz * poses[i].qz;
        if (quaternion_dot < 0.0f) {
            poses[i].qw = -poses[i].qw;
            poses[i].qx = -poses[i].qx;
            poses[i].qy = -poses[i].qy;
            poses[i].qz = -poses[i].qz;
            poses[i].workpiece_qw = -poses[i].workpiece_qw;
            poses[i].workpiece_qx = -poses[i].workpiece_qx;
            poses[i].workpiece_qy = -poses[i].workpiece_qy;
            poses[i].workpiece_qz = -poses[i].workpiece_qz;
        }
    }
    return poses;
}

static bool isFiniteWorkpieceOffset(const WorkpieceOffset3f& offset)
{
    return std::isfinite(offset.x) &&
        std::isfinite(offset.y) && std::isfinite(offset.z);
}

static bool validateFeaturePositionOffsets(
    const FeaturePositionOffsetParams& params)
{
    return isFiniteWorkpieceOffset(params.start_transition) &&
        isFiniteWorkpieceOffset(params.end_transition) &&
        isFiniteWorkpieceOffset(params.protruding_left) &&
        isFiniteWorkpieceOffset(params.protruding_right) &&
        isFiniteWorkpieceOffset(params.recessed_left) &&
        isFiniteWorkpieceOffset(params.recessed_right) &&
        isFiniteWorkpieceOffset(params.adaptive_contour);
}

static bool validateFeatureExtractionParams(
    const FeatureExtractionParams& params,
    std::string& error)
{
    auto fail = [&error](const char* message) {
        error = message;
        return false;
    };
    if (!std::isfinite(params.profile_bin_width) ||
        params.profile_bin_width <= 0.0f) {
        return fail("feature.profile_bin_width must be finite and > 0.");
    }
    if (!std::isfinite(params.flat_normal_dot_min) ||
        params.flat_normal_dot_min < 0.0f || params.flat_normal_dot_min > 1.0f ||
        !std::isfinite(params.diagonal_normal_dot_min) ||
        params.diagonal_normal_dot_min < 0.0f ||
        params.diagonal_normal_dot_min > 1.0f) {
        return fail("Feature normal dot thresholds must be within [0, 1].");
    }
    if (!std::isfinite(params.bin_label_vote_ratio) ||
        params.bin_label_vote_ratio <= 0.0f ||
        params.bin_label_vote_ratio > 1.0f) {
        return fail("feature.bin_label_vote_ratio must be within (0, 1].");
    }
    if (params.min_segment_bins < 2 ||
        params.local_fit_half_window_bins < 1) {
        return fail("Feature bin/window counts are too small.");
    }

    const float strictly_positive[] = {
        params.min_segment_span,
        params.local_fit_radius,
        params.local_line_max_median_residual,
        params.flat_segment_max_length,
        params.waist_segment_max_length,
        params.duplicate_corner_merge_distance,
        params.corner_fit_support_distance,
        params.recessed_chord_search_radius,
        params.recessed_arc_search_radius
    };
    for (float value : strictly_positive) {
        if (!std::isfinite(value) || value <= 0.0f) {
            return fail("Feature distances/radii must be finite and > 0.");
        }
    }
    const float non_negative[] = {
        params.flat_profile_slope_max,
        params.max_hole_bridge,
        params.max_corner_extrapolation,
        params.low_z_xy_tolerance,
        params.visualization_marker_radius,
        params.safe_transition_offset_y,
        params.safe_transition_offset_z
    };
    for (float value : non_negative) {
        if (!std::isfinite(value) || value < 0.0f) {
            return fail("Feature tolerances/visualization sizes must be finite and >= 0.");
        }
    }
    if (params.safe_transition_offset_y <= 0.0f &&
        params.safe_transition_offset_z <= 0.0f) {
        return fail("At least one safe transition Y/Z offset must be > 0 mm.");
    }
    if (!std::isfinite(params.min_corner_angle_deg) ||
        params.min_corner_angle_deg <= 0.0f ||
        params.min_corner_angle_deg >= 90.0f) {
        return fail("feature.min_corner_angle_deg must be within (0, 90).");
    }
    if (!std::isfinite(params.bottom_z_quantile) ||
        params.bottom_z_quantile < 0.0f || params.bottom_z_quantile > 1.0f) {
        return fail("feature.bottom_z_quantile must be within [0, 1].");
    }
    return true;
}

static bool validateAdaptiveContourParams(
    const AdaptiveContourParams& params,
    std::string& error)
{
    auto fail = [&error](const char* message) {
        error = message;
        return false;
    };
    if (params.mode != "feature_points" &&
        params.mode != "adaptive_contour") {
        return fail("path.mode must be feature_points or adaptive_contour.");
    }
    if (!std::isfinite(params.straight_spacing) ||
        !std::isfinite(params.corner_spacing) ||
        params.straight_spacing <= 0.0f || params.corner_spacing <= 0.0f ||
        params.corner_spacing > params.straight_spacing) {
        return fail("path spacing must satisfy 0 < corner_spacing <= straight_spacing.");
    }
    if (!std::isfinite(params.corner_influence_radius) ||
        !std::isfinite(params.curvature_window) ||
        params.corner_influence_radius < 0.0f ||
        params.curvature_window <= 0.0f) {
        return fail("path corner influence/window values are invalid.");
    }
    if (!std::isfinite(params.curvature_threshold_deg) ||
        params.curvature_threshold_deg <= 0.0f ||
        params.curvature_threshold_deg >= 180.0f) {
        return fail("path.curvature_threshold_deg must be within (0, 180).");
    }
    if (!std::isfinite(params.orientation_smoothing_radius) ||
        params.orientation_smoothing_radius < 0.0f) {
        return fail("path.orientation_smoothing_radius must be finite and >= 0.");
    }
    if (!std::isfinite(params.max_orientation_step_deg) ||
        params.max_orientation_step_deg <= 0.0f ||
        params.max_orientation_step_deg >= 90.0f) {
        return fail("path.max_orientation_step_deg must be within (0, 90).");
    }
    if (params.smoothing_half_window_bins < 1 ||
        params.smoothing_half_window_bins > 100) {
        return fail("path.smoothing_half_window_bins must be within [1, 100].");
    }
    if (!std::isfinite(params.outlier_max_distance) ||
        params.outlier_max_distance <= 0.0f ||
        !std::isfinite(params.max_bridge_gap) ||
        params.max_bridge_gap <= 0.0f) {
        return fail("path outlier/gap limits must be finite and > 0.");
    }
    if (params.max_points < 4 || params.max_points > 100) {
        return fail("path.max_points must be within [4, 100] including transitions.");
    }
    if (!std::isfinite(params.work_angle_deg) ||
        params.work_angle_deg <= 0.0f || params.work_angle_deg >= 90.0f ||
        !std::isfinite(params.lead_angle_deg) ||
        std::abs(params.lead_angle_deg) >= 85.0f) {
        return fail("adaptive path work/lead angles are outside the safe numeric range.");
    }
    return true;
}

static WorkpieceOffset3f selectFeaturePositionOffset(
    const WeldFeaturePoint& feature,
    const WeldPoseData& pose,
    const FeaturePositionOffsetParams& params)
{
    WorkpieceOffset3f offset;
    if (feature.is_contour_sample) offset = params.adaptive_contour;
    else if (pose.group == TORCH_PROTRUDING_LEFT) offset = params.protruding_left;
    else if (pose.group == TORCH_PROTRUDING_RIGHT) offset = params.protruding_right;
    else if (pose.group == TORCH_RECESSED_LEFT) offset = params.recessed_left;
    else offset = params.recessed_right;

    if (feature.is_transition_point) {
        // 安全点先继承相邻真实角点的四类偏置，再叠加首/末微调。这样当
        // transition.x=0 时，即使角点组设置了 X 偏置，两者 X 仍严格相同。
        const WorkpieceOffset3f& fine_adjustment = feature.is_start_transition
            ? params.start_transition : params.end_transition;
        offset.x += fine_adjustment.x;
        offset.y += fine_adjustment.y;
        offset.z += fine_adjustment.z;
    }
    return offset;
}

static const char* featurePositionOffsetGroupName(
    const WeldFeaturePoint& feature,
    const WeldPoseData& pose)
{
    if (feature.is_transition_point) {
        return feature.is_start_transition ? "start_transition" : "end_transition";
    }
    if (feature.is_contour_sample) return "adaptive_contour";
    return torchPoseGroupName(pose.group);
}

static bool applyFeaturePositionOffsets(
    std::vector<WeldFeaturePoint>& features,
    const std::vector<WeldPoseData>& poses,
    const Eigen::Vector3f& n_bottom,
    const Eigen::Vector3f& n_side,
    const Eigen::Vector3f& n_tangent,
    const FeaturePositionOffsetParams& params)
{
    if (features.size() != poses.size()) return false;
    for (size_t i = 0; i < features.size(); ++i) {
        const WorkpieceOffset3f offset =
            selectFeaturePositionOffset(features[i], poses[i], params);

        // 右手工件坐标：+X=N_tangent，+Y=-N_side（朝L侧板），+Z=N_bottom。
        features[i].world += offset.x * n_tangent -
            offset.y * n_side + offset.z * n_bottom;

        // 内部 local_y 的正方向与工件 +Y 相反，因此这里需要减去 offset.y。
        features[i].local.x() += offset.x;
        features[i].local.y() -= offset.y;
        features[i].local.z() += offset.z;
    }
    return true;
}

static float quaternionAngularDistanceDeg(
    const WeldPoseData& first,
    const WeldPoseData& second)
{
    float dot = first.qw * second.qw + first.qx * second.qx +
        first.qy * second.qy + first.qz * second.qz;
    // 绝对值同时兼容 q/-q；2*acos(|dot|) 是两姿态间的最短旋转角。
    dot = std::max(0.0f, std::min(1.0f, std::abs(dot)));
    return 2.0f * std::acos(dot) *
        static_cast<float>(180.0 / 3.14159265358979323846);
}

static bool saveFeatureCsv(
    const std::string& filename,
    const std::vector<WeldFeaturePoint>& features,
    const std::vector<WeldPoseData>& poses,
    const TorchOrientationParams& torch_params,
    const FeaturePositionOffsetParams& position_offsets,
    float workpiece_x_origin_projection)
{
    if (poses.size() != features.size()) {
        std::cerr << "Cannot write " << filename
            << ": feature/pose count mismatch." << std::endl;
        return false;
    }
    std::ofstream output(filename.c_str());
    if (!output) {
        std::cerr << "Cannot write " << filename << std::endl;
        return false;
    }
    output << "order,x,y,z,workpiece_x,workpiece_y,workpiece_z,"
              "qw,qx,qy,qz,workpiece_qw,workpiece_qx,workpiece_qy,workpiece_qz,"
              "pose_group,pose_source,"
              "work_angle_deg,lead_angle_deg,orientation_delta_from_previous_deg,"
              "world_torch_body_axis_x,world_torch_body_axis_y,"
              "world_torch_body_axis_z,tool_positive_z_direction,"
              "weld_enabled,position_offset_group,"
              "applied_offset_workpiece_x,applied_offset_workpiece_y,"
              "applied_offset_workpiece_z,feature_type,point_source,"
              "distance_to_ideal,merged_detection_count,"
              "left_segment,right_segment,"
              "world_tool_x_axis_x,world_tool_x_axis_y,world_tool_x_axis_z,"
              "tool_x_reference,tool_x_workpiece_direction\n";
    output << std::fixed << std::setprecision(6);
    for (size_t i = 0; i < features.size(); ++i) {
        const WeldFeaturePoint& feature = features[i];
        const WeldPoseData& pose = poses[i];
        const WorkpieceOffset3f applied_offset =
            selectFeaturePositionOffset(feature, pose, position_offsets);
        std::string feature_type;
        std::string point_source;
        if (feature.is_transition_point) {
            feature_type = feature.is_start_transition
                ? "start_transition" : "end_transition";
            point_source = "generated_safe_transition";
        } else if (feature.is_contour_sample) {
            feature_type = "adaptive_contour_point";
            point_source = "robust_profile_adaptive_sampling";
        } else {
            // 与四组姿态/位置偏置使用完全相同的物理分类名称，机械臂端无需
            // 再根据凹凸和腰线方向进行二次推断。
            feature_type = std::string(torchPoseGroupName(pose.group)) + "_corner";
            if (feature.topology_inferred) {
                point_source = "in_frame_periodic_boundary_completion";
            } else if (feature.protruding) {
                point_source = feature.merged_detection_count > 1
                    ? "merged_ideal_intersections" : "ideal_line_intersection";
            } else {
                point_source = feature.measured_on_arc
                    ? "measured_arc_near_chord" : "safe_chord_fallback";
            }
        }
        const char* pose_source = pose.default_without_corner
            ? "default_no_corner"
            : (pose.inherited_from_nearest_corner
                ? "inherited_nearest_corner"
                : (feature.is_contour_sample
                    ? "local_contour_tangent" : "corner_geometry"));
        const float orientation_delta = i == 0
            ? 0.0f : quaternionAngularDistanceDeg(poses[i - 1], pose);
        output << i << ','
            << feature.world.x() << ',' << feature.world.y() << ',' << feature.world.z() << ','
            << feature.local.x() - workpiece_x_origin_projection << ','
            << -feature.local.y() << ',' << feature.local.z() << ','
            << pose.qw << ',' << pose.qx << ',' << pose.qy << ',' << pose.qz << ','
            << pose.workpiece_qw << ',' << pose.workpiece_qx << ','
            << pose.workpiece_qy << ',' << pose.workpiece_qz << ','
            << torchPoseGroupName(pose.group) << ',' << pose_source << ','
            << pose.work_angle_deg << ',' << pose.lead_angle_deg << ','
            << orientation_delta << ','
            << pose.torch_body_axis_x << ',' << pose.torch_body_axis_y << ','
            << pose.torch_body_axis_z << ','
            << (torch_params.tool_positive_z_points_from_tcp_to_body
                ? "tcp_to_body" : "body_to_tcp") << ','
            << (feature.is_transition_point ? 0 : 1) << ','
            << featurePositionOffsetGroupName(feature, pose) << ','
            << applied_offset.x << ',' << applied_offset.y << ',' << applied_offset.z << ','
            << feature_type << ',' << point_source << ','
            << feature.distance_to_ideal << ',' << feature.merged_detection_count << ','
            << segmentTypeName(feature.left_segment) << ','
            << segmentTypeName(feature.right_segment) << ','
            << pose.tool_x_axis_x << ',' << pose.tool_x_axis_y << ','
            << pose.tool_x_axis_z << ',' << torch_params.tool_x_reference << ',';
        if (torch_params.tool_x_reference == "workpiece_x") {
            output << (torch_params.tool_x_points_along_positive_workpiece_x
                ? "positive_x" : "negative_x");
        } else {
            output << "not_applicable";
        }
        output << '\n';
    }
    output.flush();
    return output.good();
}

static pcl::PointCloud<PointOutT>::Ptr makeExactFeatureCloud(
    const std::vector<WeldFeaturePoint>& features)
{
    pcl::PointCloud<PointOutT>::Ptr cloud(new pcl::PointCloud<PointOutT>);
    cloud->points.reserve(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        PointOutT point;
        point.x = features[i].world.x();
        point.y = features[i].world.y();
        point.z = features[i].world.z();
        point.r = 255; point.g = 20; point.b = 147; // 粉红：安全过渡点及四类折线拐点
        cloud->points.push_back(point);
    }
    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

static bool saveExactFeaturePly(
    const std::string& filename,
    const pcl::PointCloud<PointOutT>& cloud)
{
    if (!cloud.empty()) {
        return pcl::io::savePLYFileBinary(filename, cloud) == 0;
    }

    // 视野中没有完整的四类物理拐角时，空结果也是有效结果。
    // 仍写出标准 0 顶点 PLY，使 SDK 调用方不必猜测文件是否应存在。
    std::ofstream output(filename.c_str(), std::ios::binary);
    if (!output) return false;
    output << "ply\n"
              "format ascii 1.0\n"
              "element vertex 0\n"
              "property float x\n"
              "property float y\n"
              "property float z\n"
              "property uchar red\n"
              "property uchar green\n"
              "property uchar blue\n"
              "end_header\n";
    output.flush();
    return output.good();
}

static void appendFeatureMarkers(
    pcl::PointCloud<PointOutT>& cloud,
    const std::vector<WeldFeaturePoint>& features,
    const Eigen::Vector3f& n_bottom,
    const Eigen::Vector3f& n_side,
    const Eigen::Vector3f& n_tangent,
    float radius)
{
    // 三条正交直线组成放大的粉红色三维十字星。三线交点严格等于 CSV 坐标，
    // 不再使用会遮挡中心位置的实心点球。
    const Eigen::Vector3f axes[3] = {n_tangent, n_side, n_bottom};
    for (size_t i = 0; i < features.size(); ++i) {
        const float marker_radius = radius;
        const float step = std::max(0.35f, marker_radius / 16.0f);
        PointOutT center;
        center.x = features[i].world.x();
        center.y = features[i].world.y();
        center.z = features[i].world.z();
        center.r = 255; center.g = 20; center.b = 147;
        cloud.points.push_back(center);

        for (int axis = 0; axis < 3; ++axis) {
            for (float offset = step;
                offset <= marker_radius + 1e-4f; offset += step) {
                for (int sign = -1; sign <= 1; sign += 2) {
                    const Eigen::Vector3f world = features[i].world +
                        static_cast<float>(sign) * offset * axes[axis];
                    PointOutT marker;
                    marker.x = world.x(); marker.y = world.y(); marker.z = world.z();
                    marker.r = 255; marker.g = 20; marker.b = 147;
                    cloud.points.push_back(marker);
                }
            }
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;
}

static void appendTorchBodyAxisMarkers(
    pcl::PointCloud<PointOutT>& cloud,
    const std::vector<WeldFeaturePoint>& features,
    const std::vector<WeldPoseData>& poses,
    float axis_length_mm)
{
    if (axis_length_mm <= 0.0f || features.size() != poses.size()) return;
    const float step = std::max(0.5f, axis_length_mm / 40.0f);
    for (size_t i = 0; i < features.size(); ++i) {
        Eigen::Vector3f body_axis(
            poses[i].torch_body_axis_x,
            poses[i].torch_body_axis_y,
            poses[i].torch_body_axis_z);
        if (!body_axis.allFinite() || body_axis.norm() < 1e-6f) continue;
        body_axis.normalize();
        for (float distance = step;
            distance <= axis_length_mm + 1e-4f; distance += step) {
            const Eigen::Vector3f world = features[i].world + distance * body_axis;
            PointOutT marker;
            marker.x = world.x(); marker.y = world.y(); marker.z = world.z();
            marker.r = 255; marker.g = 255; marker.b = 255; // 白色：TCP -> 焊枪枪体
            cloud.points.push_back(marker);
        }
    }
    cloud.width = static_cast<uint32_t>(cloud.points.size());
    cloud.height = 1;
    cloud.is_dense = true;
}

namespace {

struct RuntimeParameterStore {
    std::map<std::string, std::string> values;
    std::set<std::string> consumed;
};

static std::string trimText(const std::string& input)
{
    size_t first = 0;
    while (first < input.size() &&
           std::isspace(static_cast<unsigned char>(input[first]))) ++first;
    size_t last = input.size();
    while (last > first &&
           std::isspace(static_cast<unsigned char>(input[last - 1]))) --last;
    return input.substr(first, last - first);
}

static bool insertKeyValue(
    const std::string& expression,
    RuntimeParameterStore& store,
    std::string& error)
{
    const size_t equals = expression.find('=');
    if (equals == std::string::npos) {
        error = "Expected key=value, got: " + expression;
        return false;
    }
    const std::string key = trimText(expression.substr(0, equals));
    const std::string value = trimText(expression.substr(equals + 1));
    if (key.empty() || value.empty()) {
        error = "Parameter key and value must not be empty: " + expression;
        return false;
    }
    store.values[key] = value;
    return true;
}

static bool loadRuntimeParameters(
    const weld_seam_sdk::RunOptions& options,
    RuntimeParameterStore& store,
    std::string& error)
{
    if (!options.config_file.empty()) {
        std::ifstream input(options.config_file.c_str());
        if (!input) {
            error = "Cannot open config file: " + options.config_file;
            return false;
        }
        std::string line;
        size_t line_number = 0;
        while (std::getline(input, line)) {
            ++line_number;
            line = trimText(line);
            if (line.empty() || line[0] == '#' || line[0] == ';') continue;
            if (!insertKeyValue(line, store, error)) {
                error = options.config_file + ":" +
                    std::to_string(line_number) + ": " + error;
                return false;
            }
        }
    }
    // 命令行/调用方覆盖始终最后写入，因此优先级最高。
    for (const std::string& expression : options.parameter_overrides) {
        if (!insertKeyValue(expression, store, error)) return false;
    }
    return true;
}

static bool parseBoolValue(const std::string& text, bool& value)
{
    std::string lower;
    lower.reserve(text.size());
    for (char ch : text) {
        lower.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(ch))));
    }
    if (lower == "true" || lower == "1" || lower == "yes" || lower == "on") {
        value = true;
        return true;
    }
    if (lower == "false" || lower == "0" || lower == "no" || lower == "off") {
        value = false;
        return true;
    }
    return false;
}

template <typename T>
static bool parseNumberValue(const std::string& text, T& value);

template <>
bool parseNumberValue<float>(const std::string& text, float& value)
{
    try {
        size_t used = 0;
        value = std::stof(text, &used);
        return used == text.size();
    } catch (...) {
        return false;
    }
}

template <>
bool parseNumberValue<int>(const std::string& text, int& value)
{
    try {
        size_t used = 0;
        value = std::stoi(text, &used);
        return used == text.size();
    } catch (...) {
        return false;
    }
}

template <>
bool parseNumberValue<unsigned int>(const std::string& text, unsigned int& value)
{
    try {
        size_t used = 0;
        const unsigned long parsed = std::stoul(text, &used);
        if (used != text.size() ||
            parsed > std::numeric_limits<unsigned int>::max()) return false;
        value = static_cast<unsigned int>(parsed);
        return true;
    } catch (...) {
        return false;
    }
}

template <typename T>
static bool applyNumberParameter(
    RuntimeParameterStore& store,
    const char* key,
    T& target,
    std::string& error)
{
    const auto it = store.values.find(key);
    if (it == store.values.end()) return true;
    T parsed{};
    if (!parseNumberValue<T>(it->second, parsed)) {
        error = std::string("Invalid numeric value for ") + key + ": " + it->second;
        return false;
    }
    target = parsed;
    store.consumed.insert(key);
    return true;
}

static bool applyBoolParameter(
    RuntimeParameterStore& store,
    const char* key,
    bool& target,
    std::string& error)
{
    const auto it = store.values.find(key);
    if (it == store.values.end()) return true;
    bool parsed = false;
    if (!parseBoolValue(it->second, parsed)) {
        error = std::string("Invalid boolean value for ") + key + ": " + it->second;
        return false;
    }
    target = parsed;
    store.consumed.insert(key);
    return true;
}

static bool applyStringParameter(
    RuntimeParameterStore& store,
    const char* key,
    std::string& target)
{
    const auto it = store.values.find(key);
    if (it == store.values.end()) return true;
    target = trimText(it->second);
    store.consumed.insert(key);
    return true;
}

static std::string lowerText(const std::string& text)
{
    std::string lower;
    lower.reserve(text.size());
    for (char ch : text) {
        lower.push_back(static_cast<char>(
            std::tolower(static_cast<unsigned char>(ch))));
    }
    return lower;
}

static bool cloudBlobHasField(
    const pcl::PCLPointCloud2& cloud,
    const char* field_name)
{
    for (const pcl::PCLPointField& field : cloud.fields) {
        if (field.name == field_name) return true;
    }
    return false;
}

static bool rejectUnknownParameters(
    const RuntimeParameterStore& store,
    std::string& error)
{
    for (const auto& item : store.values) {
        if (store.consumed.count(item.first) == 0) {
            error = "Unknown parameter: " + item.first;
            return false;
        }
    }
    return true;
}

static bool parseCommandLine(
    int argc,
    char** argv,
    weld_seam_sdk::RunOptions& options,
    bool& show_help,
    std::string& error)
{
    show_help = false;
    std::vector<std::string> positional;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "-h" || arg == "--help") {
            show_help = true;
            return true;
        }
        auto take_value = [&](const char* name, std::string& destination) -> bool {
            if (i + 1 >= argc) {
                error = std::string("Missing value after ") + name;
                return false;
            }
            destination = argv[++i];
            return true;
        };
        if (arg == "--output-dir") {
            if (!take_value("--output-dir", options.output_directory)) return false;
        } else if (arg == "--output-prefix") {
            if (!take_value("--output-prefix", options.output_prefix)) return false;
        } else if (arg == "--config") {
            if (!take_value("--config", options.config_file)) return false;
        } else if (arg == "--set") {
            std::string expression;
            if (!take_value("--set", expression)) return false;
            options.parameter_overrides.push_back(expression);
        } else if (arg.rfind("--set=", 0) == 0) {
            options.parameter_overrides.push_back(arg.substr(6));
        } else if (!arg.empty() && arg[0] == '-') {
            error = "Unknown option: " + arg;
            return false;
        } else {
            positional.push_back(arg);
        }
    }
    if (positional.empty()) {
        error = "Missing input PLY path.";
        return false;
    }
    if (positional.size() > 3) {
        error = "Too many positional arguments.";
        return false;
    }
    options.input_ply = positional[0];
    if (positional.size() >= 2) options.output_directory = positional[1];
    if (positional.size() >= 3) options.output_prefix = positional[2];
    return true;
}

}  // namespace

static int executeWeldSeamExtraction(
    const weld_seam_sdk::RunOptions& options,
    weld_seam_sdk::RunResult& run_result)
{
    run_result = weld_seam_sdk::RunResult();
    if (options.input_ply.empty()) {
        run_result.message = "Input PLY path is empty.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (options.output_prefix.empty()) {
        run_result.message = "Output prefix must not be empty.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    RuntimeParameterStore runtime_parameters;
    std::string parameter_error;
    if (!loadRuntimeParameters(
            options, runtime_parameters, parameter_error)) {
        run_result.message = parameter_error;
        std::cerr << parameter_error << std::endl;
        return -1;
    }
    const std::chrono::steady_clock::time_point total_start_time =
        std::chrono::steady_clock::now();
    // ====================================================================
    // [用户核心调节参数区] 
    // ====================================================================
    // 1. 原始点云 ROI（使用 PLY 文件自身的 XYZ 坐标系，单位通常为 mm）。
    // 默认 false：完全不裁剪，处理相机全部视野。需要提速时改为 true，并填写
    // 六个边界。ROI 必须同时保留焊缝、足够大的 L 底面和 L 侧面，否则平面识别会失败。
    // 若相机旋转导致 PLY 坐标轴随之变化，这六个固定边界也必须重新标定。
    bool enable_roi = false;
    float roi_min_x = -std::numeric_limits<float>::infinity();
    float roi_max_x =  std::numeric_limits<float>::infinity();
    // float roi_min_y =  90.0f;
    // float roi_max_y =  135.0f;
    float roi_min_y =  -90.0f;
    float roi_max_y =  -30.0f;
    float roi_min_z = -std::numeric_limits<float>::infinity();
    float roi_max_z =  std::numeric_limits<float>::infinity();

    // 2. 法向量策略：
    // auto（默认）= PLY 有完整且足够有效的 normals 时复用，否则自动重算；
    // recompute = 无条件在 ROI + 0.5mm 体素后重算，行为与上一稳定版一致；
    // reuse = 禁止重算，输入 normals 缺失/质量不足时直接报错。
    // 复用前后都会归一化，并按相机光心(PLY原点)统一朝向，避免左右腰符号翻转。
    std::string normal_mode = "auto";
    float normal_reuse_min_valid_ratio = 0.995f;
    // K=20 对当前点云密度通常较稳定；thread_count=0 让 PCL/OpenMP 自动选线程。
    int normal_k_neighbors = 20;
    unsigned int normal_thread_count = 0;

    // 3. 七组最终位置偏置（单位 mm），全部沿“工件右手坐标系”施加：
    // +X = 焊缝 CSV 前进方向；+Y = 朝 L 侧板/开放侧；+Z = 离开蓝色底板向上。
    // 偏置不参与焊缝提取和凹凸判断，只改变最终 CSV、精确点 PLY 与粉红十字位置。
    // 每组独立生效，不要求当前视野必须拍到一个完整的四角波纹周期。
    FeaturePositionOffsetParams position_offsets;

    // 首个真实特征点安全过渡点的二次微调；非零 X 会破坏默认“X相同”的关系。
    position_offsets.start_transition.x = 0.0f;
    position_offsets.start_transition.y = 0.0f;
    position_offsets.start_transition.z = 0.0f;

    // 末个真实特征点安全过渡点的二次微调；非零 X 会破坏默认“X相同”的关系。
    position_offsets.end_transition.x = 0.0f;
    position_offsets.end_transition.y = 0.0f;
    position_offsets.end_transition.z = 0.0f;

    // 中间四类物理拐点偏置；没有检测到的类别会自然跳过，不会补造点。
    position_offsets.protruding_left.x = 2.5f;
    position_offsets.protruding_left.y = 0.0f;
    position_offsets.protruding_left.z = 0.0f;
    position_offsets.protruding_right.x = -0.5f;
    position_offsets.protruding_right.y = 0.0f;
    position_offsets.protruding_right.z = 0.0f;
    position_offsets.recessed_left.x = 0.0f;
    position_offsets.recessed_left.y = 1.0f;
    position_offsets.recessed_left.z = 0.0f;
    position_offsets.recessed_right.x = 0.0f;
    position_offsets.recessed_right.y = 1.0f;
    position_offsets.recessed_right.z = 0.0f;
    // 连续轮廓模式默认不增加额外位置偏置，直接跟随鲁棒红色焊缝轮廓。
    position_offsets.adaptive_contour.x = 0.0f;
    position_offsets.adaptive_contour.y = 0.0f;
    position_offsets.adaptive_contour.z = 0.0f;

    // 4. Z向(高度)滤波阈值：L形底面高度为 0。剔除低于此阈值的候选点。
    // 直焊缝通常低于波纹焊缝。如果直焊缝有残留，可慢慢调大 (如 0.0f, 0.5f)
    float z_filter_threshold = 0.5f; 

    // 5. Y向(深度)滤波阈值：L形侧板深度为 0。剔除贴近侧板的候选点。
    // 波纹板与侧板有 3-4mm 间隙。设为 2.0f 意味着切除侧板向内 2mm 的区域。
    float y_filter_threshold = 2.0f;  

    // 6. 主平面搜索加速参数：只用 1.0mm 辅助点云做多轮 RANSAC，随后在
    // 原 0.5mm 点云上精修；辅助搜索失败会自动回退原全分辨率流程。
    float plane_search_leaf_size = 1.0f;
    int plane_search_min_inliers = 300;
    float side_origin_low_quantile = 0.002f;

    // 7. 焊缝二次拐点提取参数。L 侧板方向为物理凸出正方向，因而靠近
    // y_local=0 的较小 y 层属于凸角。凹角内部用两侧各 5mm 构造安全弦，
    // 但每个物理拐角最终仍只输出一个点。
    FeatureExtractionParams feature_params;
    feature_params.protruding_is_larger_local_y = false;
    feature_params.corner_fit_support_distance = 5.0f;
    // 首末安全点与焊接点工件 X 相同，沿开放侧 +Y、离开底板 +Z 各避让20mm。
    feature_params.safe_transition_offset_y = 20.0f;
    feature_params.safe_transition_offset_z = 20.0f;

    // 8. 路径输出模式。默认 feature_points 完全保留 2.2.2 行为；切换为
    // adaptive_contour 后，沿红色焊缝轮廓按曲率自适应离散，且总点数硬限100。
    AdaptiveContourParams adaptive_params;

    // 9. 焊枪姿态参数：四组分别对应“凸角左腰、凸角右腰、凹角左腰、凹角右腰”。
    // 左/右腰按 CSV 前进方向下 local_y 斜率的正/负定义，不依赖相机画面的左右方向。
    // work_angle 是枪体中心轴与蓝色底板平面的夹角，默认全部45°；若凹角处枪头
    // 外壳容易碰波纹板，可在仿真/低速验证后适当增大对应 recessed 角度，使枪体更竖直。
    TorchOrientationParams torch_params;
    torch_params.protruding_left_work_angle_deg = 45.0f;
    torch_params.protruding_right_work_angle_deg = 45.0f;
    torch_params.recessed_left_work_angle_deg = 45.0f;
    torch_params.recessed_right_work_angle_deg = 45.0f;

    // lead_angle 为沿 CSV 焊接顺序的前倾/后倾角；0°最稳妥，正值向前倾，负值向后倾。
    torch_params.protruding_left_lead_angle_deg = 0.0f;
    torch_params.protruding_right_lead_angle_deg = 0.0f;
    torch_params.recessed_left_lead_angle_deg = 0.0f;
    torch_params.recessed_right_lead_angle_deg = 0.0f;

    // 必须与机器人中实际定义的焊枪工具 +Z 轴一致：
    // true 表示 +Z 从 TCP 指向枪体；若机器人 +Z 从枪体指向焊丝/TCP，则改为 false。
    torch_params.tool_positive_z_points_from_tcp_to_body = true;

    // 默认让工具 +X 跟随工件焊接前进方向，减少不同角点间绕枪轴的非工艺性旋转。
    // 方向相反时只需把布尔值改为 false；如需复现旧版本，改为 corner_bisector。
    torch_params.tool_x_reference = "workpiece_x";
    torch_params.tool_x_points_along_positive_workpiece_x = true;

    // 结果点云中白色焊枪中心轴的显示长度，便于检查凹角避让；设为0可关闭。
    torch_params.visualization_body_axis_length_mm = 25.0f;
    // ====================================================================

    // 运行时覆盖：配置文件和 --set 只改变显式给出的字段；未给出的字段继续
    // 使用上面经过样件验证的默认值，因此旧用法与原始算法行为兼容。
#define APPLY_FLOAT(KEY, TARGET) \
    do { if (!applyNumberParameter<float>(runtime_parameters, KEY, TARGET, parameter_error)) { \
        run_result.message = parameter_error; std::cerr << parameter_error << std::endl; return -1; } } while (0)
#define APPLY_INT(KEY, TARGET) \
    do { if (!applyNumberParameter<int>(runtime_parameters, KEY, TARGET, parameter_error)) { \
        run_result.message = parameter_error; std::cerr << parameter_error << std::endl; return -1; } } while (0)
#define APPLY_UINT(KEY, TARGET) \
    do { if (!applyNumberParameter<unsigned int>(runtime_parameters, KEY, TARGET, parameter_error)) { \
        run_result.message = parameter_error; std::cerr << parameter_error << std::endl; return -1; } } while (0)
#define APPLY_BOOL(KEY, TARGET) \
    do { if (!applyBoolParameter(runtime_parameters, KEY, TARGET, parameter_error)) { \
        run_result.message = parameter_error; std::cerr << parameter_error << std::endl; return -1; } } while (0)
#define APPLY_STRING(KEY, TARGET) \
    do { if (!applyStringParameter(runtime_parameters, KEY, TARGET)) { \
        run_result.message = "Invalid string parameter."; std::cerr << run_result.message << std::endl; return -1; } } while (0)

    APPLY_BOOL("roi.enable", enable_roi);
    APPLY_FLOAT("roi.min_x", roi_min_x);
    APPLY_FLOAT("roi.max_x", roi_max_x);
    APPLY_FLOAT("roi.min_y", roi_min_y);
    APPLY_FLOAT("roi.max_y", roi_max_y);
    APPLY_FLOAT("roi.min_z", roi_min_z);
    APPLY_FLOAT("roi.max_z", roi_max_z);

    APPLY_STRING("normal.mode", normal_mode);
    APPLY_FLOAT("normal.reuse_min_valid_ratio", normal_reuse_min_valid_ratio);
    APPLY_INT("normal.k_neighbors", normal_k_neighbors);
    APPLY_UINT("normal.thread_count", normal_thread_count);
    APPLY_FLOAT("primary.z_filter_threshold", z_filter_threshold);
    APPLY_FLOAT("primary.y_filter_threshold", y_filter_threshold);
    APPLY_FLOAT("primary.plane_search_leaf_size", plane_search_leaf_size);
    APPLY_INT("primary.plane_search_min_inliers", plane_search_min_inliers);
    APPLY_FLOAT("primary.side_origin_low_quantile", side_origin_low_quantile);

    APPLY_FLOAT("feature.profile_bin_width", feature_params.profile_bin_width);
    APPLY_FLOAT("feature.flat_normal_dot_min", feature_params.flat_normal_dot_min);
    APPLY_FLOAT("feature.diagonal_normal_dot_min", feature_params.diagonal_normal_dot_min);
    APPLY_FLOAT("feature.bin_label_vote_ratio", feature_params.bin_label_vote_ratio);
    APPLY_INT("feature.min_segment_bins", feature_params.min_segment_bins);
    APPLY_FLOAT("feature.min_segment_span", feature_params.min_segment_span);
    APPLY_INT("feature.local_fit_half_window_bins", feature_params.local_fit_half_window_bins);
    APPLY_FLOAT("feature.local_fit_radius", feature_params.local_fit_radius);
    APPLY_FLOAT("feature.local_line_max_median_residual", feature_params.local_line_max_median_residual);
    APPLY_FLOAT("feature.flat_profile_slope_max", feature_params.flat_profile_slope_max);
    APPLY_FLOAT("feature.flat_segment_max_length", feature_params.flat_segment_max_length);
    APPLY_FLOAT("feature.waist_segment_max_length", feature_params.waist_segment_max_length);
    APPLY_FLOAT("feature.max_hole_bridge", feature_params.max_hole_bridge);
    APPLY_FLOAT("feature.max_corner_extrapolation", feature_params.max_corner_extrapolation);
    APPLY_FLOAT("feature.min_corner_angle_deg", feature_params.min_corner_angle_deg);
    APPLY_FLOAT("feature.duplicate_corner_merge_distance", feature_params.duplicate_corner_merge_distance);
    APPLY_FLOAT("feature.corner_fit_support_distance", feature_params.corner_fit_support_distance);
    APPLY_FLOAT("feature.recessed_chord_search_radius", feature_params.recessed_chord_search_radius);
    APPLY_FLOAT("feature.safe_transition_offset_y", feature_params.safe_transition_offset_y);
    APPLY_FLOAT("feature.safe_transition_offset_z", feature_params.safe_transition_offset_z);
    APPLY_FLOAT("feature.recessed_arc_search_radius", feature_params.recessed_arc_search_radius);
    APPLY_FLOAT("feature.bottom_z_quantile", feature_params.bottom_z_quantile);
    APPLY_FLOAT("feature.low_z_xy_tolerance", feature_params.low_z_xy_tolerance);
    APPLY_BOOL("feature.protruding_is_larger_local_y", feature_params.protruding_is_larger_local_y);
    APPLY_FLOAT("visualization.marker_radius", feature_params.visualization_marker_radius);

    APPLY_STRING("path.mode", adaptive_params.mode);
    APPLY_FLOAT("path.straight_spacing", adaptive_params.straight_spacing);
    APPLY_FLOAT("path.corner_spacing", adaptive_params.corner_spacing);
    APPLY_FLOAT("path.corner_influence_radius", adaptive_params.corner_influence_radius);
    APPLY_FLOAT("path.curvature_window", adaptive_params.curvature_window);
    APPLY_FLOAT("path.curvature_threshold_deg", adaptive_params.curvature_threshold_deg);
    APPLY_FLOAT("path.orientation_smoothing_radius", adaptive_params.orientation_smoothing_radius);
    APPLY_FLOAT("path.max_orientation_step_deg", adaptive_params.max_orientation_step_deg);
    APPLY_INT("path.smoothing_half_window_bins", adaptive_params.smoothing_half_window_bins);
    APPLY_FLOAT("path.outlier_max_distance", adaptive_params.outlier_max_distance);
    APPLY_FLOAT("path.max_bridge_gap", adaptive_params.max_bridge_gap);
    APPLY_INT("path.max_points", adaptive_params.max_points);
    APPLY_FLOAT("path.work_angle_deg", adaptive_params.work_angle_deg);
    APPLY_FLOAT("path.lead_angle_deg", adaptive_params.lead_angle_deg);

    APPLY_FLOAT("orientation.protruding_left.work_angle_deg", torch_params.protruding_left_work_angle_deg);
    APPLY_FLOAT("orientation.protruding_right.work_angle_deg", torch_params.protruding_right_work_angle_deg);
    APPLY_FLOAT("orientation.recessed_left.work_angle_deg", torch_params.recessed_left_work_angle_deg);
    APPLY_FLOAT("orientation.recessed_right.work_angle_deg", torch_params.recessed_right_work_angle_deg);
    APPLY_FLOAT("orientation.protruding_left.lead_angle_deg", torch_params.protruding_left_lead_angle_deg);
    APPLY_FLOAT("orientation.protruding_right.lead_angle_deg", torch_params.protruding_right_lead_angle_deg);
    APPLY_FLOAT("orientation.recessed_left.lead_angle_deg", torch_params.recessed_left_lead_angle_deg);
    APPLY_FLOAT("orientation.recessed_right.lead_angle_deg", torch_params.recessed_right_lead_angle_deg);
    APPLY_BOOL("orientation.tool_positive_z_points_from_tcp_to_body", torch_params.tool_positive_z_points_from_tcp_to_body);
    APPLY_STRING("orientation.tool_x_reference", torch_params.tool_x_reference);
    APPLY_BOOL("orientation.tool_x_points_along_positive_workpiece_x", torch_params.tool_x_points_along_positive_workpiece_x);
    APPLY_FLOAT("visualization.body_axis_length_mm", torch_params.visualization_body_axis_length_mm);

#define APPLY_OFFSET(GROUP, MEMBER) \
    APPLY_FLOAT("offset." GROUP ".x", position_offsets.MEMBER.x); \
    APPLY_FLOAT("offset." GROUP ".y", position_offsets.MEMBER.y); \
    APPLY_FLOAT("offset." GROUP ".z", position_offsets.MEMBER.z)
    APPLY_OFFSET("start_transition", start_transition);
    APPLY_OFFSET("end_transition", end_transition);
    APPLY_OFFSET("protruding_left", protruding_left);
    APPLY_OFFSET("protruding_right", protruding_right);
    APPLY_OFFSET("recessed_left", recessed_left);
    APPLY_OFFSET("recessed_right", recessed_right);
    APPLY_OFFSET("adaptive_contour", adaptive_contour);

#undef APPLY_OFFSET
#undef APPLY_BOOL
#undef APPLY_STRING
#undef APPLY_UINT
#undef APPLY_INT
#undef APPLY_FLOAT

    if (!rejectUnknownParameters(runtime_parameters, parameter_error)) {
        run_result.message = parameter_error;
        std::cerr << parameter_error << std::endl;
        return -1;
    }

    torch_params.tool_x_reference = lowerText(torch_params.tool_x_reference);
    adaptive_params.mode = lowerText(adaptive_params.mode);
    if (!validateTorchOrientationParams(torch_params)) {
        run_result.message =
            "Invalid torch orientation: work angles must be within (0, 90), "
            "abs(lead angles) < 85, tool_x_reference must be workpiece_x or "
            "corner_bisector, and visualization length >= 0.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!validateFeaturePositionOffsets(position_offsets)) {
        run_result.message =
            "Invalid feature position offset: every XYZ value must be finite.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!validateFeatureExtractionParams(feature_params, parameter_error)) {
        run_result.message = parameter_error;
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!validateAdaptiveContourParams(adaptive_params, parameter_error)) {
        run_result.message = parameter_error;
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    normal_mode = lowerText(normal_mode);
    if (normal_mode != "auto" && normal_mode != "recompute" &&
        normal_mode != "reuse") {
        run_result.message =
            "normal.mode must be auto, recompute or reuse.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!std::isfinite(normal_reuse_min_valid_ratio) ||
        normal_reuse_min_valid_ratio <= 0.0f ||
        normal_reuse_min_valid_ratio > 1.0f) {
        run_result.message =
            "normal.reuse_min_valid_ratio must be within (0, 1].";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (normal_k_neighbors < 3 || normal_k_neighbors > 200) {
        run_result.message = "normal.k_neighbors must be within [3, 200].";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!std::isfinite(z_filter_threshold) ||
        !std::isfinite(y_filter_threshold)) {
        run_result.message = "Primary Z/Y filter thresholds must be finite.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!std::isfinite(plane_search_leaf_size) ||
        plane_search_leaf_size <= 0.0f || plane_search_min_inliers < 3) {
        run_result.message =
            "Plane-search leaf size must be > 0 and min inliers must be >= 3.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (!std::isfinite(side_origin_low_quantile) ||
        side_origin_low_quantile < 0.0f || side_origin_low_quantile > 0.5f) {
        run_result.message =
            "primary.side_origin_low_quantile must be within [0, 0.5].";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const bool roi_has_nan =
        std::isnan(roi_min_x) || std::isnan(roi_max_x) ||
        std::isnan(roi_min_y) || std::isnan(roi_max_y) ||
        std::isnan(roi_min_z) || std::isnan(roi_max_z);
    if (enable_roi && (roi_has_nan || roi_min_x > roi_max_x ||
        roi_min_y > roi_max_y || roi_min_z > roi_max_z)) {
        run_result.message =
            "Invalid ROI: bounds cannot be NaN and every minimum must be <= maximum.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // 先读取一次通用 PCLPointCloud2，检查 PLY 实际字段；避免为判断 normals
    // 重复读取大文件。XYZ 始终必需，normal_x/y/z 仅在 auto/reuse 模式下使用。
    pcl::PCLPointCloud2 cloud_blob;
    if (pcl::io::loadPLYFile(options.input_ply, cloud_blob) == -1) {
        run_result.message = "Cannot load input PLY: " + options.input_ply;
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(
        new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(cloud_blob, *cloud_xyz);
    if (cloud_xyz->empty()) {
        run_result.message = "Input PLY contains no points.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // 原始 PLY 常用 nx/ny/nz，PCL 自定义点类使用
    // normal_x/normal_y/normal_z。只改 field 元数据名，不复制数据区。
    for (pcl::PCLPointField& field : cloud_blob.fields) {
        if (field.name == "nx") field.name = "normal_x";
        else if (field.name == "ny") field.name = "normal_y";
        else if (field.name == "nz") field.name = "normal_z";
    }
    const bool input_declares_normals =
        cloudBlobHasField(cloud_blob, "normal_x") &&
        cloudBlobHasField(cloud_blob, "normal_y") &&
        cloudBlobHasField(cloud_blob, "normal_z");
    pcl::PointCloud<InputPointWithNormal>::Ptr cloud_with_input_normals;
    size_t finite_input_xyz_count = 0;
    size_t valid_input_normal_count = 0;
    float input_normal_valid_ratio = 0.0f;
    bool reuse_input_normals = false;

    if (normal_mode != "recompute" && input_declares_normals) {
        cloud_with_input_normals.reset(
            new pcl::PointCloud<InputPointWithNormal>);
        pcl::fromPCLPointCloud2(cloud_blob, *cloud_with_input_normals);
        if (cloud_with_input_normals->size() == cloud_xyz->size()) {
            for (const InputPointWithNormal& point :
                    cloud_with_input_normals->points) {
                if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                    !std::isfinite(point.z)) {
                    continue;
                }
                ++finite_input_xyz_count;
                const Eigen::Vector3f normal(
                    point.normal_x, point.normal_y, point.normal_z);
                if (normal.allFinite() && normal.squaredNorm() > 1e-8f) {
                    ++valid_input_normal_count;
                }
            }
            if (finite_input_xyz_count > 0) {
                input_normal_valid_ratio = static_cast<float>(
                    static_cast<double>(valid_input_normal_count) /
                    static_cast<double>(finite_input_xyz_count));
            }
            reuse_input_normals =
                input_normal_valid_ratio >= normal_reuse_min_valid_ratio;
        }
    }

    if (normal_mode == "reuse" && !reuse_input_normals) {
        std::ostringstream message;
        message << "normal.mode=reuse requires normal_x/normal_y/normal_z and "
                << "a valid ratio >= " << normal_reuse_min_valid_ratio
                << "; fields_present=" << (input_declares_normals ? "true" : "false")
                << ", valid_ratio=" << input_normal_valid_ratio;
        run_result.message = message.str();
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    if (normal_mode == "auto") {
        if (reuse_input_normals) {
            std::cout << "Input normals accepted: valid="
                << valid_input_normal_count << "/" << finite_input_xyz_count
                << " (ratio=" << input_normal_valid_ratio << ")." << std::endl;
        } else {
            std::cout << "Input normals unavailable or below quality threshold; "
                << "will recompute after VoxelGrid (fields_present="
                << (input_declares_normals ? "true" : "false")
                << ", valid_ratio=" << input_normal_valid_ratio << ")." << std::endl;
        }
    } else if (normal_mode == "recompute") {
        std::cout << "Normal mode=recompute: ignoring any input normals." << std::endl;
    }

    pcl::PointCloud<PointInT>::Ptr cloud_in(new pcl::PointCloud<PointInT>);
    cloud_in->points.reserve(cloud_xyz->size());
    for (size_t i = 0; i < cloud_xyz->size(); ++i) {
        const pcl::PointXYZ& source = cloud_xyz->points[i];
        if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
            !std::isfinite(source.z)) continue;
        PointInT point;
        point.x = source.x; point.y = source.y; point.z = source.z;
        point.normal_x = 0.0f; point.normal_y = 0.0f; point.normal_z = 0.0f;
        point.curvature = 0.0f;
        if (reuse_input_normals && cloud_with_input_normals &&
            i < cloud_with_input_normals->size()) {
            const InputPointWithNormal& input =
                cloud_with_input_normals->points[i];
            Eigen::Vector3f normal(
                input.normal_x, input.normal_y, input.normal_z);
            if (normal.allFinite() && normal.squaredNorm() > 1e-8f) {
                normal.normalize();
                const Eigen::Vector3f toward_view(-point.x, -point.y, -point.z);
                if (toward_view.squaredNorm() > 1e-8f &&
                    normal.dot(toward_view) < 0.0f) {
                    normal = -normal;
                }
                point.normal_x = normal.x();
                point.normal_y = normal.y();
                point.normal_z = normal.z();
                point.curvature = 0.0f;
            }
        }
        cloud_in->points.push_back(point);
    }
    const size_t discarded_invalid_xyz = cloud_xyz->size() - cloud_in->size();
    cloud_xyz.reset();
    cloud_with_input_normals.reset();
    cloud_blob.data.clear();
    cloud_blob.fields.clear();
    cloud_in->width = static_cast<uint32_t>(cloud_in->points.size());
    cloud_in->height = 1;
    cloud_in->is_dense = true;
    if (cloud_in->empty()) {
        run_result.message = "Input PLY contains no finite XYZ points.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    std::cout << "Loaded XYZ points: " << cloud_in->size()
        << "; discarded invalid XYZ: " << discarded_invalid_xyz << std::endl;
    const std::chrono::steady_clock::time_point primary_start_time =
        std::chrono::steady_clock::now();

    // ROI 在体素滤波和所有平面搜索之前执行，裁掉的点不会进入后续流程。
    // enable_roi=false 时直接复用原始点云指针，不产生一次额外的全点云拷贝。
    pcl::PointCloud<PointInT>::Ptr cloud_roi = cloud_in;
    if (enable_roi) {
        if (!(roi_min_x <= roi_max_x && roi_min_y <= roi_max_y &&
              roi_min_z <= roi_max_z)) {
            run_result.message =
                "Invalid ROI: every minimum must be <= its maximum.";
            std::cerr << run_result.message << std::endl;
            return -1;
        }

        cloud_roi.reset(new pcl::PointCloud<PointInT>);
        cloud_roi->points.reserve(cloud_in->size());
        for (size_t i = 0; i < cloud_in->size(); ++i) {
            const PointInT& point = cloud_in->points[i];
            if (!std::isfinite(point.x) || !std::isfinite(point.y) ||
                !std::isfinite(point.z)) {
                continue;
            }
            if (point.x < roi_min_x || point.x > roi_max_x ||
                point.y < roi_min_y || point.y > roi_max_y ||
                point.z < roi_min_z || point.z > roi_max_z) {
                continue;
            }
            cloud_roi->points.push_back(point);
        }
        cloud_roi->width = static_cast<uint32_t>(cloud_roi->points.size());
        cloud_roi->height = 1;
        cloud_roi->is_dense = true;

        std::cout << "ROI enabled: " << cloud_roi->size() << "/" << cloud_in->size()
            << " points, x=[" << roi_min_x << ", " << roi_max_x
            << "], y=[" << roi_min_y << ", " << roi_max_y
            << "], z=[" << roi_min_z << ", " << roi_max_z << "]" << std::endl;
        if (cloud_roi->empty()) {
            run_result.message =
                "No points remain inside the configured ROI.";
            std::cerr << run_result.message << std::endl;
            return -1;
        }
    } else {
        std::cout << "ROI disabled: using full input view ("
            << cloud_roi->size() << " points)." << std::endl;
    }

    // 1. ROI 内点云体素降采样 (0.5mm)
    pcl::PointCloud<PointInT>::Ptr cloud_filtered(new pcl::PointCloud<PointInT>);
    pcl::VoxelGrid<PointInT> sor;
    sor.setInputCloud(cloud_roi);
    // 明确对 XYZ 以外字段一起下采样，否则 auto/reuse 无法把
    // 相机 normals 传递到体素点。重算模式后面会覆盖这些字段。
    sor.setDownsampleAllData(true);
    sor.setLeafSize(0.5f, 0.5f, 0.5f); 
    sor.filter(*cloud_filtered);
    std::cout << "Points after VoxelGrid: " << cloud_filtered->size() << std::endl;
    if (cloud_filtered->empty()) {
        run_result.message = "No points remain after VoxelGrid.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // auto/reuse 下，VoxelGrid 会平均同一体素内已经统一方向的输入 normals，随后
    // 再归一化。若体素后的有效率意外下降，auto 自动回退完整重算，reuse 则报错。
    const std::chrono::steady_clock::time_point normal_start_time =
        std::chrono::steady_clock::now();
    size_t valid_normal_count = 0;
    std::string normal_source_text = "recomputed";

    auto normalize_existing_normals = [&]() -> size_t {
        size_t valid_count = 0;
        for (PointInT& target : cloud_filtered->points) {
            Eigen::Vector3f normal(
                target.normal_x, target.normal_y, target.normal_z);
            if (normal.allFinite() && normal.squaredNorm() > 1e-8f) {
                normal.normalize();
                const Eigen::Vector3f toward_view(
                    -target.x, -target.y, -target.z);
                if (toward_view.squaredNorm() > 1e-8f &&
                    normal.dot(toward_view) < 0.0f) {
                    normal = -normal;
                }
                target.normal_x = normal.x();
                target.normal_y = normal.y();
                target.normal_z = normal.z();
                if (!std::isfinite(target.curvature)) target.curvature = 0.0f;
                ++valid_count;
            } else {
                target.normal_x = 0.0f;
                target.normal_y = 0.0f;
                target.normal_z = 0.0f;
                target.curvature = 0.0f;
            }
        }
        return valid_count;
    };

    bool recompute_normals = !reuse_input_normals;
    if (reuse_input_normals) {
        valid_normal_count = normalize_existing_normals();
        const float voxel_valid_ratio = cloud_filtered->empty() ? 0.0f :
            static_cast<float>(static_cast<double>(valid_normal_count) /
                static_cast<double>(cloud_filtered->size()));
        if (voxel_valid_ratio < normal_reuse_min_valid_ratio) {
            if (normal_mode == "reuse") {
                std::ostringstream message;
                message << "Input normals became insufficient after VoxelGrid: ratio="
                        << voxel_valid_ratio << ", required="
                        << normal_reuse_min_valid_ratio;
                run_result.message = message.str();
                std::cerr << run_result.message << std::endl;
                return -1;
            }
            recompute_normals = true;
            std::cout << "Voxelized input-normal ratio=" << voxel_valid_ratio
                << " is below threshold; falling back to recomputation." << std::endl;
        } else {
            normal_source_text = "reused_from_input";
        }
    }

    if (recompute_normals) {
        pcl::NormalEstimationOMP<PointInT, pcl::Normal> normal_estimation(
            normal_thread_count);
        pcl::search::KdTree<PointInT>::Ptr normal_tree(
            new pcl::search::KdTree<PointInT>);
        normal_estimation.setInputCloud(cloud_filtered);
        normal_estimation.setSearchMethod(normal_tree);
        normal_estimation.setKSearch(normal_k_neighbors);
        // 与历史稳定版一致：把法向统一朝向 PLY 原点（通常为相机光心）。
        normal_estimation.setViewPoint(0.0f, 0.0f, 0.0f);
        pcl::PointCloud<pcl::Normal>::Ptr estimated_normals(
            new pcl::PointCloud<pcl::Normal>);
        normal_estimation.compute(*estimated_normals);
        if (estimated_normals->size() != cloud_filtered->size()) {
            run_result.message =
                "Normal estimation returned an unexpected point count.";
            std::cerr << run_result.message << std::endl;
            return -1;
        }
        valid_normal_count = 0;
        for (size_t i = 0; i < cloud_filtered->size(); ++i) {
            const pcl::Normal& source = estimated_normals->points[i];
            Eigen::Vector3f normal(
                source.normal_x, source.normal_y, source.normal_z);
            PointInT& target = cloud_filtered->points[i];
            if (normal.allFinite() && normal.squaredNorm() > 1e-8f) {
                normal.normalize();
                target.normal_x = normal.x();
                target.normal_y = normal.y();
                target.normal_z = normal.z();
                target.curvature = std::isfinite(source.curvature)
                    ? source.curvature : 0.0f;
                ++valid_normal_count;
            } else {
                target.normal_x = 0.0f;
                target.normal_y = 0.0f;
                target.normal_z = 0.0f;
                target.curvature = 0.0f;
            }
        }
        estimated_normals.reset();
        normal_source_text = reuse_input_normals
            ? "recomputed_auto_fallback" : "recomputed";
    }
    if (valid_normal_count == 0) {
        run_result.message = "No valid normals are available after processing.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const std::chrono::steady_clock::time_point normal_end_time =
        std::chrono::steady_clock::now();
    const double normal_time_ms = std::chrono::duration<double, std::milli>(
        normal_end_time - normal_start_time).count();
    std::cout << "Normals after VoxelGrid: source=" << normal_source_text
        << ", valid=" << valid_normal_count << "/" << cloud_filtered->size();
    if (normal_source_text != "reused_from_input") {
        std::cout << ", K=" << normal_k_neighbors;
    }
    std::cout << std::endl;

    // 计算点云质心 (用于确定 "上方" 和 "内侧" 的绝对物理方向)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud_filtered, centroid);
    Eigen::Vector3f C_all(centroid[0], centroid[1], centroid[2]);

    // 2. 连续提取 Top 8 平面。RANSAC 先在 1.0mm 辅助点云运行，红色焊缝
    // 仍在原 0.5mm 点云提取；若辅助点云识别失败则自动回退原全分辨率搜索。
    pcl::PointCloud<PointInT>::Ptr plane_search_cloud = cloud_filtered;
    if (plane_search_leaf_size > 0.5f) {
        pcl::PointCloud<PointInT>::Ptr coarse(new pcl::PointCloud<PointInT>);
        pcl::VoxelGrid<PointInT> plane_voxel;
        plane_voxel.setInputCloud(cloud_filtered);
        plane_voxel.setLeafSize(
            plane_search_leaf_size, plane_search_leaf_size, plane_search_leaf_size);
        plane_voxel.filter(*coarse);
        if (!coarse->empty()) plane_search_cloud = coarse;
    }
    std::cout << "Plane-search points: " << plane_search_cloud->size() << std::endl;

    const int coarse_minimum_inliers = std::max(100, std::min(
        plane_search_min_inliers,
        static_cast<int>(plane_search_cloud->size() / 100)));
    std::vector<PlaneInfo> top_planes = extractDominantPlanes(
        plane_search_cloud, 8, coarse_minimum_inliers, 1.5f);

    bool has_perpendicular_pair = false;
    for (size_t i = 0; i < top_planes.size() && !has_perpendicular_pair; ++i) {
        for (size_t j = i + 1; j < top_planes.size(); ++j) {
            if (std::abs(top_planes[i].normal.dot(top_planes[j].normal)) < 0.35f) {
                has_perpendicular_pair = true;
                break;
            }
        }
    }

    if ((top_planes.size() < 2 || !has_perpendicular_pair) &&
        plane_search_cloud != cloud_filtered) {
        std::cerr << "Coarse plane search failed; retrying full-resolution RANSAC."
            << std::endl;
        const int full_minimum_inliers = std::max(100, std::min(
            1000, static_cast<int>(cloud_filtered->size() / 100)));
        top_planes = extractDominantPlanes(
            cloud_filtered, 8, full_minimum_inliers, 1.5f);

        has_perpendicular_pair = false;
        for (size_t i = 0; i < top_planes.size() && !has_perpendicular_pair; ++i) {
            for (size_t j = i + 1; j < top_planes.size(); ++j) {
                if (std::abs(top_planes[i].normal.dot(top_planes[j].normal)) < 0.35f) {
                    has_perpendicular_pair = true;
                    break;
                }
            }
        }
    }
    if (top_planes.size() < 2 || !has_perpendicular_pair) {
        run_result.message = "Failed to find enough dominant planes.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // 3. 投票法寻找唯一的 L形底板 (Z轴平面)
    int best_bottom_idx = 0;
    int max_perp_count = -1;
    int best_bottom_support = -1;
    for (size_t i = 0; i < top_planes.size(); ++i) {
        int perp_count = 0;
        for (size_t j = 0; j < top_planes.size(); ++j) {
            if (i == j) continue;
            if (std::abs(top_planes[i].normal.dot(top_planes[j].normal)) < 0.35f) perp_count++;
        }
        if (perp_count > max_perp_count ||
            (perp_count == max_perp_count &&
                top_planes[i].point_count > best_bottom_support)) {
            max_perp_count = perp_count;
            best_bottom_support = top_planes[i].point_count;
            best_bottom_idx = i;
        }
    }

    Eigen::Vector3f N_bottom = top_planes[best_bottom_idx].normal;
    float D_bottom = top_planes[best_bottom_idx].d;

    // 辅助点云只负责快速发现方向；最终底面参数回到 0.5mm 点云做 PCA 精修。
    refinePlaneOnFullResolutionCloud(
        cloud_filtered, N_bottom, D_bottom, 1.5f,
        std::max(500, std::min(2000, static_cast<int>(cloud_filtered->size() / 200))));

    // 【核心】强制 N_bottom 指向质心所在方向 (即波纹板所在的"上方")
    if (N_bottom.dot(C_all) + D_bottom < 0) {
        N_bottom = -N_bottom;
        D_bottom = -D_bottom;
    }

    // 4. 寻找 L形侧板 (Y轴平面)
    int best_side_idx = -1;
    float max_area = -1;
    for (size_t i = 0; i < top_planes.size(); ++i) {
        if (i == best_bottom_idx) continue;
        // 找到与底面垂直的平面中，面积最大的(极大概率是波纹上底或L侧板，法向是一样的)
        if (std::abs(top_planes[i].normal.dot(N_bottom)) < 0.35f) {
            if (top_planes[i].point_count > max_area) {
                max_area = top_planes[i].point_count;
                best_side_idx = i;
            }
        }
    }

    if (best_side_idx < 0) {
        run_result.message = "Failed to identify the L-shaped side plane.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    Eigen::Vector3f N_side = top_planes[best_side_idx].normal;
    float D_side = top_planes[best_side_idx].d;
    refinePlaneOnFullResolutionCloud(
        cloud_filtered, N_side, D_side, 1.5f,
        std::max(500, std::min(2000, static_cast<int>(cloud_filtered->size() / 200))));
    // 强制 N_side 绝对垂直于 N_bottom，消除倾斜误差
    const Eigen::Vector3f orthogonal_side =
        N_side - N_side.dot(N_bottom) * N_bottom;
    if (!orthogonal_side.allFinite() || orthogonal_side.norm() < 1e-6f) {
        run_result.message = "Degenerate side-plane normal.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    N_side = orthogonal_side.normalized();

    // 寻找真正的 L形侧板的 D 值 (即坐标系Y轴零点)
    // 用低端 0.2% 分位代替单个绝对最小点，避免飞点把整个 y_local 原点拉偏。
    std::vector<float> side_projections;
    side_projections.reserve(cloud_filtered->size());
    for (const auto& pt : cloud_filtered->points) {
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;
        float y_proj = pt.x * N_side.x() + pt.y * N_side.y() + pt.z * N_side.z();
        side_projections.push_back(y_proj);
    }
    if (side_projections.empty()) {
        run_result.message = "Cannot calibrate the side-plane origin.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const float min_y_proj = fastQuantileOf(side_projections, side_origin_low_quantile);
    // 重新确立 D_side，使得 L侧面的 y_local 刚好为 0，波纹向内为正
    D_side = -min_y_proj; 

    // 计算波纹腰部的切向基准向量 (X轴)
    Eigen::Vector3f N_tangent = N_bottom.cross(N_side).normalized();
    if (!N_tangent.allFinite()) {
        run_result.message = "Invalid local coordinate frame.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // 工件 X=0 取当前 ROI 点云质心在 L侧板/底板交线上的投影。
    // 同一批点云发生刚体旋转或平移时，该相对坐标保持不变；改变 ROI 会改变此 X 原点。
    const float workpiece_x_origin_projection = N_tangent.dot(C_all);

    // 5. 提取候选点，并执行【二维联合滤波】
    pcl::PointCloud<PointInT>::Ptr seam_candidates(new pcl::PointCloud<PointInT>);
    std::vector<int> seam_original_idx; 
    const size_t candidate_reserve = std::min<size_t>(
        cloud_filtered->size(), std::max<size_t>(10000, cloud_filtered->size() / 20));
    seam_candidates->points.reserve(candidate_reserve);
    seam_original_idx.reserve(candidate_reserve);

    for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
        const auto& pt = cloud_filtered->points[i];
        if (!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) continue;

        // 计算点在局部坐标系中的坐标
        // z_local > 0 表示在底板上方； y_local > 0 表示在 L侧板内侧
        float z_local = pt.x * N_bottom.x() + pt.y * N_bottom.y() + pt.z * N_bottom.z() + D_bottom;
        float y_local = pt.x * N_side.x()   + pt.y * N_side.y()   + pt.z * N_side.z()   + D_side;
        
        Eigen::Vector3f pt_n(pt.normal_x, pt.normal_y, pt.normal_z);
        if (!pt_n.allFinite() || pt_n.squaredNorm() < 1e-8f) continue;
        pt_n.normalize();
        
        // 初筛：距离底板较近(绝对值<6mm)，且属于侧面(法向平行于地面)
        if (std::abs(z_local) < 6.0f && std::abs(N_bottom.dot(pt_n)) < 0.6f) {
            
            // 【二维联合滤波】：如果高度太低，或者离L侧板太近，就是直焊缝，统统杀掉！
            if (z_local > z_filter_threshold && y_local > y_filter_threshold) {
                seam_candidates->points.push_back(pt);
                seam_original_idx.push_back(i);
            }
        }
    }

    // 6. 欧式聚类去噪 (清除残留飞溅点)
    std::vector<bool> is_final_seam(cloud_filtered->size(), false);
    int seam_count = 0;

    if (seam_candidates->size() > 0) {
        pcl::search::KdTree<PointInT>::Ptr tree(new pcl::search::KdTree<PointInT>);
        tree->setInputCloud(seam_candidates);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointInT> ec;
        
        ec.setClusterTolerance(3.5); 
        ec.setMinClusterSize(30);    
        ec.setMaxClusterSize(500000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(seam_candidates);
        ec.extract(cluster_indices);

        for (const auto& cluster : cluster_indices) {
            for (int idx : cluster.indices) {
                is_final_seam[seam_original_idx[idx]] = true;
                seam_count++;
            }
        }
    }

    const std::chrono::steady_clock::time_point primary_end_time =
        std::chrono::steady_clock::now();

    // 6.1 两种路径模式都只消费上面已经稳定得到的红色焊缝点。
    // feature_points 保留四类拐点；adaptive_contour 沿真实轮廓自适应离散。
    const std::chrono::steady_clock::time_point feature_start_time =
        std::chrono::steady_clock::now();
    std::vector<WeldFeaturePoint> feature_points;
    std::string path_error;
    if (adaptive_params.mode == "adaptive_contour") {
        feature_points = extractAdaptiveContourPath(
            cloud_filtered, is_final_seam,
            N_bottom, D_bottom, N_side, D_side, N_tangent,
            feature_params, adaptive_params, path_error);
        if (!path_error.empty()) {
            run_result.message = path_error;
            std::cerr << run_result.message << std::endl;
            return -1;
        }
    } else {
        feature_points = extractOrderedWeldFeatures(
            cloud_filtered, is_final_seam,
            N_bottom, D_bottom, N_side, D_side, N_tangent, feature_params);
    }

    // 在 PLY 世界坐标系中生成每个点的焊枪姿态。Tool Z 使用局部开放侧
    // 角平分避让，Tool X 默认以工件焊接前进方向为参考；首末安全过渡点
    // 继承距离最近的内部角点姿态。
    TorchOrientationParams path_torch_params = torch_params;
    if (adaptive_params.mode == "adaptive_contour") {
        // 连续模式每个点的 left/right slope 都是该处的局部轮廓切线；四组角度
        // 统一为连续路径工艺角，避免分类边界产生姿态阶跃。
        path_torch_params.protruding_left_work_angle_deg = adaptive_params.work_angle_deg;
        path_torch_params.protruding_right_work_angle_deg = adaptive_params.work_angle_deg;
        path_torch_params.recessed_left_work_angle_deg = adaptive_params.work_angle_deg;
        path_torch_params.recessed_right_work_angle_deg = adaptive_params.work_angle_deg;
        path_torch_params.protruding_left_lead_angle_deg = adaptive_params.lead_angle_deg;
        path_torch_params.protruding_right_lead_angle_deg = adaptive_params.lead_angle_deg;
        path_torch_params.recessed_left_lead_angle_deg = adaptive_params.lead_angle_deg;
        path_torch_params.recessed_right_lead_angle_deg = adaptive_params.lead_angle_deg;
    }
    const std::vector<WeldPoseData> feature_poses = buildOrderedWeldPoses(
        feature_points, N_bottom, N_side, N_tangent, path_torch_params);

    // 提取、排序、凹凸分类和姿态生成完成后，才按点类型施加工件坐标偏置。
    // 因此不完整周期、视野截断和某一类角点缺失都不会影响其余点的偏置选择。
    if (!applyFeaturePositionOffsets(
            feature_points, feature_poses,
            N_bottom, N_side, N_tangent, position_offsets)) {
        run_result.message =
            "Cannot apply feature offsets: feature/pose count mismatch.";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const std::chrono::steady_clock::time_point feature_end_time =
        std::chrono::steady_clock::now();

    // 7. 全景语义精准着色
    pcl::PointCloud<PointOutT>::Ptr final_cloud(new pcl::PointCloud<PointOutT>);
    final_cloud->points.reserve(cloud_filtered->size());

    for (size_t i = 0; i < cloud_filtered->points.size(); ++i) {
        const auto& pt = cloud_filtered->points[i];
        if (!std::isfinite(pt.x)) continue;

        PointOutT p; p.x = pt.x; p.y = pt.y; p.z = pt.z;
        Eigen::Vector3f pt_n(pt.normal_x, pt.normal_y, pt.normal_z);
        const bool has_valid_normal =
            pt_n.allFinite() && pt_n.squaredNorm() > 1e-8f;
        if (has_valid_normal) pt_n.normalize();
        else pt_n.setZero();
        
        float z_local = pt.x * N_bottom.x() + pt.y * N_bottom.y() + pt.z * N_bottom.z() + D_bottom;
        float y_local = pt.x * N_side.x()   + pt.y * N_side.y()   + pt.z * N_side.z()   + D_side;
        float dot_b = std::abs(N_bottom.dot(pt_n)); 

        if (is_final_seam[i]) {
            p.r = 255; p.g = 0; p.b = 0; // 【红色】 波纹焊缝！
        }
        else if (!has_valid_normal) {
            p.r = 200; p.g = 200; p.b = 200; // 【灰色】 无有效法向量的边界/孤立点
        }
        else if (dot_b > 0.85f && std::abs(z_local) < 2.0f) {
            p.r = 0; p.g = 0; p.b = 255; // 【深蓝色】 L形底板
        } 
        else if (dot_b < 0.35f) { // 侧面集合
            float dot_s = std::abs(N_side.dot(pt_n));
            float dot_t = N_tangent.dot(pt_n);

            if (dot_s > 0.85f) {
                // 【绝杀修复】：利用 y_local 深度严格区分 L侧板 和 波纹上下底！
                if (std::abs(y_local) < 2.5f) {
                    p.r = 255; p.g = 255; p.b = 0; // 【黄色】 L形侧板 (贴在y=0处)
                } else {
                    p.r = 255; p.g = 165; p.b = 0; // 【橙色】 波纹上底、下底 (向内凹了)
                }
            } else if (dot_t > 0.15f) {
                p.r = 0; p.g = 255; p.b = 255; // 【青色】 波纹左腰
            } else if (dot_t < -0.15f) {
                p.r = 128; p.g = 0; p.b = 128; // 【紫色】 波纹右腰
            } else {
                p.r = 0; p.g = 255; p.b = 0;   // 【绿色】 过渡区/圆角
            }
        } 
        else {
            p.r = 200; p.g = 200; p.b = 200;   // 【灰色】 顶部及无关点
        }
        final_cloud->points.push_back(p);
    }

    final_cloud->width = final_cloud->points.size();
    final_cloud->height = 1;
    final_cloud->is_dense = true;

    const std::filesystem::path output_directory =
        options.output_directory.empty()
            ? std::filesystem::path(".")
            : std::filesystem::path(options.output_directory);
    std::error_code output_directory_error;
    std::filesystem::create_directories(
        output_directory, output_directory_error);
    if (output_directory_error) {
        run_result.message = "Cannot create output directory: " +
            output_directory.string() + " (" +
            output_directory_error.message() + ")";
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const std::filesystem::path output_base =
        output_directory / options.output_prefix;
    run_result.csv_path = output_base.string() + "_features.csv";
    run_result.feature_points_ply_path =
        output_base.string() + "_feature_points.ply";
    run_result.visualization_ply_path =
        output_base.string() + "_result.ply";

    // 机械臂使用的是真实、精确、已经按 local_x 连续排序的单点集合。
    if (!saveFeatureCsv(
            run_result.csv_path, feature_points, feature_poses,
            path_torch_params, position_offsets, workpiece_x_origin_projection)) {
        run_result.message = "Failed to write feature CSV: " + run_result.csv_path;
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    pcl::PointCloud<PointOutT>::Ptr exact_feature_cloud =
        makeExactFeatureCloud(feature_points);
    if (!saveExactFeaturePly(
            run_result.feature_points_ply_path, *exact_feature_cloud)) {
        run_result.message =
            "Failed to write exact feature PLY: " +
            run_result.feature_points_ply_path;
        std::cerr << run_result.message << std::endl;
        return -1;
    }

    // 直接在原来的彩色结果点云上叠加粉红色过渡点/拐点三维十字星，打开
    // weld_seam_result.ply 就能校验，不再要求查看另一个组合文件。
    appendFeatureMarkers(
        *final_cloud, feature_points,
        N_bottom, N_side, N_tangent, feature_params.visualization_marker_radius);
    appendTorchBodyAxisMarkers(
        *final_cloud, feature_points, feature_poses,
        path_torch_params.visualization_body_axis_length_mm);
    if (pcl::io::savePLYFileBinary(
            run_result.visualization_ply_path, *final_cloud) != 0) {
        run_result.message =
            "Failed to write visualization PLY: " +
            run_result.visualization_ply_path;
        std::cerr << run_result.message << std::endl;
        return -1;
    }
    const std::chrono::steady_clock::time_point total_end_time =
        std::chrono::steady_clock::now();

    // primary_start_time 位于 ROI 之前，因此原始区间包含 normal_time_ms。
    // 对外分别报告时将其扣除，避免使用者把两项相加时重复统计法向量阶段。
    const double primary_pipeline_time_ms =
        std::chrono::duration<double, std::milli>(
            primary_end_time - primary_start_time).count();
    const double primary_time_ms = std::max(
        0.0, primary_pipeline_time_ms - normal_time_ms);
    const double feature_time_ms =
        std::chrono::duration<double, std::milli>(feature_end_time - feature_start_time).count();
    const double total_time_ms =
        std::chrono::duration<double, std::milli>(total_end_time - total_start_time).count();

    std::cout << "Isolated " << seam_count << " robust corrugated seam points!" << std::endl;
    int protruding_count = 0;
    int measured_recessed_count = 0;
    int inferred_recessed_count = 0;
    int transition_count = 0;
    int contour_sample_count = 0;
    float maximum_orientation_step_deg = 0.0f;
    for (size_t i = 0; i < feature_points.size(); ++i) {
        const WeldFeaturePoint& feature = feature_points[i];
        const WeldPoseData& pose = feature_poses[i];
        const WorkpieceOffset3f applied_offset =
            selectFeaturePositionOffset(feature, pose, position_offsets);
        if (feature.is_transition_point) ++transition_count;
        else if (feature.is_contour_sample) ++contour_sample_count;
        else if (feature.protruding) ++protruding_count;
        else if (feature.measured_on_arc) ++measured_recessed_count;
        else ++inferred_recessed_count;

        std::string feature_name;
        std::string source_name;
        if (feature.is_transition_point) {
            feature_name = feature.is_start_transition
                ? "START_TRANSITION" : "END_TRANSITION";
            source_name = "generated safe transition";
        } else if (feature.is_contour_sample) {
            feature_name = "ADAPTIVE_CONTOUR_POINT";
            source_name = "robust profile adaptive sampling";
        } else {
            feature_name = feature.protruding ? "PROTRUDING_CORNER" : "RECESSED_CORNER";
            if (feature.topology_inferred) {
                source_name = "in-frame periodic boundary completion";
            } else if (feature.protruding) source_name = "ideal intersection";
            else source_name = feature.measured_on_arc
                ? "measured arc near safe chord" : "safe chord fallback";
        }
        std::cout << "Feature[" << i << "] " << feature_name << " / " << source_name;
        if (!feature.is_transition_point && feature.merged_detection_count > 1) {
            std::cout << " / merged_from=" << feature.merged_detection_count;
        }
        std::cout << " : " << feature.world.x() << ", "
            << feature.world.y() << ", " << feature.world.z()
            << " / local_z_to_bottom=" << feature.local.z() << " mm" << std::endl;
        const char* pose_source = pose.default_without_corner
            ? "default_no_corner"
            : (pose.inherited_from_nearest_corner
                ? "inherited_nearest_corner"
                : (feature.is_contour_sample
                    ? "local_contour_tangent" : "corner_geometry"));
        const float orientation_delta = i == 0
            ? 0.0f : quaternionAngularDistanceDeg(feature_poses[i - 1], pose);
        maximum_orientation_step_deg = std::max(
            maximum_orientation_step_deg, orientation_delta);
        std::cout << "  Pose[" << i << "] " << torchPoseGroupName(pose.group)
            << " / " << pose_source
            << " / work=" << pose.work_angle_deg
            << " deg, lead=" << pose.lead_angle_deg
            << " deg, delta=" << orientation_delta
            << " deg / q(wxyz)=" << pose.qw << ", " << pose.qx
            << ", " << pose.qy << ", " << pose.qz << std::endl;
        std::cout << "  WorkpiecePose[" << i << "] q(wxyz)="
            << pose.workpiece_qw << ", " << pose.workpiece_qx << ", "
            << pose.workpiece_qy << ", " << pose.workpiece_qz
            << " / xyz=" << feature.local.x() - workpiece_x_origin_projection
            << ", " << -feature.local.y() << ", " << feature.local.z()
            << " / offset_group=" << featurePositionOffsetGroupName(feature, pose)
            << " / offset_xyz=" << applied_offset.x << ", "
            << applied_offset.y << ", " << applied_offset.z << " mm" << std::endl;
    }
    if (feature_points.size() >= 3 &&
        feature_points.front().is_transition_point &&
        feature_points.back().is_transition_point) {
        const Eigen::Vector3f start_delta_local =
            feature_points.front().local - feature_points[1].local;
        const Eigen::Vector3f end_delta_local =
            feature_points.back().local -
            feature_points[feature_points.size() - 2].local;
        std::cout << "Final safe transition delta in workpiece XYZ after offsets: "
            << "start=(" << start_delta_local.x() << ", "
            << -start_delta_local.y() << ", " << start_delta_local.z()
            << ") mm, norm=" << start_delta_local.norm()
            << " mm; end=(" << end_delta_local.x() << ", "
            << -end_delta_local.y() << ", " << end_delta_local.z()
            << ") mm, norm=" << end_delta_local.norm() << " mm" << std::endl;
    }
    std::cout << "Ordered robot path points: " << feature_points.size()
        << " (safe transitions=" << transition_count
        << ", adaptive contour samples=" << contour_sample_count
        << ", protruding corners=" << protruding_count
        << ", recessed measured corners=" << measured_recessed_count
        << ", recessed safe fallbacks=" << inferred_recessed_count << ")" << std::endl;
    if (adaptive_params.mode == "adaptive_contour") {
        std::cout << "Adaptive contour weld samples: "
            << contour_sample_count << " (path.max_points="
            << adaptive_params.max_points << ", including transitions)" << std::endl;
    } else {
        std::cout << "Four-type weld feature points: "
            << (feature_points.size() >= static_cast<size_t>(transition_count)
                ? feature_points.size() - static_cast<size_t>(transition_count) : 0)
            << std::endl;
    }
    std::cout << "Path mode: " << adaptive_params.mode << std::endl;
    std::cout << "Configured safe transition workpiece offsets: X=0 mm, Y=+"
        << feature_params.safe_transition_offset_y << " mm, Z=+"
        << feature_params.safe_transition_offset_z << " mm" << std::endl;
    std::cout << "Position offsets: per feature group in workpiece XYZ "
                 "(+X=path, +Y=toward L side, +Z=up)." << std::endl;
    std::cout << "Workpiece position columns: X origin=ROI centroid projection, "
                 "Y=0 at L side, Z=0 at blue bottom." << std::endl;
    std::cout << "Workpiece axes in PLY: X=(" << N_tangent.x() << ", "
        << N_tangent.y() << ", " << N_tangent.z() << "), Y=("
        << -N_side.x() << ", " << -N_side.y() << ", " << -N_side.z()
        << "), Z=(" << N_bottom.x() << ", " << N_bottom.y() << ", "
        << N_bottom.z() << ")" << std::endl;
    std::cout << "Quaternion columns: qw/qx/qy/qz are PLY world_from_tool; "
                 "workpiece_q* are workpiece_from_tool; tool +Z="
        << (path_torch_params.tool_positive_z_points_from_tcp_to_body
            ? "TCP->torch body" : "torch body->TCP")
        << "; tool +X reference=" << path_torch_params.tool_x_reference;
    if (path_torch_params.tool_x_reference == "workpiece_x") {
        std::cout << (path_torch_params.tool_x_points_along_positive_workpiece_x
            ? " (+workpiece X)" : " (-workpiece X)");
    }
    std::cout << std::endl;
    std::cout << "Maximum adjacent orientation change: "
        << maximum_orientation_step_deg << " deg" << std::endl;
    std::cout << "Robot coordinates: " << run_result.csv_path << std::endl;
    std::cout << "Pink transition/path-point visualization: "
        << run_result.visualization_ply_path << std::endl;
    if (path_torch_params.visualization_body_axis_length_mm > 0.0f) {
        std::cout << "White TCP-to-torch-body axes: "
            << run_result.visualization_ply_path << " (length="
            << path_torch_params.visualization_body_axis_length_mm << " mm)" << std::endl;
    }
    std::cout << std::fixed << std::setprecision(3)
        << "Normal processing time: " << normal_time_ms << " ms" << std::endl
        << "Primary seam extraction time (excluding normals): "
        << primary_time_ms << " ms" << std::endl
        << "Secondary feature extraction time: " << feature_time_ms << " ms" << std::endl
        << "Total time including PLY I/O: " << total_time_ms << " ms" << std::endl;

    run_result.status = 0;
    run_result.message = "ok";
    run_result.seam_point_count = static_cast<std::size_t>(seam_count);
    run_result.path_point_count = feature_points.size();
    run_result.normal_time_ms = normal_time_ms;
    run_result.primary_time_ms = primary_time_ms;
    run_result.feature_time_ms = feature_time_ms;
    run_result.total_time_ms = total_time_ms;
    return 0;
}

weld_seam_sdk::RunResult weld_seam_sdk::run(const RunOptions& options)
{
    RunResult result;
    result.status = executeWeldSeamExtraction(options, result);
    if (result.status != 0 && result.message.empty()) {
        result.message = "Weld seam extraction failed; inspect stderr for the exact stage.";
    }
    return result;
}

const char* weld_seam_sdk::version()
{
    return "2.3.1";
}

std::string weld_seam_sdk::commandLineHelp()
{
    return
        "Usage:\n"
        "  weld_seam_extractor <input.ply> [output_directory] [output_prefix] [options]\n\n"
        "Options:\n"
        "  --output-dir <dir>       Output directory (default: .)\n"
        "  --output-prefix <name>   Prefix for the three output files (default: weld_seam)\n"
        "  --config <file>          Load key=value parameter file\n"
        "  --set <key=value>        Override one parameter; may be repeated\n"
        "  -h, --help               Show this help\n\n"
        "Precedence: compiled defaults < config file < --set/C++ overrides.\n"
        "All supported keys and current defaults are listed in config/default.conf.\n"
        "Example:\n"
        "  weld_seam_extractor part.ply out part01 --config default.conf "
        "--set roi.enable=true --set roi.min_y=-90 --set roi.max_y=-30\n";
}

#ifndef WELD_SEAM_SDK_NO_MAIN
int main(int argc, char** argv)
{
    weld_seam_sdk::RunOptions options;
    bool show_help = false;
    std::string error;
    if (!parseCommandLine(argc, argv, options, show_help, error)) {
        std::cerr << error << "\n\n" << weld_seam_sdk::commandLineHelp();
        return 2;
    }
    if (show_help) {
        std::cout << weld_seam_sdk::commandLineHelp();
        return 0;
    }
    const weld_seam_sdk::RunResult result = weld_seam_sdk::run(options);
    if (result.status != 0) {
        std::cerr << "Extraction failed: " << result.message << std::endl;
    }
    return result.status == 0 ? 0 : 1;
}
#endif
