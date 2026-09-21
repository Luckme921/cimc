#pragma once

#include <cstddef>
#include <string>
#include <vector>

#if defined(_WIN32) && defined(WELD_SEAM_SDK_SHARED)
#  if defined(WELD_SEAM_SDK_EXPORTS)
#    define WELD_SEAM_SDK_API __declspec(dllexport)
#  else
#    define WELD_SEAM_SDK_API __declspec(dllimport)
#  endif
#else
#  define WELD_SEAM_SDK_API
#endif

namespace weld_seam_sdk {

// SDK 的稳定入口。算法参数使用 key=value 覆盖，避免 ABI 由于参数结构体扩展而变化。
// 优先级：源码默认值 < config_file < parameter_overrides。
struct RunOptions {
    std::string input_ply;
    std::string output_directory = ".";
    std::string output_prefix = "weld_seam";
    std::string config_file;
    std::vector<std::string> parameter_overrides;
};

struct RunResult {
    int status = -1;
    std::string message;
    std::string csv_path;
    std::string feature_points_ply_path;
    std::string visualization_ply_path;
    std::size_t seam_point_count = 0;
    std::size_t path_point_count = 0;
    double normal_time_ms = 0.0;
    // ROI/体素/平面/焊缝阶段，不含 normal_time_ms，可直接相加。
    double primary_time_ms = 0.0;
    double feature_time_ms = 0.0;
    double total_time_ms = 0.0;
};

WELD_SEAM_SDK_API RunResult run(const RunOptions& options);
WELD_SEAM_SDK_API const char* version();
WELD_SEAM_SDK_API std::string commandLineHelp();

}  // namespace weld_seam_sdk
