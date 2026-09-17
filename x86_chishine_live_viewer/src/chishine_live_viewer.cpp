#include <3DCamera.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <iostream>
#include <limits>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

std::atomic_bool g_stop_requested{false};
std::atomic_bool g_save_requested{false};

struct RoiBounds {
    bool enabled = false;
    bool initialized = false;
    float min_x = -std::numeric_limits<float>::infinity();
    float max_x = std::numeric_limits<float>::infinity();
    float min_y = -std::numeric_limits<float>::infinity();
    float max_y = std::numeric_limits<float>::infinity();
    float min_z = -std::numeric_limits<float>::infinity();
    float max_z = std::numeric_limits<float>::infinity();
};

enum class RoiAxis { X, Y, Z };
enum class RoiLimit { Minimum, Maximum };

struct RoiControls {
    RoiBounds bounds;
    RoiAxis selected_axis = RoiAxis::X;
    RoiLimit selected_limit = RoiLimit::Minimum;
    float step_mm = 5.0f;
    bool fit_requested = false;
    bool print_requested = false;
    int pending_adjustment = 0;
    std::size_t revision = 0;
};

struct Options {
    bool network = true;
    bool usb = false;
    bool binary_ply = true;
    std::string serial;
    std::filesystem::path save_dir = "./snapshots";
    int discovery_timeout_ms = 3000;
    int discovery_attempts = 10;
    int retry_interval_ms = 1000;
    int frame_timeout_ms = 5000;
    int width = 0;
    int height = 0;
    double fps = 0.0;
    int point_size = 2;
    std::size_t max_display_points = 300000;

    // 这些成像参数默认不覆盖相机当前配置；只有命令行明确传入时才设置。
    int depth_min_mm = -1;
    int depth_max_mm = -1;
    float gain = std::numeric_limits<float>::quiet_NaN();
    float exposure = std::numeric_limits<float>::quiet_NaN();
    float frame_time = std::numeric_limits<float>::quiet_NaN();

    RoiBounds roi;
    bool roi_enable_explicit = false;
    bool roi_bound_provided = false;
    float roi_step_mm = 5.0f;
};

std::string help()
{
    return
        "Chishine3D live point-cloud viewer\n\n"
        "Usage:\n"
        "  chishine_live_viewer [options]\n\n"
        "Options:\n"
        "  --serial TEXT                 Select camera serial (default: first)\n"
        "  --network true|false          Enable network discovery (default: true)\n"
        "  --usb true|false              Enable USB/UVC discovery (default: false)\n"
        "  --attempts N                  Discovery attempts (default: 10)\n"
        "  --discovery-timeout-ms N      Per-attempt timeout (default: 3000)\n"
        "  --retry-ms N                  Delay between attempts (default: 1000)\n"
        "  --frame-timeout-ms N          Continuous frame timeout (default: 5000)\n"
        "  --width N --height N --fps F  Select Z16 stream; 0 means any\n"
        "  --point-size N                Viewer point size (default: 2)\n"
        "  --max-display-points N        Display decimation limit (default: 300000)\n"
        "  --save-dir DIR                S-key snapshot directory (default: ./snapshots)\n"
        "  --ascii                       Save ASCII PLY; binary is default\n"
        "  --depth-min-mm N --depth-max-mm N\n"
        "  --gain F --exposure F --frame-time F\n"
        "                                 Optional overrides; omitted means camera value\n"
        "  --roi true|false               Initial XYZ ROI switch (default: false)\n"
        "  --roi-min-x F --roi-max-x F    Initial camera/PLY X bounds in mm\n"
        "  --roi-min-y F --roi-max-y F    Initial camera/PLY Y bounds in mm\n"
        "  --roi-min-z F --roi-max-z F    Initial camera/PLY Z bounds in mm\n"
        "  --roi-step-mm F                Initial keyboard adjustment step (default: 5)\n"
        "  -h, --help                    Show this help\n\n"
        "Viewer keys:\n"
        "  T=ROI on/off, B=fit ROI to cloud, D=clear ROI, K=print YAML\n"
        "  X/Y/Z=select axis, N/M=select min/max, [ or ]=adjust bound\n"
        "  , or .=decrease/increase step, S=save displayed ROI PLY, Q/Esc=quit\n";
}

bool parseBool(const std::string& value)
{
    if (value == "true" || value == "1" || value == "on") return true;
    if (value == "false" || value == "0" || value == "off") return false;
    throw std::invalid_argument("Expected true/false, got: " + value);
}

const std::string& requireValue(int& index, int argc, char** argv)
{
    if (index + 1 >= argc) {
        throw std::invalid_argument(std::string("Missing value after ") + argv[index]);
    }
    static std::string value;
    value = argv[++index];
    return value;
}

Options parseOptions(int argc, char** argv)
{
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            std::cout << help();
            std::exit(0);
        } else if (arg == "--serial") {
            options.serial = requireValue(i, argc, argv);
        } else if (arg == "--network") {
            options.network = parseBool(requireValue(i, argc, argv));
        } else if (arg == "--usb") {
            options.usb = parseBool(requireValue(i, argc, argv));
        } else if (arg == "--attempts") {
            options.discovery_attempts = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--discovery-timeout-ms") {
            options.discovery_timeout_ms = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--retry-ms") {
            options.retry_interval_ms = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--frame-timeout-ms") {
            options.frame_timeout_ms = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--width") {
            options.width = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--height") {
            options.height = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--fps") {
            options.fps = std::stod(requireValue(i, argc, argv));
        } else if (arg == "--point-size") {
            options.point_size = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--max-display-points") {
            options.max_display_points = static_cast<std::size_t>(
                std::stoull(requireValue(i, argc, argv)));
        } else if (arg == "--save-dir") {
            options.save_dir = requireValue(i, argc, argv);
        } else if (arg == "--ascii") {
            options.binary_ply = false;
        } else if (arg == "--depth-min-mm") {
            options.depth_min_mm = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--depth-max-mm") {
            options.depth_max_mm = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--gain") {
            options.gain = std::stof(requireValue(i, argc, argv));
        } else if (arg == "--exposure") {
            options.exposure = std::stof(requireValue(i, argc, argv));
        } else if (arg == "--frame-time") {
            options.frame_time = std::stof(requireValue(i, argc, argv));
        } else if (arg == "--roi" || arg == "--roi-enable") {
            options.roi.enabled = parseBool(requireValue(i, argc, argv));
            options.roi_enable_explicit = true;
        } else if (arg == "--roi-min-x") {
            options.roi.min_x = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-max-x") {
            options.roi.max_x = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-min-y") {
            options.roi.min_y = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-max-y") {
            options.roi.max_y = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-min-z") {
            options.roi.min_z = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-max-z") {
            options.roi.max_z = std::stof(requireValue(i, argc, argv));
            options.roi_bound_provided = true;
        } else if (arg == "--roi-step-mm") {
            options.roi_step_mm = std::stof(requireValue(i, argc, argv));
        } else {
            throw std::invalid_argument("Unknown option: " + arg);
        }
    }

    if (options.discovery_attempts <= 0 || options.discovery_timeout_ms < 0 ||
        options.retry_interval_ms < 0 || options.frame_timeout_ms <= 0) {
        throw std::invalid_argument("Timeout/attempt parameters are invalid.");
    }
    if (options.point_size <= 0 || options.max_display_points == 0) {
        throw std::invalid_argument("Point-size and max-display-points must be > 0.");
    }
    const bool has_min = options.depth_min_mm >= 0;
    const bool has_max = options.depth_max_mm >= 0;
    if (has_min != has_max) {
        throw std::invalid_argument(
            "--depth-min-mm and --depth-max-mm must be supplied together.");
    }
    if (has_min && options.depth_max_mm <= options.depth_min_mm) {
        throw std::invalid_argument("Depth range must satisfy min < max.");
    }
    if (!options.roi_enable_explicit && options.roi_bound_provided) {
        options.roi.enabled = true;
    }
    options.roi.initialized = options.roi_bound_provided;
    if (std::isnan(options.roi.min_x) || std::isnan(options.roi.max_x) ||
        std::isnan(options.roi.min_y) || std::isnan(options.roi.max_y) ||
        std::isnan(options.roi.min_z) || std::isnan(options.roi.max_z) ||
        options.roi.min_x > options.roi.max_x ||
        options.roi.min_y > options.roi.max_y ||
        options.roi.min_z > options.roi.max_z) {
        throw std::invalid_argument(
            "ROI bounds cannot be NaN and every minimum must be <= maximum.");
    }
    if (!std::isfinite(options.roi_step_mm) || options.roi_step_mm <= 0.0f) {
        throw std::invalid_argument("--roi-step-mm must be finite and > 0.");
    }
    return options;
}

std::string errorText(ERROR_CODE code)
{
    const char* text = cs::getCameraErrorString(code);
    return text ? text : "unknown";
}

void signalHandler(int)
{
    g_stop_requested.store(true);
}

const char* axisName(RoiAxis axis)
{
    switch (axis) {
        case RoiAxis::X: return "X";
        case RoiAxis::Y: return "Y";
        case RoiAxis::Z: return "Z";
    }
    return "?";
}

const char* limitName(RoiLimit limit)
{
    return limit == RoiLimit::Minimum ? "MIN" : "MAX";
}

float& selectedBound(RoiControls& controls)
{
    if (controls.selected_axis == RoiAxis::X) {
        return controls.selected_limit == RoiLimit::Minimum
            ? controls.bounds.min_x : controls.bounds.max_x;
    }
    if (controls.selected_axis == RoiAxis::Y) {
        return controls.selected_limit == RoiLimit::Minimum
            ? controls.bounds.min_y : controls.bounds.max_y;
    }
    return controls.selected_limit == RoiLimit::Minimum
        ? controls.bounds.min_z : controls.bounds.max_z;
}

float oppositeBound(const RoiControls& controls)
{
    if (controls.selected_axis == RoiAxis::X) {
        return controls.selected_limit == RoiLimit::Minimum
            ? controls.bounds.max_x : controls.bounds.min_x;
    }
    if (controls.selected_axis == RoiAxis::Y) {
        return controls.selected_limit == RoiLimit::Minimum
            ? controls.bounds.max_y : controls.bounds.min_y;
    }
    return controls.selected_limit == RoiLimit::Minimum
        ? controls.bounds.max_z : controls.bounds.min_z;
}

float matchingBound(const RoiBounds& bounds, RoiAxis axis, RoiLimit limit)
{
    if (axis == RoiAxis::X) {
        return limit == RoiLimit::Minimum ? bounds.min_x : bounds.max_x;
    }
    if (axis == RoiAxis::Y) {
        return limit == RoiLimit::Minimum ? bounds.min_y : bounds.max_y;
    }
    return limit == RoiLimit::Minimum ? bounds.min_z : bounds.max_z;
}

std::string roiValue(float value, int precision = 1)
{
    if (std::isinf(value)) return value < 0.0f ? "-inf" : "inf";
    std::ostringstream output;
    output << std::fixed << std::setprecision(precision) << value;
    return output.str();
}

std::string roiSummary(const RoiBounds& bounds)
{
    std::ostringstream output;
    output << "ROI " << (bounds.enabled ? "ON" : "OFF")
           << " | X[" << roiValue(bounds.min_x) << ", "
           << roiValue(bounds.max_x) << "]"
           << " Y[" << roiValue(bounds.min_y) << ", "
           << roiValue(bounds.max_y) << "]"
           << " Z[" << roiValue(bounds.min_z) << ", "
           << roiValue(bounds.max_z) << "] mm";
    return output.str();
}

void printRoiYaml(const RoiControls& controls)
{
    const RoiBounds& bounds = controls.bounds;
    std::cout << "\nROI YAML overrides (camera/PLY coordinates, mm):\n"
              << "      - \"roi.enable="
              << (bounds.enabled ? "true" : "false") << "\"\n"
              << "      - \"roi.min_x=" << roiValue(bounds.min_x, 3) << "\"\n"
              << "      - \"roi.max_x=" << roiValue(bounds.max_x, 3) << "\"\n"
              << "      - \"roi.min_y=" << roiValue(bounds.min_y, 3) << "\"\n"
              << "      - \"roi.max_y=" << roiValue(bounds.max_y, 3) << "\"\n"
              << "      - \"roi.min_z=" << roiValue(bounds.min_z, 3) << "\"\n"
              << "      - \"roi.max_z=" << roiValue(bounds.max_z, 3) << "\"\n"
              << "Edit target: " << axisName(controls.selected_axis) << ' '
              << limitName(controls.selected_limit)
              << ", step=" << roiValue(controls.step_mm, 2) << " mm\n\n";
}

bool validPoint(const cs::float3& point)
{
    return std::isfinite(point.x) && std::isfinite(point.y) &&
        std::isfinite(point.z) && point.z > 0.0f;
}

bool pointInsideRoi(const cs::float3& point, const RoiBounds& bounds)
{
    if (!validPoint(point)) return false;
    if (!bounds.enabled) return true;
    return point.x >= bounds.min_x && point.x <= bounds.max_x &&
        point.y >= bounds.min_y && point.y <= bounds.max_y &&
        point.z >= bounds.min_z && point.z <= bounds.max_z;
}

bool cloudBounds(
    const std::vector<cs::float3>& vertices, RoiBounds& output)
{
    output.min_x = output.min_y = output.min_z =
        std::numeric_limits<float>::infinity();
    output.max_x = output.max_y = output.max_z =
        -std::numeric_limits<float>::infinity();
    bool found = false;
    for (const cs::float3& point : vertices) {
        if (!validPoint(point)) continue;
        found = true;
        output.min_x = std::min(output.min_x, point.x);
        output.max_x = std::max(output.max_x, point.x);
        output.min_y = std::min(output.min_y, point.y);
        output.max_y = std::max(output.max_y, point.y);
        output.min_z = std::min(output.min_z, point.z);
        output.max_z = std::max(output.max_z, point.z);
    }
    output.initialized = found;
    return found;
}

void processRoiRequests(
    RoiControls& controls, const std::vector<cs::float3>& vertices)
{
    bool changed = false;
    if (controls.fit_requested ||
        (controls.pending_adjustment != 0 && !controls.bounds.initialized)) {
        RoiBounds fitted;
        if (cloudBounds(vertices, fitted)) {
            fitted.enabled = true;
            controls.bounds = fitted;
            changed = true;
            std::cout << "ROI fitted to the current valid cloud.\n";
        } else {
            std::cerr << "Warning: cannot fit ROI because the current cloud is empty.\n";
        }
        controls.fit_requested = false;
    }

    if (controls.pending_adjustment != 0 && controls.bounds.initialized) {
        float& active = selectedBound(controls);
        if (!std::isfinite(active)) {
            RoiBounds full_bounds;
            if (cloudBounds(vertices, full_bounds)) {
                active = matchingBound(
                    full_bounds, controls.selected_axis, controls.selected_limit);
            }
        }

        if (std::isfinite(active)) {
            const float candidate = active +
                controls.step_mm * static_cast<float>(controls.pending_adjustment);
            const float opposite = oppositeBound(controls);
            const bool valid_order = controls.selected_limit == RoiLimit::Minimum
                ? candidate <= opposite : candidate >= opposite;
            if (valid_order) {
                active = candidate;
                controls.bounds.enabled = true;
                changed = true;
            } else {
                std::cerr << "Warning: rejected ROI adjustment because MIN must be <= MAX.\n";
            }
        }
        controls.pending_adjustment = 0;
    }

    if (changed) {
        ++controls.revision;
        controls.print_requested = true;
    }
    if (controls.print_requested) {
        printRoiYaml(controls);
        controls.print_requested = false;
    }
}

void keyboardCallback(
    const pcl::visualization::KeyboardEvent& event, void* context)
{
    if (!event.keyDown()) return;
    const std::string key = event.getKeySym();
    auto* controls = static_cast<RoiControls*>(context);
    if (key == "s" || key == "S") {
        g_save_requested.store(true);
    } else if (key == "q" || key == "Q" || key == "Escape") {
        g_stop_requested.store(true);
    } else if (!controls) {
        return;
    } else if (key == "t" || key == "T") {
        controls->bounds.enabled = !controls->bounds.enabled;
        if (controls->bounds.enabled && !controls->bounds.initialized) {
            controls->fit_requested = true;
        }
        ++controls->revision;
        controls->print_requested = true;
    } else if (key == "b" || key == "B") {
        controls->fit_requested = true;
    } else if (key == "d" || key == "D") {
        controls->bounds = RoiBounds{};
        ++controls->revision;
        controls->print_requested = true;
    } else if (key == "k" || key == "K") {
        controls->print_requested = true;
    } else if (key == "x" || key == "X") {
        controls->selected_axis = RoiAxis::X;
    } else if (key == "y" || key == "Y") {
        controls->selected_axis = RoiAxis::Y;
    } else if (key == "z" || key == "Z") {
        controls->selected_axis = RoiAxis::Z;
    } else if (key == "n" || key == "N") {
        controls->selected_limit = RoiLimit::Minimum;
    } else if (key == "m" || key == "M") {
        controls->selected_limit = RoiLimit::Maximum;
    } else if (key == "bracketleft" || key == "[") {
        --controls->pending_adjustment;
    } else if (key == "bracketright" || key == "]") {
        ++controls->pending_adjustment;
    } else if (key == "comma" || key == ",") {
        controls->step_mm = std::max(0.1f, controls->step_mm * 0.5f);
        std::cout << "ROI adjustment step: " << controls->step_mm << " mm\n";
    } else if (key == "period" || key == ".") {
        controls->step_mm = std::min(100.0f, controls->step_mm * 2.0f);
        std::cout << "ROI adjustment step: " << controls->step_mm << " mm\n";
    }
}

void processViewerEvents(
    const pcl::visualization::PCLVisualizer::Ptr& viewer,
    int wait_milliseconds)
{
    // Ubuntu 22.04 自带 PCL 1.12.1 + VTK 9.1 的 spinOnce() 在 X11 下
    // 存在已知段错误：内部临时定时器会结束并销毁 Display，随后又调用
    // XPending。直接使用 VTK 9 的 ProcessEvents() 可避免 Start/TerminateApp
    // 路径，同时仍能处理鼠标、键盘和关闭窗口事件。
    const vtkSmartPointer<vtkRenderWindow> window = viewer->getRenderWindow();
    if (window) {
        window->Render();
        vtkRenderWindowInteractor* interactor = window->GetInteractor();
        if (interactor) interactor->ProcessEvents();
    }
    if (wait_milliseconds > 0) {
        std::this_thread::sleep_for(
            std::chrono::milliseconds(wait_milliseconds));
    }
}

std::vector<CameraInfo> discover(const Options& options)
{
    cs::setSdkEnableNetworking(options.network);
    cs::setEnableNetworking(options.network);
    cs::setSdkEnableLibuvc(options.usb);
    const cs::ISystemPtr system = cs::getSystemPtr();

    std::vector<CameraInfo> cameras;
    ERROR_CODE result = ERROR_UNKNOW;
    for (int attempt = 1; attempt <= options.discovery_attempts; ++attempt) {
        cameras.clear();
        result = system->queryCameras(cameras, options.discovery_timeout_ms);
        std::cout << "Discovery " << attempt << "/" << options.discovery_attempts
                  << ": error=" << static_cast<int>(result) << " ("
                  << errorText(result) << "), devices=" << cameras.size() << '\n';
        if (result == SUCCESS && !cameras.empty()) return cameras;
        if (attempt < options.discovery_attempts && options.retry_interval_ms > 0) {
            std::this_thread::sleep_for(
                std::chrono::milliseconds(options.retry_interval_ms));
        }
    }
    throw std::runtime_error(
        "No camera found. Stop the ROS camera node, check the camera network, "
        "and retry with the same permission method used by the verified camera test.");
}

CameraInfo selectCamera(
    const Options& options, const std::vector<CameraInfo>& cameras)
{
    if (options.serial.empty()) return cameras.front();
    for (const CameraInfo& info : cameras) {
        if (options.serial == info.serial) return info;
    }
    throw std::runtime_error("Camera serial not found: " + options.serial);
}

bool streamMatches(const StreamInfo& info, const Options& options)
{
    return info.format == STREAM_FORMAT_Z16 &&
        (options.width <= 0 || info.width == options.width) &&
        (options.height <= 0 || info.height == options.height) &&
        (options.fps <= 0.0 || std::abs(info.fps - options.fps) < 0.01);
}

class CameraGuard {
public:
    explicit CameraGuard(cs::ICameraPtr camera) : camera_(std::move(camera)) {}
    ~CameraGuard()
    {
        if (!camera_) return;
        if (depth_started_) camera_->stopStream(STREAM_TYPE_DEPTH);
        camera_->disconnect();
    }
    cs::ICameraPtr& camera() { return camera_; }
    void markDepthStarted() { depth_started_ = true; }

private:
    cs::ICameraPtr camera_;
    bool depth_started_ = false;
};

void setOptionalProperties(const Options& options, cs::ICameraPtr& camera)
{
    auto set_float = [&](PROPERTY_TYPE property, float value, const char* name) {
        if (!std::isfinite(value)) return;
        const ERROR_CODE code = camera->setProperty(STREAM_TYPE_DEPTH, property, value);
        if (code != SUCCESS) {
            std::cerr << "Warning: cannot set " << name << ": "
                      << errorText(code) << '\n';
        }
    };

    // 若需要增加曝光，应先增大 frame time，避免曝光超过当前帧时间上限。
    set_float(PROPERTY_FRAMETIME, options.frame_time, "frame time");
    set_float(PROPERTY_GAIN, options.gain, "gain");
    set_float(PROPERTY_EXPOSURE, options.exposure, "exposure");

    if (options.depth_min_mm >= 0) {
        PropertyExtension range{};
        range.depthRange.min = options.depth_min_mm;
        range.depthRange.max = options.depth_max_mm;
        const ERROR_CODE code = camera->setPropertyExtension(
            PROPERTY_EXT_DEPTH_RANGE, range);
        if (code != SUCCESS) {
            std::cerr << "Warning: cannot set depth range: "
                      << errorText(code) << '\n';
        }
    }
}

void printEffectiveDepthProperties(cs::ICameraPtr& camera)
{
    float gain = std::numeric_limits<float>::quiet_NaN();
    float exposure = std::numeric_limits<float>::quiet_NaN();
    float frame_time = std::numeric_limits<float>::quiet_NaN();
    const bool gain_ok =
        camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_GAIN, gain) == SUCCESS;
    const bool exposure_ok =
        camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_EXPOSURE, exposure) == SUCCESS;
    const bool frame_time_ok =
        camera->getProperty(STREAM_TYPE_DEPTH, PROPERTY_FRAMETIME, frame_time) == SUCCESS;

    PropertyExtension range{};
    const bool range_ok = camera->getPropertyExtension(
        PROPERTY_EXT_DEPTH_RANGE, range) == SUCCESS;

    std::cout << "Effective depth settings:";
    if (range_ok) {
        std::cout << " range=" << range.depthRange.min << ".."
                  << range.depthRange.max << " mm";
    } else {
        std::cout << " range=unavailable";
    }
    if (gain_ok) std::cout << ", gain=" << gain;
    if (exposure_ok) std::cout << ", exposure=" << exposure << " us";
    if (frame_time_ok) std::cout << ", frame_time=" << frame_time << " us";
    std::cout << '\n';
}

std::filesystem::path snapshotPath(const std::filesystem::path& directory)
{
    const auto now = std::chrono::system_clock::now();
    const auto time = std::chrono::system_clock::to_time_t(now);
    const auto milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    std::tm local{};
    localtime_r(&time, &local);

    std::ostringstream name;
    name << "live_" << std::put_time(&local, "%Y%m%d_%H%M%S")
         << '_' << std::setfill('0') << std::setw(3) << milliseconds.count()
         << ".ply";
    return std::filesystem::absolute(directory / name.str());
}

std::uint8_t channel(float value)
{
    return static_cast<std::uint8_t>(
        std::clamp(value, 0.0f, 1.0f) * 255.0f);
}

void depthColor(float t, std::uint8_t& r, std::uint8_t& g, std::uint8_t& b)
{
    // 简洁的蓝->青->绿->黄->红深度色带。
    const float four = 4.0f * std::clamp(t, 0.0f, 1.0f);
    r = channel(std::min(four - 1.5f, -four + 4.5f));
    g = channel(std::min(four - 0.5f, -four + 3.5f));
    b = channel(std::min(four + 0.5f, -four + 2.5f));
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr makeDisplayCloud(
    const std::vector<cs::float3>& vertices,
    const RoiBounds& roi,
    std::size_t max_points,
    std::size_t& retained_points)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    retained_points = 0;
    if (vertices.empty()) return cloud;

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const cs::float3& point : vertices) {
        if (!pointInsideRoi(point, roi)) continue;
        ++retained_points;
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    if (retained_points == 0 || !(max_z >= min_z)) return cloud;

    const std::size_t stride = std::max<std::size_t>(
        1, (retained_points + max_points - 1) / max_points);
    cloud->reserve((retained_points + stride - 1) / stride);
    const float span = std::max(1.0e-6f, max_z - min_z);

    std::size_t retained_index = 0;
    for (const cs::float3& source : vertices) {
        if (!pointInsideRoi(source, roi)) continue;
        if (retained_index++ % stride != 0) continue;
        pcl::PointXYZRGB point;
        point.x = source.x;
        point.y = source.y;
        point.z = source.z;
        depthColor((source.z - min_z) / span, point.r, point.g, point.b);
        cloud->push_back(point);
    }
    cloud->width = static_cast<std::uint32_t>(cloud->size());
    cloud->height = 1;
    cloud->is_dense = true;
    return cloud;
}

std::size_t filterPointcloudInPlace(
    cs::Pointcloud& pointcloud, const RoiBounds& roi)
{
    auto& vertices = pointcloud.getVertices();
    auto& normals = pointcloud.getNormals();
    auto& texcoords = pointcloud.getTexcoords();
    const std::size_t original_size = vertices.size();
    if (normals.size() != original_size) {
        throw std::runtime_error(
            "Cannot save PLY: SDK vertex/normal array sizes do not match.");
    }
    const bool textures_aligned = texcoords.size() == original_size;

    std::size_t output_index = 0;
    for (std::size_t input_index = 0; input_index < original_size; ++input_index) {
        if (!pointInsideRoi(vertices[input_index], roi)) continue;
        if (output_index != input_index) {
            vertices[output_index] = vertices[input_index];
            normals[output_index] = normals[input_index];
            if (textures_aligned) {
                texcoords[output_index] = texcoords[input_index];
            }
        }
        ++output_index;
    }
    vertices.resize(output_index);
    normals.resize(output_index);
    if (textures_aligned) texcoords.resize(output_index);
    return output_index;
}

bool finiteRoiBox(const RoiBounds& bounds)
{
    return bounds.enabled &&
        std::isfinite(bounds.min_x) && std::isfinite(bounds.max_x) &&
        std::isfinite(bounds.min_y) && std::isfinite(bounds.max_y) &&
        std::isfinite(bounds.min_z) && std::isfinite(bounds.max_z) &&
        bounds.min_x < bounds.max_x && bounds.min_y < bounds.max_y &&
        bounds.min_z < bounds.max_z;
}

void updateRoiBox(
    const pcl::visualization::PCLVisualizer::Ptr& viewer,
    const RoiControls& controls,
    bool& box_exists,
    std::size_t& displayed_revision)
{
    if (displayed_revision == controls.revision) return;
    if (box_exists) {
        viewer->removeShape("roi_box");
        box_exists = false;
    }
    if (finiteRoiBox(controls.bounds)) {
        const RoiBounds& bounds = controls.bounds;
        box_exists = viewer->addCube(
            bounds.min_x, bounds.max_x,
            bounds.min_y, bounds.max_y,
            bounds.min_z, bounds.max_z,
            1.0, 0.85, 0.1, "roi_box");
        if (box_exists) {
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION,
                pcl::visualization::PCL_VISUALIZER_REPRESENTATION_WIREFRAME,
                "roi_box");
            viewer->setShapeRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_LINE_WIDTH,
                2.0, "roi_box");
        }
    }
    displayed_revision = controls.revision;
}

void runViewer(const Options& options)
{
    const std::vector<CameraInfo> cameras = discover(options);
    const CameraInfo selected = selectCamera(options, cameras);
    std::cout << "Selected camera: name=" << selected.name
              << ", serial=" << selected.serial
              << ", unique_id=" << selected.uniqueId << '\n';

    CameraGuard guard(cs::getCameraPtr());
    ERROR_CODE result = guard.camera()->connect(selected);
    if (result != SUCCESS) {
        throw std::runtime_error("Camera connect failed: " + errorText(result));
    }

    std::vector<StreamInfo> streams;
    result = guard.camera()->getStreamInfos(STREAM_TYPE_DEPTH, streams);
    if (result != SUCCESS) {
        throw std::runtime_error("Cannot query depth streams: " + errorText(result));
    }

    bool started = false;
    for (const StreamInfo& info : streams) {
        std::cout << "Depth stream: format=" << static_cast<int>(info.format)
                  << ", " << info.width << 'x' << info.height
                  << " @ " << info.fps << " fps\n";
        if (!started && streamMatches(info, options)) {
            result = guard.camera()->startStream(STREAM_TYPE_DEPTH, info);
            if (result == SUCCESS) {
                started = true;
                guard.markDepthStarted();
                std::cout << "Selected Z16 stream: " << info.width << 'x'
                          << info.height << " @ " << info.fps << " fps\n";
            }
        }
    }
    if (!started) {
        throw std::runtime_error("No requested Z16 stream could be started.");
    }

    Intrinsics intrinsics{};
    result = guard.camera()->getIntrinsics(STREAM_TYPE_DEPTH, intrinsics);
    if (result != SUCCESS) {
        throw std::runtime_error("Cannot read depth intrinsics: " + errorText(result));
    }

    setOptionalProperties(options, guard.camera());
    printEffectiveDepthProperties(guard.camera());

    // 实时定位使用连续输出模式；正式生产节点仍保持软件触发单帧模式。
    PropertyExtension trigger{};
    trigger.triggerMode = TRIGGER_MODE_OFF;
    result = guard.camera()->setPropertyExtension(
        PROPERTY_EXT_TRIGGER_MODE, trigger);
    if (result != SUCCESS) {
        throw std::runtime_error(
            "Cannot enable continuous depth mode: " + errorText(result));
    }

    PropertyExtension scale_property{};
    float depth_scale = 0.1f;
    if (guard.camera()->getPropertyExtension(
            PROPERTY_EXT_DEPTH_SCALE, scale_property) == SUCCESS &&
        std::isfinite(scale_property.depthScale) &&
        scale_property.depthScale > 0.0f) {
        depth_scale = scale_property.depthScale;
    }

    std::filesystem::create_directories(options.save_dir);

    RoiControls roi_controls;
    roi_controls.bounds = options.roi;
    roi_controls.step_mm = options.roi_step_mm;
    roi_controls.fit_requested =
        roi_controls.bounds.enabled && !roi_controls.bounds.initialized;
    roi_controls.print_requested =
        roi_controls.bounds.enabled || options.roi_bound_provided;

    auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>(
        "Chishine3D Live Point Cloud");
    viewer->setBackgroundColor(0.04, 0.04, 0.06);
    viewer->addCoordinateSystem(100.0);
    viewer->addText("Waiting for depth frame...", 10, 10, 14, 1.0, 1.0, 1.0,
                    "status");
    viewer->addText("ROI OFF | camera/PLY XYZ in mm", 10, 32, 14,
                    0.3, 1.0, 0.4, "roi_status");
    viewer->addText("Edit X MIN | step 5.0 mm", 10, 54, 14,
                    0.4, 0.9, 1.0, "roi_edit");
    viewer->addText(
        "T:ROI  B:fit  D:clear  X/Y/Z:axis  N/M:min/max  [ ]:adjust  , .:step  K:print  S:save  Q:quit",
        10, 76, 12, 0.9, 0.9, 0.3, "help");
    viewer->registerKeyboardCallback(keyboardCallback, &roi_controls);
    if (viewer->getRenderWindow() &&
        viewer->getRenderWindow()->GetInteractor()) {
        viewer->getRenderWindow()->GetInteractor()->Initialize();
    }

    bool cloud_added = false;
    std::size_t frame_number = 0;
    auto previous_time = std::chrono::steady_clock::now();
    double smoothed_fps = 0.0;
    bool roi_box_exists = false;
    std::size_t displayed_roi_revision =
        std::numeric_limits<std::size_t>::max();

    std::cout
        << "Live viewer started. Default ROI is OFF (full cloud).\n"
        << "ROI keys: T=on/off, B=fit current cloud, D=clear, K=print YAML,\n"
        << "          X/Y/Z=axis, N/M=min/max, [ ]=adjust, , .=step.\n"
        << "S saves exactly the current ROI cloud; Q/Esc quits.\n";
    while (!g_stop_requested.load() && !viewer->wasStopped()) {
        cs::IFramePtr frame;
        result = guard.camera()->getFrame(
            STREAM_TYPE_DEPTH, frame, options.frame_timeout_ms);
        if (result != SUCCESS || !frame) {
            std::cerr << "Warning: depth frame failed: " << errorText(result) << '\n';
            processViewerEvents(viewer, 10);
            continue;
        }
        if (frame->getFormat() != STREAM_FORMAT_Z16 || !frame->getData()) {
            std::cerr << "Warning: ignored non-Z16 or empty frame.\n";
            processViewerEvents(viewer, 10);
            continue;
        }

        const auto* depth_data = reinterpret_cast<const unsigned short*>(
            frame->getData());
        const std::size_t depth_pixel_count =
            static_cast<std::size_t>(frame->getWidth()) *
            static_cast<std::size_t>(frame->getHeight());
        std::size_t raw_valid_count = 0;
        unsigned short raw_min = std::numeric_limits<unsigned short>::max();
        unsigned short raw_max = 0;
        for (std::size_t i = 0; i < depth_pixel_count; ++i) {
            if (depth_data[i] == 0) continue;
            ++raw_valid_count;
            raw_min = std::min(raw_min, depth_data[i]);
            raw_max = std::max(raw_max, depth_data[i]);
        }

        cs::Pointcloud pointcloud;
        pointcloud.generatePoints(
            const_cast<unsigned short*>(depth_data),
            frame->getWidth(), frame->getHeight(), depth_scale,
            &intrinsics, nullptr, nullptr, true);

        const std::vector<cs::float3>& vertices = pointcloud.getVertices();
        processRoiRequests(roi_controls, vertices);
        std::size_t roi_point_count = 0;
        auto display_cloud = makeDisplayCloud(
            vertices, roi_controls.bounds,
            options.max_display_points, roi_point_count);

        if (!cloud_added) {
            viewer->addPointCloud<pcl::PointXYZRGB>(display_cloud, "live_cloud");
            viewer->setPointCloudRenderingProperties(
                pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
                options.point_size, "live_cloud");
            viewer->resetCameraViewpoint("live_cloud");
            viewer->resetCamera();
            cloud_added = true;
        } else {
            viewer->updatePointCloud<pcl::PointXYZRGB>(
                display_cloud, "live_cloud");
        }
        updateRoiBox(
            viewer, roi_controls, roi_box_exists, displayed_roi_revision);

        const auto now = std::chrono::steady_clock::now();
        const double seconds = std::chrono::duration<double>(
            now - previous_time).count();
        previous_time = now;
        const double instant_fps = seconds > 0.0 ? 1.0 / seconds : 0.0;
        smoothed_fps = frame_number == 0
            ? instant_fps
            : 0.90 * smoothed_fps + 0.10 * instant_fps;
        ++frame_number;

        std::ostringstream status;
        const double valid_percent = depth_pixel_count == 0
            ? 0.0
            : 100.0 * static_cast<double>(raw_valid_count) /
                static_cast<double>(depth_pixel_count);
        status << "Frame " << frame_number
               << " | valid " << raw_valid_count << "/" << depth_pixel_count
               << " (" << std::fixed << std::setprecision(1)
               << valid_percent << "%)";
        if (raw_valid_count > 0) {
            status << " | Z " << std::setprecision(0)
                   << raw_min * depth_scale << ".."
                   << raw_max * depth_scale << " mm";
        }
        status << " | displayed " << display_cloud->size()
               << " | " << std::setprecision(1)
               << smoothed_fps << " Hz";
        viewer->updateText(status.str(), 10, 10, "status");
        std::ostringstream roi_status;
        roi_status << roiSummary(roi_controls.bounds)
                   << " | retained " << roi_point_count;
        viewer->updateText(roi_status.str(), 10, 32, "roi_status");
        std::ostringstream roi_edit;
        roi_edit << "Edit " << axisName(roi_controls.selected_axis) << ' '
                 << limitName(roi_controls.selected_limit)
                 << " | step " << roiValue(roi_controls.step_mm, 2) << " mm";
        viewer->updateText(roi_edit.str(), 10, 54, "roi_edit");
        processViewerEvents(viewer, 1);

        // 键盘事件在 ProcessEvents() 内触发；立即应用到本帧，保证随后按 S
        // 保存时使用刚刚看到并调好的 ROI，而不是上一轮边界。
        processRoiRequests(roi_controls, vertices);

        if (g_save_requested.exchange(false)) {
            const std::size_t original_points =
                static_cast<std::size_t>(pointcloud.size());
            const std::size_t saved_points = roi_controls.bounds.enabled
                ? filterPointcloudInPlace(pointcloud, roi_controls.bounds)
                : original_points;
            if (saved_points == 0) {
                std::cerr
                    << "Warning: current ROI contains zero points; snapshot was not saved.\n";
                continue;
            }
            const std::filesystem::path path = snapshotPath(options.save_dir);
            pointcloud.exportToFile(
                path.string(), nullptr, 0, 0, options.binary_ply);
            if (std::filesystem::exists(path) &&
                std::filesystem::file_size(path) > 0) {
                std::cout << "Saved snapshot: " << path
                          << " (points=" << saved_points << "/"
                          << original_points
                          << ", ROI="
                          << (roi_controls.bounds.enabled ? "on" : "off")
                          << ")\n";
                printRoiYaml(roi_controls);
            } else {
                std::cerr << "Warning: snapshot write failed: " << path << '\n';
            }
        }
    }
}

}  // namespace

int main(int argc, char** argv)
{
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    try {
        runViewer(parseOptions(argc, argv));
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << "\n\n" << help();
        return 1;
    }
}
