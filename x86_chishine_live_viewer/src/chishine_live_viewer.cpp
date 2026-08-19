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
        "  -h, --help                    Show this help\n\n"
        "Viewer keys: S=save current full PLY, Q/Esc=quit.\n";
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

void keyboardCallback(
    const pcl::visualization::KeyboardEvent& event, void*)
{
    if (!event.keyDown()) return;
    const std::string key = event.getKeySym();
    if (key == "s" || key == "S") {
        g_save_requested.store(true);
    } else if (key == "q" || key == "Q" || key == "Escape") {
        g_stop_requested.store(true);
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
    const std::vector<cs::float3>& vertices, std::size_t max_points)
{
    auto cloud = pcl::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    if (vertices.empty()) return cloud;

    float min_z = std::numeric_limits<float>::max();
    float max_z = std::numeric_limits<float>::lowest();
    for (const cs::float3& point : vertices) {
        if (!std::isfinite(point.z) || point.z <= 0.0f) continue;
        min_z = std::min(min_z, point.z);
        max_z = std::max(max_z, point.z);
    }
    if (!(max_z >= min_z)) return cloud;

    const std::size_t stride = std::max<std::size_t>(
        1, (vertices.size() + max_points - 1) / max_points);
    cloud->reserve((vertices.size() + stride - 1) / stride);
    const float span = std::max(1.0e-6f, max_z - min_z);

    for (std::size_t i = 0; i < vertices.size(); i += stride) {
        const cs::float3& source = vertices[i];
        if (!std::isfinite(source.x) || !std::isfinite(source.y) ||
            !std::isfinite(source.z) || source.z <= 0.0f) {
            continue;
        }
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

    auto viewer = pcl::make_shared<pcl::visualization::PCLVisualizer>(
        "Chishine3D Live Point Cloud");
    viewer->setBackgroundColor(0.04, 0.04, 0.06);
    viewer->addCoordinateSystem(100.0);
    viewer->addText("Waiting for depth frame...", 10, 10, 14, 1.0, 1.0, 1.0,
                    "status");
    viewer->addText("S: save PLY    Q/Esc: quit", 10, 32, 14,
                    0.9, 0.9, 0.3, "help");
    viewer->registerKeyboardCallback(keyboardCallback, nullptr);
    if (viewer->getRenderWindow() &&
        viewer->getRenderWindow()->GetInteractor()) {
        viewer->getRenderWindow()->GetInteractor()->Initialize();
    }

    bool cloud_added = false;
    std::size_t frame_number = 0;
    auto previous_time = std::chrono::steady_clock::now();
    double smoothed_fps = 0.0;

    std::cout << "Live viewer started. S=save full PLY, Q/Esc=quit.\n";
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
        auto display_cloud = makeDisplayCloud(
            vertices, options.max_display_points);

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
        processViewerEvents(viewer, 1);

        if (g_save_requested.exchange(false)) {
            const std::filesystem::path path = snapshotPath(options.save_dir);
            pointcloud.exportToFile(
                path.string(), nullptr, 0, 0, options.binary_ply);
            if (std::filesystem::exists(path) &&
                std::filesystem::file_size(path) > 0) {
                std::cout << "Saved snapshot: " << path
                          << " (points=" << pointcloud.size() << ")\n";
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