#include <3DCamera.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace {

struct Options {
    bool list_only = true;
    bool network = true;
    bool usb = true;
    bool binary_ply = true;
    std::string output_ply;
    std::string serial;
    int discovery_timeout_ms = 3000;
    int discovery_attempts = 10;
    int retry_interval_ms = 1000;
    int capture_timeout_ms = 5000;
    int width = 0;
    int height = 0;
    double fps = 0.0;
    int depth_min_mm = 460;
    int depth_max_mm = 520;
    float gain = 1.0f;
    float exposure = 8000.0f;
    float frame_time = 10000.0f;
};

std::string help()
{
    return
        "Usage:\n"
        "  chishine_camera_test --list [options]\n"
        "  chishine_camera_test --capture output.ply [options]\n\n"
        "Options:\n"
        "  --serial TEXT                 Select a camera serial (default: first)\n"
        "  --network true|false          Enable network discovery (default: true)\n"
        "  --usb true|false              Enable USB/UVC discovery (default: true)\n"
        "  --attempts N                  Discovery attempts (default: 10)\n"
        "  --discovery-timeout-ms N      Per-attempt timeout (default: 3000)\n"
        "  --retry-ms N                  Delay between attempts (default: 1000)\n"
        "  --capture-timeout-ms N        Frame timeout (default: 5000)\n"
        "  --width N --height N --fps F  Select depth stream; 0 means any\n"
        "  --depth-min-mm N --depth-max-mm N\n"
        "  --gain F --exposure F --frame-time F\n"
        "  --ascii                       Export ASCII PLY (binary is default)\n"
        "  -h, --help                    Show this help\n";
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
        } else if (arg == "--list") {
            options.list_only = true;
            options.output_ply.clear();
        } else if (arg == "--capture") {
            options.output_ply = requireValue(i, argc, argv);
            options.list_only = false;
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
        } else if (arg == "--capture-timeout-ms") {
            options.capture_timeout_ms = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--width") {
            options.width = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--height") {
            options.height = std::stoi(requireValue(i, argc, argv));
        } else if (arg == "--fps") {
            options.fps = std::stod(requireValue(i, argc, argv));
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
        } else if (arg == "--ascii") {
            options.binary_ply = false;
        } else {
            throw std::invalid_argument("Unknown option: " + arg);
        }
    }
    if (options.discovery_attempts <= 0 || options.discovery_timeout_ms < 0 ||
        options.retry_interval_ms < 0 || options.capture_timeout_ms <= 0) {
        throw std::invalid_argument("Timeout/attempt parameters are invalid.");
    }
    if (options.depth_min_mm < 0 ||
        options.depth_max_mm <= options.depth_min_mm) {
        throw std::invalid_argument("Depth range must satisfy 0 <= min < max.");
    }
    return options;
}

std::string errorText(ERROR_CODE code)
{
    const char* text = cs::getCameraErrorString(code);
    return text ? text : "unknown";
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
        "No camera found. Check NIC link/IP/subnet/firewall for network cameras, "
        "or lsusb/cable/power/permissions for USB cameras. Last SDK error=" +
        std::to_string(static_cast<int>(result)) + " (" + errorText(result) + ")");
}

void printCameras(const std::vector<CameraInfo>& cameras)
{
    for (std::size_t i = 0; i < cameras.size(); ++i) {
        const CameraInfo& info = cameras[i];
        std::cout << "[" << i << "] name=" << info.name
                  << ", serial=" << info.serial
                  << ", unique_id=" << info.uniqueId
                  << ", firmware=" << info.firmwareVersion
                  << ", algorithm=" << info.algorithmVersion << '\n';
    }
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

void capture(const Options& options, const std::vector<CameraInfo>& cameras)
{
    CameraInfo selected = cameras.front();
    if (!options.serial.empty()) {
        bool found = false;
        for (const CameraInfo& info : cameras) {
            if (options.serial == info.serial) {
                selected = info;
                found = true;
                break;
            }
        }
        if (!found) throw std::runtime_error("Camera serial not found: " + options.serial);
    }

    CameraGuard guard(cs::getCameraPtr());
    if (guard.camera()->connect(selected) != SUCCESS) {
        throw std::runtime_error("Cannot connect camera serial=" +
            std::string(selected.serial));
    }

    std::vector<StreamInfo> streams;
    ERROR_CODE result = guard.camera()->getStreamInfos(STREAM_TYPE_DEPTH, streams);
    if (result != SUCCESS) {
        throw std::runtime_error("Cannot query depth streams: " + errorText(result));
    }
    bool started = false;
    for (const StreamInfo& info : streams) {
        std::cout << "Depth stream: format=" << static_cast<int>(info.format)
                  << ", " << info.width << "x" << info.height
                  << " @ " << info.fps << " fps\n";
        if (!started && streamMatches(info, options)) {
            result = guard.camera()->startStream(STREAM_TYPE_DEPTH, info);
            if (result == SUCCESS) {
                guard.markDepthStarted();
                started = true;
                std::cout << "Selected Z16 stream: " << info.width << "x"
                          << info.height << " @ " << info.fps << " fps\n";
            }
        }
    }
    if (!started) throw std::runtime_error("No requested Z16 stream could be started.");

    Intrinsics intrinsics{};
    if (guard.camera()->getIntrinsics(STREAM_TYPE_DEPTH, intrinsics) != SUCCESS) {
        throw std::runtime_error("Cannot read depth intrinsics.");
    }

    auto setOptional = [&](PROPERTY_TYPE property, float value, const char* name) {
        const ERROR_CODE code = guard.camera()->setProperty(
            STREAM_TYPE_DEPTH, property, value);
        if (code != SUCCESS) {
            std::cerr << "Warning: cannot set " << name << ": "
                      << errorText(code) << '\n';
        }
    };
    setOptional(PROPERTY_GAIN, options.gain, "gain");
    setOptional(PROPERTY_FRAMETIME, options.frame_time, "frame time");
    setOptional(PROPERTY_EXPOSURE, options.exposure, "exposure");

    PropertyExtension range{};
    range.depthRange.min = options.depth_min_mm;
    range.depthRange.max = options.depth_max_mm;
    result = guard.camera()->setPropertyExtension(PROPERTY_EXT_DEPTH_RANGE, range);
    if (result != SUCCESS) {
        std::cerr << "Warning: cannot set depth range: " << errorText(result) << '\n';
    }

    PropertyExtension trigger{};
    trigger.triggerMode = TRIGGER_MODE_SOFTWAER;
    result = guard.camera()->setPropertyExtension(PROPERTY_EXT_TRIGGER_MODE, trigger);
    if (result != SUCCESS) {
        throw std::runtime_error("Cannot enable software trigger: " + errorText(result));
    }
    result = guard.camera()->softTrigger();
    if (result != SUCCESS) {
        throw std::runtime_error("softTrigger failed: " + errorText(result));
    }

    cs::IFramePtr frame;
    result = guard.camera()->getFrame(
        STREAM_TYPE_DEPTH, frame, options.capture_timeout_ms);
    if (result != SUCCESS || !frame) {
        throw std::runtime_error("Depth frame failed: " + errorText(result));
    }
    if (frame->getFormat() != STREAM_FORMAT_Z16 || !frame->getData()) {
        throw std::runtime_error("Returned frame is not valid Z16 data.");
    }

    PropertyExtension scale_property{};
    float scale = 0.1f;
    if (guard.camera()->getPropertyExtension(
            PROPERTY_EXT_DEPTH_SCALE, scale_property) == SUCCESS) {
        scale = scale_property.depthScale;
    }
    if (!std::isfinite(scale) || scale <= 0.0f) scale = 0.1f;

    cs::Pointcloud pointcloud;
    pointcloud.generatePoints(
        reinterpret_cast<unsigned short*>(const_cast<char*>(frame->getData())),
        frame->getWidth(), frame->getHeight(), scale,
        &intrinsics, nullptr, nullptr, true);
    if (pointcloud.size() == 0) {
        throw std::runtime_error("Point reconstruction returned zero valid points.");
    }

    const std::filesystem::path output =
        std::filesystem::absolute(options.output_ply);
    if (!output.parent_path().empty()) {
        std::filesystem::create_directories(output.parent_path());
    }
    pointcloud.exportToFile(output.string(), nullptr, 0, 0, options.binary_ply);
    if (!std::filesystem::exists(output) || std::filesystem::file_size(output) == 0) {
        throw std::runtime_error("SDK did not create a valid PLY file.");
    }
    std::cout << "Captured " << pointcloud.size() << " points to "
              << output.string() << '\n';
}

}  // namespace

int main(int argc, char** argv)
{
    try {
        const Options options = parseOptions(argc, argv);
        const std::vector<CameraInfo> cameras = discover(options);
        printCameras(cameras);
        if (!options.list_only) capture(options, cameras);
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "Error: " << exception.what() << "\n\n" << help();
        return 1;
    }
}