#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <3DCamera.hpp>

#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

namespace {

template <typename T>
bool streamMatches(const T& info, int width, int height, double fps)
{
    return (width <= 0 || info.width == width) &&
           (height <= 0 || info.height == height) &&
           (fps <= 0.0 || std::abs(info.fps - fps) < 0.01);
}

std::string timestampSuffix()
{
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::to_string(
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
}

std::filesystem::path defaultDataRoot()
{
    if (const char* configured = std::getenv("SCUT_WELD_DATA_ROOT")) {
        if (*configured) return configured;
    }
    if (const char* home = std::getenv("HOME")) {
        if (*home) return std::filesystem::path(home) / "scut_weld_data";
    }
    return std::filesystem::temp_directory_path() / "scut_weld_data";
}

}  // namespace

class ChishineCameraNode : public rclcpp::Node {
public:
    ChishineCameraNode()
        : Node("chishine_camera_node")
    {
        camera_serial_ = declare_parameter<std::string>("camera_serial", "");
        // 厂商 SDK 按“设备发现 -> 序列号选择 -> connect(CameraInfo)”连接，
        // 不提供 connect(camera_ip, host_ip) 形式的公开 C++ 接口。
        enable_network_discovery_ = declare_parameter<bool>(
            "enable_network_discovery", true);
        enable_usb_discovery_ = declare_parameter<bool>(
            "enable_usb_discovery", true);
        discovery_timeout_ms_ = declare_parameter<int>(
            "discovery_timeout_ms", 3000);
        // 厂商示例也会重复枚举。相机/网卡刚上电时，一次枚举为空不应立即退出。
        discovery_attempts_ = declare_parameter<int>("discovery_attempts", 10);
        discovery_retry_interval_ms_ = declare_parameter<int>(
            "discovery_retry_interval_ms", 1000);
        output_directory_ = declare_parameter<std::string>(
            "output_directory", "");
        if (output_directory_.empty()) {
            output_directory_ = (defaultDataRoot() / "pointclouds").string();
        }
        file_prefix_ = declare_parameter<std::string>("file_prefix", "capture");
        capture_timeout_ms_ = declare_parameter<int>("capture_timeout_ms", 5000);
        depth_width_ = declare_parameter<int>("depth_width", 0);
        depth_height_ = declare_parameter<int>("depth_height", 0);
        depth_fps_ = declare_parameter<double>("depth_fps", 0.0);
        rgb_width_ = declare_parameter<int>("rgb_width", 0);
        rgb_height_ = declare_parameter<int>("rgb_height", 0);
        rgb_fps_ = declare_parameter<double>("rgb_fps", 0.0);
        // 焊缝算法只消费 XYZ；默认关闭 RGB 可减少取帧时间，并保留
        // 所有有效深度点。需要带颜色的原始 PLY 时再通过 YAML 开启。
        enable_rgb_ = declare_parameter<bool>("enable_rgb", false);
        binary_ply_ = declare_parameter<bool>("binary_ply", true);
        depth_min_mm_ = declare_parameter<int>("depth_min_mm", 460);
        depth_max_mm_ = declare_parameter<int>("depth_max_mm", 520);
        depth_gain_ = declare_parameter<double>("depth_gain", 1.0);
        depth_exposure_ = declare_parameter<double>("depth_exposure", 8000.0);
        depth_frame_time_ = declare_parameter<double>("depth_frame_time", 10000.0);

        if (discovery_timeout_ms_ < 0) {
            throw std::invalid_argument("discovery_timeout_ms must be >= 0.");
        }
        if (discovery_attempts_ <= 0 || discovery_retry_interval_ms_ < 0) {
            throw std::invalid_argument(
                "discovery_attempts must be > 0 and "
                "discovery_retry_interval_ms must be >= 0.");
        }
        if (capture_timeout_ms_ <= 0) {
            throw std::invalid_argument("capture_timeout_ms must be > 0.");
        }
        if (depth_min_mm_ < 0 || depth_max_mm_ <= depth_min_mm_) {
            throw std::invalid_argument(
                "Depth range must satisfy 0 <= depth_min_mm < depth_max_mm.");
        }
        if (!std::isfinite(depth_gain_) || depth_gain_ < 0.0 ||
            !std::isfinite(depth_exposure_) || depth_exposure_ <= 0.0 ||
            !std::isfinite(depth_frame_time_) || depth_frame_time_ <= 0.0) {
            throw std::invalid_argument(
                "Depth gain/exposure/frame time contains an invalid value.");
        }

        pointcloud_path_pub_ = create_publisher<std_msgs::msg::String>(
            "/camera/pointcloud_file", rclcpp::QoS(1).transient_local());
        capture_service_ = create_service<std_srvs::srv::Trigger>(
            "/camera/capture",
            std::bind(&ChishineCameraNode::captureCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        try {
            initializeCamera();
        } catch (...) {
            shutdownCameraNoThrow();
            throw;
        }
        RCLCPP_INFO(get_logger(),
            "Camera ready; output=%s. Call: ros2 service call /camera/capture "
            "std_srvs/srv/Trigger {} ", output_directory_.c_str());
    }

    ~ChishineCameraNode() override
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        shutdownCameraNoThrow();
    }

private:
    void initializeCamera()
    {
        // 显式启用厂商 SDK 的网络/USB 发现，避免依赖动态库内部默认值。
        // Jetson 主机 IP 仍由 Linux 网卡配置，必须与网络相机可路由/同子网。
        // 3.2.52 同时公开了 SDK 层和网络功能层两个开关；保持二者一致，
        // 避免某个平台只启用了其中一层而导致 queryCameras() 返回空列表。
        cs::setSdkEnableNetworking(enable_network_discovery_);
        cs::setEnableNetworking(enable_network_discovery_);
        cs::setSdkEnableLibuvc(enable_usb_discovery_);
        RCLCPP_INFO(get_logger(),
            "Camera discovery: network=%s, usb_uvc=%s, timeout=%d ms, "
            "attempts=%d, retry_interval=%d ms",
            enable_network_discovery_ ? "enabled" : "disabled",
            enable_usb_discovery_ ? "enabled" : "disabled",
            discovery_timeout_ms_, discovery_attempts_,
            discovery_retry_interval_ms_);

        std::vector<CameraInfo> cameras;
        const cs::ISystemPtr system = cs::getSystemPtr();
        ERROR_CODE query_result = ERROR_UNKNOW;
        for (int attempt = 1; attempt <= discovery_attempts_; ++attempt) {
            cameras.clear();
            query_result = system->queryCameras(cameras, discovery_timeout_ms_);
            if (query_result == SUCCESS && !cameras.empty()) {
                RCLCPP_INFO(get_logger(),
                    "Camera discovery succeeded on attempt %d/%d: %zu device(s).",
                    attempt, discovery_attempts_, cameras.size());
                break;
            }

            const char* error_text = cs::getCameraErrorString(query_result);
            RCLCPP_WARN(get_logger(),
                "Camera discovery attempt %d/%d returned error=%d (%s), "
                "devices=%zu.",
                attempt, discovery_attempts_, static_cast<int>(query_result),
                error_text ? error_text : "unknown", cameras.size());
            if (attempt < discovery_attempts_ &&
                discovery_retry_interval_ms_ > 0) {
                std::this_thread::sleep_for(std::chrono::milliseconds(
                    discovery_retry_interval_ms_));
            }
        }
        if (query_result != SUCCESS || cameras.empty()) {
            const char* error_text = cs::getCameraErrorString(query_result);
            throw std::runtime_error(
                "No Chishine3D camera was detected after " +
                std::to_string(discovery_attempts_) +
                " attempts; last query error=" +
                std::to_string(static_cast<int>(query_result)) + " (" +
                (error_text ? std::string(error_text) : std::string("unknown")) +
                "). Run the vendor SampleSystemQueryCameras first. For an IP "
                "camera, check the Jetson NIC link/address/subnet/route/firewall; "
                "for USB, check lsusb, cable, power and permissions.");
        }

        CameraInfo selected = cameras.front();
        if (!camera_serial_.empty()) {
            bool found = false;
            for (const CameraInfo& info : cameras) {
                if (camera_serial_ == info.serial) {
                    selected = info;
                    found = true;
                    break;
                }
            }
            if (!found) {
                throw std::runtime_error(
                    "Configured camera serial was not found: " + camera_serial_);
            }
        }

        camera_ = cs::getCameraPtr();
        const ERROR_CODE connect_result = camera_->connect(selected);
        if (connect_result != SUCCESS) {
            throw std::runtime_error(
                "Camera connect failed, error=" + std::to_string(connect_result));
        }
        RCLCPP_INFO(get_logger(),
            "Connected camera serial=%s, name=%s, unique_id=%s",
            selected.serial, selected.name, selected.uniqueId);

        startDepthStream();
        if (enable_rgb_ && !startRgbStream()) {
            enable_rgb_ = false;
            RCLCPP_WARN(get_logger(),
                "RGB8 is unavailable; continuing with depth-only PLY output.");
        }

        if (camera_->getIntrinsics(STREAM_TYPE_DEPTH, depth_intrinsics_) != SUCCESS) {
            throw std::runtime_error("Cannot read depth intrinsics.");
        }
        if (enable_rgb_) {
            if (camera_->getIntrinsics(STREAM_TYPE_RGB, rgb_intrinsics_) != SUCCESS ||
                camera_->getExtrinsics(rgb_extrinsics_) != SUCCESS) {
                camera_->stopStream(STREAM_TYPE_RGB);
                rgb_started_ = false;
                enable_rgb_ = false;
                RCLCPP_WARN(get_logger(),
                    "Cannot read RGB calibration; continuing with depth-only PLY output.");
            }
        }

        auto set_optional_property = [this](
            PROPERTY_TYPE property, float value, const char* name) {
            const ERROR_CODE result = camera_->setProperty(
                STREAM_TYPE_DEPTH, property, value);
            if (result != SUCCESS) {
                RCLCPP_WARN(get_logger(), "Cannot set %s, error=%d; using camera value.",
                    name, static_cast<int>(result));
            }
        };
        set_optional_property(
            PROPERTY_GAIN, static_cast<float>(depth_gain_), "depth gain");
        set_optional_property(
            PROPERTY_FRAMETIME, static_cast<float>(depth_frame_time_),
            "depth frame time");
        set_optional_property(
            PROPERTY_EXPOSURE, static_cast<float>(depth_exposure_),
            "depth exposure");

        PropertyExtension depth_range{};
        depth_range.depthRange.min = depth_min_mm_;
        depth_range.depthRange.max = depth_max_mm_;
        const ERROR_CODE range_result = camera_->setPropertyExtension(
            PROPERTY_EXT_DEPTH_RANGE, depth_range);
        if (range_result != SUCCESS) {
            RCLCPP_WARN(get_logger(),
                "Cannot set depth range, error=%d; using camera value.",
                static_cast<int>(range_result));
        }

        PropertyExtension trigger_mode{};
        trigger_mode.triggerMode = TRIGGER_MODE_SOFTWAER;
        const ERROR_CODE trigger_result = camera_->setPropertyExtension(
            PROPERTY_EXT_TRIGGER_MODE, trigger_mode);
        if (trigger_result != SUCCESS) {
            throw std::runtime_error(
                "Cannot enable software trigger, error=" +
                std::to_string(trigger_result));
        }
    }

    void startDepthStream()
    {
        std::vector<StreamInfo> infos;
        if (camera_->getStreamInfos(STREAM_TYPE_DEPTH, infos) != SUCCESS) {
            throw std::runtime_error("Cannot query depth stream formats.");
        }
        for (const StreamInfo& info : infos) {
            if (info.format == STREAM_FORMAT_Z16 &&
                streamMatches(info, depth_width_, depth_height_, depth_fps_)) {
                if (camera_->startStream(STREAM_TYPE_DEPTH, info) != SUCCESS) {
                    throw std::runtime_error("Cannot start the selected depth stream.");
                }
                depth_started_ = true;
                RCLCPP_INFO(get_logger(), "Depth stream: %dx%d @ %.1f fps",
                    info.width, info.height, info.fps);
                return;
            }
        }
        throw std::runtime_error("No matching Z16 depth stream was found.");
    }

    bool startRgbStream()
    {
        std::vector<StreamInfo> infos;
        if (camera_->getStreamInfos(STREAM_TYPE_RGB, infos) != SUCCESS) {
            return false;
        }
        for (const StreamInfo& info : infos) {
            if (info.format == STREAM_FORMAT_RGB8 &&
                streamMatches(info, rgb_width_, rgb_height_, rgb_fps_)) {
                if (camera_->startStream(STREAM_TYPE_RGB, info) != SUCCESS) {
                    return false;
                }
                rgb_started_ = true;
                RCLCPP_INFO(get_logger(), "RGB stream: %dx%d @ %.1f fps",
                    info.width, info.height, info.fps);
                return true;
            }
        }
        return false;
    }

    void shutdownCameraNoThrow() noexcept
    {
        if (!camera_) return;
        if (depth_started_) {
            camera_->stopStream(STREAM_TYPE_DEPTH);
            depth_started_ = false;
        }
        if (rgb_started_) {
            camera_->stopStream(STREAM_TYPE_RGB);
            rgb_started_ = false;
        }
        camera_->disconnect();
        camera_.reset();
    }

    void captureCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        std::lock_guard<std::mutex> lock(camera_mutex_);
        try {
            const std::string path = captureOnePointCloud();
            response->success = true;
            response->message = path;
            std_msgs::msg::String message;
            message.data = path;
            pointcloud_path_pub_->publish(message);
            RCLCPP_INFO(get_logger(), "Captured point cloud: %s", path.c_str());
        } catch (const std::exception& exception) {
            response->success = false;
            response->message = exception.what();
            RCLCPP_ERROR(get_logger(), "Capture failed: %s", exception.what());
        }
    }

    std::string captureOnePointCloud()
    {
        if (!camera_) throw std::runtime_error("Camera is not connected.");
        const ERROR_CODE trigger_result = camera_->softTrigger();
        if (trigger_result != SUCCESS) {
            throw std::runtime_error(
                "softTrigger failed, error=" + std::to_string(trigger_result));
        }

        cs::IFramePtr depth_frame;
        cs::IFramePtr rgb_frame;
        ERROR_CODE frame_result = SUCCESS;
        if (enable_rgb_) {
            frame_result = camera_->getPairedFrame(
                depth_frame, rgb_frame, capture_timeout_ms_);
        } else {
            frame_result = camera_->getFrame(
                STREAM_TYPE_DEPTH, depth_frame, capture_timeout_ms_);
        }
        if (frame_result != SUCCESS || !depth_frame) {
            throw std::runtime_error(
                "Frame acquisition failed, error=" + std::to_string(frame_result));
        }

        PropertyExtension scale_property{};
        float depth_scale = 0.1f;
        if (camera_->getPropertyExtension(
                PROPERTY_EXT_DEPTH_SCALE, scale_property) == SUCCESS) {
            depth_scale = scale_property.depthScale;
        }
        if (!std::isfinite(depth_scale) || depth_scale <= 0.0f) {
            RCLCPP_WARN(get_logger(),
                "Camera returned invalid depth scale; falling back to 0.1 mm/unit.");
            depth_scale = 0.1f;
        }

        // startDepthStream()/startRgbStream() 已保证整帧分别是 Z16/RGB8。
        // 厂商 SamplePointReconstruction 对点云重建使用无参 getData()；
        // FRAME_DATA_FORMAT_RGB8 在 3.2.52 中不存在，不能当作帧分量枚举使用。
        auto* depth_data = reinterpret_cast<unsigned short*>(
            const_cast<char*>(depth_frame->getData()));
        if (!depth_data) throw std::runtime_error("Z16 depth data is null.");
        if (depth_frame->getFormat() != STREAM_FORMAT_Z16) {
            throw std::runtime_error("Depth frame is not STREAM_FORMAT_Z16.");
        }

        cs::Pointcloud pointcloud;
        unsigned char* rgb_data = nullptr;
        int rgb_width = 0;
        int rgb_height = 0;
        if (enable_rgb_ && rgb_frame) {
            if (rgb_frame->getFormat() != STREAM_FORMAT_RGB8) {
                throw std::runtime_error("RGB frame is not STREAM_FORMAT_RGB8.");
            }
            rgb_data = reinterpret_cast<unsigned char*>(
                const_cast<char*>(rgb_frame->getData()));
            if (rgb_data) {
                rgb_width = rgb_frame->getWidth();
                rgb_height = rgb_frame->getHeight();
                pointcloud.generatePoints(
                    depth_data, depth_frame->getWidth(), depth_frame->getHeight(),
                    depth_scale, &depth_intrinsics_, &rgb_intrinsics_,
                    &rgb_extrinsics_, true);
            }
        }
        if (!rgb_data) {
            pointcloud.generatePoints(
                depth_data, depth_frame->getWidth(), depth_frame->getHeight(),
                depth_scale, &depth_intrinsics_, nullptr, nullptr, true);
        }
        if (pointcloud.size() == 0) {
            throw std::runtime_error("Point reconstruction produced zero valid points.");
        }

        const std::filesystem::path directory(output_directory_);
        std::error_code directory_error;
        std::filesystem::create_directories(directory, directory_error);
        if (directory_error) {
            throw std::runtime_error(
                "Cannot create output directory: " + directory_error.message());
        }
        const std::filesystem::path output_path =
            directory / (file_prefix_ + "_" + timestampSuffix() + ".ply");
        pointcloud.exportToFile(
            output_path.string(), rgb_data, rgb_width, rgb_height, binary_ply_);
        if (!std::filesystem::exists(output_path) ||
            std::filesystem::file_size(output_path) == 0) {
            throw std::runtime_error("Point-cloud export did not create a valid file.");
        }
        return std::filesystem::absolute(output_path).string();
    }

    std::string camera_serial_;
    bool enable_network_discovery_ = true;
    bool enable_usb_discovery_ = true;
    int discovery_timeout_ms_ = 3000;
    int discovery_attempts_ = 10;
    int discovery_retry_interval_ms_ = 1000;
    std::string output_directory_;
    std::string file_prefix_;
    int capture_timeout_ms_ = 5000;
    int depth_width_ = 0;
    int depth_height_ = 0;
    double depth_fps_ = 0.0;
    int rgb_width_ = 0;
    int rgb_height_ = 0;
    double rgb_fps_ = 0.0;
    bool enable_rgb_ = false;
    bool binary_ply_ = true;
    int depth_min_mm_ = 460;
    int depth_max_mm_ = 520;
    double depth_gain_ = 1.0;
    double depth_exposure_ = 8000.0;
    double depth_frame_time_ = 10000.0;

    cs::ICameraPtr camera_;
    Intrinsics depth_intrinsics_{};
    Intrinsics rgb_intrinsics_{};
    Extrinsics rgb_extrinsics_{};
    bool depth_started_ = false;
    bool rgb_started_ = false;
    std::mutex camera_mutex_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pointcloud_path_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr capture_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    try {
        rclcpp::spin(std::make_shared<ChishineCameraNode>());
    } catch (const std::exception& exception) {
        RCLCPP_FATAL(rclcpp::get_logger("chishine_camera_node"),
            "%s", exception.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
