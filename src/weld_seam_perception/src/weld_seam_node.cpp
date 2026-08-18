#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>

#include <weld_seam_sdk/weld_seam_sdk.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <functional>
#include <map>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

std::vector<std::string> splitCsvLine(const std::string& line)
{
    std::vector<std::string> fields;
    std::stringstream stream(line);
    std::string field;
    while (std::getline(stream, field, ',')) fields.push_back(field);
    return fields;
}

std::string jsonEscape(const std::string& text)
{
    std::string result;
    result.reserve(text.size());
    for (char ch : text) {
        if (ch == '\\' || ch == '"') result.push_back('\\');
        if (ch == '\n') result += "\\n";
        else result.push_back(ch);
    }
    return result;
}

std::string normalizeInputPath(const std::string& raw_path)
{
    const auto first = std::find_if_not(raw_path.begin(), raw_path.end(),
        [](unsigned char ch) { return std::isspace(ch) != 0; });
    const auto last = std::find_if_not(raw_path.rbegin(), raw_path.rend(),
        [](unsigned char ch) { return std::isspace(ch) != 0; }).base();
    std::string path = first < last ? std::string(first, last) : std::string();

#ifndef _WIN32
    // ROS String 中的 '~' 不经过 shell，因此不会自动展开。
    if (path == "~" || path.rfind("~/", 0) == 0) {
        const char* home = std::getenv("HOME");
        if (home && *home) {
            path = std::string(home) + path.substr(1);
        }
    }
#endif
    return path;
}

std::filesystem::path defaultDataRoot()
{
    if (const char* configured = std::getenv("SCUT_WELD_DATA_ROOT")) {
        if (*configured) {
            return std::filesystem::path(normalizeInputPath(configured));
        }
    }
    if (const char* home = std::getenv("HOME")) {
        if (*home) return std::filesystem::path(home) / "scut_weld_data";
    }
    return std::filesystem::temp_directory_path() / "scut_weld_data";
}

}  // namespace

class WeldSeamNode : public rclcpp::Node {
public:
    WeldSeamNode()
        : Node("weld_seam_node")
    {
        input_topic_ = declare_parameter<std::string>(
            "input_ply_topic", "/camera/pointcloud_file");
        output_directory_ = normalizeInputPath(declare_parameter<std::string>(
            "output_directory", ""));
        if (output_directory_.empty()) {
            output_directory_ =
                (defaultDataRoot() / "weld_results").string();
        }
        output_prefix_ = declare_parameter<std::string>(
            "output_prefix", "weld_seam");
        config_file_ = normalizeInputPath(
            declare_parameter<std::string>("config_file", ""));
        frame_id_ = declare_parameter<std::string>("frame_id", "camera_link");
        position_scale_to_ros_ = declare_parameter<double>(
            "position_scale_to_ros", 0.001);
        auto_process_ = declare_parameter<bool>("auto_process", true);
        parameter_overrides_ = declare_parameter<std::vector<std::string>>(
            "algorithm_overrides", std::vector<std::string>{});
        if (!std::isfinite(position_scale_to_ros_) ||
            position_scale_to_ros_ <= 0.0) {
            throw std::invalid_argument(
                "position_scale_to_ros must be finite and > 0.");
        }

        csv_pub_ = create_publisher<std_msgs::msg::String>(
            "/weld_seam/result_csv", rclcpp::QoS(1).transient_local());
        visualization_pub_ = create_publisher<std_msgs::msg::String>(
            "/weld_seam/result_visualization", rclcpp::QoS(1).transient_local());
        status_pub_ = create_publisher<std_msgs::msg::String>(
            "/weld_seam/status", rclcpp::QoS(10));
        pose_pub_ = create_publisher<geometry_msgs::msg::PoseArray>(
            "/weld_seam/poses_camera_frame", rclcpp::QoS(1).transient_local());

        input_sub_ = create_subscription<std_msgs::msg::String>(
            input_topic_, rclcpp::QoS(1).transient_local(),
            std::bind(&WeldSeamNode::inputCallback, this, std::placeholders::_1));
        extract_service_ = create_service<std_srvs::srv::Trigger>(
            "/weld_seam/extract_latest",
            std::bind(&WeldSeamNode::extractLatestCallback, this,
                std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(get_logger(),
            "Weld seam SDK %s ready; input topic=%s, output=%s, auto_process=%s",
            weld_seam_sdk::version(), input_topic_.c_str(),
            output_directory_.c_str(),
            auto_process_ ? "true" : "false");
    }

private:
    void inputCallback(const std_msgs::msg::String::SharedPtr message)
    {
        const std::string input_path = normalizeInputPath(message->data);
        if (input_path.empty()) {
            RCLCPP_ERROR(get_logger(), "Received an empty point-cloud path.");
            return;
        }
        if (input_path != message->data) {
            RCLCPP_INFO(get_logger(), "Normalized input path: %s",
                input_path.c_str());
        }
        {
            std::lock_guard<std::mutex> lock(latest_path_mutex_);
            latest_input_ply_ = input_path;
        }
        if (auto_process_) process(input_path, nullptr);
    }

    void extractLatestCallback(
        const std_srvs::srv::Trigger::Request::SharedPtr,
        std_srvs::srv::Trigger::Response::SharedPtr response)
    {
        std::string input;
        {
            std::lock_guard<std::mutex> lock(latest_path_mutex_);
            input = latest_input_ply_;
        }
        if (input.empty()) {
            response->success = false;
            response->message = "No point-cloud file has been received yet.";
            return;
        }
        process(input, response.get());
    }

    void process(
        const std::string& input_ply,
        std_srvs::srv::Trigger::Response* service_response)
    {
        std::unique_lock<std::mutex> processing_lock(processing_mutex_, std::try_to_lock);
        if (!processing_lock.owns_lock()) {
            const std::string message = "Extraction is already running; request was rejected.";
            RCLCPP_WARN(get_logger(), "%s", message.c_str());
            if (service_response) {
                service_response->success = false;
                service_response->message = message;
            }
            return;
        }

        weld_seam_sdk::RunOptions options;
        options.input_ply = input_ply;
        options.output_directory = output_directory_;
        const std::filesystem::path input_path(input_ply);
        options.output_prefix = output_prefix_ + "_" + input_path.stem().string();
        options.config_file = config_file_;
        options.parameter_overrides = parameter_overrides_;

        RCLCPP_INFO(get_logger(), "Extracting weld seam from: %s", input_ply.c_str());
        const weld_seam_sdk::RunResult result = weld_seam_sdk::run(options);
        publishStatus(result, input_ply);
        if (result.status != 0) {
            RCLCPP_ERROR(get_logger(), "Extraction failed: %s", result.message.c_str());
            if (service_response) {
                service_response->success = false;
                service_response->message = result.message;
            }
            return;
        }

        publishString(csv_pub_, result.csv_path);
        publishString(visualization_pub_, result.visualization_ply_path);
        publishPoseArray(result.csv_path);
        RCLCPP_INFO(get_logger(),
            "Extraction succeeded: seam=%zu, path=%zu, total=%.3f ms",
            result.seam_point_count, result.path_point_count, result.total_time_ms);
        if (service_response) {
            service_response->success = true;
            service_response->message = result.csv_path;
        }
    }

    void publishString(
        const rclcpp::Publisher<std_msgs::msg::String>::SharedPtr& publisher,
        const std::string& text)
    {
        std_msgs::msg::String message;
        message.data = text;
        publisher->publish(message);
    }

    void publishStatus(
        const weld_seam_sdk::RunResult& result,
        const std::string& input_ply)
    {
        std_msgs::msg::String status;
        std::ostringstream stream;
        stream << "{\"status\":" << result.status
               << ",\"message\":\"" << jsonEscape(result.message) << "\""
               << ",\"input_ply\":\"" << jsonEscape(input_ply) << "\""
               << ",\"csv\":\"" << jsonEscape(result.csv_path) << "\""
               << ",\"seam_points\":" << result.seam_point_count
               << ",\"path_points\":" << result.path_point_count
               << ",\"normal_ms\":" << result.normal_time_ms
               << ",\"primary_ms\":" << result.primary_time_ms
               << ",\"feature_ms\":" << result.feature_time_ms
               << ",\"total_ms\":" << result.total_time_ms << "}";
        status.data = stream.str();
        status_pub_->publish(status);
    }

    void publishPoseArray(const std::string& csv_path)
    {
        std::ifstream input(csv_path);
        if (!input) {
            RCLCPP_ERROR(get_logger(), "Cannot reopen CSV: %s", csv_path.c_str());
            return;
        }
        std::string line;
        if (!std::getline(input, line)) return;
        const std::vector<std::string> header = splitCsvLine(line);
        std::map<std::string, size_t> column;
        for (size_t i = 0; i < header.size(); ++i) column[header[i]] = i;
        const char* required[] = {"x", "y", "z", "qw", "qx", "qy", "qz"};
        for (const char* name : required) {
            if (column.count(name) == 0) {
                RCLCPP_ERROR(get_logger(), "CSV is missing column: %s", name);
                return;
            }
        }

        geometry_msgs::msg::PoseArray array;
        array.header.stamp = now();
        array.header.frame_id = frame_id_;
        while (std::getline(input, line)) {
            if (line.empty()) continue;
            const std::vector<std::string> fields = splitCsvLine(line);
            try {
                geometry_msgs::msg::Pose pose;
                // 算法/PLY/CSV 默认是 mm；ROS REP-103 位置使用 m。
                pose.position.x = position_scale_to_ros_ *
                    std::stod(fields.at(column["x"]));
                pose.position.y = position_scale_to_ros_ *
                    std::stod(fields.at(column["y"]));
                pose.position.z = position_scale_to_ros_ *
                    std::stod(fields.at(column["z"]));
                pose.orientation.w = std::stod(fields.at(column["qw"]));
                pose.orientation.x = std::stod(fields.at(column["qx"]));
                pose.orientation.y = std::stod(fields.at(column["qy"]));
                pose.orientation.z = std::stod(fields.at(column["qz"]));
                array.poses.push_back(pose);
            } catch (const std::exception& exception) {
                RCLCPP_WARN(get_logger(), "Skipped malformed CSV row: %s",
                    exception.what());
            }
        }
        pose_pub_->publish(array);
    }

    std::string input_topic_;
    std::string output_directory_;
    std::string output_prefix_;
    std::string config_file_;
    std::string frame_id_;
    double position_scale_to_ros_ = 0.001;
    bool auto_process_ = true;
    std::vector<std::string> parameter_overrides_;

    std::mutex latest_path_mutex_;
    std::mutex processing_mutex_;
    std::string latest_input_ply_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr input_sub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr csv_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr visualization_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr pose_pub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr extract_service_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeldSeamNode>());
    rclcpp::shutdown();
    return 0;
}
