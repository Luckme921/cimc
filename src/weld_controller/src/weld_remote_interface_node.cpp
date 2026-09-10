#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <sstream>
#include <stdexcept>
#include <string>

namespace
{

std::string normalize_command(std::string command)
{
  const auto first = command.find_first_not_of(" \t\r\n");
  if (first == std::string::npos) {
    return "";
  }
  const auto last = command.find_last_not_of(" \t\r\n");
  command = command.substr(first, last - first + 1);
  std::transform(command.begin(), command.end(), command.begin(), [](unsigned char value) {
    return static_cast<char>(std::toupper(value));
  });
  return command;
}

}  // namespace

class WeldRemoteInterfaceNode : public rclcpp::Node
{
public:
  WeldRemoteInterfaceNode()
  : Node("weld_remote_interface_node")
  {
    output_enabled_ = declare_parameter<bool>("output_enabled", false);
    require_gas_before_weld_ = declare_parameter<bool>("require_gas_before_weld", true);
    min_current_a_ = declare_parameter<double>("min_current_a", 1.0);
    max_current_a_ = declare_parameter<double>("max_current_a", 350.0);
    min_rotation_speed_rps_ = declare_parameter<double>("min_rotation_speed_rps", 0.0);
    max_rotation_speed_rps_ = declare_parameter<double>("max_rotation_speed_rps", 10.0);
    unary_voltage_placeholder_v_ =
      declare_parameter<double>("unary_voltage_placeholder_v", 20.0);

    const auto command_topic =
      declare_parameter<std::string>("command_topic", "/weld/remote/command");
    const auto setpoints_topic =
      declare_parameter<std::string>("setpoints_topic", "/weld/remote/setpoints");
    const auto status_topic =
      declare_parameter<std::string>("status_topic", "/weld/remote/status");
    const auto weld_control_topic =
      declare_parameter<std::string>("weld_control_topic", "/weld/control");
    const auto weld_parameter_topic =
      declare_parameter<std::string>("weld_parameter_topic", "/weld/set_param_real");
    const auto motor_speed_topic =
      declare_parameter<std::string>("motor_speed_topic", "/cimc/motor_speed");

    validate_parameters(
      command_topic, setpoints_topic, status_topic, weld_control_topic,
      weld_parameter_topic, motor_speed_topic);

    weld_control_pub_ = create_publisher<std_msgs::msg::String>(weld_control_topic, 10);
    weld_parameter_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(weld_parameter_topic, 10);
    motor_speed_pub_ = create_publisher<std_msgs::msg::Float32>(motor_speed_topic, 10);
    status_pub_ = create_publisher<std_msgs::msg::String>(status_topic, 10);

    command_sub_ = create_subscription<std_msgs::msg::String>(
      command_topic, 10,
      std::bind(&WeldRemoteInterfaceNode::command_callback, this, std::placeholders::_1));
    setpoints_sub_ = create_subscription<std_msgs::msg::Float32MultiArray>(
      setpoints_topic, 10,
      std::bind(&WeldRemoteInterfaceNode::setpoints_callback, this, std::placeholders::_1));

    RCLCPP_INFO(
      get_logger(),
      "Remote weld interface ready: output_enabled=%s, mode=unary, command=%s, setpoints=%s",
      output_enabled_ ? "true" : "false", command_topic.c_str(), setpoints_topic.c_str());
    if (!output_enabled_) {
      RCLCPP_WARN(
        get_logger(),
        "Dry-run is active: remote commands are validated and reported but not forwarded to hardware topics.");
    }
  }

private:
  enum class State
  {
    IDLE,
    GAS_ON,
    WELDING
  };

  void validate_parameters(
    const std::string & command_topic,
    const std::string & setpoints_topic,
    const std::string & status_topic,
    const std::string & weld_control_topic,
    const std::string & weld_parameter_topic,
    const std::string & motor_speed_topic) const
  {
    if (
      command_topic.empty() || setpoints_topic.empty() || status_topic.empty() ||
      weld_control_topic.empty() || weld_parameter_topic.empty() || motor_speed_topic.empty())
    {
      throw std::invalid_argument("all remote weld topic parameters must be non-empty");
    }
    if (
      !std::isfinite(min_current_a_) || !std::isfinite(max_current_a_) ||
      min_current_a_ < 0.0 || max_current_a_ <= min_current_a_)
    {
      throw std::invalid_argument("current limits must satisfy 0 <= min < max");
    }
    if (
      !std::isfinite(min_rotation_speed_rps_) ||
      !std::isfinite(max_rotation_speed_rps_) ||
      min_rotation_speed_rps_ < 0.0 ||
      max_rotation_speed_rps_ <= min_rotation_speed_rps_)
    {
      throw std::invalid_argument("rotation speed limits must satisfy 0 <= min < max");
    }
    if (!std::isfinite(unary_voltage_placeholder_v_)) {
      throw std::invalid_argument("unary_voltage_placeholder_v must be finite");
    }
  }

  const char * state_name() const
  {
    switch (state_) {
      case State::IDLE:
        return "idle";
      case State::GAS_ON:
        return "gas_on";
      case State::WELDING:
        return "welding";
    }
    return "unknown";
  }

  void publish_status(
    bool accepted,
    const std::string & command,
    const std::string & message,
    bool forwarded)
  {
    std::ostringstream stream;
    stream << "{\"accepted\":" << (accepted ? "true" : "false")
           << ",\"forwarded\":" << (forwarded ? "true" : "false")
           << ",\"dry_run\":" << (output_enabled_ ? "false" : "true")
           << ",\"state\":\"" << state_name() << "\""
           << ",\"command\":\"" << command << "\""
           << ",\"message\":\"" << message << "\"}";
    std_msgs::msg::String status;
    status.data = stream.str();
    status_pub_->publish(status);
    if (accepted) {
      RCLCPP_INFO(
        get_logger(), "%s: %s (forwarded=%s, state=%s)",
        command.c_str(), message.c_str(), forwarded ? "true" : "false", state_name());
    } else {
      RCLCPP_WARN(
        get_logger(), "%s: %s (state=%s)",
        command.empty() ? "EMPTY" : command.c_str(), message.c_str(), state_name());
    }
  }

  void publish_weld_control(const std::string & command)
  {
    if (!output_enabled_) {
      return;
    }
    std_msgs::msg::String message;
    message.data = command;
    weld_control_pub_->publish(message);
  }

  void publish_motor_speed(double speed_rps)
  {
    if (!output_enabled_) {
      return;
    }
    std_msgs::msg::Float32 message;
    message.data = static_cast<float>(speed_rps);
    motor_speed_pub_->publish(message);
  }

  void command_callback(const std_msgs::msg::String::SharedPtr message)
  {
    const std::string command = normalize_command(message->data);
    if (command.empty()) {
      publish_status(false, "", "empty command rejected", false);
      return;
    }

    if (command == "GAS_ON" || command == "START_GAS") {
      if (state_ == State::WELDING) {
        publish_status(false, command, "gas command rejected while welding", false);
        return;
      }
      // The low-level driver defaults to unary mode. Reassert that mode and
      // start CAN polling before requesting gas, without any startup action in
      // this interface node itself.
      publish_weld_control("use_builtin_curve");
      publish_weld_control("start_system");
      publish_weld_control("start_gas");
      state_ = State::GAS_ON;
      publish_status(
        true, command,
        "unary mode + start_system + start_gas requested",
        output_enabled_);
      return;
    }

    if (command == "WELD_START" || command == "START_WELDING") {
      if (require_gas_before_weld_ && state_ != State::GAS_ON) {
        publish_status(false, command, "GAS_ON is required before WELD_START", false);
        return;
      }
      publish_weld_control("start_welding");
      state_ = State::WELDING;
      publish_status(true, command, "start_welding requested", output_enabled_);
      return;
    }

    if (
      command == "WELD_STOP" || command == "STOP_WELDING" ||
      command == "STOP_ALL")
    {
      publish_weld_control("stop_welding");
      publish_motor_speed(0.0);
      state_ = State::IDLE;
      publish_status(
        true, command, "stop_welding + motor_speed=0 requested", output_enabled_);
      return;
    }

    if (command == "FAULT_RESET") {
      publish_weld_control("fault_reset");
      publish_status(true, command, "fault_reset requested", output_enabled_);
      return;
    }

    publish_status(false, command, "unsupported command", false);
  }

  void setpoints_callback(const std_msgs::msg::Float32MultiArray::SharedPtr message)
  {
    constexpr std::size_t expected_size = 2;
    if (message->data.size() != expected_size) {
      publish_status(
        false, "SETPOINTS",
        "expected [current_A, rotation_speed_rps]", false);
      return;
    }

    const double current_a = message->data[0];
    const double rotation_speed_rps = message->data[1];
    if (
      !std::isfinite(current_a) || current_a < min_current_a_ ||
      current_a > max_current_a_)
    {
      publish_status(false, "SETPOINTS", "current is outside configured limits", false);
      return;
    }
    if (
      !std::isfinite(rotation_speed_rps) ||
      rotation_speed_rps < min_rotation_speed_rps_ ||
      rotation_speed_rps > max_rotation_speed_rps_)
    {
      publish_status(
        false, "SETPOINTS", "rotation speed is outside configured limits", false);
      return;
    }

    if (output_enabled_) {
      std_msgs::msg::Float32MultiArray weld_parameters;
      weld_parameters.data = {
        static_cast<float>(current_a),
        static_cast<float>(unary_voltage_placeholder_v_)};
      weld_parameter_pub_->publish(weld_parameters);
      publish_motor_speed(rotation_speed_rps);
    }

    std::ostringstream detail;
    detail << "unary setpoints accepted: current=" << current_a
           << "A, rotation_speed=" << rotation_speed_rps << "rps";
    publish_status(true, "SETPOINTS", detail.str(), output_enabled_);
  }

  bool output_enabled_{false};
  bool require_gas_before_weld_{true};
  double min_current_a_{1.0};
  double max_current_a_{350.0};
  double min_rotation_speed_rps_{0.0};
  double max_rotation_speed_rps_{10.0};
  double unary_voltage_placeholder_v_{20.0};
  State state_{State::IDLE};

  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr weld_control_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr weld_parameter_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr motor_speed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr command_sub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr setpoints_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WeldRemoteInterfaceNode>());
  rclcpp::shutdown();
  return 0;
}
