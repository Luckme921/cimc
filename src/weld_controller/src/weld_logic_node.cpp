#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <map>
#include <vector>
#include <string>
#include <chrono>

using namespace std::chrono_literals;

// =========================================================================
// ⏱️ 时间与延时参数配置区 (单位：毫秒 ms)
// =========================================================================
const int TIME_IGNITION_STABILIZE_MS = 200;  
const int TIME_CRATER_DELAY_MS       = 1000; 

// =========================================================================
// 🟢 焊接工艺参数配置区 
// =========================================================================
struct BaseParam {
    float current;
    float voltage;
    float spin_speed;
    std::string name;
};

struct SubParam {
    float current;      
    float voltage;      
    float spin_speed;   
    int duration_ms;    
    std::string name;   
};

struct ProcessParam {
    std::vector<SubParam> stages; 
};

// 专用起弧与收弧参数
const BaseParam IGNITION_PARAM = {240.0, 29.6, 6.0, "引弧预热"}; 
const BaseParam CRATER_PARAM   = {245.0, 29.8, 6.0, "自动收弧"}; 

// =========================================================================
// 映射表：工艺段索引 -> 对应的焊接参数数组
// 业务逻辑说明 (以物理轨迹的 9 个点 P1~P9 为例，程序会自动适应 ABB 的标号):
// 第 1 个点 (P1): 安全过度点
// 第 2 个点 (P2): 开始焊接点 (触发起弧 IGNITION_PARAM)
// 第 3~8 个点 (P3~P8): 执行具体的工艺段，映射下方字典的索引 2~7。
// 第 8 个点 (P8): 也就是倒数第 2 个点(结束焊接点)，触发索引 7 的工艺，同时启动收弧倒计时！
// 第 9 个点 (P9): 也就是最后 1 个点(安全离开点)，彻底关闭电弧和电机。
// =========================================================================
std::map<int, ProcessParam> WELD_PROCESS_PARAMS = {
    {2,  ProcessParam{ { {250.0, 30.0, 6.0, 0, "底边"} } }}, 
    {3,  ProcessParam{ { {250.0, 30.0, 6.0, 0, "右边斜边"} } }},  
    {4,  ProcessParam{ { {250.0, 30.0, 6.0, 0, "顶边"} } }},      
    {5,  ProcessParam{ { {250.0, 30.4, 6.0, 0, "左边斜边"} } }}, 
    {6,  ProcessParam{ { 
        {250.0, 30.0, 6.0, 2000, "底边-前段"}, 
        {250.0, 30.0, 6.0, 2000, "底边-中段"}, 
        {245.0, 29.8, 6.0, 0,    "底边-后段(提前转斜边参数)"} 
    } }}, 
    {7,  ProcessParam{ { {250.0, 30.0, 6.0, 0, "右边斜边"} } }}
};

enum class WeldState {
    IDLE,               
    WAIT_ARC_SUCCESS,   
    STABILIZING,        
    WELDING             
};

class WeldLogicNode : public rclcpp::Node {
public:
    WeldLogicNode() : Node("weld_logic_node"), is_auto_mode_(true), state_(WeldState::IDLE), index_offset_(-1) {
        
        pub_weld_control_ = this->create_publisher<std_msgs::msg::String>("/weld/control", 10);
        pub_weld_param_   = this->create_publisher<std_msgs::msg::Float32MultiArray>("/weld/set_param_real", 10);
        pub_motor_speed_  = this->create_publisher<std_msgs::msg::Float32>("/cimc/motor_speed", 10);

        sub_abb_status_ = this->create_subscription<std_msgs::msg::String>(
            "/abb/raw_text", 10, std::bind(&WeldLogicNode::abb_status_callback, this, std::placeholders::_1));
        
        sub_weld_status_ = this->create_subscription<std_msgs::msg::String>(
            "/weld/status", 10, std::bind(&WeldLogicNode::weld_status_callback, this, std::placeholders::_1));

        sub_override_cmd_ = this->create_subscription<std_msgs::msg::String>(
            "/cimc/override_cmd", 10, std::bind(&WeldLogicNode::override_cmd_callback, this, std::placeholders::_1));

        sub_override_param_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "/cimc/override_param", 10, std::bind(&WeldLogicNode::override_param_callback, this, std::placeholders::_1));

        last_weld_heartbeat_time_ = this->now();

        init_timer_ = this->create_wall_timer(
            std::chrono::seconds(3), std::bind(&WeldLogicNode::system_init_callback, this));
        
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::seconds(1), std::bind(&WeldLogicNode::watchdog_callback, this));

        RCLCPP_INFO(this->get_logger(), "🧠 焊接逻辑中枢已启动，处于 [自动模式]，具备自动对齐基准防错能力！");
    }

private:
    void system_init_callback() {
        init_timer_->cancel();
        send_weld_control("start_system");
        RCLCPP_INFO(this->get_logger(), "✅ 已下发焊机首发激活指令 (start_system)");
    }

    void watchdog_callback() {
        if (!is_auto_mode_) return; 
        auto now = this->now();
        if ((now - last_weld_heartbeat_time_).seconds() >= 3.0) {
            send_weld_control("start_system");
            RCLCPP_WARN(this->get_logger(), "🔌 警告: 焊机反馈丢失超 3 秒！尝试重新激活通讯...");
            last_weld_heartbeat_time_ = now;
        }
    }

    void weld_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data.find("[底层掉线]") == std::string::npos) {
            last_weld_heartbeat_time_ = this->now();
        }

        if (!is_auto_mode_) return;

        if (state_ == WeldState::WAIT_ARC_SUCCESS) {
            if (msg->data.find("起弧成功[✅]") != std::string::npos || msg->data.find("起弧成功[🔥]") != std::string::npos) {
                RCLCPP_INFO(this->get_logger(), "✅ 检测到物理【起弧成功】！保持引弧参数预热熔池 %d 毫秒...", TIME_IGNITION_STABILIZE_MS);
                state_ = WeldState::STABILIZING;

                delay_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIME_IGNITION_STABILIZE_MS), [this]() {
                    delay_timer_->cancel();
                    RCLCPP_INFO(this->get_logger(), "🚀 预热完毕！瞬间切入第一段工艺！");
                    state_ = WeldState::WELDING;
                    apply_process_params(2); // 起弧后的第一段工艺在字典里是 2
                });
            }
        }
    }

    void abb_status_callback(const std_msgs::msg::String::SharedPtr msg) {
        if (!is_auto_mode_) return;

        std::string status = msg->data;

        if (status == "START_GAS") {
            RCLCPP_INFO(this->get_logger(), "==> 收到 START_GAS，开启保护气，重置坐标基准点");
            index_offset_ = -1; // 每次重新开始流程时，清空基准
            send_weld_control("start_gas");
            return;
        }

        try {
            int raw_idx = std::stoi(status); 
            
            // 【核心防呆修正】：自动适配 ABB 的起始偏移量
            if (index_offset_ == -1) {
                index_offset_ = raw_idx;
                RCLCPP_INFO(this->get_logger(), "🔄 自动适配 ABB 序号！检测到流程起点(第1个点)的原始序号为 %d，已作为偏移基准。", raw_idx);
            }
            
            // 归一化序号：强行抹平 ABB 序号跳动的影响，让内部计算永远从 0 开始。
            // 例如：第 1 个物理点计算得出 point_idx = 0，第 2 个物理点 point_idx = 1
            int point_idx = raw_idx - index_offset_; 
            int max_process_index = WELD_PROCESS_PARAMS.rbegin()->first;

            if (point_idx == 0) {
                // 第 1 个物理点：P1
                RCLCPP_INFO(this->get_logger(), "==> 到达第 1 个点 (P1)：安全过度点 (ABB原始序号: %d)", raw_idx);
            }
            else if (point_idx == 1) {
                // 第 2 个物理点：P2
                RCLCPP_INFO(this->get_logger(), "✅ 到达第 2 个点 (P2)：开始焊接点 (ABB原始序号: %d)，下发【%s】起弧参数...", raw_idx, IGNITION_PARAM.name.c_str());
                send_weld_params(IGNITION_PARAM.current, IGNITION_PARAM.voltage);
                send_motor_speed(IGNITION_PARAM.spin_speed);
                send_weld_control("start_welding");
                state_ = WeldState::WAIT_ARC_SUCCESS; 
            }
            else if (point_idx >= 2 && point_idx <= max_process_index) {
                // 第 3 个物理点及以后
                RCLCPP_INFO(this->get_logger(), "==> 到达中间工艺点 (ABB原始序号: %d)，切入字典配置索引 P%d", raw_idx, point_idx);
                apply_process_params(point_idx); 
                
                // 到达字典的最后一个工艺段（倒数第2个物理点）
                if (point_idx == max_process_index) {
                    RCLCPP_INFO(this->get_logger(), "⏳ 到达倒数第 2 个点：结束焊接点！后台倒计时 %d 毫秒后自动收弧...", TIME_CRATER_DELAY_MS);
                    crater_timer_ = this->create_wall_timer(std::chrono::milliseconds(TIME_CRATER_DELAY_MS), [this]() {
                        crater_timer_->cancel();
                        if (sub_stage_timer_) sub_stage_timer_->cancel(); 
                        send_weld_params(CRATER_PARAM.current, CRATER_PARAM.voltage);
                        send_motor_speed(CRATER_PARAM.spin_speed);
                        RCLCPP_INFO(this->get_logger(), "📉 [收弧触发] 强行切入 [%s] 段: 电流=%.1f A, 电压=%.1f V, 旋弧=%.1f r/s", 
                                    CRATER_PARAM.name.c_str(), CRATER_PARAM.current, CRATER_PARAM.voltage, CRATER_PARAM.spin_speed);
                    });
                }
            }
            else if (point_idx == max_process_index + 1) {
                // 最后的物理点
                RCLCPP_INFO(this->get_logger(), "✅ 到达最后 1 个点：安全离开点 (ABB原始序号: %d)，停止焊接并关闭电机！", raw_idx);
                stop_all_timers();
                send_weld_control("stop_welding");
                send_motor_speed(0.0);
            }
            else if (point_idx == max_process_index + 2) {
                RCLCPP_INFO(this->get_logger(), "==> 任务完全结束，整个焊接流程圆满完成！");
                index_offset_ = -1; // 跑完全程，彻底清空基准，迎接下一次焊接
            }

        } catch (const std::exception& e) {}
    }

    void apply_process_params(int index) {
        if (WELD_PROCESS_PARAMS.count(index) > 0) {
            current_segment_idx_ = index; 
            current_stage_idx_ = 0;       
            execute_current_stage();      
        } else {
            RCLCPP_WARN(this->get_logger(), "⚠️ 未找到索引 %d 的工艺参数配置！", index);
        }
    }

    void execute_current_stage() {
        if (sub_stage_timer_) sub_stage_timer_->cancel(); 

        auto stages = WELD_PROCESS_PARAMS[current_segment_idx_].stages;
        if (current_stage_idx_ < (int)stages.size()) {
            auto p = stages[current_stage_idx_];
            send_weld_params(p.current, p.voltage);
            send_motor_speed(p.spin_speed);
            
            if (stages.size() > 1) {
                RCLCPP_INFO(this->get_logger(), "   -> 执行 [%s] (第 %d/%d 小段): 电流=%.1f A, 电压=%.1f V, 旋弧=%.1f r/s", 
                            p.name.c_str(), current_stage_idx_ + 1, (int)stages.size(), p.current, p.voltage, p.spin_speed);
            } else {
                RCLCPP_INFO(this->get_logger(), "   -> 执行 [%s]: 电流=%.1f A, 电压=%.1f V, 旋弧=%.1f r/s", 
                            p.name.c_str(), p.current, p.voltage, p.spin_speed);
            }

            if (current_stage_idx_ < (int)stages.size() - 1 && p.duration_ms > 0) {
                sub_stage_timer_ = this->create_wall_timer(
                    std::chrono::milliseconds(p.duration_ms), 
                    std::bind(&WeldLogicNode::advance_sub_stage, this)
                );
            } else {
                if (stages.size() > 1) {
                    RCLCPP_INFO(this->get_logger(), "      (进入后段保持阶段，等待下一坐标到来...)");
                }
            }
        }
    }

    void advance_sub_stage() {
        current_stage_idx_++;
        execute_current_stage();
    }

    void stop_all_timers() {
        if (delay_timer_) delay_timer_->cancel();
        if (sub_stage_timer_) sub_stage_timer_->cancel(); 
        if (crater_timer_) crater_timer_->cancel(); 
        state_ = WeldState::IDLE;
    }

    void override_cmd_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;

        if (cmd == "MODE_MANUAL") {
            is_auto_mode_ = false;
            stop_all_timers();
            RCLCPP_WARN(this->get_logger(), "⚠️ 已切换为 [手动接管模式]，屏蔽 ABB 自动逻辑");
        } else if (cmd == "MODE_AUTO") {
            is_auto_mode_ = true;
            state_ = WeldState::IDLE;
            index_offset_ = -1; // 切换回自动时重置基准点
            last_weld_heartbeat_time_ = this->now();
            RCLCPP_INFO(this->get_logger(), "♻️ 已恢复为 [自动模式]");
        } else if (cmd == "ESTOP") {
            is_auto_mode_ = false;
            stop_all_timers();
            send_weld_control("stop_welding");
            send_motor_speed(0.0);
            RCLCPP_ERROR(this->get_logger(), "🚨 紧急停止！已切断电弧和电机，进入手动模式");
        } 
        // ================= 新增策略切换指令 =================
        else if (cmd == "USE_BUILTIN") {
            send_weld_control("use_builtin_curve");
            RCLCPP_INFO(this->get_logger(), "🔧 已向底层下发：强制使用焊机内置专家曲线(压强0)");
        } else if (cmd == "USE_CALC") {
            send_weld_control("use_calc_curve");
            RCLCPP_INFO(this->get_logger(), "🔧 已向底层下发：启用公式动态计算压强干预");
        } 
        // ====================================================
        else if (!is_auto_mode_) {
            send_weld_control(cmd);
            RCLCPP_INFO(this->get_logger(), "[手动执行] %s", cmd.c_str());
        } else {
            RCLCPP_WARN(this->get_logger(), "请先发送 MODE_MANUAL 切入手动模式，再执行单步动作");
        }
    }

    void override_param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 3) {
            float current = msg->data[0];
            float voltage = msg->data[1];
            float speed   = msg->data[2];
            send_weld_params(current, voltage);
            send_motor_speed(speed);
            RCLCPP_WARN(this->get_logger(), "⚡ [飞车调参] 强制覆写: 电流=%.1fA, 电压=%.1fV, 旋弧=%.1fr/s", 
                        current, voltage, speed);
        }
    }

    void send_weld_control(const std::string& cmd) {
        std_msgs::msg::String msg;
        msg.data = cmd;
        pub_weld_control_->publish(msg);
    }

    void send_weld_params(float current, float voltage) {
        std_msgs::msg::Float32MultiArray msg;
        msg.data.push_back(current);
        msg.data.push_back(voltage);
        pub_weld_param_->publish(msg);
    }

    void send_motor_speed(float speed) {
        std_msgs::msg::Float32 msg;
        msg.data = speed;
        pub_motor_speed_->publish(msg);
    }

    bool is_auto_mode_;
    WeldState state_;
    
    int current_segment_idx_;
    int current_stage_idx_;
    int index_offset_; // 记录第一次到来的 ABB 序号，实现无缝动态对齐

    rclcpp::Time last_weld_heartbeat_time_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
    
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_weld_control_;
    rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_weld_param_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr pub_motor_speed_;

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_abb_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_weld_status_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_override_cmd_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_override_param_;
    
    rclcpp::TimerBase::SharedPtr init_timer_;
    rclcpp::TimerBase::SharedPtr delay_timer_;
    rclcpp::TimerBase::SharedPtr sub_stage_timer_; 
    rclcpp::TimerBase::SharedPtr crater_timer_; 
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeldLogicNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}