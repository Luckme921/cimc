#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <pthread.h>
#include <unistd.h>
#include <math.h>
#include <string.h>
#include <iomanip>
#include <sstream>
#include "controlcan.h" 

// ==================== 宏定义配置 ====================
#define DEV_TYPE            VCI_USBCAN2
#define DEV_INDEX           0
#define CAN_CH              0
#define CAN_BAUD_T0         0x03    // 125kbps
#define CAN_BAUD_T1         0x1C

#define WELD_NODE_ID        0x02
#define COB_ID_NMT          0x000
#define COB_ID_RPDO1        (0x200 + WELD_NODE_ID)  // 主机发给焊机
#define COB_ID_TPDO1        (0x180 + WELD_NODE_ID)  // 焊机发给主机

#define NMT_CMD_START_NODE        0x01
#define WELD_CMD_STOP             0x00
#define WELD_CMD_START_WELD_FULL  0x07 // E08+E09+E10
#define WELD_CMD_GAS_DETECT       0x02 // E09
#define WELD_CMD_WIRE_FORWARD     0x04 // E10
#define WELD_CMD_WIRE_BACKWARD    0x08 // E11
#define WELD_CMD_FAULT_RESET      0x10 // E12

// ==================== 结构体定义 ====================
typedef struct {
    uint16_t weld_current;
    float    target_voltage; // 存储大脑发来的绝对电压，用于可能的公式计算
    uint8_t  weld_cmd;
} Weld_Param_T;

typedef struct {
    bool weld_ready;        // A00
    bool weld_fault;        // A01
    bool locate_success;    // A02
    bool arc_success;       // A08
    bool arc_error;         // A09
    bool gas_error;         // A10
    bool wire_stick;        // A11
    bool param_out_range;   // A15
    uint16_t feedback_current; // A16-A31
    float    feedback_voltage; // A32-A47
    uint8_t  raw_data[6];      // 原始 6 字节报文
    bool     has_data;         // 是否接收过数据
} Weld_Status_T;

// ==================== ROS 2 节点类 ====================
class WeldControllerNode : public rclcpp::Node {
public:
    WeldControllerNode() : Node("weld_controller_node"), run_flag_(true), polling_active_(false) {
        
        weld_param_ = {210, 20.0f, WELD_CMD_STOP};
        
        // 【核心控制位】：开机默认强制使用焊机内置专家曲线（压强=0），解决大飞溅问题！
        use_builtin_curve_ = true; 
        
        memset(&weld_status_, 0, sizeof(Weld_Status_T));
        memset(&last_tx_frame_, 0, sizeof(VCI_CAN_OBJ));

        pthread_mutex_init(&can_mutex_, NULL);
        pthread_mutex_init(&param_mutex_, NULL);
        pthread_mutex_init(&status_mutex_, NULL);

        last_rx_time_ = this->now();

        sub_control_ = this->create_subscription<std_msgs::msg::String>(
            "weld/control", 10, std::bind(&WeldControllerNode::control_callback, this, std::placeholders::_1));
        
        sub_param_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
            "weld/set_param_real", 10, std::bind(&WeldControllerNode::set_param_callback, this, std::placeholders::_1));

        status_pub_ = this->create_publisher<std_msgs::msg::String>("weld/status", 10);

        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500), 
            std::bind(&WeldControllerNode::dashboard_timer_callback, this));

        init_can_device();
        RCLCPP_INFO(this->get_logger(), "焊机控制器已启动！锁定【一元化+风冷】。默认开启低飞溅内置曲线模式(压强0)。");
    }

    ~WeldControllerNode() {
        RCLCPP_INFO(this->get_logger(), "正在安全停止焊机系统...");
        run_flag_ = false;
        polling_active_ = false;
        usleep(150000); 

        weld_param_.weld_cmd = WELD_CMD_STOP;
        send_rpdo1_now(&weld_param_);
        usleep(200000);

        VCI_ResetCAN(DEV_TYPE, DEV_INDEX, CAN_CH);
        VCI_CloseDevice(DEV_TYPE, DEV_INDEX);
        pthread_mutex_destroy(&can_mutex_);
        pthread_mutex_destroy(&param_mutex_);
        pthread_mutex_destroy(&status_mutex_);
    }

private:
    float get_base_voltage(uint16_t current) {
        if (current == 200) return 21.0f;
        if (current == 240) return 24.4f;
        if (current == 280) return 27.4f;
        return 0.08f * current + 5.0f; 
    }

    int8_t calculate_intensity(uint16_t current, float target_voltage) {
        float base_v = get_base_voltage(current);
        float k = ((target_voltage / base_v) - 1.0f) * 100.0f;
        int8_t intensity = (int8_t)round(k);
        if (intensity > 30) intensity = 30;
        if (intensity < -30) intensity = -30;
        return intensity;
    }

    void parse_tpdo1_frame(uint8_t *data) {
        pthread_mutex_lock(&status_mutex_);
        
        last_rx_time_ = this->now();
        memcpy(weld_status_.raw_data, data, 6);
        
        weld_status_.weld_ready = (data[0] & 0x01) != 0;      
        weld_status_.weld_fault = (data[0] & 0x02) != 0;      
        weld_status_.locate_success = (data[0] & 0x04) != 0;  
        
        weld_status_.arc_success = (data[1] & 0x01) != 0;     
        weld_status_.arc_error = (data[1] & 0x02) != 0;       
        weld_status_.gas_error = (data[1] & 0x04) != 0;       
        weld_status_.wire_stick = (data[1] & 0x08) != 0;      
        weld_status_.param_out_range = (data[1] & 0x80) != 0; 
        
        weld_status_.feedback_current = ((uint16_t)data[2] << 8) | data[3];
        uint16_t raw_volt = ((uint16_t)data[4] << 8) | data[5];
        weld_status_.feedback_voltage = raw_volt * 0.1f;
        
        weld_status_.has_data = true;
        pthread_mutex_unlock(&status_mutex_);
    }

    void dashboard_timer_callback() {
        if (!polling_active_) return;

        Weld_Param_T current_param;
        pthread_mutex_lock(&param_mutex_);
        current_param = weld_param_;
        pthread_mutex_unlock(&param_mutex_);

        VCI_CAN_OBJ tx_frame;
        pthread_mutex_lock(&can_mutex_);
        tx_frame = last_tx_frame_;
        pthread_mutex_unlock(&can_mutex_);

        Weld_Status_T status;
        rclcpp::Time rx_time;
        pthread_mutex_lock(&status_mutex_);
        status = weld_status_;
        rx_time = last_rx_time_;
        pthread_mutex_unlock(&status_mutex_);

        double dt = (this->now() - rx_time).seconds();
        if (status.has_data && dt > 1.5) {
            status.has_data = false; 
            pthread_mutex_lock(&status_mutex_);
            weld_status_.has_data = false;
            weld_status_.weld_ready = false; 
            memset(weld_status_.raw_data, 0, 6); 
            pthread_mutex_unlock(&status_mutex_);
        }

        std::stringstream ss;
        ss << "\n==================== 焊机实时监控仪表盘 (2Hz) ====================\n";
        
        std::string mode_str = use_builtin_curve_ ? "默认低飞溅曲线(压强锁定0)" : "自算压强干预";
        ss << "[-> TX 下发 | 一元化: " << mode_str << "] CAN_ID: 0x" << std::hex << std::uppercase << COB_ID_RPDO1 << std::dec << " | 原始帧: ";
        for (int i = 0; i < 8; i++) {
            ss << std::hex << std::setw(2) << std::setfill('0') << (int)tx_frame.Data[i] << " ";
        }
        
        int8_t show_intensity = use_builtin_curve_ ? 0 : calculate_intensity(current_param.weld_current, current_param.target_voltage);

        ss << std::dec << "\n    --> 解析: 指令位=0x" << std::hex << (int)current_param.weld_cmd << std::dec 
           << " | 设定电流=" << current_param.weld_current << "A"
           << " | 实际下发压强=" << (int)show_intensity << "%\n";

        ss << "[<- RX 接收 | 实时反馈] CAN_ID: 0x" << std::hex << std::uppercase << COB_ID_TPDO1 << std::dec;
        
        if (status.has_data) {
            ss << " | 原始帧: ";
            for (int i = 0; i < 6; i++) {
                ss << std::hex << std::setw(2) << std::setfill('0') << (int)status.raw_data[i] << " ";
            }
            ss << std::dec 
               << "\n    --> 【运行状态】: 就绪[" << (status.weld_ready ? "✅" : "❌") << "]  起弧[" << (status.arc_success ? "✅" : "❌") << "]  寻位[" << (status.locate_success ? "✅" : "❌") << "]\n"
               << "    --> 【报警状态】: 综合故障[" << (status.weld_fault ? "❌" : "✅") << "]  电弧异常[" << (status.arc_error ? "❌" : "✅") << "]  气流[" << (status.gas_error ? "❌" : "✅") << "]  粘丝[" << (status.wire_stick ? "❌" : "✅") << "]\n"
               << "    --> 【传感反馈】: 真实电流 = " << status.feedback_current << " A\n"
               << "                      真实电压 = " << std::fixed << std::setprecision(1) << status.feedback_voltage << " V";
        } else {
            ss << "\n ❌ [底层掉线] 尚未收到焊机回传数据，或焊机已关机/物理掉线！";
        }
        ss << "\n==================================================================";

        std::string dashboard_str = ss.str();
        RCLCPP_INFO(this->get_logger(), "%s", dashboard_str.c_str());

        std_msgs::msg::String msg;
        msg.data = dashboard_str;
        status_pub_->publish(msg);
    }

    void set_param_callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg) {
        if (msg->data.size() >= 2) {
            uint16_t current = static_cast<uint16_t>(msg->data[0]);
            float target_voltage = msg->data[1];

            // 限制最大电流，防止终端误输入导致硬件损坏
            if (current > 350) current = 350;

            pthread_mutex_lock(&param_mutex_);
            weld_param_.weld_current = current;
            weld_param_.target_voltage = target_voltage;
            pthread_mutex_unlock(&param_mutex_);
        }
    }

    void control_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string cmd = msg->data;
        
        // 响应大脑发来的策略切换指令
        if (cmd == "use_builtin_curve") {
            use_builtin_curve_ = true;
            RCLCPP_WARN(this->get_logger(), "🔧 底层已切换：强制压强为0 (启用低飞溅专家曲线)");
            return;
        } else if (cmd == "use_calc_curve") {
            use_builtin_curve_ = false;
            RCLCPP_WARN(this->get_logger(), "🔧 底层已切换：启用动态压强计算 (允许改变电弧长度)");
            return;
        }

        if (cmd == "start_system") {
            if (!polling_active_) {
                send_nmt(NMT_CMD_START_NODE);
                polling_active_ = true;
                pthread_create(&t_poll_, NULL, &WeldControllerNode::polling_thread_wrapper, this);
                pthread_create(&t_recv_, NULL, &WeldControllerNode::receive_thread_wrapper, this);
                pthread_detach(t_poll_);
                pthread_detach(t_recv_);
            } else {
                send_nmt(NMT_CMD_START_NODE);
            }
            return;
        }

        pthread_mutex_lock(&param_mutex_);
        if (cmd == "start_welding") weld_param_.weld_cmd = WELD_CMD_START_WELD_FULL;
        else if (cmd == "stop_welding") weld_param_.weld_cmd = WELD_CMD_STOP;
        else if (cmd == "start_gas") weld_param_.weld_cmd = WELD_CMD_GAS_DETECT;
        else if (cmd == "wire_forward") weld_param_.weld_cmd = WELD_CMD_WIRE_FORWARD;
        else if (cmd == "wire_backward") weld_param_.weld_cmd = WELD_CMD_WIRE_BACKWARD;
        else if (cmd == "fault_reset") weld_param_.weld_cmd = WELD_CMD_FAULT_RESET;
        
        Weld_Param_T p = weld_param_;
        pthread_mutex_unlock(&param_mutex_);

        if (cmd == "fault_reset" && polling_active_) {
            send_rpdo1_now(&p);
            usleep(200000);
            pthread_mutex_lock(&param_mutex_);
            weld_param_.weld_cmd = WELD_CMD_STOP;
            pthread_mutex_unlock(&param_mutex_);
        }
    }

    void init_can_device() {
        if (VCI_OpenDevice(DEV_TYPE, DEV_INDEX, 0) != STATUS_OK) return;
        VCI_INIT_CONFIG config = {};
        config.AccMask = 0xFFFFFFFF;
        config.Filter = 1;
        config.Timing0 = CAN_BAUD_T0;
        config.Timing1 = CAN_BAUD_T1;
        VCI_InitCAN(DEV_TYPE, DEV_INDEX, CAN_CH, &config);
        VCI_StartCAN(DEV_TYPE, DEV_INDEX, CAN_CH);
        VCI_ClearBuffer(DEV_TYPE, DEV_INDEX, CAN_CH);
    }

    void send_nmt(uint8_t cmd) {
        VCI_CAN_OBJ frame = {};
        frame.ID = COB_ID_NMT;
        frame.SendType = 1;
        frame.DataLen = 2;
        frame.Data[0] = cmd;
        frame.Data[1] = WELD_NODE_ID;
        pthread_mutex_lock(&can_mutex_);
        VCI_Transmit(DEV_TYPE, DEV_INDEX, CAN_CH, &frame, 1);
        pthread_mutex_unlock(&can_mutex_);
    }

    void send_rpdo1_now(Weld_Param_T *param) {
        VCI_CAN_OBJ frame = {};
        frame.ID = COB_ID_RPDO1;
        frame.SendType = 1;
        frame.DataLen = 8;
        frame.Data[1] = param->weld_cmd;
        
        // 0x02 = 0000 0010 (Bit1=1一元化, Bit2=0风冷, Bit0=0电流)
        frame.Data[2] = 0x02; 
        
        frame.Data[4] = (param->weld_current >> 8) & 0xFF;
        frame.Data[5] = param->weld_current & 0xFF;
        
        // 根据标志位决定真实下发的压强
        int8_t intensity = 0;
        if (!use_builtin_curve_) {
            intensity = calculate_intensity(param->weld_current, param->target_voltage);
        }
        
        uint8_t volt_send = intensity + 30; // 偏移计算
        
        frame.Data[6] = 0x00; 
        frame.Data[7] = volt_send;

        pthread_mutex_lock(&can_mutex_);
        VCI_Transmit(DEV_TYPE, DEV_INDEX, CAN_CH, &frame, 1);
        last_tx_frame_ = frame;
        pthread_mutex_unlock(&can_mutex_);
    }

    static void* polling_thread_wrapper(void* context) {
        static_cast<WeldControllerNode*>(context)->polling_loop();
        return NULL;
    }

    static void* receive_thread_wrapper(void* context) {
        static_cast<WeldControllerNode*>(context)->receive_loop();
        return NULL;
    }

    void polling_loop() {
        while (run_flag_ && polling_active_) {
            Weld_Param_T p;
            pthread_mutex_lock(&param_mutex_);
            p = weld_param_;
            pthread_mutex_unlock(&param_mutex_);
            send_rpdo1_now(&p);
            usleep(20000); 
        }
    }

    void receive_loop() {
        VCI_CAN_OBJ recv_frames[100];
        while (run_flag_ && polling_active_) {
            pthread_mutex_lock(&can_mutex_);
            int len = VCI_Receive(DEV_TYPE, DEV_INDEX, CAN_CH, recv_frames, 100, 0);
            pthread_mutex_unlock(&can_mutex_);

            for (int i = 0; i < len; i++) {
                if (recv_frames[i].ID == COB_ID_TPDO1 && recv_frames[i].DataLen >= 6) {
                    parse_tpdo1_frame(recv_frames[i].Data);
                }
            }
            usleep(10000); 
        }
    }

    bool run_flag_;
    bool polling_active_;
    bool use_builtin_curve_; 

    pthread_t t_poll_;
    pthread_t t_recv_;
    pthread_mutex_t can_mutex_;
    pthread_mutex_t param_mutex_;
    pthread_mutex_t status_mutex_;
    
    Weld_Param_T weld_param_;
    Weld_Status_T weld_status_;
    VCI_CAN_OBJ last_tx_frame_; 
    
    rclcpp::Time last_rx_time_; 

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_control_;
    rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_param_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeldControllerNode>();
    rclcpp::spin(node); 
    rclcpp::shutdown();
    return 0;
}