#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import time
from typing import Optional

class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')
        
        # 1. 串口配置：默认值保持用户最新稳定版，也可在 ros2 run 后临时覆盖。
        self.serial_port_name = self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-FTDI_FT232R_USB_UART_BG027XPY-if00-port0'
        ).value
        self.baud_rate = int(self.declare_parameter('baud_rate', 115200).value)
        self.reconnect_interval_s = float(
            self.declare_parameter('reconnect_interval_s', 3.0).value)
        self.speed_topic = self.declare_parameter(
            'speed_topic', '/cimc/motor_speed').value
        if not self.serial_port_name or self.baud_rate <= 0:
            raise ValueError('serial_port must be non-empty and baud_rate must be > 0')
        if self.reconnect_interval_s <= 0.0 or not self.speed_topic:
            raise ValueError(
                'reconnect_interval_s must be > 0 and speed_topic must be non-empty')
        self.ser: Optional[serial.Serial] = None
        
        # 2. 状态变量
        self.current_speed = 0.0
        
        # 3. 初始连接尝试
        self.init_serial()
        
        # 4. 重连定时器：每隔 3.0 秒触发一次健康检查与重连
        self.reconnect_timer = self.create_timer(
            self.reconnect_interval_s, self.check_reconnect)
        
        # 5. 订阅话题
        self.subscription = self.create_subscription(
            Float32,
            self.speed_topic,
            self.speed_callback,
            10  
        )
        self.get_logger().info(
            f"电机控制节点已启动，topic={self.speed_topic}, "
            f"serial={self.serial_port_name}, baud={self.baud_rate}")

    def init_serial(self):
        """初始化串口连接"""
        try:
            self.ser = serial.Serial(
                port=self.serial_port_name,
                baudrate=self.baud_rate,
                timeout=1,  
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            if self.ser.is_open:
                self.get_logger().info(f"✅ 串口 {self.serial_port_name} 打开成功")
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"❌ 串口打开失败，等待下次定时器重试：{e}")
            self.ser = None

    def check_reconnect(self):
        """定时器回调：主动探针检查，掉线或变僵尸则断开并重连"""
        # ⭐ 核心优化1：主动探针，专门对付“拔掉USB但未发数据”的假在线状态
        if self.ser is not None and self.ser.is_open:
            try:
                # 尝试访问底层文件描述符属性（如检测缓冲区大小），若硬件已丢失，会在此刻强制触发异常
                _ = self.ser.in_waiting
            except (serial.SerialException, OSError, Exception) as e:
                self.get_logger().error(f"⚠️ 定时器检测到潜在的僵尸句柄（物理硬件已断开）：{e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.ser = None

        # 真正触发重连
        if self.ser is None or not self.ser.is_open:
            self.get_logger().warn("⚠️ 串口未连接或已断开，正在尝试重新连接...")
            self.init_serial()

    def speed_callback(self, msg: Float32):
        """话题回调函数：解析转速指令并发送"""
        target_speed = msg.data  
        self.get_logger().info(f"接收到目标转速：{target_speed} r/s")

        if self.ser is None or not self.ser.is_open:
            self.get_logger().error("🚫 串口未就绪，指令被丢弃，等待系统自动重连！")
            return

        try:
            # 逻辑1：停止电机（目标转速为0）
            if target_speed == 0.0:
                self.ser.write(b"OFFOFF")  
                self.get_logger().info(f"发送停止指令：OFFOFF")
                self.current_speed = 0.0

            # 逻辑2：启动电机（从0→非0）
            elif self.current_speed == 0.0 and target_speed > 0.0:
                self.ser.write(b"ONONON")
                self.get_logger().info(f"发送启动指令：ONONON（默认5r/s）")
                time.sleep(0.1)  
                
                speed_cmd = f"V_r/s:{target_speed:.1f}".encode('utf-8')
                self.ser.write(speed_cmd)
                self.get_logger().info(f"发送转速指令：{speed_cmd.decode('utf-8')}")
                self.current_speed = target_speed

            # 逻辑3：修改转速（非0→非0）
            elif self.current_speed != 0.0 and target_speed > 0.0:
                speed_cmd = f"V_r/s:{target_speed:.1f}".encode('utf-8')
                self.ser.write(speed_cmd)
                self.get_logger().info(f"发送转速指令：{speed_cmd.decode('utf-8')}")
                self.current_speed = target_speed

        # ⭐ 核心优化2：扩大异常捕获范围，包容系统层面的 OSError 崩溃
        except (serial.SerialException, OSError) as e:
            self.get_logger().error(f"❌ 串口通信期间发生严重错误（设备可能被强拔）：{e}")
            if self.ser:
                try:
                    self.ser.close()
                except Exception:
                    pass
            self.ser = None  # 置为 None，交给 3s 定时器重新去捕捉物理设备

    def destroy_node(self):
        """节点销毁时关闭串口"""
        if self.ser is not None and self.ser.is_open:
            self.ser.close()
            self.get_logger().info("串口已安全关闭")
        super().destroy_node()
        
def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
        node.destroy_node()
    except Exception as e:
        node.get_logger().error(f"节点运行异常：{e}")
        node.destroy_node()
    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
