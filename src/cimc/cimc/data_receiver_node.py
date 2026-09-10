#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import json
import math
import socket
import threading
import queue
import time

# 引入 ROS 2 的标准消息类型
from std_msgs.msg import String, UInt8MultiArray
from std_msgs.msg import Float32, Float32MultiArray
from geometry_msgs.msg import Point


THIRD_PARTY_PROTOCOL_VERSION = 1


def encode_protocol_frame(frame):
    """Encode one versioned JSON object as an ASCII JSON-Lines frame."""
    return (json.dumps(
        frame, ensure_ascii=True, separators=(',', ':'), allow_nan=False
    ) + '\n').encode('ascii')


def decode_protocol_frame(line):
    """Decode and validate the common fields of one JSON-Lines frame."""
    try:
        frame = json.loads(line.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise ValueError(f'invalid JSON: {exc}') from exc
    if not isinstance(frame, dict):
        raise ValueError('frame must be a JSON object')
    if frame.get('version') != THIRD_PARTY_PROTOCOL_VERSION:
        raise ValueError(
            f'unsupported version: {frame.get("version")!r}; '
            f'expected {THIRD_PARTY_PROTOCOL_VERSION}')
    sequence = frame.get('seq')
    if isinstance(sequence, bool) or not isinstance(sequence, int) or sequence < 0:
        raise ValueError('seq must be a non-negative integer')
    frame_type = frame.get('type')
    if not isinstance(frame_type, str) or not frame_type:
        raise ValueError('type must be a non-empty string')
    return frame


def extract_jsonl_lines(receive_buffer, received, max_line_bytes):
    """Append a TCP chunk and return all complete JSONL lines."""
    receive_buffer.extend(received)
    if len(receive_buffer) > max_line_bytes and b'\n' not in receive_buffer:
        raise ValueError('third-party frame exceeds maximum line length')

    lines = []
    while b'\n' in receive_buffer:
        line, _, remainder = receive_buffer.partition(b'\n')
        receive_buffer[:] = remainder
        line = line.rstrip(b'\r')
        if line:
            lines.append(bytes(line))
    return lines


class DataReceiverNode(Node):
    def __init__(self):
        super().__init__('data_receiver_node')

        # 必须把状态变量放在最前面，防止后面的线程启动时找不到该变量而崩溃！
        self.is_running = True
        self.server_socket = None
        # ABB is the TCP client, therefore all outbound robot data must reuse
        # the accepted socket owned by this node. A second ROS node must not
        # bind the same listen address/port.
        self.abb_connection = None
        self.abb_connection_lock = threading.Lock()
        self.abb_tx_queue = queue.Queue(maxsize=100)
        self.forward_sequence = 0
        self.forward_sequence_lock = threading.Lock()
        self.forward_drop_count = 0
        self.last_third_party_request_seq = -1
        self.third_party_welding_active = False

        # 1. 网络配置 (ABB 机器人端)。默认值与用户最新版一致。
        self.host = self.declare_parameter(
            'listen_host', '192.168.125.2').value
        self.port = int(self.declare_parameter('listen_port', 45000).value)
        self.target_ip = self.declare_parameter(
            'abb_allowed_ip', '192.168.125.1').value

        # 2. TCP 异步可靠转发配置 (发给第三方设备)
        self.forward_ip = self.declare_parameter(
            'forward_ip', '192.168.3.5').value
        self.forward_port = int(
            self.declare_parameter('forward_port', 50000).value)
        self.forward_queue_size = int(
            self.declare_parameter('forward_queue_size', 500).value)
        self.weld_feedback_topic = self.declare_parameter(
            'weld_feedback_topic', '/weld/feedback_raw').value
        self.third_party_command_enabled = bool(self.declare_parameter(
            'third_party_command_enabled', True).value)
        self.third_party_max_line_bytes = int(self.declare_parameter(
            'third_party_max_line_bytes', 4096).value)
        self.unary_voltage_placeholder_v = float(self.declare_parameter(
            'unary_voltage_placeholder_v', 20.0).value)
        self.min_current_a = float(self.declare_parameter(
            'min_current_a', 1.0).value)
        self.max_current_a = float(self.declare_parameter(
            'max_current_a', 350.0).value)
        self.min_rotation_speed_rps = float(self.declare_parameter(
            'min_rotation_speed_rps', 0.0).value)
        self.max_rotation_speed_rps = float(self.declare_parameter(
            'max_rotation_speed_rps', 6.0).value)
        self.stop_on_third_party_disconnect = bool(self.declare_parameter(
            'stop_on_third_party_disconnect', True).value)
        self.weld_control_topic = self.declare_parameter(
            'weld_control_topic', '/weld/control').value
        self.weld_parameter_topic = self.declare_parameter(
            'weld_parameter_topic', '/weld/set_param_real').value
        self.motor_speed_topic = self.declare_parameter(
            'motor_speed_topic', '/cimc/motor_speed').value
        self.third_party_status_topic = self.declare_parameter(
            'third_party_status_topic', '/third_party/status').value
        if not self.host or not self.target_ip or not self.forward_ip:
            raise ValueError('listen/ABB/forward IP parameters must be non-empty')
        if not 1 <= self.port <= 65535 or not 1 <= self.forward_port <= 65535:
            raise ValueError('TCP ports must be within [1, 65535]')
        if self.forward_queue_size <= 0 or not self.weld_feedback_topic:
            raise ValueError(
                'forward_queue_size must be > 0 and weld_feedback_topic must be non-empty')
        if self.third_party_max_line_bytes <= 0:
            raise ValueError('third_party_max_line_bytes must be > 0')
        if not all(math.isfinite(value) for value in (
                self.unary_voltage_placeholder_v,
                self.min_current_a, self.max_current_a,
                self.min_rotation_speed_rps, self.max_rotation_speed_rps)):
            raise ValueError('third-party numeric parameters must be finite')
        if self.min_current_a < 0.0 or self.max_current_a <= self.min_current_a:
            raise ValueError('current limits must satisfy 0 <= min < max')
        if (self.min_rotation_speed_rps < 0.0 or
                self.max_rotation_speed_rps <= self.min_rotation_speed_rps):
            raise ValueError('rotation speed limits must satisfy 0 <= min < max')
        if not all((self.weld_control_topic, self.weld_parameter_topic,
                    self.motor_speed_topic, self.third_party_status_topic)):
            raise ValueError('third-party ROS topic parameters must be non-empty')

        # 创建容量为 500 的消息队列（缓冲池），存放原始字节流
        self.forward_queue = queue.Queue(maxsize=self.forward_queue_size)

        # 3. 创建 ROS 2 发布者 (Publishers)
        self.point_pub = self.create_publisher(Point, '/abb/weld_point', 10)
        self.raw_text_pub = self.create_publisher(String, '/abb/raw_text', 10)
        self.abb_tx_status_pub = self.create_publisher(
            String, '/abb/tx_status', 10)
        self.abb_tx_sub = self.create_subscription(
            String, '/abb/tx_text', self.abb_tx_callback, 10)
        self.weld_control_pub = self.create_publisher(
            String, self.weld_control_topic, 10)
        self.weld_parameter_pub = self.create_publisher(
            Float32MultiArray, self.weld_parameter_topic, 10)
        self.motor_speed_pub = self.create_publisher(
            Float32, self.motor_speed_topic, 10)
        self.third_party_status_pub = self.create_publisher(
            String, self.third_party_status_topic, 10)

        # 直接订阅焊机每个 TPDO1 的 6 字节真实反馈帧。
        # /weld/status 仍保留给人工诊断，不再从其 2 Hz 文本中提取字节。
        self.weld_feedback_sub = self.create_subscription(
            UInt8MultiArray,
            self.weld_feedback_topic,
            self.weld_feedback_callback,
            10
        )

        # 4. ROS 接口就绪后再启动第三方 TCP 线程，避免刚连接就收到命令时
        # 发布者尚未创建。
        self.forward_thread = threading.Thread(target=self.tcp_forwarder_loop)
        self.forward_thread.daemon = True
        self.forward_thread.start()

        # 5. 启动 ABB 接收主线程
        self.receive_thread = threading.Thread(target=self.tcp_server_loop)
        self.receive_thread.daemon = True
        self.receive_thread.start()
        self.abb_sender_thread = threading.Thread(target=self.abb_sender_loop)
        self.abb_sender_thread.daemon = True
        self.abb_sender_thread.start()

        self.get_logger().info(f"ABB 数据接收节点已启动，纯文本模式监听: {self.host}:{self.port} ...")
        self.get_logger().info(
            f"第三方 JSONL v{THIRD_PARTY_PROTOCOL_VERSION} 双向通道: "
            f"{self.forward_ip}:{self.forward_port}, "
            f"command_enabled={self.third_party_command_enabled}")

    def next_forward_sequence(self):
        with self.forward_sequence_lock:
            self.forward_sequence += 1
            return self.forward_sequence

    def build_outbound_frame(self, frame_type, **fields):
        frame = {
            'version': THIRD_PARTY_PROTOCOL_VERSION,
            'type': frame_type,
            'seq': self.next_forward_sequence(),
        }
        frame.update(fields)
        return encode_protocol_frame(frame)

    def queue_outbound_frame(self, frame_type, **fields):
        try:
            self.forward_queue.put_nowait(
                self.build_outbound_frame(frame_type, **fields))
        except queue.Full:
            self.forward_drop_count += 1
            if self.forward_drop_count == 1 or self.forward_drop_count % 100 == 0:
                self.get_logger().warn(
                    f'第三方发送队列已满，累计丢弃 '
                    f'{self.forward_drop_count} 帧；最新类型={frame_type}')

    def publish_third_party_status(self, accepted, request_seq, message):
        status = String()
        status.data = json.dumps({
            'accepted': accepted,
            'request_seq': request_seq,
            'message': message,
        }, ensure_ascii=True, separators=(',', ':'))
        self.third_party_status_pub.publish(status)

    def build_ack_frame(self, request_seq, accepted, message):
        return self.build_outbound_frame(
            'ack', request_seq=request_seq,
            accepted=accepted, message=message)

    def weld_feedback_callback(self, msg):
        """Queue one parsed and framed TPDO1 feedback message for 192.168.3.5."""
        if len(msg.data) != 6:
            self.get_logger().warn(
                f'Ignoring weld feedback with invalid length {len(msg.data)}; expected 6')
            return
        raw = bytes(msg.data)
        status_0 = raw[0]
        status_1 = raw[1]
        current_a = (raw[2] << 8) | raw[3]
        voltage_v = ((raw[4] << 8) | raw[5]) * 0.1
        self.queue_outbound_frame(
            'weld_feedback',
            weld_ready=bool(status_0 & 0x01),
            weld_fault=bool(status_0 & 0x02),
            locate_success=bool(status_0 & 0x04),
            arc_success=bool(status_1 & 0x01),
            arc_error=bool(status_1 & 0x02),
            gas_error=bool(status_1 & 0x04),
            wire_stick=bool(status_1 & 0x08),
            param_out_of_range=bool(status_1 & 0x80),
            current_a=current_a,
            voltage_v=round(voltage_v, 1),
            raw_hex=raw.hex().upper())

    def publish_weld_control(self, command):
        message = String()
        message.data = command
        self.weld_control_pub.publish(message)

    def publish_motor_speed(self, speed_rps):
        message = Float32()
        message.data = float(speed_rps)
        self.motor_speed_pub.publish(message)

    def handle_third_party_frame(self, line):
        """Translate one 192.168.3.5 JSONL request directly to low-level topics."""
        request_seq = None
        try:
            frame = decode_protocol_frame(line)
            request_seq = frame['seq']
            if not self.third_party_command_enabled:
                raise ValueError('third-party command input is disabled')
            if request_seq <= self.last_third_party_request_seq:
                raise ValueError(
                    'seq must increase strictly within one TCP connection')

            frame_type = frame['type']
            if frame_type == 'command':
                command = frame.get('command')
                if not isinstance(command, str):
                    raise ValueError('command must be a string')
                command = command.strip().upper()
                if command == 'GAS_ON':
                    # 一元模式：重申内置曲线，然后启动 CAN 通信并送气。
                    self.publish_weld_control('use_builtin_curve')
                    self.publish_weld_control('start_system')
                    self.publish_weld_control('start_gas')
                    message = 'published unary mode, start_system and start_gas'
                elif command == 'WELD_START':
                    self.publish_weld_control('start_welding')
                    self.third_party_welding_active = True
                    message = 'published start_welding'
                elif command == 'WELD_STOP':
                    self.publish_weld_control('stop_welding')
                    self.publish_motor_speed(0.0)
                    self.third_party_welding_active = False
                    message = 'published stop_welding and motor_speed=0'
                elif command == 'FAULT_RESET':
                    self.publish_weld_control('fault_reset')
                    message = 'published fault_reset'
                else:
                    raise ValueError(f'unsupported command: {command!r}')
            elif frame_type == 'setpoints':
                current_a = frame.get('current_a')
                speed_rps = frame.get('rotation_speed_rps')
                if (isinstance(current_a, bool) or
                        not isinstance(current_a, (int, float)) or
                        isinstance(speed_rps, bool) or
                        not isinstance(speed_rps, (int, float))):
                    raise ValueError(
                        'current_a and rotation_speed_rps must be numbers')
                current_a = float(current_a)
                speed_rps = float(speed_rps)
                if not math.isfinite(current_a) or not math.isfinite(speed_rps):
                    raise ValueError('setpoints must be finite')
                if not self.min_current_a <= current_a <= self.max_current_a:
                    raise ValueError(
                        f'current_a must be within '
                        f'[{self.min_current_a}, {self.max_current_a}]')
                if not (self.min_rotation_speed_rps <= speed_rps <=
                        self.max_rotation_speed_rps):
                    raise ValueError(
                        f'rotation_speed_rps must be within '
                        f'[{self.min_rotation_speed_rps}, '
                        f'{self.max_rotation_speed_rps}]')

                weld_parameters = Float32MultiArray()
                weld_parameters.data = [
                    current_a, self.unary_voltage_placeholder_v]
                self.weld_parameter_pub.publish(weld_parameters)
                self.publish_motor_speed(speed_rps)
                message = (
                    f'published current={current_a} A and '
                    f'rotation_speed={speed_rps} r/s')
            else:
                raise ValueError(f'unsupported request type: {frame_type!r}')

            self.last_third_party_request_seq = request_seq
            self.publish_third_party_status(True, request_seq, message)
            return self.build_ack_frame(request_seq, True, message)
        except ValueError as exc:
            message = str(exc)
            self.publish_third_party_status(False, request_seq, message)
            self.get_logger().warn(
                f'拒绝第三方协议帧 seq={request_seq}: {message}')
            return self.build_ack_frame(request_seq, False, message)

    def stop_after_third_party_disconnect(self):
        if (not self.stop_on_third_party_disconnect or
                not self.third_party_welding_active):
            return
        self.publish_weld_control('stop_welding')
        self.publish_motor_speed(0.0)
        self.third_party_welding_active = False
        self.publish_third_party_status(
            True, None,
            'third-party TCP disconnected while welding; fail-safe stop published')
        self.get_logger().error(
            '第三方 TCP 在焊接状态下断开，已发布停焊和电机停止指令')

    def abb_tx_callback(self, msg):
        """Queue newline-framed ASCII generated by the ABB bridge."""
        if not msg.data:
            return
        try:
            self.abb_tx_queue.put_nowait(msg.data.encode('ascii'))
        except UnicodeEncodeError:
            self.publish_abb_tx_status(False, 'outbound ABB text must be ASCII')
        except queue.Full:
            self.publish_abb_tx_status(False, 'ABB transmit queue is full')

    def publish_abb_tx_status(self, success, message):
        status = String()
        status.data = ('OK:' if success else 'ERROR:') + message
        self.abb_tx_status_pub.publish(status)

    def abb_sender_loop(self):
        """Send queued trajectories over the already accepted ABB socket."""
        while rclpy.ok() and self.is_running:
            try:
                payload = self.abb_tx_queue.get(timeout=1.0)
            except queue.Empty:
                continue
            with self.abb_connection_lock:
                connection = self.abb_connection
            if connection is None:
                self.publish_abb_tx_status(
                    False, 'ABB is disconnected; outbound message was dropped')
                continue
            try:
                connection.sendall(payload)
                self.publish_abb_tx_status(
                    True, f'sent {len(payload)} bytes to ABB')
            except (OSError, socket.error) as exc:
                self.publish_abb_tx_status(False, f'ABB send failed: {exc}')

    def process_received_text(self, text_data):
        """处理接收到的纯文本数据，打印并发布给 ROS"""

        clean_text = text_data.strip()
        if not clean_text:
            return

        self.get_logger().info(f"[接收文本] {clean_text}")

        # 发布原始文本，供拍照任务协调、监视和记录使用。
        raw_msg = String()
        raw_msg.data = clean_text
        self.raw_text_pub.publish(raw_msg)

        # 按行分割数据，防止粘包时一条消息里有多行
        messages = clean_text.split('\n')

        for msg in messages:
            msg = msg.strip()
            if not msg:
                continue

            try:
                # 解析类似 P1:538.91,211.97... 的格式
                if msg.startswith('P') and ':' in msg:
                    parts = msg.split(':')
                    coord_part = parts[1]

                    coords = coord_part.split(',')
                    # 兼容新版的四元数格式，只提取前三个 X, Y, Z 作为轨迹点
                    if len(coords) >= 3:
                        x = float(coords[0])
                        y = float(coords[1])
                        z = float(coords[2])

                        point_msg = Point()
                        point_msg.x = x
                        point_msg.y = y
                        point_msg.z = z
                        self.point_pub.publish(point_msg)

            except Exception as e:
                self.get_logger().error(f"解析失败: {msg}, 错误: {e}")

    def tcp_server_loop(self):
        """主接收循环：接收 ABB 数据，存入队列并解析"""
        try:
            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            self.server_socket.bind((self.host, self.port))
            self.server_socket.listen(1)
            self.server_socket.settimeout(1.0)

            while rclpy.ok() and self.is_running:
                try:
                    conn, addr = self.server_socket.accept()
                    client_ip = addr[0]

                    if client_ip == self.target_ip:
                        self.get_logger().info(f"🚀 成功连接到 ABB 机器人: {client_ip}:{addr[1]}")
                        conn.settimeout(1.0)
                        conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                        with self.abb_connection_lock:
                            self.abb_connection = conn

                        while rclpy.ok() and self.is_running:
                            try:
                                data = conn.recv(4096)
                                if not data:
                                    self.get_logger().warn("ABB 机器人已断开连接。")
                                    break

                                # 轨道1：按固定 JSONL 协议封装 ABB 原始块，第三方可
                                # 用 raw_hex 无损重建，也可直接读取 payload_ascii。
                                self.queue_outbound_frame(
                                    'abb_rx',
                                    payload_ascii=data.decode(
                                        'ascii', errors='replace'),
                                    raw_hex=data.hex().upper())

                                # 轨道2：解码为字符串，交给 ROS 节点内部处理
                                text_data = data.decode('ascii', errors='ignore')
                                self.process_received_text(text_data)

                            except socket.timeout:
                                continue
                            except Exception as e:
                                self.get_logger().error(f"接收异常: {e}")
                                break
                    else:
                        self.get_logger().warn(f"拒绝非法 IP 连接: {client_ip}")
                    with self.abb_connection_lock:
                        if self.abb_connection is conn:
                            self.abb_connection = None
                    conn.close()
                except socket.timeout:
                    pass
        except Exception as e:
            self.get_logger().error(f"Socket 启动失败: {e}")
        finally:
            if self.server_socket:
                self.server_socket.close()

    def tcp_forwarder_loop(self):
        """Maintain the full-duplex JSONL link to the third-party device."""
        forward_sock = None
        receive_buffer = bytearray()

        while rclpy.ok() and self.is_running:
            try:
                # 1. 自动连接配置的第三方设备（当前默认 192.168.3.5）
                if forward_sock is None:
                    forward_sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                    # 【本次新增】：禁用 Nagle 算法，实现真正意义上的 0 延迟，发 1 个字节也瞬间发出去！
                    forward_sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)

                    forward_sock.settimeout(2.0)
                    try:
                        forward_sock.connect((self.forward_ip, self.forward_port))
                        forward_sock.settimeout(0.05)
                        receive_buffer.clear()
                        self.last_third_party_request_seq = -1
                        # 断线期间积累的是过时的实时状态；重连后丢弃旧队列，
                        # 等待焊机和 ABB 的新帧，避免第三方误用历史数据。
                        while True:
                            try:
                                self.forward_queue.get_nowait()
                            except queue.Empty:
                                break
                        self.forward_drop_count = 0
                        self.get_logger().info(
                            f"TCP JSONL v{THIRD_PARTY_PROTOCOL_VERSION} 已连接"
                            f"第三方设备 {self.forward_ip}:{self.forward_port}")
                    except Exception:
                        forward_sock.close()
                        forward_sock = None
                        time.sleep(2.0)  # 如果连不上，默默等 2 秒再重连
                        continue

                # 2. 先批量发送反馈，避免高频 TPDO1 在队列中持续积压。
                try:
                    for _ in range(128):
                        try:
                            data_to_send = self.forward_queue.get_nowait()
                        except queue.Empty:
                            break
                        forward_sock.sendall(data_to_send)

                    # 3. 接收 192.168.3.5 的逐行 JSON 指令。buffer 同时处理
                    # TCP 半包、粘包和一批多条指令。
                    try:
                        received = forward_sock.recv(4096)
                    except socket.timeout:
                        continue
                    if not received:
                        raise ConnectionError('third-party device closed connection')
                    for line in extract_jsonl_lines(
                            receive_buffer, received,
                            self.third_party_max_line_bytes):
                        if len(line) > self.third_party_max_line_bytes:
                            response = self.build_ack_frame(
                                None, False,
                                'third-party frame exceeds maximum line length')
                        else:
                            response = self.handle_third_party_frame(line)
                        # ACK 优先直接发送，避免被高频焊机反馈排在队尾。
                        forward_sock.sendall(response)
                except Exception as e:
                    self.get_logger().warn(
                        f"第三方 TCP 连接异常 {self.forward_ip}: "
                        f"{e}")
                    self.stop_after_third_party_disconnect()
                    forward_sock.close()
                    forward_sock = None
                    receive_buffer.clear()

            except Exception:
                time.sleep(1.0)

        if forward_sock:
            forward_sock.close()

    def destroy_node(self):
        self.stop_after_third_party_disconnect()
        self.is_running = False
        with self.abb_connection_lock:
            connection = self.abb_connection
            self.abb_connection = None
        if connection:
            try:
                connection.shutdown(socket.SHUT_RDWR)
                connection.close()
            except OSError:
                pass
        if self.server_socket:
            self.server_socket.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DataReceiverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("节点被手动终止")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
