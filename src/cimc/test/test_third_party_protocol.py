import unittest

from geometry_msgs.msg import Pose, PoseArray
from std_msgs.msg import UInt8MultiArray

from cimc.data_receiver_node import (
    DataReceiverNode,
    decode_protocol_frame,
    encode_protocol_frame,
    extract_abb_lines,
    extract_jsonl_lines,
)
from cimc.handeye_abb_bridge_node import HandeyeAbbBridgeNode


class _PublisherRecorder:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class _LoggerRecorder:
    def __init__(self):
        self.warnings = []

    def warn(self, message):
        self.warnings.append(message)


class _BridgeStub:
    """Only the state used by DataReceiverNode protocol callbacks."""

    def __init__(self):
        self.third_party_command_enabled = True
        self.last_third_party_request_seq = -1
        self.third_party_welding_active = False
        self.min_current_a = 1.0
        self.max_current_a = 350.0
        self.min_rotation_speed_rps = 0.0
        self.max_rotation_speed_rps = 6.0
        self.unary_voltage_placeholder_v = 20.0
        self.weld_parameter_pub = _PublisherRecorder()
        self.control_commands = []
        self.motor_speeds = []
        self.statuses = []
        self.outbound = []
        self.logger = _LoggerRecorder()

    def publish_weld_control(self, command):
        self.control_commands.append(command)

    def publish_motor_speed(self, speed):
        self.motor_speeds.append(float(speed))

    def publish_third_party_status(self, accepted, request_seq, message):
        self.statuses.append((accepted, request_seq, message))

    def build_ack_frame(self, request_seq, accepted, message):
        return encode_protocol_frame({
            'version': 1,
            'type': 'ack',
            'seq': 100,
            'request_seq': request_seq,
            'accepted': accepted,
            'message': message,
        })

    def queue_outbound_frame(self, frame_type, **fields):
        self.outbound.append((frame_type, fields))

    def get_logger(self):
        return self.logger


def _request(frame_type, sequence, **fields):
    frame = {'version': 1, 'type': frame_type, 'seq': sequence}
    frame.update(fields)
    return encode_protocol_frame(frame).rstrip(b'\n')


class ThirdPartyProtocolTest(unittest.TestCase):
    def test_abb_trajectory_wire_format(self):
        bridge = type('BridgeStub', (), {'protocol_precision': 6})()
        trajectory = PoseArray()
        pose = Pose()
        pose.position.x = 1.090441957
        pose.position.y = -0.272560220
        pose.position.z = 0.689289440
        pose.orientation.w = 0.765853257
        pose.orientation.x = -0.320902583
        pose.orientation.y = -0.215710919
        pose.orientation.z = 0.513769519
        trajectory.poses.append(pose)

        payload = HandeyeAbbBridgeNode._serialize_for_abb(
            bridge, trajectory)

        self.assertEqual(
            payload,
            'TRAJECTORY_BEGIN:1\n'
            'P1:1090.441957,-272.560220,689.289440,'
            '0.765853,-0.320903,-0.215711,0.513770\n'
            'TRAJECTORY_END\n')

    def test_abb_tcp_half_packet_sticky_packet_and_crlf(self):
        first = (
            b'START_CAPTURE:947.35,-79.0,717.49,'
            b'0.38469,-0.77967,0.40899,-0.27723\n')
        second = b'P1:1000,200,700,1,0,0,0\r\n'
        receive_buffer = bytearray()

        split = len(first) // 2
        self.assertEqual(
            extract_abb_lines(receive_buffer, first[:split], 4096), [])
        lines = extract_abb_lines(
            receive_buffer, first[split:] + second, 4096)
        self.assertEqual(lines, [first.rstrip(b'\n'), second.rstrip(b'\r\n')])
        self.assertEqual(receive_buffer, bytearray())

        with self.assertRaisesRegex(ValueError, 'maximum line length'):
            extract_abb_lines(bytearray(), b'x' * 9, 8)

    def test_common_frame_round_trip_and_validation(self):
        encoded = _request(
            'setpoints', 2, current_a=250.0, rotation_speed_rps=6.0)
        decoded = decode_protocol_frame(encoded)
        self.assertEqual(decoded['seq'], 2)
        self.assertEqual(decoded['current_a'], 250.0)

        with self.assertRaisesRegex(ValueError, 'unsupported version'):
            decode_protocol_frame(b'{"version":2,"type":"command","seq":1}')
        with self.assertRaisesRegex(ValueError, 'non-negative integer'):
            decode_protocol_frame(b'{"version":1,"type":"command","seq":true}')

    def test_jsonl_tcp_half_packet_and_sticky_packet(self):
        first = _request('command', 1, command='GAS_ON') + b'\n'
        second = _request('command', 2, command='WELD_START') + b'\r\n'
        receive_buffer = bytearray()

        split = len(first) // 2
        self.assertEqual(
            extract_jsonl_lines(receive_buffer, first[:split], 4096), [])
        lines = extract_jsonl_lines(
            receive_buffer, first[split:] + second, 4096)
        self.assertEqual(len(lines), 2)
        self.assertEqual(decode_protocol_frame(lines[0])['seq'], 1)
        self.assertEqual(decode_protocol_frame(lines[1])['seq'], 2)
        self.assertEqual(receive_buffer, bytearray())

        with self.assertRaisesRegex(ValueError, 'maximum line length'):
            extract_jsonl_lines(bytearray(), b'x' * 9, 8)

    def test_command_and_setpoint_mapping(self):
        bridge = _BridgeStub()

        gas_ack = DataReceiverNode.handle_third_party_frame(
            bridge, _request('command', 1, command='GAS_ON'))
        self.assertEqual(bridge.control_commands, [
            'use_builtin_curve', 'start_system', 'start_gas'])
        self.assertTrue(decode_protocol_frame(gas_ack.rstrip(b'\n'))['accepted'])

        DataReceiverNode.handle_third_party_frame(
            bridge,
            _request(
                'setpoints', 2, current_a=250.0,
                rotation_speed_rps=6.0))
        self.assertEqual(
            list(bridge.weld_parameter_pub.messages[-1].data),
            [250.0, 20.0])
        self.assertEqual(bridge.motor_speeds[-1], 6.0)

        DataReceiverNode.handle_third_party_frame(
            bridge, _request('command', 3, command='WELD_START'))
        DataReceiverNode.handle_third_party_frame(
            bridge, _request('command', 4, command='WELD_STOP'))
        self.assertEqual(bridge.control_commands[-2:], [
            'start_welding', 'stop_welding'])
        self.assertEqual(bridge.motor_speeds[-1], 0.0)
        self.assertFalse(bridge.third_party_welding_active)

    def test_invalid_or_duplicate_request_does_not_publish(self):
        bridge = _BridgeStub()
        DataReceiverNode.handle_third_party_frame(
            bridge, _request('command', 5, command='FAULT_RESET'))
        before = list(bridge.control_commands)

        ack = DataReceiverNode.handle_third_party_frame(
            bridge, _request('command', 5, command='WELD_START'))
        self.assertEqual(bridge.control_commands, before)
        self.assertFalse(decode_protocol_frame(ack.rstrip(b'\n'))['accepted'])

        ack = DataReceiverNode.handle_third_party_frame(
            bridge,
            _request(
                'setpoints', 6, current_a=351.0,
                rotation_speed_rps=6.0))
        self.assertEqual(bridge.weld_parameter_pub.messages, [])
        self.assertFalse(decode_protocol_frame(ack.rstrip(b'\n'))['accepted'])

        ack = DataReceiverNode.handle_third_party_frame(
            bridge,
            _request(
                'setpoints', 7, current_a=250.0,
                rotation_speed_rps=6.1))
        self.assertEqual(bridge.weld_parameter_pub.messages, [])
        self.assertFalse(decode_protocol_frame(ack.rstrip(b'\n'))['accepted'])

    def test_weld_feedback_fields_and_raw_bytes(self):
        bridge = _BridgeStub()
        message = UInt8MultiArray()
        message.data = [0x01, 0x01, 0x00, 0xF8, 0x01, 0x29]

        DataReceiverNode.weld_feedback_callback(bridge, message)

        frame_type, fields = bridge.outbound[-1]
        self.assertEqual(frame_type, 'weld_feedback')
        self.assertTrue(fields['weld_ready'])
        self.assertTrue(fields['arc_success'])
        self.assertEqual(fields['current_a'], 248)
        self.assertEqual(fields['voltage_v'], 29.7)
        self.assertEqual(fields['raw_hex'], '010100F80129')


if __name__ == '__main__':
    unittest.main()
