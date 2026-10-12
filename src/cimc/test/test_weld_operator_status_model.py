"""Dashboard summary tests without ROS graph or hardware."""

import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cimc.weld_operator_status_model import StatusModel  # noqa: E402


class StatusModelTest(unittest.TestCase):
    def test_separate_prepared_send_success_and_ack(self):
        model = StatusModel()
        model.accept('/abb/connection_status', '{"state":"connected","peer":"192.168.3.2:5"}', '10:00:00')
        self.assertIn('已连接', model.connection)
        model.accept('/abb/tx_text', 'TRAJECTORY_BEGIN:2\nP1:1,2,3,1,0,0,0\nP2:2,2,3,1,0,0,0\nTRAJECTORY_END')
        self.assertIn('TRAJECTORY_BEGIN:2', model.prepared)
        self.assertEqual(model.sent, '—')
        model.accept('/abb/tx_status', 'OK:sent 100 bytes to ABB')
        self.assertIn('OK:', model.sent)
        self.assertEqual(model.ack, '—')
        model.accept('/abb/raw_text', 'TRAJECTORY_ACK:1,2')
        self.assertEqual(model.ack, 'TRAJECTORY_ACK:1,2')
        model.accept('/abb/raw_text', 'START_CAPTURE:1,2,3,1,0,0,0')
        self.assertEqual(model.ack, '等待本次 ABB ACK')
        self.assertEqual(model.sent, '等待本次发送结果')

    def test_timing_and_algorithm(self):
        model = StatusModel()
        model.accept('/weld_seam/status',
                     '{"status":0,"path_points":29,"total_ms":804.94}')
        self.assertIn('29', model.algorithm)
        model.accept('/abb/task_timing',
                     '{"task_id":3,"success":true,"elapsed_ms":1204.7,'
                     '"success_count":2,"average_ms":1250.0,'
                     '"minimum_ms":1204.7,"maximum_ms":1295.3}')
        self.assertIn('1204.7', model.timing)
        self.assertIn('平均', model.statistics)


if __name__ == '__main__':
    unittest.main()
