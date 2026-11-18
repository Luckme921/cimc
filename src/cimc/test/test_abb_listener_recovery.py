"""Listener failure/retry tests use fake sockets only; no network connection."""
import errno
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from cimc.data_receiver_node import DataReceiverNode, explain_listen_error


class ListenerRecoveryTest(unittest.TestCase):
    def test_missing_local_address_is_explained(self):
        state, reason = explain_listen_error(OSError(errno.EADDRNOTAVAIL, 'missing'),
                                            '192.168.3.100', 45000)
        self.assertEqual(state, 'waiting_address')
        self.assertIn('本机尚无监听地址', reason)
        self.assertIn('workflow_mode:=offline', reason)

    def test_rebind_recovers_after_address_becomes_available(self):
        bad, good = Mock(), Mock()
        bad.bind.side_effect = OSError(errno.EADDRNOTAVAIL, 'missing')
        node = SimpleNamespace(host='192.168.3.100', port=45000, is_running=True,
                               server_socket=None, listen_retry_seconds=3,
                               listen_stop_event=Mock(), publish_abb_connection_status=Mock(),
                               get_logger=Mock(return_value=Mock()))
        node.listen_stop_event.wait.return_value = False
        with patch('cimc.data_receiver_node.rclpy.ok', return_value=True), \
                patch('cimc.data_receiver_node.socket.socket', side_effect=[bad, good]):
            self.assertTrue(DataReceiverNode.open_abb_listener(node))
        bad.close.assert_called_once()
        good.bind.assert_called_once_with(('192.168.3.100', 45000))
        self.assertIs(node.server_socket, good)
        self.assertEqual(node.publish_abb_connection_status.call_args_list[0].args[0], 'waiting_address')
        self.assertEqual(node.publish_abb_connection_status.call_args_list[-1].args[0], 'listening')

    def test_cancel_stops_retries_without_widening_listen_address(self):
        bad = Mock()
        bad.bind.side_effect = OSError(errno.EADDRINUSE, 'busy')
        node = SimpleNamespace(host='192.168.3.100', port=45000, is_running=True,
                               server_socket=None, listen_retry_seconds=3,
                               listen_stop_event=Mock(), publish_abb_connection_status=Mock(),
                               get_logger=Mock(return_value=Mock()))
        node.listen_stop_event.wait.return_value = True
        with patch('cimc.data_receiver_node.rclpy.ok', return_value=True), \
                patch('cimc.data_receiver_node.socket.socket', return_value=bad):
            self.assertFalse(DataReceiverNode.open_abb_listener(node))
        bad.bind.assert_called_once_with(('192.168.3.100', 45000))
        self.assertIsNone(node.server_socket)
