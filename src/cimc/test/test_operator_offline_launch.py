"""Inspect launch actions only; never execute a launch or any ROS node."""
import importlib.util
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from launch.actions import ExecuteProcess
from launch_ros.actions import Node


class OfflineLaunchTest(unittest.TestCase):
    def test_offline_branch_has_no_receiver_or_runtime_nodes(self):
        source = Path(__file__).resolve().parents[1] / 'launch/camera_weld_operator_abb.launch.py'
        spec = importlib.util.spec_from_file_location('operator_launch_under_test', source)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        context = SimpleNamespace(perform_substitution=lambda _value: 'offline')
        with patch.object(module, '_validate_abb_test_configuration', side_effect=AssertionError('must not preflight ABB')), \
                patch.object(module, '_manual_runtime', side_effect=AssertionError('must not start runtime')), \
                patch.object(module, 'LogInfo', return_value=Mock()):
            actions = module._select_mode(context, 'mode', '/tmp/workspace', 'camera.yaml',
                                          'weld.yaml', 'handeye.yaml', 'task.yaml', 'abb.yaml', 'false', 'false')
        self.assertFalse(any(isinstance(action, Node) for action in actions))
        self.assertEqual(sum(isinstance(action, ExecuteProcess) for action in actions), 1)
