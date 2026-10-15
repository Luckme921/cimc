"""Offline checks for the operator form; no Tk display or hardware needed."""

import sys
import hashlib
import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import Mock, patch

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cimc.weld_operator_setup import (  # noqa: E402
    CAMERA_FIELDS, GROUP_DEFAULTS, ROI_KEYS, WELD_COMMON, SetupWindow,
    active_overrides, camera_save_directory, replace_active_overrides,
    replace_camera_parameters, replace_handeye_parameters,
    inspect_handeye_matrix, viewer_command, camera_direct_environment,
)


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value

    def set(self, value):
        self.value = value


class OperatorConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[3]
        cls.weld_text = (root / 'src/weld_seam_perception/config/weld_seam.yaml').read_text()
        cls.camera_text = (root / 'src/chishine_camera_ros2/config/camera.yaml').read_text()
        cls.handeye_text = (root / 'src/cimc/config/handeye_bridge.yaml').read_text()

    def make_form(self):
        form = SetupWindow.__new__(SetupWindow)
        form.original = active_overrides(self.weld_text)
        form.camera_params = yaml.safe_load(self.camera_text)[
            'chishine_camera_node']['ros__parameters']
        form.vars = {'roi.enable': Value(form.original.get('roi.enable', 'false'))}
        for key in ROI_KEYS[1:]:
            form.vars[key] = Value(form.original.get(
                key, '-inf' if 'min_' in key else 'inf'))
        form.vars['path.mode'] = Value(form.original.get('path.mode', 'feature_points'))
        for group, defaults in GROUP_DEFAULTS.items():
            for axis, default in zip('xyz', defaults):
                key = f'offset.{group}.{axis}'
                form.vars[key] = Value(form.original.get(key, str(default)))
        for key, _label, default in WELD_COMMON:
            form.vars[key] = Value(form.original.get(key, default))
        form.camera_vars = {key: Value(str(form.camera_params[key]))
                            for key, _label in CAMERA_FIELDS}
        return form

    def test_unchanged_inputs_do_not_rewrite_yaml(self):
        form = self.make_form()
        self.assertEqual(form.changes(), {})
        self.assertEqual(form.camera_changes(), {})

    def test_camera_edit_reaches_viewer_and_yaml(self):
        form = self.make_form()
        form.camera_vars['depth_exposure'].value = '15000'
        changes = form.camera_changes()
        self.assertEqual(changes, {'depth_exposure': '15000.0'})
        updated = replace_camera_parameters(self.camera_text, changes)
        params = yaml.safe_load(updated)['chishine_camera_node']['ros__parameters']
        self.assertEqual(params['depth_exposure'], 15000)
        command = viewer_command('/tmp/viewer', camera_save_directory(params),
                                 '/tmp/manifest', form.current_camera_params(),
                                 {key: form.vars[key].get() for key in ROI_KEYS})
        self.assertEqual(command[command.index('--exposure') + 1], '15000.0')
        self.assertEqual(command[command.index('--attempts') + 1], '3')
        self.assertEqual(command[command.index('--max-display-points') + 1], '150000')

    def test_viewer_s_key_imports_roi_before_viewer_exits(self):
        form = self.make_form()
        form.ply_label = Mock()
        form.status = Mock()
        form.update_review = Mock()
        form.preview_signature = 'stale'
        form.viewer_manifest_time = None
        form.viewer_saved_this_run = False
        with tempfile.TemporaryDirectory(prefix='operator_viewer_test_', dir='/tmp') as directory:
            form.manifest = Path(directory) / 'snapshot.yaml'
            saved = Path(directory) / 'saved.ply'
            saved.touch()
            form.manifest.write_text(yaml.safe_dump({
                'ply': str(saved), 'roi.enable': True,
                'roi.min_x': -20.5, 'roi.max_x': 120.0,
            }), encoding='utf-8')
            form.import_viewer_snapshot()
            self.assertEqual(form.ply, saved)
            self.assertEqual(form.vars['roi.enable'].get(), 'true')
            self.assertEqual(form.vars['roi.min_x'].get(), '-20.5')
            self.assertTrue(form.viewer_saved_this_run)
            self.assertIsNone(form.preview_signature)
            form.update_review.assert_called_once()
            form.import_viewer_snapshot()
            form.update_review.assert_called_once()

    def test_viewer_poll_keeps_gui_responsive_until_q(self):
        form = self.make_form()
        form.root = Mock()
        form.status = Mock()
        form.viewer_process = Mock()
        form.viewer_process.poll.side_effect = [None, 0]
        form.viewer_manifest_time = None
        form.viewer_saved_this_run = True
        form.viewer_running = True
        with tempfile.TemporaryDirectory(prefix='operator_viewer_test_', dir='/tmp') as directory:
            form.manifest = Path(directory) / 'missing.yaml'
            form.poll_viewer()
            form.root.after.assert_called_once()
            self.assertTrue(form.viewer_running)
            form.poll_viewer()
            self.assertFalse(form.viewer_running)
            self.assertIsNone(form.viewer_process)

    def test_camera_viewer_bypasses_proxy_without_changing_parent(self):
        with patch.dict(os.environ, {'HTTP_PROXY': 'http://127.0.0.1:7688',
                                     'NO_PROXY': 'localhost'}):
            environment = camera_direct_environment()
            self.assertEqual(environment['NO_PROXY'], '*')
            self.assertEqual(environment['no_proxy'], '*')
            self.assertEqual(os.environ['NO_PROXY'], 'localhost')

    def test_common_weld_edit_reaches_algorithm_override(self):
        form = self.make_form()
        form.vars['path.rounded_corner_spacing'].value = '4'
        changes = form.changes()
        self.assertEqual(changes, {'path.rounded_corner_spacing': '4'})
        updated = replace_active_overrides(self.weld_text, changes)
        self.assertEqual(active_overrides(updated)['path.rounded_corner_spacing'], '4')

    def test_reject_exposure_longer_than_frame_and_even_corner_count(self):
        form = self.make_form()
        form.camera_vars['depth_exposure'].value = '25000'
        with self.assertRaises(ValueError):
            form.camera_changes()
        form.vars['path.rounded_corner_point_count'].value = '4'
        with self.assertRaises(ValueError):
            form.changes()

    def test_handeye_matrix_validation_and_direction_write(self):
        params = yaml.safe_load(self.handeye_text)[
            'handeye_abb_bridge_node']['ros__parameters']
        matrix, digest = inspect_handeye_matrix(params['matrix_file'], params['matrix_key'])
        self.assertEqual(matrix.shape, (4, 4))
        self.assertEqual(len(digest), 64)
        changed = replace_handeye_parameters(
            self.handeye_text, {'matrix_direction': 'camera_from_tcp'})
        self.assertEqual(yaml.safe_load(changed)[
            'handeye_abb_bridge_node']['ros__parameters']['matrix_direction'],
            'camera_from_tcp')
        self.assertIn('send_to_abb: true', changed)

    def test_final_approval_writes_three_temp_configs_only(self):
        form = self.make_form()
        params = yaml.safe_load(self.handeye_text)[
            'handeye_abb_bridge_node']['ros__parameters']
        form.handeye_params = params
        form.handeye_vars = {key: Value(str(params[key])) for key in (
            'matrix_file', 'matrix_key', 'matrix_direction', 'matrix_translation_unit')}
        form.handeye_vars['matrix_direction'].value = 'camera_from_tcp'
        form.camera_vars['depth_exposure'].value = '15000'
        form.vars['path.rounded_corner_spacing'].value = '4'
        form.original_text = self.weld_text
        form.original_hash = hashlib.sha256(self.weld_text.encode()).hexdigest()
        form.camera_hash = hashlib.sha256(self.camera_text.encode()).hexdigest()
        form.handeye_hash = hashlib.sha256(self.handeye_text.encode()).hexdigest()
        _matrix, digest = inspect_handeye_matrix(params['matrix_file'], params['matrix_key'])
        form.initial_matrix_hash = digest
        form.verified_matrix_digest = digest
        form.matrix_verified = Value(True)
        form.viewer_running = False
        form.ply = None
        form.preview_rows = [{'order': '0'}]
        form.preview_result_ply = None
        form.preview_warnings = []
        form.preview_signature = form.signature(
            form.changes(), form.camera_changes(), form.handeye_changes()[0], digest)
        form.root = Mock()
        with tempfile.TemporaryDirectory(prefix='operator_approve_test_', dir='/tmp') as directory:
            base = Path(directory)
            form.workspace = base
            form.session = base / 'session'
            form.session.mkdir()
            form.config_path = base / 'weld_seam.yaml'
            form.camera_path = base / 'camera.yaml'
            form.handeye_path = base / 'handeye_bridge.yaml'
            form.config_path.write_text(self.weld_text)
            form.camera_path.write_text(self.camera_text)
            form.handeye_path.write_text(self.handeye_text)
            with patch('cimc.weld_operator_setup.messagebox.askyesno', return_value=True), \
                    patch('cimc.weld_operator_setup.messagebox.showerror') as error:
                form.approved = False
                form.approve()
                error.assert_not_called()
            self.assertTrue(form.approved)
            self.assertEqual(active_overrides(form.config_path.read_text())[
                'path.rounded_corner_spacing'], '4')
            self.assertEqual(yaml.safe_load(form.camera_path.read_text())[
                'chishine_camera_node']['ros__parameters']['depth_exposure'], 15000)
            self.assertEqual(yaml.safe_load(form.handeye_path.read_text())[
                'handeye_abb_bridge_node']['ros__parameters']['matrix_direction'],
                'camera_from_tcp')
            self.assertTrue((form.session / 'approval.json').is_file())


if __name__ == '__main__':
    unittest.main()
