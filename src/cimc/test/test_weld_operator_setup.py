"""Offline checks for the operator form; no Tk display or hardware needed."""

import sys
import unittest
from pathlib import Path

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from cimc.weld_operator_setup import (  # noqa: E402
    CAMERA_FIELDS, GROUP_DEFAULTS, ROI_KEYS, WELD_COMMON, SetupWindow,
    active_overrides, camera_save_directory, replace_active_overrides,
    replace_camera_parameters, viewer_command,
)


class Value:
    def __init__(self, value):
        self.value = value

    def get(self):
        return self.value


class OperatorConfigTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        root = Path(__file__).resolve().parents[3]
        cls.weld_text = (root / 'src/weld_seam_perception/config/weld_seam.yaml').read_text()
        cls.camera_text = (root / 'src/chishine_camera_ros2/config/camera.yaml').read_text()

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


if __name__ == '__main__':
    unittest.main()
