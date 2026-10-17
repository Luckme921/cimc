#!/usr/bin/env python3
"""Offline safety-pose regression against the saved field CSV; never starts ROS/ABB."""

import csv
import math
from pathlib import Path
import subprocess
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DATA = Path('/home/mini/scut_weld_data/pointclouds')
INSTALLED = WORKSPACE / 'x86_weld_seam_test/install/bin/weld_seam_extractor'
NEW = WORKSPACE / 'x86_weld_seam_test/build/weld_seam_extractor'
YAML = WORKSPACE / 'src/weld_seam_perception/config/weld_seam.yaml'
FIELD_CSV = Path('/home/mini/scut_weld_data/weld_results/'
                 'weld_seam_capture_1792068502726_features.csv')


def run(extractor, ply, overrides, output, prefix):
    command = [str(extractor), str(ply), str(output), prefix]
    for value in overrides.values():
        command.extend(('--set', value))
    completed = subprocess.run(command, capture_output=True, text=True, check=False)
    if completed.returncode:
        raise AssertionError(f'{prefix}: extraction failed: {completed.stderr[-1000:]}')
    with (output / f'{prefix}_features.csv').open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def angle(first, second):
    fields = ('qw', 'qx', 'qy', 'qz')
    dot = abs(sum(float(first[key]) * float(second[key]) for key in fields))
    first_norm = math.sqrt(sum(float(first[key]) ** 2 for key in fields))
    second_norm = math.sqrt(sum(float(second[key]) ** 2 for key in fields))
    dot /= first_norm * second_norm
    return math.degrees(2.0 * math.acos(min(1.0, dot)))


def assert_same_path(reference, candidate, name, ignore_transition_pose=False):
    assert len(reference) == len(candidate), f'{name}: point count changed'
    assert len(reference) <= 100, f'{name}: ABB point limit exceeded'
    for old, new in zip(reference, candidate):
        assert old['feature_type'] == new['feature_type'], f'{name}: feature topology changed'
        assert old['point_source'] == new['point_source'], f'{name}: point source changed'
        for key in ('x', 'y', 'z', 'raw_x', 'raw_y', 'raw_z'):
            assert abs(float(old[key]) - float(new[key])) < 0.05, \
                f'{name}: {key} changed at {old["order"]}'
        if not ignore_transition_pose or old['weld_enabled'] == '1':
            assert angle(old, new) < 0.1, \
                f'{name}: weld pose changed {angle(old, new):.3f}° at {old["order"]}'


def main():
    params = yaml.safe_load(YAML.read_text(encoding='utf-8'))[
        'weld_seam_node']['ros__parameters']
    common = {item.split('=', 1)[0]: item for item in params['algorithm_overrides']}
    for key in ('orientation.transition_waist_flat_blend',
                'orientation.transition_waist_max_turn_deg'):
        common.pop(key, None)
    # The approved 2026-10-15 ROI reproduces the field's two straight-waist
    # endpoints. The current YAML ROI was subsequently changed and no longer
    # represents that exposure, so using it here would test a different weld.
    common.update({
        'roi.min_x': 'roi.min_x=-203.332',
        'roi.max_x': 'roi.max_x=203.674',
        'roi.min_y': 'roi.min_y=-28.5993',
        'roi.max_y': 'roi.max_y=155.179',
    })
    ply = DATA / 'capture_1792068502726.ply'
    assert ply.is_file(), f'missing field PLY: {ply}'
    assert INSTALLED.is_file() and NEW.is_file(), 'build and install the SDK first'
    assert FIELD_CSV.is_file(), f'missing saved field CSV: {FIELD_CSV}'
    with tempfile.TemporaryDirectory(prefix='weld_transition_regression_') as folder:
        output = Path(folder)
        with FIELD_CSV.open(newline='', encoding='utf-8') as stream:
            old = list(csv.DictReader(stream))
        unchanged = run(NEW, ply, common, output, 'new_blend_off')
        assert_same_path(old, unchanged, 'saved field result vs new, blend off')
        changed_settings = dict(common)
        changed_settings['orientation.transition_waist_flat_blend'] = \
            'orientation.transition_waist_flat_blend=0.5'
        changed_settings['orientation.transition_waist_max_turn_deg'] = \
            'orientation.transition_waist_max_turn_deg=15.0'
        changed = run(NEW, ply, changed_settings, output, 'new_blend_on')
        assert_same_path(unchanged, changed, 'blend on', ignore_transition_pose=True)
        assert len(changed) == 33, f'expected field path of 33 points, got {len(changed)}'
        for index, neighbor in ((0, 1), (-1, -2)):
            safety = changed[index]
            assert safety['pose_source'] == 'safety_waist_to_flat_blend', \
                'waist boundary was not recognized'
            step = angle(safety, changed[neighbor])
            assert 1.0 < step <= 15.1, f'unsafe or ineffective safety turn: {step:.2f}°'
            print(f'{safety["feature_type"]}: turn {step:.2f}°, '
                  f'pose group {safety["pose_group"]}')
        print(f'PASS: {len(changed)} points, all weld XYZ/quaternions unchanged; '
              'only two non-weld safety quaternions changed')

        # Other proven point clouds use corner/flat boundaries. This option
        # must not alter them or activate unrelated extraction changes.
        full_roi = dict(common)
        for axis in 'xyz':
            full_roi[f'roi.min_{axis}'] = f'roi.min_{axis}=-inf'
            full_roi[f'roi.max_{axis}'] = f'roi.max_{axis}=inf'
        for stem in ('capture_1792068502726', 'capture_1790216105559',
                     'live_20260924_095031_427', '20260616测试视野-2'):
            cloud = DATA / f'{stem}.ply'
            if not cloud.is_file():
                raise AssertionError(f'missing regression PLY: {cloud}')
            before = run(INSTALLED, cloud, full_roi, output, f'{stem}_installed')
            after = run(NEW, cloud, full_roi, output, f'{stem}_new')
            assert_same_path(before, after, f'{stem} installed vs build')
            with_blend = dict(full_roi)
            with_blend.update({
                'orientation.transition_waist_flat_blend':
                    'orientation.transition_waist_flat_blend=0.5',
                'orientation.transition_waist_max_turn_deg':
                    'orientation.transition_waist_max_turn_deg=15.0',
            })
            after_blend = run(NEW, cloud, with_blend, output, f'{stem}_blend')
            assert_same_path(after, after_blend, f'{stem} with waist blend',
                             ignore_transition_pose=True)
            for index, neighbor in ((0, 1), (-1, -2)):
                if after_blend[index]['pose_source'] == 'safety_waist_to_flat_blend':
                    assert angle(after_blend[index], after_blend[neighbor]) <= 15.1
                else:
                    assert angle(after[index], after_blend[index]) < 0.1
            print(f'PASS: {stem}, {len(after)} points, weld poses preserved')


if __name__ == '__main__':
    main()
