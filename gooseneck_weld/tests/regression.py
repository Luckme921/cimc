#!/usr/bin/env python3
"""Real historical PLY regression. Does not start ROS or any device."""
import csv
import math
from pathlib import Path
import subprocess
import tempfile
import numpy as np

ROOT = Path(__file__).resolve().parents[2]
DATA = Path('/home/mini/scut_weld_data/pointclouds')
SDK = ROOT / 'gooseneck_weld/install/bin/gooseneck_weld_extractor'


def rows(path):
    with path.open(newline='', encoding='utf-8') as stream:
        return list(csv.DictReader(stream))


def run(ply, directory, prefix, *settings):
    cmd = [str(SDK), str(ply), str(directory), prefix]
    for setting in settings:
        cmd += ['--set', setting]
    result = subprocess.run(cmd, capture_output=True, text=True, timeout=30)
    if result.returncode:
        raise AssertionError(result.stderr + result.stdout[-1000:])
    return rows(directory / f'{prefix}_features.csv')


def quaternion(row):
    q = np.array([float(row[key]) for key in ('qw', 'qx', 'qy', 'qz')])
    assert abs(np.linalg.norm(q) - 1) < 1e-8
    return q


def axes(row):
    w, x, y, z = quaternion(row)
    matrix = np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])
    assert np.allclose(matrix.T @ matrix, np.eye(3), atol=1e-8)
    assert abs(np.linalg.det(matrix) - 1) < 1e-8
    return matrix


def check_tilt_and_lean(cloud, out, original, slot=1):
    """Correct physical directions without moving TCP targets or flipping tool X."""
    reference_x = axes(original[0])[:, 0]
    for tilt in (-15, 15):
        tilted = run(cloud, out, f'tilt_{slot}_{tilt}',
                     f'gooseneck.groove_index={slot}', f'gooseneck.body_tilt_deg={tilt}')
        for before, after in zip(original, tilted):
            np.testing.assert_allclose([float(after[k]) for k in 'xyz'],
                                       [float(before[k]) for k in 'xyz'], atol=1e-9)
            np.testing.assert_allclose(axes(after)[:, 0], axes(before)[:, 0], atol=1e-9)
            turn = math.degrees(2*math.acos(min(1., abs(quaternion(before) @ quaternion(after)))))
            assert abs(turn-abs(tilt)) < 1e-5
            assert tilt*(float(after['body_angle_to_base_deg'])-
                         float(before['body_angle_to_base_deg'])) > 0
    leaned = run(cloud, out, f'lean_{slot}', f'gooseneck.groove_index={slot}',
                 'gooseneck.side_lean_deg=10')
    reversed_x = run(cloud, out, f'lean_flip_x_{slot}', f'gooseneck.groove_index={slot}',
                     'gooseneck.side_lean_deg=10', 'gooseneck.tool_x_positive=false')
    reversed_z = run(cloud, out, f'lean_flip_z_{slot}', f'gooseneck.groove_index={slot}',
                     'gooseneck.side_lean_deg=10', 'gooseneck.tool_z_to_body=true')
    combined = run(cloud, out, f'lean_tilt_{slot}', f'gooseneck.groove_index={slot}',
                   'gooseneck.side_lean_deg=10', 'gooseneck.body_tilt_deg=-10')
    for i, (before, after, flip_x, flip_z, both) in enumerate(
            zip(original, leaned, reversed_x, reversed_z, combined)):
        for candidate in (after, flip_x, flip_z, both):
            np.testing.assert_allclose([float(candidate[k]) for k in 'xyz'],
                                       [float(before[k]) for k in 'xyz'], atol=1e-9)
            axes(candidate)
        np.testing.assert_allclose(axes(after)[:, 0] @ reference_x, math.cos(math.radians(10)), atol=1e-9)
        body = np.array([float(after[f'world_torch_body_axis_{k}']) for k in 'xyz'])
        assert abs(body @ reference_x - (-1 if i<=3 else 1)*math.sin(math.radians(10))) < 1e-9
        for flipped in (flip_x, flip_z):
            np.testing.assert_allclose(body, [float(flipped[f'world_torch_body_axis_{k}']) for k in 'xyz'], atol=1e-9)
        assert after['pose_source'] == 'outward_side_lean_frame'
    np.testing.assert_allclose(quaternion(leaned[0]), quaternion(leaned[1]), atol=1e-9)
    np.testing.assert_allclose(quaternion(leaned[6]), quaternion(leaned[7]), atol=1e-9)
    max_turn = max(math.degrees(2*math.acos(min(1., abs(quaternion(a)@quaternion(b)))))
                   for a,b in zip(leaned, leaned[1:]))
    assert abs(max_turn-20) < 1e-5


def main():
    with tempfile.TemporaryDirectory(prefix='gooseneck_regression_') as folder:
        out = Path(folder)
        ply = DATA / 'live_20261104_160336_814.ply'
        original = run(ply, out, 'original', 'gooseneck.tool_x_positive=true')
        current = run(ply, out, 'current_default')
        assert current == original, 'SDK zero-adjustment default must preserve portable +X'
        check_tilt_and_lean(ply, out, original)
        for key, value in (('body_tilt_deg', '31'), ('body_tilt_deg', 'nan'),
                           ('side_lean_deg', '-1'), ('side_lean_deg', '21')):
            rejected = subprocess.run([str(SDK), str(ply), str(out), 'rejected_angle',
                                       '--set', f'gooseneck.{key}={value}'], capture_output=True)
            assert rejected.returncode != 0
            assert not (out/'rejected_angle_features.csv').exists()
        flipped = run(ply, out, 'flipped', 'gooseneck.tool_x_positive=false')
        assert len(original) == len(flipped) == 8
        assert [row['weld_enabled'] for row in flipped] == ['0', '1', '1', '1', '1', '1', '1', '0']
        for a, b in zip(original, flipped):
            assert all(float(a[key]) == float(b[key]) for key in 'xyz')
            ma, mb = axes(a), axes(b)
            assert np.allclose(ma[:, 2], mb[:, 2], atol=1e-8)
            assert np.allclose(ma[:, 0], -mb[:, 0], atol=1e-8)
            assert np.allclose(ma[:, 1], -mb[:, 1], atol=1e-8)
        repeated = run(ply, out, 'repeated', 'gooseneck.tool_x_positive=false')
        assert repeated == flipped, 'non-deterministic output'
        z_reversed = run(ply, out, 'z_reversed', 'gooseneck.tool_z_to_body=true',
                         'gooseneck.tool_x_positive=false')
        for a, b in zip(flipped, z_reversed):
            assert np.allclose(axes(a)[:, 2], -axes(b)[:, 2], atol=1e-8)
            assert np.allclose([float(a[f'world_torch_body_axis_{axis}']) for axis in 'xyz'],
                               [float(b[f'world_torch_body_axis_{axis}']) for axis in 'xyz'])
        offset = run(ply, out, 'offset', 'gooseneck.corner_2.dx=1.25',
                     'gooseneck.corner_2.roll=7', 'gooseneck.weld_start.dz=2',
                     'gooseneck.tool_x_positive=false')
        for i, (a, b) in enumerate(zip(flipped, offset)):
            delta = np.array([float(b[k])-float(a[k]) for k in 'xyz'])
            expected = [1.25, 0, 0] if i == 3 else ([0, 0, 2] if i in (0, 1) else [0, 0, 0])
            assert np.allclose(delta, expected, atol=1e-7), (i, delta)
            qa, qb = quaternion(a), quaternion(b)
            turn = math.degrees(2*math.acos(min(1., abs(qa @ qb))))
            assert abs(turn - (7 if i == 3 else 0)) < 0.001
        shorter = run(ply, out, 'shorter', 'gooseneck.vertical_length_mm=25')
        assert any(abs(float(shorter[1][k])-float(flipped[1][k])) > 1 for k in 'xyz')
        farther = run(ply, out, 'farther', 'gooseneck.clearance_mm=40')
        for i, neighbor in ((0, 1), (7, 6)):
            assert abs(np.linalg.norm([float(farther[i][k])-float(farther[neighbor][k])
                                       for k in 'xyz']) - 40) < 1e-7
        invalid = subprocess.run([str(SDK), str(ply), str(out), 'invalid', '--set',
                                  'gooseneck.groove_index=2'], capture_output=True)
        assert invalid.returncode != 0
        assert not (out / 'invalid_features.csv').exists()
        # Compare with an untouched build of the desktop portable C++ source.
        subprocess.run(['unzip', '-q', str(ROOT / 'gooseneck_weld/upstream/gooseneck_weld_portable.zip'),
                        '-d', str(out / 'upstream')], check=True)
        baseline = out / 'baseline_extractor'
        subprocess.run(['c++', '-std=c++17', '-O2',
                        str(out / 'upstream/gooseneck_weld/weld_extract.cpp'),
                        '-o', str(baseline)], check=True)
        if baseline.is_file():
            staged = out / 'baseline.ply'
            subprocess.run(['cp', str(ply), str(staged)], check=True)
            result = subprocess.run([str(baseline), '--trajectory', str(staged)],
                                    capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
            legacy = rows(out / 'baseline_U1_trajectory.csv')
            for a, b in zip(legacy, original):
                assert all(abs(float(a[k])-float(b[k])) < 1e-7 for k in 'xyz')
                assert abs(quaternion(a) @ quaternion(b)) > 1 - 1e-8
        for stem, count in (('live_20261104_160300_749', 1),
                            ('live_20261104_160017_391', 2),
                            ('live_20261104_155850_206', 2)):
            cloud = DATA / f'{stem}.ply'
            staged = out / f'{stem}.ply'
            subprocess.run(['cp', str(cloud), str(staged)], check=True)
            result = subprocess.run([str(baseline), '--trajectory', str(staged)],
                                    capture_output=True, text=True)
            assert result.returncode == 0, result.stderr
            for slot in range(1, count+1):
                expected = rows(out / f'{stem}_U{slot}_trajectory.csv')
                actual = run(cloud, out, f'{stem}_{slot}',
                             f'gooseneck.groove_index={slot}', 'gooseneck.tool_x_positive=true')
                assert len(actual) == len(expected) == 8
                assert all(row['groove'] == str(slot) for row in actual)
                for a, b in zip(expected, actual):
                    assert all(abs(float(a[k])-float(b[k])) < 1e-7 for k in 'xyz')
                    assert abs(quaternion(a) @ quaternion(b)) > 1 - 1e-8
                check_tilt_and_lean(cloud, out, actual, slot)
            print(f'PASS: {stem}, {count} groove(s), every slot matches portable SDK')
        for stem in ('live_20261104_161852_937', 'live_20261104_170234_609'):
            cloud = DATA / f'{stem}.ply'
            staged = out / f'{stem}.ply'
            subprocess.run(['cp', str(cloud), str(staged)], check=True)
            old = subprocess.run([str(baseline), '--trajectory', str(staged)], capture_output=True)
            new = subprocess.run([str(SDK), str(cloud), str(out), stem], capture_output=True)
            assert old.returncode != 0 and new.returncode != 0
            assert b'Expected two parallel upright faces and a base face' in old.stderr
            assert b'Expected two parallel upright faces and a base face' in new.stderr
            print(f'PASS: {stem}, both portable and SDK reject unsupported plane geometry')
        print('PASS: 8 poses; portable zero-adjustment parity; signed body tilt; mirrored side lean; '
              'fixed XYZ and nearest orthogonal tool X; physical lean independent of X/Z signs; '
              'right-handed quaternions; bounded angles; offsets; endpoint length; clearance; deterministic results')


if __name__ == '__main__':
    main()
