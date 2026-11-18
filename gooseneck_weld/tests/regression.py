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


def main():
    with tempfile.TemporaryDirectory(prefix='gooseneck_regression_') as folder:
        out = Path(folder)
        ply = DATA / 'live_20261104_160336_814.ply'
        original = run(ply, out, 'original', 'gooseneck.tool_x_positive=true')
        flipped = run(ply, out, 'flipped')
        assert len(original) == len(flipped) == 8
        assert [row['weld_enabled'] for row in flipped] == ['0', '1', '1', '1', '1', '1', '1', '0']
        for a, b in zip(original, flipped):
            assert all(float(a[key]) == float(b[key]) for key in 'xyz')
            ma, mb = axes(a), axes(b)
            assert np.allclose(ma[:, 2], mb[:, 2], atol=1e-8)
            assert np.allclose(ma[:, 0], -mb[:, 0], atol=1e-8)
            assert np.allclose(ma[:, 1], -mb[:, 1], atol=1e-8)
        repeated = run(ply, out, 'repeated')
        assert repeated == flipped, 'non-deterministic output'
        z_reversed = run(ply, out, 'z_reversed', 'gooseneck.tool_z_to_body=true')
        for a, b in zip(flipped, z_reversed):
            assert np.allclose(axes(a)[:, 2], -axes(b)[:, 2], atol=1e-8)
            assert np.allclose([float(a[f'world_torch_body_axis_{axis}']) for axis in 'xyz'],
                               [float(b[f'world_torch_body_axis_{axis}']) for axis in 'xyz'])
        offset = run(ply, out, 'offset', 'gooseneck.corner_2.dx=1.25',
                     'gooseneck.corner_2.roll=7', 'gooseneck.weld_start.dz=2')
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
        print('PASS: 8 poses; portable geometry/pose parity; proper X/Y flip with Z fixed; '
              'position/rotation offsets; endpoint length; clearance; deterministic results; invalid slot rejected')


if __name__ == '__main__':
    main()
