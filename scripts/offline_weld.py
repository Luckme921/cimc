#!/usr/bin/env python3
"""Run either installed SDK with a ROS YAML, without starting ROS or hardware."""
import argparse
from pathlib import Path
import subprocess
import sys
import yaml

WORKSPACE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(WORKSPACE / 'src/cimc'))
from cimc.weld_algorithm import preview_command  # noqa: E402


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('input_ply', type=Path)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('prefix')
    parser.add_argument('--yaml', type=Path, default=WORKSPACE / 'src/weld_seam_perception/config/weld_seam.yaml')
    parser.add_argument('--set', action='append', default=[])
    args = parser.parse_args()
    try:
        params = yaml.safe_load(args.yaml.read_text(encoding='utf-8'))['weld_seam_node']['ros__parameters']
        overrides = {}
        for item in params.get('algorithm_overrides', []):
            key, value = item.split('=', 1)
            if key in overrides:
                raise ValueError(f'YAML重复参数：{key}')
            overrides[key] = value
        for item in args.set:
            key, value = item.split('=', 1)
            overrides[key] = value
        return subprocess.run(preview_command(WORKSPACE, args.input_ply, args.output_directory,
                              args.prefix, overrides, params.get('config_file', '')), check=False).returncode
    except (OSError, ValueError) as error:
        print(error, file=sys.stderr)
        return 1


if __name__ == '__main__':
    sys.exit(main())
