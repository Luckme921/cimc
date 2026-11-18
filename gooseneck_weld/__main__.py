"""Standalone offline invocation using the CIMC CLI argument convention."""

import argparse
from pathlib import Path
import sys

from . import run_preview


def main():
    parser = argparse.ArgumentParser(description='Generate CIMC gooseneck welding poses from PLY')
    parser.add_argument('input_ply', type=Path)
    parser.add_argument('output_directory', type=Path)
    parser.add_argument('output_prefix')
    parser.add_argument('--set', action='append', default=[], metavar='KEY=VALUE')
    parser.add_argument('--workspace', type=Path, default=Path(__file__).resolve().parent.parent)
    args = parser.parse_args()
    overrides = {}
    for item in args.set:
        key, equals, value = item.partition('=')
        if not equals or not key or not value or key in overrides:
            parser.error(f'Invalid or duplicate --set argument: {item}')
        overrides[key] = value
    try:
        result, rows = run_preview(args.workspace, args.output_directory, args.output_prefix,
                                   args.input_ply, overrides,
                                   args.output_directory / f'{args.output_prefix}_extract.log')
    except (OSError, ValueError, RuntimeError) as exc:
        print(exc, file=sys.stderr)
        return 1
    print(f'Poses: {len(rows)} ({len(rows)//8} grooves)')
    print(f'PLY: {result}')
    print(f'CSV: {result.with_name(args.output_prefix + "_features.csv")}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
