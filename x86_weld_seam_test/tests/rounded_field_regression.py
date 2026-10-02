#!/usr/bin/env python3
"""Offline safety regression for the rounded path using the active ROS YAML."""

import argparse
import csv
from pathlib import Path
import subprocess
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CASES = (
    ("live_20260924_095031_427", True),
    ("live_20260924_091242_789", False),
    ("live_20260924_085227_114", False),
    ("20260616测试视野1", True),
    ("20260616测试视野-2", True),
)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--data-root", type=Path,
        default=Path("/home/mini/scut_weld_data/pointclouds"))
    parser.add_argument(
        "--extractor", type=Path,
        default=WORKSPACE / "x86_weld_seam_test/build/weld_seam_extractor")
    parser.add_argument(
        "--yaml", type=Path,
        default=WORKSPACE / "src/weld_seam_perception/config/weld_seam.yaml")
    args = parser.parse_args()

    with args.yaml.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["weld_seam_node"]["ros__parameters"]
    overrides = params["algorithm_overrides"]
    if "path.mode=rounded_features" not in overrides:
        parser.error("active ROS YAML must select path.mode=rounded_features")

    failures = []
    with tempfile.TemporaryDirectory(prefix="rounded_field_regression_") as directory:
        output_dir = Path(directory)
        for name, must_succeed in CASES:
            input_ply = args.data_root / (name + ".ply")
            if not input_ply.is_file():
                failures.append(f"{name}: missing input PLY")
                continue
            command = [str(args.extractor), str(input_ply), str(output_dir), name]
            for override in overrides:
                command.extend(("--set", override))
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            csv_path = output_dir / (name + "_features.csv")
            if result.returncode == 0:
                if not csv_path.is_file():
                    failures.append(f"{name}: succeeded without CSV")
                    continue
                with csv_path.open(newline="", encoding="utf-8") as stream:
                    rows = list(csv.DictReader(stream))
                # 6 physical corners = 2 safe + 2 endpoint singles +
                # 4*5 internal samples + 5 straight midpoints = 29.
                if len(rows) < 29 or len(rows) > 100:
                    failures.append(f"{name}: unsafe partial/oversized path ({len(rows)} points)")
                for row in rows:
                    if row["weld_enabled"] == "1" and \
                            row["point_source"] != "measured_red_seam_point":
                        failures.append(f"{name}: non-measured weld point at order {row['order']}")
                        break
                maximum_turn = max(
                    float(row["orientation_delta_from_previous_deg"])
                    for row in rows)
                weld_rows = [row for row in rows if row["weld_enabled"] == "1"]
                min_raw_z = min(float(row["raw_workpiece_z"]) for row in weld_rows)
                min_target_z = min(float(row["workpiece_z"]) for row in weld_rows)
                below_bottom = sum(
                    float(row["workpiece_z"]) < -0.5 for row in weld_rows)
                print(f"{name}: OK, {len(rows)} points, max adjacent tool turn "
                      f"{maximum_turn:.2f} deg, raw/target minimum workpiece Z "
                      f"{min_raw_z:.2f}/{min_target_z:.2f} mm, "
                      f"target points >0.5 mm below fitted bottom: {below_bottom}")
            else:
                if must_succeed:
                    failures.append(f"{name}: unexpected failure: {result.stderr.strip()[-400:]}")
                if csv_path.exists():
                    failures.append(f"{name}: failed but left a trajectory CSV")
                print(f"{name}: rejected incomplete/uncertain trajectory")
    if failures:
        raise SystemExit("\n".join(failures))
    print("Offline rounded field regression passed; no hardware was commanded.")


if __name__ == "__main__":
    main()
