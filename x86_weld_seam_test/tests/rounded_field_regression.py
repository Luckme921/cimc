#!/usr/bin/env python3
"""Offline safety regression for the rounded path using the active ROS YAML."""

import argparse
from contextlib import nullcontext
import csv
from pathlib import Path
import subprocess
import tempfile

import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CASES = (
    ("capture_1790216105559", False),
    ("live_20260924_095031_427", True),
    ("live_20260924_091242_789", True),
    ("live_20260924_085227_114", False),
    ("20260616测试视野1", False),
    ("20260616测试视野-2", False),
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
    parser.add_argument(
        "--output-dir", type=Path,
        help="Keep offline output files here; otherwise use a temporary directory")
    args = parser.parse_args()

    with args.yaml.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["weld_seam_node"]["ros__parameters"]
    overrides = params["algorithm_overrides"]
    if "path.mode=rounded_features" not in overrides:
        parser.error("active ROS YAML must select path.mode=rounded_features")

    failures = []
    output_context = (
        nullcontext(str(args.output_dir)) if args.output_dir else
        tempfile.TemporaryDirectory(prefix="rounded_field_regression_"))
    with output_context as directory:
        output_dir = Path(directory)
        output_dir.mkdir(parents=True, exist_ok=True)
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
                if not must_succeed:
                    failures.append(f"{name}: unexpectedly published a previously rejected path")
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
                            row["point_source"] not in (
                                "measured_red_seam_point",
                                "modeled_small_hole_from_adjacent_lines"):
                        failures.append(f"{name}: unsupported point source at order {row['order']}")
                        break
                    if row["weld_enabled"] == "1" and \
                            row["point_source"] == "measured_red_seam_point" and \
                            float(row["distance_to_ideal"]) > 2.5001:
                        failures.append(f"{name}: remote measured snap at order {row['order']}")
                for previous, current in zip(rows[1:-2], rows[2:-1]):
                    if float(current["raw_workpiece_x"]) - \
                            float(previous["raw_workpiece_x"]) <= 0.25:
                        failures.append(f"{name}: non-monotonic raw target order")
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
            visual_prefix = name + "_visual_only"
            visual_command = [
                str(args.extractor), str(input_ply), str(output_dir), visual_prefix]
            for override in overrides:
                visual_command.extend(("--set", override))
            visual_command.extend(("--set", "path.rounded_visualization_only=true"))
            visual = subprocess.run(
                visual_command, capture_output=True, text=True, check=False)
            if visual.returncode == 0 or \
                    "INSPECTION ONLY" not in visual.stderr or \
                    not (output_dir / (visual_prefix + "_result.ply")).is_file() or \
                    (output_dir / (visual_prefix + "_features.csv")).exists() or \
                    (output_dir / (visual_prefix + "_feature_points.ply")).exists():
                failures.append(f"{name}: visualization-only isolation failed")
    if failures:
        raise SystemExit("\n".join(failures))
    print("Offline rounded field regression passed; no hardware was commanded.")


if __name__ == "__main__":
    main()
