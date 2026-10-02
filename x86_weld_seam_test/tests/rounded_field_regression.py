#!/usr/bin/env python3
"""Offline safety regression for the rounded path using the active ROS YAML."""

import argparse
from contextlib import nullcontext
import csv
from pathlib import Path
import subprocess
import tempfile

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
CASES = (
    ("capture_1790216105559", True),
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
                if len(rows) != 29 or len(rows) > 100:
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
                rounded_groups = []
                current_group = []
                for row in rows:
                    if row["feature_type"].endswith("_rounded_corner_sample"):
                        current_group.append(row)
                    elif current_group:
                        rounded_groups.append(current_group)
                        current_group = []
                if current_group:
                    rounded_groups.append(current_group)
                if sum(len(group) == 5 for group in rounded_groups) != 4:
                    failures.append(f"{name}: an internal five-point corner is missing")
                for group in rounded_groups:
                    if len(group) != 5:
                        if group[0]["point_source"] != "measured_red_seam_point":
                            failures.append(f"{name}: endpoint corner is not measured")
                        continue
                    if sum(row["point_source"] ==
                           "modeled_small_hole_from_adjacent_lines"
                           for row in group) > 3:
                        failures.append(f"{name}: corner relies on too many modeled points")
                    direction = 1 if float(group[-1]["raw_workpiece_y"]) >= \
                        float(group[0]["raw_workpiece_y"]) else -1
                    for previous, current in zip(group, group[1:]):
                        if direction * (float(current["raw_workpiece_y"]) -
                                        float(previous["raw_workpiece_y"])) < -0.1:
                            failures.append(f"{name}: a rounded corner reverses Y")
                            break
                    # The existing tool-frame convention should keep the five
                    # torch-axis lines roughly concurrent at a rounded corner.
                    matrix = np.zeros((3, 3))
                    vector = np.zeros(3)
                    axes = []
                    positions = []
                    for row in group:
                        axis = np.array([float(row[key]) for key in (
                            "world_torch_body_axis_x", "world_torch_body_axis_y",
                            "world_torch_body_axis_z")])
                        axis /= np.linalg.norm(axis)
                        position = np.array([float(row[key]) for key in (
                            "raw_x", "raw_y", "raw_z")])
                        projector = np.eye(3) - np.outer(axis, axis)
                        matrix += projector
                        vector += projector @ position
                        axes.append(axis)
                        positions.append(position)
                    center = np.linalg.solve(matrix, vector)
                    maximum_axis_miss = max(np.linalg.norm(
                        np.cross(center - position, axis))
                        for position, axis in zip(positions, axes))
                    if maximum_axis_miss > 2.5:
                        failures.append(f"{name}: corner tool axes miss their "
                                        f"common center by {maximum_axis_miss:.2f} mm")
                maximum_turn = max(
                    float(row["orientation_delta_from_previous_deg"])
                    for row in rows)
                if maximum_turn > 30.0:
                    failures.append(f"{name}: adjacent tool turn {maximum_turn:.2f} deg")
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

        # The two files were captured/cropped under different ROI choices.
        # Do not relax the production boundary guard just to pass one global ROI.
        for name in ("live_20260924_091242_789", "live_20260924_095031_427"):
            input_ply = args.data_root / (name + ".ply")
            prefix = name + "_full_x"
            command = [str(args.extractor), str(input_ply), str(output_dir), prefix]
            for override in overrides + ["roi.min_x=-inf", "roi.max_x=inf"]:
                command.extend(("--set", override))
            result = subprocess.run(command, capture_output=True, text=True, check=False)
            csv_path = output_dir / (prefix + "_features.csv")
            if result.returncode != 0 or not csv_path.is_file():
                failures.append(f"{name}: full-X ROI should yield a complete path")
                continue
            with csv_path.open(newline="", encoding="utf-8") as stream:
                if len(list(csv.DictReader(stream))) != 29:
                    failures.append(f"{name}: full-X ROI path has wrong point count")
    if failures:
        raise SystemExit("\n".join(failures))
    print("Offline rounded field regression passed; no hardware was commanded.")


if __name__ == "__main__":
    main()
