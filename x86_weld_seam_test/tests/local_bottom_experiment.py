#!/usr/bin/env python3
"""Compare the stable global-bottom gate with the opt-in local-bottom gate.

Only the standalone, offline extractor is invoked. The active ROS YAML is read
but not changed; ROI is disabled because these historical PLYs have different
camera positions/crops. Neither ROS nor any industrial device is contacted.
"""

import argparse
import csv
import math
from pathlib import Path
import re
import subprocess

import numpy as np
import yaml


WORKSPACE = Path(__file__).resolve().parents[2]
DEFAULT_CASES = (
    "capture_1790216105559",
    "live_20260924_095031_427",
    "live_20260924_094336_444",
    "live_20260924_092948_107",
    "live_20260924_091242_789",
    "live_20260924_085227_114",
    "live_20260923_204704_730",
    "live_20260923_165120_398",
    "20260616测试视野1",
    "20260616测试视野-2",
)
VERTEX_FIELDS = (
    ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
    ("red", "u1"), ("green", "u1"), ("blue", "u1"),
)


def red_points(path):
    with path.open("rb") as stream:
        header = []
        while True:
            line = stream.readline().decode("ascii").strip()
            header.append(line)
            if line == "end_header":
                break
        if "format binary_little_endian 1.0" not in header:
            raise ValueError(f"unexpected PLY format: {path}")
        vertex_count = int(next(
            line.split()[2] for line in header if line.startswith("element vertex ")))
        vertex_line = next(i for i, line in enumerate(header)
                           if line.startswith("element vertex "))
        properties = []
        for line in header[vertex_line + 1:]:
            if line.startswith("element ") or line == "end_header":
                break
            if line.startswith("property "):
                properties.append(tuple(line.split()[1:]))
        expected = [("float", key) for key in ("x", "y", "z")]
        expected += [("uchar", key) for key in ("red", "green", "blue")]
        if properties != expected:
            raise ValueError(f"unexpected PLY vertex fields: {properties}")
        points = np.fromfile(stream, dtype=np.dtype(list(VERTEX_FIELDS)), count=vertex_count)
        if len(points) != vertex_count:
            raise ValueError(f"incomplete PLY vertices: {path}")
    red = (points["red"] == 255) & (points["green"] == 0) & (points["blue"] == 0)
    return np.column_stack([points[key][red] for key in ("x", "y", "z")])


def run_case(extractor, ply, directory, prefix, overrides, local):
    command = [str(extractor), str(ply), str(directory), prefix]
    for setting in overrides:
        command.extend(("--set", setting))
    command.extend(("--set", f"primary.local_bottom.enable={'true' if local else 'false'}"))
    result = subprocess.run(command, text=True, capture_output=True, check=False)
    log_path = directory / f"{prefix}.log"
    log_path.write_text(result.stdout + result.stderr, encoding="utf-8")
    info = {"exit_code": result.returncode, "log": log_path.name}
    match = re.search(r"Experimental local bottom: ([^\n]+)", result.stdout)
    info["bottom_profile"] = match.group(1) if match else "global plane"
    if result.returncode:
        info["error"] = (result.stderr.strip() or result.stdout.strip())[-350:]
        return info, None
    csv_path = directory / f"{prefix}_features.csv"
    with csv_path.open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    info["path_points"] = len(rows)
    info["red_points"] = len(red_points(directory / f"{prefix}_result.ply"))
    info["result_ply"] = f"{prefix}_result.ply"
    info["features_csv"] = csv_path.name
    return info, rows


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--extractor", type=Path,
                        default=WORKSPACE / "x86_weld_seam_test/build/weld_seam_extractor")
    parser.add_argument("--data-root", type=Path,
                        default=Path("/home/mini/scut_weld_data/pointclouds"))
    parser.add_argument("--yaml", type=Path,
                        default=WORKSPACE / "src/weld_seam_perception/config/weld_seam.yaml")
    parser.add_argument("--output-dir", type=Path, default=WORKSPACE /
                        "x86_weld_seam_test/output/local_bottom_experiment_20261012")
    parser.add_argument("--case", action="append", help="PLY stem; repeat to select cases")
    args = parser.parse_args()
    if not args.extractor.is_file():
        parser.error(f"missing extractor: {args.extractor}")
    with args.yaml.open(encoding="utf-8") as stream:
        params = yaml.safe_load(stream)["weld_seam_node"]["ros__parameters"]
    overrides = [item for item in params["algorithm_overrides"]
                 if not item.startswith("roi.") and
                 not item.startswith("primary.local_bottom.")]
    overrides.append("roi.enable=false")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    report = []
    for stem in args.case or DEFAULT_CASES:
        ply = args.data_root / (stem + ".ply")
        if not ply.is_file():
            report.append({"case": stem, "error": "missing PLY"})
            continue
        baseline, old_rows = run_case(
            args.extractor, ply, args.output_dir, stem + "_global", overrides, False)
        candidate, new_rows = run_case(
            args.extractor, ply, args.output_dir, stem + "_local", overrides, True)
        record = {"case": stem, "global": baseline, "local": candidate}
        if old_rows is not None and new_rows is not None:
            old_red = red_points(args.output_dir / baseline["result_ply"])
            new_red = red_points(args.output_dir / candidate["result_ply"])
            old_set = set(map(tuple, old_red))
            new_set = set(map(tuple, new_red))
            record["red_added"] = len(new_set - old_set)
            record["red_removed"] = len(old_set - new_set)
            record["path_count_changed"] = len(old_rows) != len(new_rows)
            if len(old_rows) == len(new_rows) and all(
                    old["feature_type"] == new["feature_type"]
                    for old, new in zip(old_rows, new_rows)):
                position_shifts = []
                orientation_shifts = []
                for old, new in zip(old_rows, new_rows):
                    old_xyz = np.array([float(old[key]) for key in
                                        ("raw_x", "raw_y", "raw_z")])
                    new_xyz = np.array([float(new[key]) for key in
                                        ("raw_x", "raw_y", "raw_z")])
                    position_shifts.append(float(np.linalg.norm(old_xyz - new_xyz)))
                    dot = abs(sum(float(old[key]) * float(new[key]) for key in
                                  ("qw", "qx", "qy", "qz")))
                    orientation_shifts.append(2.0 * math.degrees(
                        math.acos(min(1.0, dot))))
                record["max_raw_target_shift_mm"] = max(position_shifts)
                record["max_orientation_shift_deg"] = max(orientation_shifts)
        report.append(record)
        print(stem, "global", baseline.get("red_points", "FAIL"),
              "local", candidate.get("red_points", "FAIL"),
              "added", record.get("red_added", "-"),
              "removed", record.get("red_removed", "-"),
              "path", baseline.get("path_points", "-"),
              "->", candidate.get("path_points", "-"),
              "max_raw_shift_mm",
              round(record.get("max_raw_target_shift_mm", float("nan")), 3),
              candidate["bottom_profile"])
    report_path = args.output_dir / "comparison.csv"
    with report_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(("case", "global_exit", "local_exit", "global_red",
                         "local_red", "red_added", "red_removed", "global_path",
                         "local_path", "max_raw_target_shift_mm",
                         "max_orientation_shift_deg", "bottom_profile"))
        for record in report:
            old = record.get("global", {})
            new = record.get("local", {})
            writer.writerow((record["case"], old.get("exit_code", ""),
                             new.get("exit_code", ""), old.get("red_points", ""),
                             new.get("red_points", ""), record.get("red_added", ""),
                             record.get("red_removed", ""),
                             old.get("path_points", ""), new.get("path_points", ""),
                             record.get("max_raw_target_shift_mm", ""),
                             record.get("max_orientation_shift_deg", ""),
                             new.get("bottom_profile", "")))
    print("comparison:", report_path)
    if any(record.get("global", {}).get("exit_code", 1) or
           record.get("local", {}).get("exit_code", 1) for record in report):
        raise SystemExit(1)


if __name__ == "__main__":
    main()
