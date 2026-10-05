#!/usr/bin/env python3
"""Validate a collected dataset run: sample counts, real vs. declared Hz,
timestamp range/gaps/duplicates, NaN/empty data, frame/tick monotonicity,
ground truth presence -- see ../colecta_biguasim/docs/dataset_format.md for
the layout this assumes and ../colecta_biguasim/docs/synchronization.md for
the Hz/timestamp semantics it checks against.

Usage
-----
    python3 tools/validate_dataset.py dataset/colecta_simple/runs/<run_id>
    python3 tools/validate_dataset.py <run_dir> --gap-factor 3.0 --json report.json

Exit code is 0 if no ERROR-level finding was raised, 1 otherwise (WARNING
findings never affect the exit code). This script does not need BiguaSim
or ROS 2 -- it only reads files already on disk.
"""

from __future__ import annotations

import argparse
import csv
import glob
import json
import math
import re
import sys
from dataclasses import dataclass, field
from pathlib import Path
from typing import Optional

import numpy as np
import yaml

FILENAME_RE = re.compile(r"^(\d+)_([0-9.]+)\.")


@dataclass
class Finding:
    level: str  # "ERROR" | "WARNING" | "INFO"
    writer: str
    message: str


@dataclass
class WriterReport:
    name: str
    kind: str  # "tabular" | "files"
    num_samples: int = 0
    declared_hz: Optional[float] = None
    real_hz: Optional[float] = None
    t_start: Optional[float] = None
    t_end: Optional[float] = None
    num_duplicated_timestamps: int = 0
    num_nan_rows: int = 0
    num_empty_samples: int = 0
    max_gap_s: Optional[float] = None
    findings: list = field(default_factory=list)


def load_yaml(path: Path) -> dict:
    with open(path, "r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def find_tabular_files(run_dir: Path, output_rel: str) -> list[Path]:
    """A tabular writer's 'output' entry in dataset_manifest.yaml can be
    the folder itself (filename defaults to '<sensor>.csv' inside it, or
    an explicit 'filename:') -- just glob for the one CSV in that dir."""
    output_dir = run_dir / output_rel
    if output_dir.is_file():
        return [output_dir]
    if not output_dir.is_dir():
        return []
    return sorted(output_dir.glob("*.csv"))


def validate_tabular(name: str, csv_path: Path, declared_hz: Optional[float],
                      gap_factor: float) -> WriterReport:
    report = WriterReport(name=name, kind="tabular", declared_hz=declared_hz)

    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            report.findings.append(Finding("ERROR", name, f"{csv_path}: empty file, no header."))
            return report

        data_cols = header[3:]  # frame_id, tick, timestamp, *fields
        timestamps = []
        frame_ids = []
        prev_frame_id = None
        nan_rows = 0
        for row_num, row in enumerate(reader, start=2):
            if len(row) != len(header):
                report.findings.append(Finding(
                    "ERROR", name,
                    f"{csv_path}:{row_num}: {len(row)} columns, expected {len(header)}."))
                continue
            frame_id = int(row[0])
            ts = float(row[2])
            timestamps.append(ts)
            frame_ids.append(frame_id)
            if prev_frame_id is not None and frame_id <= prev_frame_id:
                report.findings.append(Finding(
                    "ERROR", name,
                    f"{csv_path}:{row_num}: frame_id {frame_id} is not strictly increasing "
                    f"(previous: {prev_frame_id})."))
            prev_frame_id = frame_id

            values = row[3:]
            has_nan = any(v.strip().lower() in ("nan", "-nan") for v in values)
            if not has_nan:
                try:
                    has_nan = any(math.isnan(float(v)) for v in values)
                except ValueError:
                    pass
            if has_nan:
                nan_rows += 1

        report.num_samples = len(timestamps)
        report.num_nan_rows = nan_rows
        if nan_rows:
            report.findings.append(Finding(
                "ERROR", name, f"{nan_rows} row(s) with NaN in {csv_path}."))

        if timestamps:
            report.t_start = timestamps[0]
            report.t_end = timestamps[-1]
            duration = timestamps[-1] - timestamps[0]
            report.real_hz = (len(timestamps) - 1) / duration if duration > 0 else float("inf")

            sorted_ts = sorted(timestamps)
            if sorted_ts != timestamps:
                report.findings.append(Finding(
                    "ERROR", name, "Timestamps are not strictly increasing (out of order)."))

            dup = len(timestamps) - len(set(timestamps))
            report.num_duplicated_timestamps = dup
            if dup:
                report.findings.append(Finding(
                    "ERROR", name, f"{dup} duplicated timestamp(s)."))

            if declared_hz and len(timestamps) > 1:
                expected_dt = 1.0 / declared_hz
                gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
                max_gap = max(gaps)
                report.max_gap_s = max_gap
                bad_gaps = [g for g in gaps if g > expected_dt * gap_factor]
                if bad_gaps:
                    report.findings.append(Finding(
                        "WARNING", name,
                        f"{len(bad_gaps)} gap(s) > {gap_factor}x the expected period "
                        f"({expected_dt:.4f}s); the largest is {max_gap:.4f}s."))
        else:
            report.findings.append(Finding("WARNING", name, "0 samples."))

    return report


def validate_files(name: str, output_dir: Path, declared_hz: Optional[float],
                    gap_factor: float) -> WriterReport:
    report = WriterReport(name=name, kind="files", declared_hz=declared_hz)
    if not output_dir.is_dir():
        report.findings.append(Finding("ERROR", name, f"Missing directory: {output_dir}"))
        return report

    files = sorted(f for f in output_dir.iterdir() if f.is_file())
    entries = []
    for f in files:
        m = FILENAME_RE.match(f.name)
        if not m:
            report.findings.append(Finding("WARNING", name, f"Unexpected file name: {f.name}"))
            continue
        frame_id, ts = int(m.group(1)), float(m.group(2))
        entries.append((frame_id, ts, f))

    entries.sort(key=lambda e: e[0])
    report.num_samples = len(entries)

    prev_frame_id = None
    timestamps = []
    empty_count = 0
    nan_count = 0
    for frame_id, ts, f in entries:
        if prev_frame_id is not None and frame_id <= prev_frame_id:
            report.findings.append(Finding(
                "ERROR", name, f"{f.name}: frame_id {frame_id} not increasing (previous {prev_frame_id})."))
        prev_frame_id = frame_id
        timestamps.append(ts)

        if f.stat().st_size == 0:
            report.findings.append(Finding("ERROR", name, f"{f.name}: 0-byte file."))
            continue

        if f.suffix == ".npy":
            try:
                arr = np.load(f, allow_pickle=False)
            except Exception as exc:  # noqa: BLE001
                report.findings.append(Finding("ERROR", name, f"{f.name}: could not be read ({exc})."))
                continue
            if arr.size == 0:
                empty_count += 1
            elif np.issubdtype(arr.dtype, np.floating) and np.isnan(arr).any():
                nan_count += 1

    report.num_empty_samples = empty_count
    report.num_nan_rows = nan_count
    if nan_count:
        report.findings.append(Finding("ERROR", name, f"{nan_count} .npy file(s) with NaN."))
    if empty_count:
        # Not necessarily an error: e.g. an ImagingSonar pointcloud with no
        # in-range return is legitimately a 0-length array (see
        # docs/sensors.md); still surfaced for the user to judge.
        report.findings.append(Finding(
            "INFO", name,
            f"{empty_count} empty .npy file(s) (size=0) -- normal for 'pointcloud' with no "
            f"in-range return, check it if it is another sensor."))

    if timestamps:
        report.t_start = timestamps[0]
        report.t_end = timestamps[-1]
        duration = timestamps[-1] - timestamps[0]
        report.real_hz = (len(timestamps) - 1) / duration if duration > 0 else float("inf")

        sorted_ts = sorted(timestamps)
        if sorted_ts != timestamps:
            report.findings.append(Finding("ERROR", name, "Timestamps not strictly increasing."))
        dup = len(timestamps) - len(set(timestamps))
        report.num_duplicated_timestamps = dup
        if dup:
            report.findings.append(Finding("ERROR", name, f"{dup} duplicated timestamp(s)."))

        if declared_hz and len(timestamps) > 1:
            expected_dt = 1.0 / declared_hz
            gaps = [b - a for a, b in zip(timestamps, timestamps[1:])]
            max_gap = max(gaps)
            report.max_gap_s = max_gap
            bad_gaps = [g for g in gaps if g > expected_dt * gap_factor]
            if bad_gaps:
                report.findings.append(Finding(
                    "WARNING", name,
                    f"{len(bad_gaps)} gap(s) > {gap_factor}x the expected period "
                    f"({expected_dt:.4f}s); the largest is {max_gap:.4f}s."))
    else:
        report.findings.append(Finding("WARNING", name, "0 samples."))

    return report


def validate_run(run_dir: Path, gap_factor: float) -> tuple[list[WriterReport], dict]:
    manifest_path = run_dir / "dataset_manifest.yaml"
    metadata_path = run_dir / "dataset_metadata.yaml"

    if not manifest_path.exists() or not metadata_path.exists():
        print(f"ERROR: {run_dir} does not look like a valid run "
              f"(dataset_manifest.yaml or dataset_metadata.yaml is missing).")
        sys.exit(2)

    manifest = load_yaml(manifest_path)
    metadata = load_yaml(metadata_path)
    sensor_hz = metadata.get("sensor_hz", {})

    reports = []
    for writer_name, output_rel in manifest.get("paths", {}).items():
        if writer_name in ("configs", "metadata"):
            continue
        output_dir = run_dir / output_rel

        tabular_files = find_tabular_files(run_dir, output_rel)
        if tabular_files:
            # Guess the declared Hz for this writer by matching its output
            # dir name against a sensor name in sensor_hz -- best-effort,
            # tabular writers name their sensor 1:1 with dataset.yaml's
            # 'sensor:' key which we don't have here, so fall back to None
            # (still reports real_hz/samples/gaps, just skips the
            # declared-vs-real Hz check).
            declared_hz = _guess_hz(writer_name, sensor_hz)
            for csv_path in tabular_files:
                reports.append(validate_tabular(writer_name, csv_path, declared_hz, gap_factor))
        else:
            declared_hz = _guess_hz(writer_name, sensor_hz)
            reports.append(validate_files(writer_name, output_dir, declared_hz, gap_factor))

    # Ground truth presence check (task requirement: explicit GT check).
    gt_reports = [r for r in reports if r.name in ("ground_truth", "dynamics", "pose")]
    if not gt_reports:
        reports.append(WriterReport(
            name="ground_truth", kind="tabular",
            findings=[Finding("ERROR", "ground_truth",
                               "No ground truth writer found "
                               "(expected 'ground_truth', 'dynamics' or 'pose' in dataset_manifest.yaml).")],
        ))
    elif all(r.num_samples == 0 for r in gt_reports):
        reports.append(WriterReport(
            name="ground_truth", kind="tabular",
            findings=[Finding("ERROR", "ground_truth", "The ground truth writer exists but has 0 samples.")],
        ))

    # Cross-sensor temporal coherence: every writer's [t_start, t_end]
    # should fall inside the run's declared [start_time_s, end_time_s].
    run_start = metadata.get("start_time_s")
    run_end = metadata.get("end_time_s")
    coherence_findings = []
    if run_start is not None and run_end is not None:
        for r in reports:
            if r.t_start is None:
                continue
            tol = 1e-3
            if r.t_start < run_start - tol or r.t_end > run_end + tol:
                coherence_findings.append(Finding(
                    "WARNING", r.name,
                    f"Time range [{r.t_start:.3f}, {r.t_end:.3f}] outside the run "
                    f"range [{run_start:.3f}, {run_end:.3f}]."))
    reports.append(WriterReport(name="_coherence", kind="tabular", findings=coherence_findings))

    return reports, metadata


def _guess_hz(writer_name: str, sensor_hz: dict) -> Optional[float]:
    # Writer names in dataset.yaml are lower_snake_case labels chosen by
    # the user (e.g. 'ground_truth' -> DynamicsSensor, 'sonar_raw' ->
    # ImagingSonar); sensor_hz is keyed by the real BiguaSim sensor name.
    # Best-effort case-insensitive substring match; falls back to None
    # (still validates everything except the declared-vs-real Hz check).
    lname = writer_name.lower()
    for sensor_name, hz in sensor_hz.items():
        if sensor_name.lower() in lname or lname in sensor_name.lower():
            return hz
    aliases = {"ground_truth": "DynamicsSensor", "dynamics": "DynamicsSensor", "pose": "DynamicsSensor"}
    if writer_name in aliases:
        return sensor_hz.get(aliases[writer_name])
    return None


def print_report(run_dir: Path, reports: list[WriterReport], metadata: dict) -> int:
    print(f"=== Dataset validation: {run_dir} ===")
    print(f"Trajectory: {metadata.get('trajectory_type')}  "
          f"Duration (sim): {metadata.get('duration_s'):.2f}s  "
          f"Frames: {metadata.get('num_frames')}")
    print()

    num_errors = 0
    num_warnings = 0
    for r in reports:
        if r.name == "_coherence":
            continue
        print(f"--- {r.name} ({r.kind}) ---")
        print(f"  samples: {r.num_samples}"
              + (f"  declared Hz: {r.declared_hz}" if r.declared_hz else "")
              + (f"  actual Hz: {r.real_hz:.2f}" if r.real_hz is not None else ""))
        if r.t_start is not None:
            print(f"  t_start={r.t_start:.4f}s  t_end={r.t_end:.4f}s"
                  + (f"  max_gap={r.max_gap_s:.4f}s" if r.max_gap_s is not None else ""))
        if r.num_duplicated_timestamps:
            print(f"  duplicated timestamps: {r.num_duplicated_timestamps}")
        if r.num_nan_rows:
            print(f"  rows/files with NaN: {r.num_nan_rows}")
        if r.num_empty_samples:
            print(f"  empty samples: {r.num_empty_samples}")
        for finding in r.findings:
            print(f"  [{finding.level}] {finding.message}")
            if finding.level == "ERROR":
                num_errors += 1
            elif finding.level == "WARNING":
                num_warnings += 1
        print()

    coherence = next((r for r in reports if r.name == "_coherence"), None)
    if coherence and coherence.findings:
        print("--- temporal coherence across sensors ---")
        for finding in coherence.findings:
            print(f"  [{finding.level}] {finding.message}")
            if finding.level == "WARNING":
                num_warnings += 1
        print()

    print(f"=== Summary: {num_errors} error(s), {num_warnings} warning(s) ===")
    return 1 if num_errors else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("run_dir", help="Run directory, e.g. dataset/<name>/runs/<run_id>")
    parser.add_argument("--gap-factor", type=float, default=3.0,
                         help="A gap > gap_factor x the expected period (1/Hz) is reported as WARNING.")
    parser.add_argument("--json", default=None, help="Also write the report as JSON to this path.")
    args = parser.parse_args()

    run_dir = Path(args.run_dir).resolve()
    reports, metadata = validate_run(run_dir, args.gap_factor)
    exit_code = print_report(run_dir, reports, metadata)

    if args.json:
        payload = {
            "run_dir": str(run_dir),
            "metadata": metadata,
            "writers": [
                {
                    "name": r.name, "kind": r.kind, "num_samples": r.num_samples,
                    "declared_hz": r.declared_hz, "real_hz": r.real_hz,
                    "t_start": r.t_start, "t_end": r.t_end,
                    "num_duplicated_timestamps": r.num_duplicated_timestamps,
                    "num_nan_rows": r.num_nan_rows, "num_empty_samples": r.num_empty_samples,
                    "max_gap_s": r.max_gap_s,
                    "findings": [{"level": f.level, "message": f.message} for f in r.findings],
                }
                for r in reports
            ],
        }
        Path(args.json).write_text(json.dumps(payload, indent=2))

    sys.exit(exit_code)


if __name__ == "__main__":
    main()
