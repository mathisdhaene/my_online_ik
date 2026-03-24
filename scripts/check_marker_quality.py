#!/usr/bin/env python3
import argparse
import csv
import math
import os
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Sequence, Tuple
import xml.etree.ElementTree as ET


@dataclass
class MarkerStats:
    marker: str
    total_frames: int = 0
    valid_frames: int = 0
    nan_frames: int = 0
    zero_frames: int = 0
    max_gap_frames: int = 0
    jump_spikes: int = 0
    vel_spikes: int = 0
    max_jump_m: float = 0.0
    max_vel_mps: float = 0.0


def _split_ws(line: str) -> List[str]:
    return line.strip().split()


def parse_units(lines: Sequence[str]) -> str:
    for i in range(min(16, len(lines) - 1)):
        keys = _split_ws(lines[i])
        vals = _split_ws(lines[i + 1])
        if not keys or not vals:
            continue
        for c, k in enumerate(keys):
            if k.lower() == "units" and c < len(vals):
                return vals[c]
    return "unknown"


def unit_scale_to_m(units: str) -> float:
    u = units.lower()
    if u in {"mm", "millimeter", "millimeters"}:
        return 0.001
    if u in {"m", "meter", "meters"}:
        return 1.0
    return 1.0


def find_header_idx(lines: Sequence[str]) -> int:
    for i, line in enumerate(lines):
        toks = _split_ws(line)
        if len(toks) >= 2 and toks[0].lower().startswith("frame") and toks[1].lower() == "time":
            return i
    raise ValueError("Could not find TRC header row with 'Frame# Time'.")


def parse_marker_names(lines: Sequence[str], header_idx: int) -> List[str]:
    row1 = _split_ws(lines[header_idx])
    row2 = _split_ws(lines[header_idx + 1]) if header_idx + 1 < len(lines) else []

    # Typical TRC: row2 starts with X1 Y1 Z1 ... for each marker triplet.
    triplets = 0
    if len(row2) >= 5:
        triplets = (len(row2) - 2) // 3

    names = row1[2:]
    if triplets > 0 and len(names) >= triplets:
        return names[:triplets]

    # Fallback from numeric suffixes X1 Y1 Z1 X2 ...
    if triplets > 0:
        return [f"marker_{i+1}" for i in range(triplets)]

    # Final fallback based on row1 length.
    if len(names) % 3 == 0 and len(names) > 0:
        return [names[i] for i in range(0, len(names), 3)]

    raise ValueError("Could not infer marker names from TRC header.")


def parse_data_rows(lines: Sequence[str], start_idx: int, n_markers: int) -> Tuple[List[float], Dict[str, List[Tuple[float, float, float]]]]:
    times: List[float] = []
    data: Dict[str, List[Tuple[float, float, float]]] = {}

    for line in lines[start_idx:]:
        toks = _split_ws(line)
        if len(toks) < 2 + 3 * n_markers:
            continue
        try:
            t = float(toks[1])
        except ValueError:
            continue
        times.append(t)

        coords = toks[2:2 + 3 * n_markers]
        for m in range(n_markers):
            x_s, y_s, z_s = coords[3 * m: 3 * m + 3]
            def to_f(v: str) -> float:
                try:
                    return float(v)
                except ValueError:
                    return math.nan
            p = (to_f(x_s), to_f(y_s), to_f(z_s))
            key = str(m)
            data.setdefault(key, []).append(p)

    return times, data


def _is_finite_xyz(p: Tuple[float, float, float]) -> bool:
    return math.isfinite(p[0]) and math.isfinite(p[1]) and math.isfinite(p[2])


def _norm(p: Tuple[float, float, float]) -> float:
    return math.sqrt(p[0] * p[0] + p[1] * p[1] + p[2] * p[2])


def _dist(a: Tuple[float, float, float], b: Tuple[float, float, float]) -> float:
    return _norm((a[0] - b[0], a[1] - b[1], a[2] - b[2]))


def analyze_marker(
    name: str,
    series: Sequence[Tuple[float, float, float]],
    times: Sequence[float],
    scale_to_m: float,
    zero_eps_m: float,
    jump_thresh_m: float,
    vel_thresh_mps: float,
) -> Tuple[MarkerStats, List[Tuple[str, int, float, float]]]:
    s = MarkerStats(marker=name, total_frames=len(series))
    events: List[Tuple[str, int, float, float]] = []  # (kind, frame_idx, value, t)

    prev_valid_idx: Optional[int] = None
    prev_valid_p: Optional[Tuple[float, float, float]] = None
    current_gap = 0

    for i, raw_p in enumerate(series):
        p = (raw_p[0] * scale_to_m, raw_p[1] * scale_to_m, raw_p[2] * scale_to_m)
        finite = _is_finite_xyz(p)
        zeroish = finite and (_norm(p) <= zero_eps_m)
        valid = finite and not zeroish

        if not finite:
            s.nan_frames += 1
        if zeroish:
            s.zero_frames += 1

        if not valid:
            current_gap += 1
            if current_gap > s.max_gap_frames:
                s.max_gap_frames = current_gap
            continue

        s.valid_frames += 1
        current_gap = 0

        if prev_valid_p is not None and prev_valid_idx is not None:
            dt = times[i] - times[prev_valid_idx]
            if dt <= 0:
                dt = 1.0 / 30.0
            jump = _dist(p, prev_valid_p)
            vel = jump / dt
            if jump > s.max_jump_m:
                s.max_jump_m = jump
            if vel > s.max_vel_mps:
                s.max_vel_mps = vel
            if jump > jump_thresh_m:
                s.jump_spikes += 1
                events.append(("jump", i, jump, times[i]))
            if vel > vel_thresh_mps:
                s.vel_spikes += 1
                events.append(("velocity", i, vel, times[i]))

        prev_valid_idx = i
        prev_valid_p = p

    return s, events


def load_model_marker_names(model_path: Path) -> List[str]:
    root = ET.parse(model_path).getroot()
    names = []
    for elem in root.findall(".//MarkerSet/objects/Marker"):
        n = elem.get("name")
        if n:
            names.append(n)
    return names


def collect_trc_files(trc: Optional[Path], input_root: Optional[Path], trc_glob: str) -> List[Path]:
    if trc is not None:
        return [trc]
    assert input_root is not None
    return sorted([p for p in input_root.rglob(trc_glob) if p.is_file()])


def main() -> int:
    ap = argparse.ArgumentParser(description="Audit TRC marker quality and locate problematic markers/frames.")
    src = ap.add_mutually_exclusive_group(required=True)
    src.add_argument("--trc", type=Path, help="Single TRC file to analyze")
    src.add_argument("--input-root", type=Path, help="Root folder to scan recursively for TRC files")
    ap.add_argument("--trc-glob", default="sample_*.trc", help="Glob used with --input-root")
    ap.add_argument("--model", type=Path, help="Optional .osim model to compare marker sets")
    ap.add_argument("--out-dir", type=Path, default=Path("marker_qc_results"), help="Output directory")
    ap.add_argument("--jump-thresh-m", type=float, default=0.18, help="Jump threshold in meters")
    ap.add_argument("--vel-thresh-mps", type=float, default=6.0, help="Velocity threshold in m/s")
    ap.add_argument("--zero-eps-m", type=float, default=1e-8, help="Norm <= eps considered zero marker")
    args = ap.parse_args()

    trc_files = collect_trc_files(args.trc, args.input_root, args.trc_glob)
    if not trc_files:
        print("[ERROR] No TRC files found.")
        return 1

    args.out_dir.mkdir(parents=True, exist_ok=True)
    marker_csv = args.out_dir / "marker_quality.csv"
    event_csv = args.out_dir / "marker_events.csv"
    file_csv = args.out_dir / "file_summary.csv"

    model_markers: Optional[List[str]] = None
    model_set = set()
    if args.model:
        model_markers = load_model_marker_names(args.model)
        model_set = set(model_markers)

    marker_rows = []
    event_rows = []
    file_rows = []

    for trc_path in trc_files:
        text = trc_path.read_text(errors="replace")
        lines = text.splitlines()

        units = parse_units(lines)
        scale_to_m = unit_scale_to_m(units)

        hidx = find_header_idx(lines)
        marker_names = parse_marker_names(lines, hidx)
        times, indexed_data = parse_data_rows(lines, hidx + 2, len(marker_names))

        if not times:
            print(f"[WARN] {trc_path}: no data rows parsed")
            continue

        trc_set = set(marker_names)
        missing_in_trc = sorted(model_set - trc_set) if model_markers is not None else []
        extra_in_trc = sorted(trc_set - model_set) if model_markers is not None else []

        worst_score = -1.0
        worst_marker = ""
        total_spikes = 0

        for i, name in enumerate(marker_names):
            series = indexed_data.get(str(i), [])
            stats, events = analyze_marker(
                name=name,
                series=series,
                times=times,
                scale_to_m=scale_to_m,
                zero_eps_m=args.zero_eps_m,
                jump_thresh_m=args.jump_thresh_m,
                vel_thresh_mps=args.vel_thresh_mps,
            )

            valid_ratio = (stats.valid_frames / stats.total_frames) if stats.total_frames else 0.0
            score = (1.0 - valid_ratio) + 0.3 * stats.jump_spikes + 0.3 * stats.vel_spikes
            if score > worst_score:
                worst_score = score
                worst_marker = name

            total_spikes += stats.jump_spikes + stats.vel_spikes

            marker_rows.append({
                "trc_file": str(trc_path),
                "marker": stats.marker,
                "total_frames": stats.total_frames,
                "valid_frames": stats.valid_frames,
                "valid_ratio": f"{valid_ratio:.6f}",
                "nan_frames": stats.nan_frames,
                "zero_frames": stats.zero_frames,
                "max_gap_frames": stats.max_gap_frames,
                "jump_spikes": stats.jump_spikes,
                "vel_spikes": stats.vel_spikes,
                "max_jump_m": f"{stats.max_jump_m:.6f}",
                "max_vel_mps": f"{stats.max_vel_mps:.6f}",
            })

            for kind, frame_idx, val, t in events:
                event_rows.append({
                    "trc_file": str(trc_path),
                    "marker": name,
                    "event": kind,
                    "frame_idx": frame_idx,
                    "time_s": f"{t:.6f}",
                    "value": f"{val:.6f}",
                })

        file_rows.append({
            "trc_file": str(trc_path),
            "units": units,
            "n_frames": len(times),
            "n_markers": len(marker_names),
            "worst_marker": worst_marker,
            "total_spikes": total_spikes,
            "missing_model_markers_count": len(missing_in_trc),
            "missing_model_markers": ";".join(missing_in_trc),
            "extra_trc_markers_count": len(extra_in_trc),
            "extra_trc_markers": ";".join(extra_in_trc),
        })

    with marker_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "trc_file", "marker", "total_frames", "valid_frames", "valid_ratio",
            "nan_frames", "zero_frames", "max_gap_frames", "jump_spikes", "vel_spikes",
            "max_jump_m", "max_vel_mps"
        ])
        writer.writeheader()
        writer.writerows(marker_rows)

    with event_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "trc_file", "marker", "event", "frame_idx", "time_s", "value"
        ])
        writer.writeheader()
        writer.writerows(event_rows)

    with file_csv.open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=[
            "trc_file", "units", "n_frames", "n_markers", "worst_marker", "total_spikes",
            "missing_model_markers_count", "missing_model_markers",
            "extra_trc_markers_count", "extra_trc_markers"
        ])
        writer.writeheader()
        writer.writerows(file_rows)

    print(f"[DONE] Wrote: {marker_csv}")
    print(f"[DONE] Wrote: {event_csv}")
    print(f"[DONE] Wrote: {file_csv}")
    print(f"[INFO] TRC analyzed: {len(file_rows)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
