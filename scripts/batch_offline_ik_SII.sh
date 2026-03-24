#!/usr/bin/env bash
set -euo pipefail

IK_BIN="${IK_BIN:-./build/online_ik_offline}"
INPUT_ROOT="${INPUT_ROOT:-/media/mathis/PANO/gst/analyse/data/repaired_videos}"
OUTPUT_ROOT="${OUTPUT_ROOT:-/media/mathis/PANO/gst/analyse/data/markerless_results}"
MODEL_SUFFIX="_upperlimb-biorob_nomuscle.osim"
TRC_GLOB="SII*.trc"

if [[ ! -x "$IK_BIN" ]]; then
  echo "[ERROR] IK binary not found or not executable: $IK_BIN"
  exit 1
fi

mkdir -p "$OUTPUT_ROOT"

shopt -s nullglob

found_any_folder=0
for person_dir in "$INPUT_ROOT"/*_raw; do
  [[ -d "$person_dir" ]] || continue
  found_any_folder=1

  folder_name="$(basename "$person_dir")"
  person_name="${folder_name%_raw}"
  model_path="$person_dir/${person_name}${MODEL_SUFFIX}"

  if [[ ! -f "$model_path" ]]; then
    echo "[WARN] Missing model for $folder_name: $model_path"
    continue
  fi

  out_dir="$OUTPUT_ROOT/$folder_name"
  mkdir -p "$out_dir"

  trc_files=("$person_dir"/$TRC_GLOB)
  if (( ${#trc_files[@]} == 0 )); then
    echo "[WARN] No TRC files matching '$TRC_GLOB' in $person_dir"
    continue
  fi

  mapfile -t trc_files < <(printf '%s\n' "${trc_files[@]}" | sort -V)

  echo "[INFO] Processing $folder_name (${#trc_files[@]} files)"

  for trc_path in "${trc_files[@]}"; do
    trc_base="$(basename "$trc_path" .trc)"
    tmp_out="$out_dir/.tmp_${trc_base}"
    mkdir -p "$tmp_out"

    echo "  [RUN] $trc_base"
    "$IK_BIN" "$model_path" "$trc_path" "$tmp_out" "$@"

    mot_src="$tmp_out/output_results.mot"
    mot_dst="$out_dir/${trc_base}.mot"

    if [[ -f "$mot_src" ]]; then
      mv -f "$mot_src" "$mot_dst"
      rmdir "$tmp_out" 2>/dev/null || true
      echo "  [OK ] $mot_dst"
    else
      echo "  [ERR] Missing expected output: $mot_src"
      exit 2
    fi
  done

done

if (( found_any_folder == 0 )); then
  echo "[ERROR] No '*_raw' folders found under: $INPUT_ROOT"
  exit 1
fi

echo "[DONE] Batch IK completed. Outputs in: $OUTPUT_ROOT"
