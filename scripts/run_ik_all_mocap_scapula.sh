#!/usr/bin/env bash
set -euo pipefail

IK_BIN="${IK_BIN:-}"
INPUT_ROOT="${INPUT_ROOT:-/media/mathis/PANO/gst/analyse/data/mocap_scapula}"
TRC_GLOB="${TRC_GLOB:-sample_*.trc}"
MODEL_SUFFIX="${MODEL_SUFFIX:-_scapula_scaled.osim}"
START_TIME_FROM_MOT_ROOT="${START_TIME_FROM_MOT_ROOT:-}"
START_TIME_MOT_SUFFIX="${START_TIME_MOT_SUFFIX:-_ik.mot}"
IK_TASKS_FILE="${IK_TASKS_FILE:-}"
CHECK_MODELS_ONLY=0
FORWARD_ARGS=()

for arg in "$@"; do
  if [[ "$arg" == "--check-models" ]]; then
    CHECK_MODELS_ONLY=1
  elif [[ "$arg" == --start-time-from-mot-root=* ]]; then
    START_TIME_FROM_MOT_ROOT="${arg#*=}"
  elif [[ "$arg" == --start-time-mot-suffix=* ]]; then
    START_TIME_MOT_SUFFIX="${arg#*=}"
  elif [[ "$arg" == --ik-tasks=* ]]; then
    IK_TASKS_FILE="${arg#*=}"
  else
    FORWARD_ARGS+=("$arg")
  fi
done

if [[ -z "$IK_BIN" ]]; then
  if [[ -x "./build/online_ik_offline" ]]; then
    IK_BIN="./build/online_ik_offline"
  elif [[ -x "/home/mathis/pano2kinematics-stack/deps/my_online_ik/build/online_ik_offline" ]]; then
    IK_BIN="/home/mathis/pano2kinematics-stack/deps/my_online_ik/build/online_ik_offline"
  else
    IK_BIN="./build/online_ik_offline"
  fi
fi

if [[ ! -x "$IK_BIN" ]]; then
  echo "[ERROR] IK binary not found or not executable: $IK_BIN"
  echo "[HINT] Set IK_BIN=/absolute/path/to/online_ik_offline"
  exit 1
fi

if [[ ! -d "$INPUT_ROOT" ]]; then
  echo "[ERROR] Input root does not exist: $INPUT_ROOT"
  exit 1
fi

if [[ -n "$START_TIME_FROM_MOT_ROOT" && ! -d "$START_TIME_FROM_MOT_ROOT" ]]; then
  echo "[ERROR] START_TIME_FROM_MOT_ROOT does not exist: $START_TIME_FROM_MOT_ROOT"
  exit 1
fi

if [[ -n "$IK_TASKS_FILE" && ! -f "$IK_TASKS_FILE" ]]; then
  echo "[ERROR] IK_TASKS_FILE does not exist: $IK_TASKS_FILE"
  exit 1
fi

shopt -s nullglob

found_any_subject=0
missing_model_count=0
for subject_dir in "$INPUT_ROOT"/*; do
  [[ -d "$subject_dir" ]] || continue
  found_any_subject=1

  subject_name="$(basename "$subject_dir")"
  model_path="$subject_dir/${subject_name}${MODEL_SUFFIX}"
  out_dir="$subject_dir/ik_results"
  mkdir -p "$out_dir"

  if [[ ! -f "$model_path" ]]; then
    echo "[WARN] Missing model for $subject_name: $model_path"
    ((missing_model_count+=1))
    continue
  fi

  if (( CHECK_MODELS_ONLY == 1 )); then
    continue
  fi

  trc_files=("$subject_dir"/$TRC_GLOB)
  if (( ${#trc_files[@]} == 0 )); then
    echo "[WARN] No TRC files matching '$TRC_GLOB' in $subject_dir"
    continue
  fi

  mapfile -t trc_files < <(printf '%s\n' "${trc_files[@]}" | sort -V)
  echo "[INFO] Processing $subject_name (${#trc_files[@]} files)"

  for trc_path in "${trc_files[@]}"; do
    trc_base="$(basename "$trc_path" .trc)"
    tmp_out="$out_dir/.tmp_${trc_base}"
    mot_src="$tmp_out/output_results.mot"
    mot_dst="$out_dir/${trc_base}_ik.mot"

    mkdir -p "$tmp_out"
    echo "  [RUN] $trc_base"
    per_run_args=("${FORWARD_ARGS[@]}")
    if [[ -n "$IK_TASKS_FILE" ]]; then
      ik_tasks_arg="$IK_TASKS_FILE"
      if [[ "$ik_tasks_arg" != *.xml ]]; then
        ik_tasks_arg="$tmp_out/ik_tasks.xml"
        ln -sf "$IK_TASKS_FILE" "$ik_tasks_arg"
      fi
      per_run_args=("$ik_tasks_arg" "${per_run_args[@]}")
    fi
    if [[ -n "$START_TIME_FROM_MOT_ROOT" ]]; then
      start_ref_mot="$START_TIME_FROM_MOT_ROOT/$subject_name/ik_results/${trc_base}${START_TIME_MOT_SUFFIX}"
      if [[ ! -f "$start_ref_mot" ]]; then
        echo "  [WARN] Missing reference MOT for start-time: $start_ref_mot"
      else
        start_time="$(
          awk '
            BEGIN{after=0}
            tolower($0)=="endheader"{after=1; next}
            after==1 && NF>0 {
              # Skip column header rows like: "time trunk_tx ..."
              if ($1 ~ /^[-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?$/) {
                print $1;
                exit;
              }
            }
          ' "$start_ref_mot"
        )"
        if [[ -n "$start_time" ]]; then
          per_run_args+=("--start-time" "$start_time")
          echo "  [INFO] start-time from ${start_ref_mot##*/}: $start_time s"
        else
          echo "  [WARN] Could not extract start-time from: $start_ref_mot"
        fi
      fi
    fi

    "$IK_BIN" "$model_path" "$trc_path" "$tmp_out" "${per_run_args[@]}"

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

if (( found_any_subject == 0 )); then
  echo "[ERROR] No subject folders found under: $INPUT_ROOT"
  exit 1
fi

if (( CHECK_MODELS_ONLY == 1 )); then
  if (( missing_model_count > 0 )); then
    echo "[DONE] Model check completed: $missing_model_count missing model file(s)."
    exit 3
  fi
  echo "[DONE] Model check completed: no missing model files."
  exit 0
fi

echo "[DONE] Batch IK completed under: $INPUT_ROOT"
