#!/usr/bin/env bash

ROOT_DIR="$1"

if [ -z "$ROOT_DIR" ]; then
    echo "Usage: ./run_ik_all.sh /path/to/data_root"
    exit 1
fi

for subject in "$ROOT_DIR"/alessandro; do

    [ -d "$subject" ] || continue

    echo "=== Processing subject: $subject ==="

    # Automatically detect the .osim model (should be exactly one)
    MODEL_LIST=($(find "$subject" -maxdepth 1 -type f -name "*.osim"))

    if [ ${#MODEL_LIST[@]} -eq 0 ]; then
        echo "  ❌ No .osim model found in $subject, skipping."
        continue
    elif [ ${#MODEL_LIST[@]} -gt 1 ]; then
        echo "  ⚠️ Multiple .osim files found in $subject. Using the first one:"
        for m in "${MODEL_LIST[@]}"; do echo "     - $m"; done
    fi

    MODEL="${MODEL_LIST[0]}"
    echo "  -> Using model: $MODEL"

    mkdir -p "$subject/ik_results"

    # Loop through all TRC files
    for trc in "$subject"/*.trc; do
        [ -f "$trc" ] || continue

        base=$(basename "$trc" .trc)
        OUT="$subject/ik_results/${base}_ik.mot"
        IKXML="$subject/ik_${base}.xml"

        # Create IK XML from the template
        sed \
            -e "s|@MODEL@|${MODEL}|g" \
            -e "s|@TRC@|${trc}|g" \
            -e "s|@OUT@|${OUT}|g" \
            "$ROOT_DIR/ik_setup_template.xml" > "$IKXML"

        echo "  Running IK for $trc"
        opensim-cmd run-tool "$IKXML"
    done

    echo "=== Done with subject: $subject ==="
done
