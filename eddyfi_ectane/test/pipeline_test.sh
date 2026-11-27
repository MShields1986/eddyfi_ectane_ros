#!/bin/bash
# Runs the driver and the reconstruction against fake_sdk.py, checks every channel they publish, then checks that
# Ctrl-C shuts the driver down and disconnects from the SDK.
# usage: pipeline_test.sh <configuration> <number of frequencies> [stream-first]
#   stream-first: streaming starts before the reconstruction does
CONFIG=$1; NUM_FREQS=$2; MODE=$3
source "$(dirname "$0")/common.sh"
OUT=$(log_dir "pipeline_${CONFIG}${MODE:+_$MODE}")
echo "logs in $OUT"
start_roscore "$OUT"

python3 "$FAKE_SDK" "$CONFIG" --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 & PIDS+=($!)
roslaunch eddyfi_ectane init.launch configuration:="$CONFIG" no_robot_mode:=true rviz:=false > "$OUT/init.log" 2>&1 &
PIDS+=($!); LAUNCHES=($!)

recon() {
    roslaunch eddyfi_ectane distributed_reconstruction.launch > "$OUT/recon.log" 2>&1 & PIDS+=($!); LAUNCHES+=($!)
}

if [ "$MODE" = stream-first ]; then
    # The reconstruction starts after streaming, so it first sees latched clouds from before its TF listener
    timeout 150 python3 "$TEST_DIR/check_pipeline.py" "$OUT/fake_sdk.log" "$NUM_FREQS" stream-first > "$OUT/check.out" 2>&1 &
    CHECKER=$!
    until grep -q 'streaming started' "$OUT/check.out" 2>/dev/null || ! kill -0 $CHECKER 2>/dev/null; do sleep 0.5; done
    sleep 8
    recon
    wait $CHECKER; STATUS=$?
    cat "$OUT/check.out"
else
    sleep 2
    recon
    timeout 150 python3 "$TEST_DIR/check_pipeline.py" "$OUT/fake_sdk.log" "$NUM_FREQS"; STATUS=$?
fi
check "[ $STATUS -eq 0 ]" "observations and clouds match the stream"
echo "dropped observations: $(cat "$OUT/init.log" "$OUT/recon.log" 2>/dev/null | grep -c 'Dropping observation')"

# Ctrl-C the launches, as from a terminal
for p in "${LAUNCHES[@]}"; do kill -INT -- -"$p"; done
for p in "${LAUNCHES[@]}"; do timeout 30 tail --pid="$p" -f /dev/null; done
check "exited ${LAUNCHES[*]}" "the launches exit on Ctrl-C"
check "! cat '$OUT/init.log' '$OUT/recon.log' 2>/dev/null | grep -q escalating" "the nodes stop without roslaunch escalating to SIGTERM"
check "grep -q '^command 7' '$OUT/fake_sdk.log'" "the driver disconnects from the SDK"

grep -h -E '\[(ERROR|FATAL)\]' "$OUT/init.log" "$OUT/recon.log" 2>/dev/null | sort -u | head -10
exit $((FAILURES > 0))
