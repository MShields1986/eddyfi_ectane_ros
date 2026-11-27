#!/bin/bash
# Runs the driver and the reconstruction against fake_sdk.py, checks every channel they publish, then checks that
# Ctrl-C shuts the driver down and disconnects from the SDK.
# usage: pipeline_test.sh <configuration> <number of frequencies> [composed|stream-first]
#   composed:     the reconstruction runs in the driver's container, with reconstruction:=true
#   stream-first: streaming starts before the reconstruction does
CONFIG=$1; NUM_FREQS=$2; MODE=$3
source "$(dirname "$0")/common.sh"
OUT=$(log_dir "pipeline_${CONFIG}${MODE:+_$MODE}")
echo "logs in $OUT"
SETTINGS=$SHARE/config/$([ "$CONFIG" = example ] && echo simulated_ectane_mode.yaml || echo settings.yaml)

python3 "$FAKE_SDK" "$CONFIG" --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 & PIDS+=($!)
# The checker reads the stream from the fake SDK's log, which it writes first
for _ in $(seq 60); do grep -q '^stream ' "$OUT/fake_sdk.log" 2>/dev/null && break; sleep 0.5; done
ros2 launch eddyfi_ectane init.launch.py configuration:="$CONFIG" no_robot_mode:=true rviz:=false \
    reconstruction:=$([ "$MODE" = composed ] && echo true || echo false) > "$OUT/init.log" 2>&1 & PIDS+=($!)
LAUNCHES=($!)

recon() {
    ros2 launch eddyfi_ectane distributed_reconstruction.launch.py configuration:="$CONFIG" > "$OUT/recon.log" 2>&1 &
    PIDS+=($!); LAUNCHES+=($!)
}

if [ "$MODE" = stream-first ]; then
    # The reconstruction starts after streaming, so it first sees clouds from before its TF listener
    timeout 150 python3 "$TEST_DIR/check_pipeline.py" "$OUT/fake_sdk.log" "$NUM_FREQS" "$SETTINGS" stream-first \
        > "$OUT/check.out" 2>&1 &
    CHECKER=$!
    until grep -q 'streaming started' "$OUT/check.out" 2>/dev/null || ! kill -0 $CHECKER 2>/dev/null; do sleep 0.5; done
    sleep 8
    recon
    wait $CHECKER; STATUS=$?
    cat "$OUT/check.out"
else
    sleep 2
    [ "$MODE" = composed ] || recon
    timeout 150 python3 "$TEST_DIR/check_pipeline.py" "$OUT/fake_sdk.log" "$NUM_FREQS" "$SETTINGS"; STATUS=$?
fi
check "[ $STATUS -eq 0 ]" "observations and clouds match the stream"
echo "dropped observations: $(cat "$OUT/init.log" "$OUT/recon.log" 2>/dev/null | grep -c 'Dropping observation')"

# Ctrl-C the launches, as from a terminal
for p in "${LAUNCHES[@]}"; do kill -INT -- -"$p"; done
for p in "${LAUNCHES[@]}"; do timeout 20 tail --pid="$p" -f /dev/null; done
check "exited ${LAUNCHES[*]}" "the launches exit on Ctrl-C"
check "! cat '$OUT/init.log' '$OUT/recon.log' 2>/dev/null | grep -q escalating" "the nodes stop without launch escalating to SIGTERM"
check "grep -q '^command 7' '$OUT/fake_sdk.log'" "the driver disconnects from the SDK"

grep -h -E '\[(ERROR|FATAL)\]' "$OUT/init.log" "$OUT/recon.log" 2>/dev/null | sort -u | head -10
exit $((FAILURES > 0))
