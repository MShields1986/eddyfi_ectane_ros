#!/bin/bash
# Checks that the driver waits for data without spinning, and that Ctrl-C still shuts it down promptly and
# disconnects from the SDK, when no data is arriving.
# usage: shutdown_test.sh <stopped|stalled>
#   stopped: streaming is stopped with the service, so the acquisition thread waits for data that will not come
#   stalled: the fake SDK is frozen mid-stream, as if the Ectane dropped out
SCENARIO=$1
source "$(dirname "$0")/common.sh"
OUT=$(log_dir "shutdown_$SCENARIO")
echo "logs in $OUT"
CONFIG=hf_probe_axial_and_transversal_1_channel

python3 "$FAKE_SDK" $CONFIG --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 & PIDS+=($!)
SDK=$!
ros2 launch eddyfi_ectane init.launch.py configuration:=$CONFIG no_robot_mode:=true rviz:=false > "$OUT/init.log" 2>&1 &
PIDS+=($!)
LAUNCH=$!

for _ in $(seq 120); do grep -q "Loaded node '/ectane/ectane_handler'" "$OUT/init.log" && break; sleep 0.5; done
CONTAINER=$(pgrep -f component_container_mt | head -1)
TCK=$(getconf CLK_TCK)
cpu() {   # Percentage of a CPU the component container uses over 3 s
    local a b
    a=$(awk '{print $14 + $15}' /proc/$CONTAINER/stat); sleep 3; b=$(awk '{print $14 + $15}' /proc/$CONTAINER/stat)
    echo $(( (b - a) * 100 / (3 * TCK) ))
}

ros2 service call /ectane/stream_data eddyfi_ectane/srv/StreamData "{stream_data: true}" > /dev/null
check "timeout 20 ros2 topic echo --once /ectane/eddy_current_data --field header > /dev/null 2>&1" \
    "observations are published while streaming"

if [ "$SCENARIO" = stalled ]; then
    kill -STOP $SDK
else
    ros2 service call /ectane/stream_data eddyfi_ectane/srv/StreamData "{stream_data: false}" > /dev/null
fi
sleep 1
WAITING_CPU=$(cpu)
check "[ $WAITING_CPU -lt 30 ]" "the driver uses $WAITING_CPU% of a CPU waiting for data"

# Ctrl-C the launch, as from a terminal
START=$(date +%s.%N)
kill -INT -- -$LAUNCH
timeout 20 tail --pid=$LAUNCH -f /dev/null
check "exited $LAUNCH" "the launch exits on Ctrl-C, after $(awk "BEGIN {printf \"%.2f\", $(date +%s.%N) - $START}") s"
check "! grep -q escalating '$OUT/init.log'" "the driver stops without launch escalating to SIGTERM"
kill -CONT $SDK; sleep 1
check "grep -q '^command 7' '$OUT/fake_sdk.log'" "the driver disconnects from the SDK"

grep -h -E '\[(ERROR|FATAL)\]' "$OUT/init.log" | sort -u | head -10
exit $((FAILURES > 0))
