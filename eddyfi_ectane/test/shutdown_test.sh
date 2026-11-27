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
start_roscore "$OUT"
CONFIG=hf_probe_axial_and_transversal_1_channel

python3 "$FAKE_SDK" $CONFIG --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 & PIDS+=($!)
SDK=$!
roslaunch eddyfi_ectane init.launch configuration:=$CONFIG no_robot_mode:=true rviz:=false > "$OUT/init.log" 2>&1 &
PIDS+=($!)
LAUNCH=$!

for _ in $(seq 120); do rosservice list 2>/dev/null | grep -q /ectane/stream_data && break; sleep 0.5; done
MANAGER=$(pgrep -f '__name:=ectane_nodelet_manager' | head -1)
TCK=$(getconf CLK_TCK)
cpu() {   # Percentage of a CPU the nodelet manager uses over 3 s
    local a b
    a=$(awk '{print $14 + $15}' /proc/$MANAGER/stat); sleep 3; b=$(awk '{print $14 + $15}' /proc/$MANAGER/stat)
    echo $(( (b - a) * 100 / (3 * TCK) ))
}

rosservice call /ectane/stream_data "stream_data: true" > /dev/null
check "timeout 20 rostopic echo -n 1 /ectane/eddy_current_data/header > /dev/null 2>&1" \
    "observations are published while streaming"

if [ "$SCENARIO" = stalled ]; then
    kill -STOP $SDK
else
    rosservice call /ectane/stream_data "stream_data: false" > /dev/null
fi
sleep 1
WAITING_CPU=$(cpu)
check "[ $WAITING_CPU -lt 30 ]" "the driver uses $WAITING_CPU% of a CPU waiting for data"

# Ctrl-C the launch, as from a terminal
START=$(date +%s.%N)
kill -INT -- -$LAUNCH
timeout 30 tail --pid=$LAUNCH -f /dev/null
check "exited $LAUNCH" "the launch exits on Ctrl-C, after $(awk "BEGIN {printf \"%.2f\", $(date +%s.%N) - $START}") s"
check "! grep -q escalating '$OUT/init.log'" "the driver stops without roslaunch escalating to SIGTERM"
kill -CONT $SDK; sleep 1
check "grep -q '^command 7' '$OUT/fake_sdk.log'" "the driver disconnects from the SDK"

grep -h -E '\[(ERROR|FATAL)\]' "$OUT/init.log" | sort -u | head -10
exit $((FAILURES > 0))
