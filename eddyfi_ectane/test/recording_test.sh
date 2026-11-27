#!/bin/bash
# Checks that logging:=true records the observations and transforms while streaming.
source "$(dirname "$0")/common.sh"
OUT=$(log_dir recording)
echo "logs in $OUT"
cd "$OUT"
export ROS_LOG_DIR=$OUT/roslog
CONFIG=hf_probe_axial_and_transversal_1_channel

python3 "$FAKE_SDK" $CONFIG --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 & PIDS+=($!)
ros2 launch eddyfi_ectane init.launch.py configuration:=$CONFIG no_robot_mode:=true rviz:=false logging:=true \
    > "$OUT/init.log" 2>&1 & PIDS+=($!)
LAUNCH=$!

for _ in $(seq 120); do grep -q "Loaded node '/ectane/ectane_handler'" "$OUT/init.log" && break; sleep 0.5; done
sleep 2
ros2 service call /ectane/stream_data eddyfi_ectane/srv/StreamData "{stream_data: true}" > /dev/null
sleep 4
ros2 service call /ectane/stream_data eddyfi_ectane/srv/StreamData "{stream_data: false}" > /dev/null

# Ctrl-C the launch, which closes the bag
kill -INT -- -$LAUNCH
timeout 20 tail --pid=$LAUNCH -f /dev/null

ros2 bag info "$OUT/ectane_recording" > "$OUT/bag_info.txt" 2>&1
cat "$OUT/bag_info.txt"
count() { grep -o -E "Topic: $1 \| Type: [^|]+ \| Count: [0-9]+" "$OUT/bag_info.txt" | grep -o -E '[0-9]+$'; }
check "[ \"\$(count /ectane/eddy_current_data)\" -gt 0 ] 2>/dev/null" "the bag holds observations"
check "[ \"\$(count /tf)\" -gt 0 ] 2>/dev/null" "the bag holds transforms"
exit $((FAILURES > 0))
