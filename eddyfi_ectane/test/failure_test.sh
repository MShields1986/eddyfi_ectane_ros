#!/bin/bash
# Checks that the driver reports bad settings and does not start the Ectane, and that an unknown configuration stops
# the launch with an error.
source "$(dirname "$0")/common.sh"
OUT=$(log_dir failure)
echo "logs in $OUT"
start_roscore "$OUT"

# Four frequency configuration with two frequencies set
python3 "$FAKE_SDK" hf_probe_axial_and_transversal_4_channels --log "$OUT/fake_sdk.log" > "$OUT/fake_sdk.out" 2>&1 &
PIDS+=($!)
roslaunch "$TEST_DIR/bad_frequencies.launch" > "$OUT/frequencies.log" 2>&1 & PIDS+=($!)
for _ in $(seq 60); do grep -q 'Invalid frequency configuration' "$OUT/frequencies.log" && break; sleep 0.5; done
check "grep -q -F 'uses 4 frequencies but 2 frequencies and 2 voltages are set' '$OUT/frequencies.log'" \
    "frequencies: the driver reports the mismatch"
check "grep -q -F 'Invalid frequency configuration, not initialising the Ectane' '$OUT/frequencies.log'" \
    "frequencies: the driver does not initialise the Ectane"
sleep 2
check "! grep -q '^command 5' '$OUT/fake_sdk.log'" "frequencies: the driver does not acquire data"

timeout 30 roslaunch eddyfi_ectane init.launch configuration:=hf_probe_typo > "$OUT/configuration.log" 2>&1
STATUS=$?
check "[ $STATUS -ne 0 ] && [ $STATUS -ne 124 ] && grep -q crosstalk_hf_probe_typo.yaml '$OUT/configuration.log'" \
    "configuration: roslaunch exits $STATUS, as there is no crosstalk_hf_probe_typo.yaml"

exit $((FAILURES > 0))
