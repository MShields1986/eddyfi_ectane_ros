#!/bin/bash
# Checks that the driver stops with a clear error, rather than running misconfigured, when its settings do not match
# its configuration.
source "$(dirname "$0")/common.sh"
OUT=$(log_dir failure)
echo "logs in $OUT"
export ROS_LOG_DIR=$OUT/roslog
C=$SHARE/config
P=$C/probes

# Fails if the command succeeds or times out, or does not print the error
expect_error() {
    local name=$1 error=$2; shift 2
    timeout 30 "$@" > "$OUT/$name.log" 2>&1
    local status=$?
    check "[ $status -ne 0 ] && [ $status -ne 124 ] && grep -q -F \"$error\" '$OUT/$name.log'" \
        "$name: exits $status with \"$error\""
}

expect_error frequencies "uses 4 frequencies but 2 frequencies and 2 voltages are set" \
    ros2 run eddyfi_ectane ectane_node --ros-args --params-file $C/settings.yaml \
    --params-file $P/crosstalk_hf_probe_axial_and_transversal_4_channels.yaml \
    --params-file $P/hf_axial_geometry.yaml --params-file $P/hf_transversal_geometry.yaml \
    -p configuration:=hf_probe_axial_and_transversal_4_channels -p 'frequencies:=[5000, 10000]' -p 'voltages:=[7.0, 7.0]'

expect_error crosstalk "No crosstalk parameters" \
    ros2 run eddyfi_ectane ectane_node --ros-args --params-file $C/settings.yaml -p configuration:=hf_probe_axial_1_channel

expect_error configuration "Unknown configuration 'hf_probe_typo'" \
    ros2 launch eddyfi_ectane init.launch.py configuration:=hf_probe_typo

exit $((FAILURES > 0))
