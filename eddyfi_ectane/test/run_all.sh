#!/bin/bash
# Runs every test, printing the logs of any that fail, and exits non-zero if any did.
TEST_DIR=$(cd "$(dirname "$0")" && pwd)
TESTS=(
    "pipeline_test.sh hf_probe_axial_and_transversal_1_channel 1"
    "pipeline_test.sh hf_probe_transversal_1_channel 1"
    "pipeline_test.sh lf_probe_axial_1_channel 1"
    "pipeline_test.sh hf_probe_axial_and_transversal_4_channels 4"
    "pipeline_test.sh hf_probe_axial_and_transversal_4_channels 4 stream-first"
    "pipeline_test.sh lf_probe_axial_and_transversal_4_channels 4"
    "shutdown_test.sh stopped"
    "shutdown_test.sh stalled"
    "failure_test.sh"
)

RESULTS=()
FAILED=0
OUTPUT=$(mktemp)
for test in "${TESTS[@]}"; do
    echo "===== $test"
    # To a file rather than a pipe, which a daemon the ros2 command line starts could hold open
    "$TEST_DIR"/$test < /dev/null > "$OUTPUT" 2>&1
    STATUS=$?
    cat "$OUTPUT"
    if [ $STATUS -ne 0 ]; then
        FAILED=$((FAILED + 1))
        LOGS=$(sed -n 's/^logs in //p' "$OUTPUT")
        for log in "$LOGS"/*.log "$LOGS"/*.out; do
            [ -f "$log" ] && { echo "--- $log"; tail -n 40 "$log"; }
        done
    fi
    RESULTS+=("$([ $STATUS -eq 0 ] && echo passed || echo FAILED)  $test")
done

echo "===== $((${#TESTS[@]} - FAILED)) of ${#TESTS[@]} passed"
printf '%s\n' "${RESULTS[@]}"
exit $((FAILED > 0))
