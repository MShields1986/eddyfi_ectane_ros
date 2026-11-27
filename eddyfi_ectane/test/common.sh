# Sourced by the tests, which run against the workspace whose setup.bash is sourced.
TEST_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
FAKE_SDK=$TEST_DIR/../scripts/fake_sdk.py
export PYTHONUNBUFFERED=1

# A new directory for a test's logs, under TEST_LOG_DIR if it is set
log_dir() {
    local root=${TEST_LOG_DIR:-${TMPDIR:-/tmp}}
    mkdir -p "$root" && mktemp -d "$root/$1.XXXX"
}

# Background jobs get their own process groups and keep SIGINT, as they would from a terminal
set -m
PIDS=()

# Ctrl-C each process group started, last first so the roscore goes last, killing any left after 20 s
stop_all() {
    local i p
    for ((i = ${#PIDS[@]} - 1; i >= 0; i--)); do
        p=${PIDS[i]}
        kill -INT -- -"$p" 2>/dev/null
        timeout 20 tail --pid="$p" -f /dev/null
        kill -KILL -- -"$p" 2>/dev/null
    done
    PIDS=()
}
trap stop_all EXIT

# A roscore for the test, on its own port so it does not meet one already running
start_roscore() {
    export ROS_MASTER_URI=http://localhost:${ROS_PORT:-11411} ROS_LOG_DIR=$1/roslog ROS_HOME=$1/roshome
    roscore -p "${ROS_PORT:-11411}" > "$1/roscore.log" 2>&1 & PIDS+=($!)
    for _ in $(seq 60); do rosparam get /run_id > /dev/null 2>&1 && return; sleep 0.5; done
    echo "FAILED: roscore did not start"
    exit 1
}

# Whether all the processes given have exited
exited() {
    local p
    for p in "$@"; do kill -0 "$p" 2>/dev/null && return 1; done
    return 0
}

FAILURES=0
check() {
    if eval "$1"; then echo "ok:     $2"; else echo "FAILED: $2"; FAILURES=$((FAILURES + 1)); fi
}
