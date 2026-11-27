# Sourced by the tests, which run against the workspace whose setup.bash is sourced.
TEST_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
FAKE_SDK=$TEST_DIR/../scripts/fake_sdk.py
SHARE=$(ros2 pkg prefix eddyfi_ectane)/share/eddyfi_ectane

# Kept to this machine, so tests running alongside each other or a robot do not see each other
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42} ROS_LOCALHOST_ONLY=1 PYTHONUNBUFFERED=1

# A new directory for a test's logs, under TEST_LOG_DIR if it is set
log_dir() {
    local root=${TEST_LOG_DIR:-${TMPDIR:-/tmp}}
    mkdir -p "$root" && mktemp -d "$root/$1.XXXX"
}

# Background jobs get their own process groups and keep SIGINT, as they would from a terminal
set -m
PIDS=()

# Ctrl-C each process group started, then kill any left after 20 s
stop_all() {
    local p
    for p in "${PIDS[@]}"; do kill -INT -- -"$p" 2>/dev/null; done
    for p in "${PIDS[@]}"; do timeout 20 tail --pid="$p" -f /dev/null; kill -KILL -- -"$p" 2>/dev/null; done
    PIDS=()
}
trap stop_all EXIT

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
