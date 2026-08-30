#!/usr/bin/env bash
set -euo pipefail
config="$1"
log="${TMPDIR:-/tmp}/sentry-rviz2-smoke.$$.log"
set +e
timeout 10 xvfb-run -a rviz2 -d "$config" >"$log" 2>&1
status=$?
set -e
# timeout is expected: the purpose is to prove the configured plugin loads.
if [[ "$status" -ne 124 ]]; then
  cat "$log" >&2
  exit "$status"
fi
if grep -Eiq 'Failed to load|Failed to create|Class .* does not exist|PluginlibException|sentry_rviz_tools.*error' "$log"; then
  cat "$log" >&2
  exit 1
fi
