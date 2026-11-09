#!/usr/bin/env bash
# Run only the new arena, localization interface, planner, MPC and safety output; no decision or MCU.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SENTRY_LOCALIZATION="${SENTRY_LOCALIZATION:-truth}"
export SENTRY_PHYSICS="${SENTRY_PHYSICS:-false}"
export SENTRY_FULL_SYSTEM=false
exec bash "$ROOT/scripts/gazebo_autonomy.sh" "${1:-start}"
