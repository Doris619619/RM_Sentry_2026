#!/usr/bin/env bash
# Complete simulated decision/NDT/planning/MPC/MCU/force-plant chain; never select a hardware serial port.
set -eo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export SENTRY_LOCALIZATION=ndt SENTRY_PHYSICS=true SENTRY_FULL_SYSTEM=true
exec bash "$ROOT/scripts/gazebo_autonomy.sh" "${1:-start}"
