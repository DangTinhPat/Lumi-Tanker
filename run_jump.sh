#!/usr/bin/env bash
set -euo pipefail
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
profile="${1:-rated}"
if (( $# )); then shift; fi
case "$profile" in
  rated)
    config=jump.yaml
    speed=12.566370614359172
    ;;
  manual-curve)
    config=jump_manual_curve.yaml
    speed=28.274333882308138
    ;;
  *)
    printf 'Usage: %s [rated|manual-curve] [gui:=false ...]\n' "$0" >&2
    exit 2
    ;;
esac
for argument in "$@"; do
  case "$argument" in
    jump_model:=*|jump_leg_speed:=*|jump_leg_velocity_damping:=*|controller_config:=*)
      printf 'The selected jump profile owns model, speed and controller configuration.\n' >&2
      exit 2
      ;;
  esac
done
exec "$workspace/run_sim.sh" jump_model:=true "jump_leg_speed:=$speed" jump_leg_velocity_damping:=0.6 \
  "controller_config:=$workspace/src/tap_control/config/$config" "$@"
