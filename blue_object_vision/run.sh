#!/usr/bin/env bash
set -euo pipefail

vision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
cd "$vision_dir"
exec python3 track_black_square.py "$@"
