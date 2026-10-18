#!/usr/bin/env bash
set -euo pipefail

vision_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
vision_python="${LUMI_VISION_PYTHON:-$HOME/Downloads/yolov5-master/yolov5-master/.venv/bin/python}"
person_model="${LUMI_PERSON_MODEL:-$vision_dir/../yolov8n_1(1).onnx}"
cd "$vision_dir"
exec "$vision_python" track_black_square.py --model "$person_model" "$@"
