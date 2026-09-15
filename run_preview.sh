#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$workspace/install/setup.bash" ]]; then
    printf 'Build this workspace with colcon build --symlink-install first.\n' >&2
    exit 1
fi
source "$workspace/install/setup.bash"
python_prefix="$(python3 -c 'import sys; print(sys.prefix)')"
if [[ -d "$python_prefix/conda-meta" ]]; then
    export CONDA_PREFIX="$python_prefix"
fi
# Invoke ROS directly to avoid machine-local wrappers sourcing another workspace.
exec python3 -c 'from ros2cli.cli import main; raise SystemExit(main())' \
    launch tap_robot_description cad_preview.launch.py "$@"
