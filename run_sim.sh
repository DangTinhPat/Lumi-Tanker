#!/usr/bin/env bash
set -e
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
if [[ ! -f "$workspace/install/tap_sim/share/tap_sim/package.xml" ]]; then
    printf 'Build the TAP workspace first: colcon build --symlink-install\n' >&2
    exit 1
fi
source "$workspace/install/setup.bash"
python_prefix="$(python3 -c 'import sys; print(sys.prefix)')"
if [[ -d "$python_prefix/conda-meta" ]]; then
    export CONDA_PREFIX="$python_prefix"
    export LD_LIBRARY_PATH="$python_prefix/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
fi
exec python3 -c 'from ros2cli.cli import main; raise SystemExit(main())' \
    launch tap_sim sim.launch.py "$@"
