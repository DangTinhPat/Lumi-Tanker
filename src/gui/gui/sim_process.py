"""Resolve the TAP workspace root and directly own a `ros2 launch tap_sim
sim.launch.py` subprocess -- matching the wheel_leg reference GUI's own
pattern of the GUI backend directly constructing and running the literal
`ros2 launch` invocation itself, rather than shelling out to a separate
wrapper script.

Kept separate from dashboard.py so the workspace-resolution trick and the
process-lifecycle wrapping live in exactly one place, and so both are
testable without Tk/rclpy.

Unlike run_sim.sh (still used standalone by tools/*_smoke.py and manual
headless runs -- untouched, not removed), this does not need to re-source
install/setup.bash or fix up CONDA_PREFIX/LD_LIBRARY_PATH: the GUI process
itself is already running from a correctly sourced ROS environment (`ros2
run gui gui` requires that), and `subprocess.Popen` inherits the parent's
environment by default, so the launched sim gets the same environment for
free.

This runs the literal `ros2` command (PATH lookup), matching wheel_leg's own
GUI exactly (it shells out to `exec ros2 launch balance_firmware_core
sim.launch.py`) -- an earlier version of this module avoided that in favor
of invoking the ros2cli Python entry point inline, worried that this
machine's `~/bin/ros2` wrapper (which also sources
/home/dvt/wheel_leg/install/setup.bash) would shadow tap_sim. Verified
empirically: since subprocess.Popen inherits this process's environment,
AMENT_PREFIX_PATH already has this workspace's install/tap_sim entry before
the wrapper runs, and the wrapper's own sourcing only prepends wheel_leg's
paths in front rather than replacing the inherited ones, so `ros2 pkg prefix
tap_sim` (and therefore `ros2 launch tap_sim ...`) still resolves correctly
even through the wrapper. That collision risk is real only for the
ambiguously-named `gui` package itself (see src/gui/README.md), which is
never what this module launches.
"""
from pathlib import Path
import sys
import time


def resolve_workspace():
    """Walk up from this file to the TAP workspace root.

    This repo always builds with `colcon build --symlink-install`, so an
    installed ament_cmake_python module resolves, via Path.resolve(), back
    through the symlink to its real path under src/gui/... — the same trick
    run_sim.sh uses in bash via `dirname "${BASH_SOURCE[0]}"`.
    """
    here = Path(__file__).resolve()
    for candidate in (here, *here.parents):
        if (candidate / 'tools' / 'owned_process.py').is_file() and (candidate / 'run_sim.sh').is_file():
            return candidate
    raise RuntimeError(
        f'Could not locate the TAP workspace root from {here}; expected an '
        'ancestor directory containing both tools/owned_process.py and run_sim.sh.')


WORKSPACE = resolve_workspace()
sys.path.insert(0, str(WORKSPACE / 'tools'))
from owned_process import cleanup_group, start_owned_process  # noqa: E402

# activate_controllers's own proven get_logger().info bring-up milestones;
# do not invent unverified controller_manager/spawn log strings here.
STAGES = (
    ('Enabled and running after', 'Running'),
    ('Controllers ready; paused.', 'Controllers ready, waiting for Enable'),
)


def sim_stage(log_path, elapsed):
    text = 'Starting Gazebo...'
    try:
        blob = Path(log_path).read_text(errors='replace')
        for marker, label in STAGES:
            if marker in blob:
                text = label
                break
    except OSError:
        pass
    return f'{text} ({int(elapsed)}s)'


def launch_sim(extra_args=(), log_dir=None):
    """Launch `ros2 launch tap_sim sim.launch.py <extra_args>` as an owned
    subprocess, built and run directly by this module (no run_sim.sh). No env
    overrides: the GUI's own rclpy node and its later gz_world calls must land
    on the same ROS_DOMAIN_ID/GZ_PARTITION as the sim this launches, unlike
    the isolated QA smoke tools, which deliberately randomize those for
    concurrent trials.
    """
    if not (WORKSPACE / 'install' / 'tap_sim' / 'share' / 'tap_sim' / 'package.xml').is_file():
        raise RuntimeError('Build the TAP workspace first: colcon build --symlink-install')
    log_dir = log_dir or WORKSPACE / 'log' / f'gui_sim_{time.time_ns()}'
    log_dir.mkdir(parents=True)
    log_path = log_dir / 'sim.log'
    stream = log_path.open('w')
    command = ['ros2', 'launch', 'tap_sim', 'sim.launch.py', *extra_args]
    process = start_owned_process(command, cwd=WORKSPACE, stdout=stream, stderr=stream)
    return process, log_path, stream


def stop_sim(process, stream):
    try:
        return cleanup_group(process)
    finally:
        stream.close()
