"""Direct `ros2 launch` command builders.

Replaces the run_sim.sh / run_jump.sh / run_preview.sh shell wrappers
(removed 2026-09-16 per explicit direction: launch via `ros2 launch`
everywhere, not shell scripts - matching src/gui/gui/sim_process.py's own
"no run_sim.sh" pattern for the GUI's Launch sim button). Callers are
expected to already run from a sourced ROS 2 environment, the same
assumption every tools/*.py smoke script and sim_process.py already make
(`source install/setup.bash && python3 tools/...`); this module does no
environment sourcing or conda/LD_LIBRARY_PATH fixup of its own.
"""

# profile -> (controller_config filename under src/tap_control/config/, jump_leg_speed rad/s)
JUMP_PROFILES = {
    'rated': ('jump.yaml', 12.566370614359172),
    'manual-curve': ('jump_manual_curve.yaml', 28.274333882308138),
}
# Args a named jump profile owns; a caller passing any of these is almost
# certainly trying to override the profile rather than layer onto it.
_JUMP_OWNED_PREFIXES = ('jump_model:=', 'jump_leg_speed:=',
                        'jump_leg_velocity_damping:=', 'controller_config:=')


def sim_launch_command(*extra_args):
    """`ros2 launch tap_sim sim.launch.py <extra_args>`."""
    return ['ros2', 'launch', 'tap_sim', 'sim.launch.py', *extra_args]


def preview_launch_command(*extra_args):
    """`ros2 launch tap_robot_description cad_preview.launch.py <extra_args>`."""
    return ['ros2', 'launch', 'tap_robot_description', 'cad_preview.launch.py', *extra_args]


def jump_launch_command(workspace, profile, extra_args=()):
    """Port of run_jump.sh's profile selection and argument guard.

    `workspace` is the TAP workspace root (a Path or str), used to build an
    absolute controller_config path exactly as run_jump.sh did. Raises
    ValueError (never prints/exits) for an unknown profile or an attempt to
    override an argument the profile owns - the caller decides how to
    report that, since this runs in-process rather than as a subprocess.
    """
    try:
        config, speed = JUMP_PROFILES[profile]
    except KeyError:
        raise ValueError(
            f'Unknown jump profile {profile!r}; choose one of {sorted(JUMP_PROFILES)}')
    for argument in extra_args:
        if argument.startswith(_JUMP_OWNED_PREFIXES):
            raise ValueError(
                'The selected jump profile owns model, speed and controller configuration.')
    return sim_launch_command(
        'jump_model:=true', f'jump_leg_speed:={speed}', 'jump_leg_velocity_damping:=0.6',
        f'controller_config:={workspace}/src/tap_control/config/{config}', *extra_args)
