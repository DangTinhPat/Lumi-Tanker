"""Simulation-only Gazebo world control for the dashboard's reset-standup test convenience.

Uses only the plain, non-"/blocking"-suffixed gz services already proven in
this repo (tools/recovery_smoke.py, src/tap_sim/scripts/activate_controllers):
`/world/empty/control` (gz.msgs.WorldControl) and `/world/empty/set_pose`
(gz.msgs.Pose), both replying gz.msgs.Boolean. Never call this against
hardware; it exists to reposition the Gazebo world, not a physical robot.

`/world/empty/set_physics` (gz.msgs.Physics) was tried here too, to zero
gravity during the teleport settle window like the wheel_leg reference GUI
does — but measured to have no effect on this Gazebo Sim 8.10 build (the
service replies `data: true` while the robot keeps falling at normal gravity
speed; see docs/GUI_RESET_STANDUP_QA.md for the measured trace). It is not
used here as a result.
"""
import subprocess

_SERVICES = {'control': 'gz.msgs.WorldControl', 'set_pose': 'gz.msgs.Pose'}


def wrench(name, fx, fy, timeout=6):
    """Publish one EntityWrench (gz topic, not a service) to shove `name`.

    Simulation-only disturbance-rejection test convenience, same framing as
    the rest of this module. Mirrors the wheel_leg reference GUI's own
    shove: a `gz topic` publish is a single message, not a held wrench, and
    Gazebo applies it for one physics step (max_step_size 1 ms here, see
    src/tap_sim/CONTRACT.md), so passing a *force* in newtons that already
    equals impulse[N s] / 0.001 s gives a clean, repeatable impulse rather
    than a duration-dependent push.
    """
    result = subprocess.run(
        ['gz', 'topic', '-t', '/world/empty/wrench', '-m', 'gz.msgs.EntityWrench',
         '-p', f'entity: {{name: "{name}" type: MODEL}} '
               f'wrench: {{force: {{x: {fx:.3f} y: {fy:.3f}}}}}'],
        capture_output=True, text=True, timeout=timeout)
    if result.returncode:
        raise RuntimeError(f'wrench failed: {result.stdout} {result.stderr}')


def _service(name, request, timeout=5):
    reqtype = _SERVICES[name]
    result = subprocess.run(
        ['gz', 'service', '-s', f'/world/empty/{name}', '--reqtype', reqtype,
         '--reptype', 'gz.msgs.Boolean', '--timeout', '3000', '--req', request],
        capture_output=True, text=True, timeout=timeout)
    if result.returncode or 'data: true' not in result.stdout:
        raise RuntimeError(f'{name} failed: {result.stdout} {result.stderr}')


def pause(value):
    _service('control', f"pause: {'true' if value else 'false'}")


def set_pose(name, x, y, z, w=1., qx=0., qy=0., qz=0.):
    _service('set_pose',
             f'name: "{name}" position {{x: {x} y: {y} z: {z}}} '
             f'orientation {{w: {w} x: {qx} y: {qy} z: {qz}}}')


def multi_step(count):
    """Step the (paused) world `count` physics steps, then re-assert pause.

    Matches the wheel_leg reference GUI's own `_gz_step` helper: `multi_step`
    silently clears the world's pause flag as a side effect, so the caller
    must explicitly re-pause afterward to stay in control of when the world
    resumes running freely.

    NOT currently used by dashboard.py's reset-standup flow, deliberately:
    wheel_leg steps a just-teleported pose forward a few steps at a time
    with motors off specifically to let the base settle, and that is only
    safe there because it ALSO zeros gravity for the same window (see
    set_pose's caller in dashboard.py and docs/GUI_RESET_STANDUP_QA.md).
    Gravity-zero (`/world/empty/set_physics`) has been confirmed, three
    times, to have no effect on runtime physics on this Gazebo Sim 8.10
    build despite replying `data: true`. Stepping with motors off under
    this build's real, un-zeroable gravity is not a safe substitute: it was
    tried and measured to let an already-balanced robot start tipping
    during what should have been a no-op reset. Kept here, tested, as a
    primitive for a future caller that has a genuine reason to step the
    paused world (e.g. under a scenario where gravity truly does not
    matter) -- it is not a general-purpose "settle" tool on its own.
    """
    _service('control', f'multi_step: {int(count)}',
             timeout=max(5, int(count) // 20))
    pause(True)
