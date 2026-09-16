"""Tk desktop dashboard.

Can launch/stop its own simulation (Launch sim / Stop sim) and reposition the
Gazebo world only through the labeled, simulation-only "Reset standup" test
convenience below. That teleport-based sequence is a testing shortcut, never
evidence of real recovery capability: see .agents/skills/tap-ros-gazebo/
SKILL.md ("Keep Gazebo world-reset commands simulation-only; reflect
unsupported actions honestly") and tap-qa-regression/SKILL.md ("do not
inherit reference robot results").
"""
import argparse
import csv
import math
import os
import threading
import time
from collections import deque
from pathlib import Path
import tkinter as tk
from tkinter import ttk, messagebox

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from rosgraph_msgs.msg import Clock
from std_msgs.msg import Bool, Float64MultiArray, String

from tap_teleop.pose_curves import JOINTS, generate, sample
from tap_teleop.show_sequence import ShowSequence
from tap_teleop.deep_squat import DeepSquatTrajectory, generate_deep_squat, generate_stand_tall

from . import gz_world, sim_process

# wheel_leg reference dashboard's own dark palette (/home/dvt/wheel_leg/gui/
# scripts/gui) - reused verbatim per the user's "redesign like wheel_leg" ask.
BG = '#16181d'
CARD = '#22252e'
FG = '#e8eaf0'
MUTED = '#98a0b0'
ACCENT = '#4c8dff'
GOOD = '#3ecf8e'
WARN = '#ffb454'
BAD = '#ff5c68'
# All six physical joints in CLAUDE.md's documented order, for the Telemetry
# tab's table/plots - wider than tap_teleop's JOINTS (hip/knee only, used by
# the pose-control state machine, left untouched).
ALL_JOINTS = ('cad_left_hip', 'cad_left_knee', 'cad_left_wheel',
              'cad_right_hip', 'cad_right_knee', 'cad_right_wheel')
HISTORY_S = 12.0

try:
    import matplotlib
    matplotlib.use('TkAgg')
    from matplotlib.figure import Figure
    from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
    HAVE_MPL = True
    MPL_ERR = ''
except Exception as exc:  # pragma: no cover - environment dependent
    HAVE_MPL = False
    MPL_ERR = str(exc)


class StripChart(tk.Canvas):
    """Minimal dependency-free strip chart, used only if matplotlib is
    unavailable (confirmed present in this environment; this is a defensive
    fallback, not the common path). Ported near-verbatim from wheel_leg's
    own StripChart."""

    def __init__(self, master, series, ylabel, **kwargs):
        super().__init__(master, bg='#1b1e26', highlightthickness=0, **kwargs)
        self.series = series  # [(label, color), ...]
        self.ylabel = ylabel
        self.data = {label: [] for label, _ in series}
        self.bind('<Configure>', lambda _e: self.redraw())

    def push(self, label, t, value):
        if label not in self.data:
            self.data[label] = []
        buf = self.data[label]
        buf.append((t, value))
        cutoff = t - HISTORY_S
        while buf and buf[0][0] < cutoff:
            buf.pop(0)

    def redraw(self):
        self.delete('all')
        w, h = self.winfo_width(), self.winfo_height()
        if w < 2 or h < 2:
            return
        all_vals = [v for buf in self.data.values() for _, v in buf]
        if not all_vals:
            return
        lo, hi = min(all_vals), max(all_vals)
        if hi - lo < 1e-6:
            lo, hi = lo - 1, hi + 1
        all_ts = [t for buf in self.data.values() for t, _ in buf]
        if not all_ts:
            return
        t1 = max(all_ts)
        t0 = t1 - HISTORY_S
        self.create_text(4, 4, anchor='nw', fill=MUTED, font=('', 8),
                         text=self.ylabel)
        for label, color in self.series:
            buf = self.data.get(label, [])
            pts = []
            for t, v in buf:
                x = w * (t - t0) / max(1e-6, t1 - t0)
                y = h - h * (v - lo) / (hi - lo)
                pts.extend((x, y))
            if len(pts) >= 4:
                self.create_line(*pts, fill=color, width=1)

DEEP_SQUAT_CANDIDATE_MAX_RAD = 1.6
DEEP_SQUAT_GEOMETRY_CEILING_RAD = 1.6
DEEP_SQUAT_GENERATION_HIP_BOUNDS = (-.6, 1.05)
DEEP_SQUAT_FULL_DEPTH_MIN_TRANSITION_S = 11.
# 0.5 rad stays well inside the monotonic-height region found by sampling FK
# height vs. knee (peaks ~0.57 rad, then folds back down - see
# generate_stand_tall's docstring); generated unconditionally (unlike Deep
# squat's --deep-squat-knee opt-in) since this magnitude never approaches the
# CAD wheel/thigh mesh-overlap risk documented in docs/DEEP_SQUAT.md for the
# crouch direction, and generate_stand_tall's own monotonicity check rejects
# anything past the real geometric peak regardless.
STAND_TALL_KNEE_RAD = 0.5
# Mirrors sim.launch.py's own terrain-conditioned spawn_x/spawn_y/spawn_z
# defaults (src/tap_sim/launch/sim.launch.py) so Launch sim and Reset standup
# agree on where "home" is for the selected terrain. 0.2372 m is the flat
# wheel-ground height (src/tap_sim/CONTRACT.md's 0.23715084313 m); rough's
# (-4.6, 0, 0.4372) is the dedicated test step in rough_training.sdf.
TERRAIN_SPAWN = {
    'none': (0.0, 0.0, 0.2372),
    'rough': (-4.6, 0.0, 0.4372),
}
# Launch sim's Profile dropdown -> config filename under
# src/tap_control/config/ (None = the launch's own default,
# balance_controller.yaml alone). Deliberately a short, curated list of
# GUI-relevant profiles, not every experimental config in that directory.
SIM_PROFILES = {
    'Default': None,
    'Deep squat': 'deep_squat.yaml',
}
RESET_STANDUP_CONFIRM_S = 2.
RESET_STANDUP_CONFIRM_TIMEOUT_S = 15.


class Dashboard:
    def __init__(self, root, node, curves, deep_curves=None, stand_curves=None, experimental_jump=False):
        self.root, self.node, self.curves = root, node, curves
        self.deep_curves = deep_curves
        self.stand_curves = stand_curves
        # Which extreme profile is currently in play ('Deep squat' or
        # 'Standup'); drives status text and return_deep's target radio
        # value so the two share one trajectory/hold state machine below.
        self.deep_mode = 'Deep squat'
        self.updating_controls = False
        self.experimental_jump = experimental_jump
        self.jump_locked = False
        self.jump_fault = False
        self.jump_seen_active = False
        self.jump_reset_pending = False
        self.jump_ready_status = False
        self.jump_last_status = 0.
        self.jump_text = 'Jump unavailable'
        self.jump_reason = ''
        self.jump_pose_pending = False
        self.pose_commands_blocked = False
        self.jump_status_sequence = 0
        self.jump_request_sequence = 0
        self.controller_pose_bounds = None
        self.controller_fresh = False
        self.deep_geometry_ready = False
        self.deep_text = ''
        self.deep_trajectory = None
        self.deep_returning = False
        self.deep_paused = False
        self.deep_next_mode = 'Native joints'
        self.deep_tracking_since = None
        self.operator_zero = False
        self.enabled = False
        self.estopped = False
        self.drive = (0., 0.)
        self.last_tick = time.monotonic()
        self.last_mode = 0.
        self.feedback_grace_until = 0.
        self.show = None
        self.show_epoch = 0.
        self.show_text = 'Show idle'
        self.sim_time = None
        self.clock_wall = 0.
        self.station_ready = False
        self.measured_pose = None
        self.measured_wall = 0.
        self.show_tracking_since = None
        self.phase = 0.
        self.pose = list(curves['rest_pose'])
        self.mode_text = 'No controller feedback'
        self.telemetry_text = 'Pitch: -- | Reference: -- | CoM: -- | Sensors: -- | Speed: --'
        self.motor_text = 'No motor commands'
        self.last_pitch = None
        self.last_pose_z = None
        self.last_contact = None
        # Owned Gazebo subprocess launched via the "Launch sim" button, if any.
        self.sim_process = None
        self.sim_stream = None
        self.sim_log_path = None
        self.sim_started = 0.
        # Sim-only reset-standup state machine; see reset_standup() below.
        self.reset_standup_stage = None
        self.reset_standup_error = ''
        self.reset_standup_confirm_start = None
        self.reset_standup_confirm_deadline = 0.
        self._reset_standup_target = TERRAIN_SPAWN['none']
        # Telemetry tab: rolling ~HISTORY_S-second history for the plots,
        # appended by on_mode (pitch/pitch_rate) and on_joint_states (all six
        # joints' position/velocity/effort) - additive to those callbacks'
        # existing parsing, no change to the state they already compute.
        self.pitch_history = []       # [(t, pitch, pitch_rate), ...]
        self.joint_history = {name: [] for name in ALL_JOINTS}  # name -> [(t,p,v,e), ...]
        self.push_mag = tk.DoubleVar(value=0.6)   # shove impulse [N s]
        # Log tab.
        self.recording = tk.BooleanVar(value=False)
        self.csv_writer = None
        self.csv_file = None
        self._key_release_job = None
        self.pub = {topic: node.create_publisher(msg, '/' + topic, 10)
                    for topic, msg in [('cmd_vel', Twist), ('target_pose', Float64MultiArray),
                                       ('enable', Bool), ('emergency_stop', Bool),
                                       ('stand_up', Bool), ('reset_controller', Bool)]}
        node.create_subscription(String, '/control_mode', self.on_mode, 10)
        node.create_subscription(JointState, '/motor_commands', self.on_motors, 1)
        node.create_subscription(Clock, '/clock', self.on_clock, 1)
        node.create_subscription(JointState, '/joint_states', self.on_joint_states, 1)
        self.pub['jump_request'] = node.create_publisher(Bool, '/jump_request', 10)
        node.create_subscription(String, '/jump_status', self.on_jump_status, 10)
        root.title('TAP robot control')
        root.configure(bg=BG)
        root.geometry('1040x760')
        root.minsize(900, 640)
        root.protocol('WM_DELETE_WINDOW', self.close)
        root.bind('<FocusOut>', lambda event: self.stop_drive())
        root.bind('<ButtonRelease-1>', lambda event: self.stop_drive())
        root.bind('<Escape>', lambda event: self.estop())
        self._style()
        self.nb = ttk.Notebook(root)
        self.nb.pack(fill='both', expand=True, padx=8, pady=8)
        self._tab_control(curves)
        self._tab_pose(curves)
        self._tab_telemetry()
        self._tab_log()
        self._bind_keys()
        self.refresh_controls(time.monotonic())
        root.after(20, self.tick)
        root.after(100, self._redraw_plots)

    def _style(self):
        s = ttk.Style()
        try:
            s.theme_use('clam')
        except tk.TclError:
            pass
        s.configure('.', background=BG, foreground=FG, font=('', 10))
        s.configure('TNotebook', background=BG, borderwidth=0)
        s.configure('TNotebook.Tab', background=CARD, foreground=MUTED,
                   padding=(16, 7), font=('', 10, 'bold'))
        s.map('TNotebook.Tab', background=[('selected', ACCENT)],
             foreground=[('selected', '#0b1220')])
        s.configure('TFrame', background=BG)
        s.configure('Card.TFrame', background=CARD)
        s.configure('TLabel', background=BG, foreground=FG)
        s.configure('Card.TLabel', background=CARD, foreground=FG)
        s.configure('Muted.TLabel', background=BG, foreground=MUTED, font=('', 9))
        s.configure('TButton', background='#2f3440', foreground=FG,
                   borderwidth=0, padding=7)
        s.map('TButton', background=[('active', '#3a404e'), ('pressed', ACCENT)])
        s.configure('Accent.TButton', background=ACCENT, foreground='#0b1220',
                   font=('', 10, 'bold'))
        s.configure('Danger.TButton', background=BAD, foreground='#180a0c',
                   font=('', 10, 'bold'))
        s.configure('Drive.TButton', font=('', 15, 'bold'), padding=10)
        s.configure('Treeview', background=CARD, fieldbackground=CARD,
                   foreground=FG, borderwidth=0, rowheight=24)
        s.configure('Treeview.Heading', background='#2f3440', foreground=MUTED,
                   font=('', 9, 'bold'))

    # ---------------- Control tab ----------------
    def _tab_control(self, curves):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text='Control')
        outer = ttk.Frame(tab, padding=14)
        outer.pack(fill='both', expand=True)

        head = ttk.Frame(outer)
        head.pack(fill='x', pady=(0, 10))
        ttk.Label(head, text='TAP robot control', font=('', 16, 'bold')).pack(side='left')
        self.mode_pill = tk.Label(head, text=' offline ', bg='#333', fg=FG,
                                  font=('', 11, 'bold'), padx=12, pady=4)
        self.mode_pill.pack(side='right')

        simulation = ttk.Frame(outer, style='Card.TFrame', padding=10)
        simulation.pack(fill='x', pady=(0, 10))
        self.sim_button = ttk.Button(simulation, text='Launch sim', style='Accent.TButton',
                                     command=self.toggle_sim)
        self.sim_button.pack(side='left')
        ttk.Label(simulation, text='Terrain', style='Card.TLabel').pack(side='left', padx=(8, 2))
        self.sim_terrain = tk.StringVar(value='none')
        self.sim_terrain_menu = ttk.Combobox(simulation, textvariable=self.sim_terrain,
                                             values=['none', 'rough'], state='readonly', width=6)
        self.sim_terrain_menu.pack(side='left')
        self.sim_terrain_menu.bind('<<ComboboxSelected>>', self.on_terrain_changed)
        # One-click profile select for Launch sim, so a controller_config
        # override (e.g. Deep squat's wider pose_knee_max/hip_max, needed
        # for the Pose tab's Deep squat/Standup buttons to actually reach
        # full depth instead of being silently clamped) no longer requires
        # a separate terminal running `ros2 launch tap_sim sim.launch.py
        # controller_config:=...` alongside the GUI - matches wheel_leg's
        # "press button, it launches" expectation.
        ttk.Label(simulation, text='Profile', style='Card.TLabel').pack(side='left', padx=(8, 2))
        self.sim_profile = tk.StringVar(value='Default')
        self.sim_profile_menu = ttk.Combobox(simulation, textvariable=self.sim_profile,
                                             values=list(SIM_PROFILES), state='readonly', width=10)
        self.sim_profile_menu.pack(side='left')
        # Spawn X/Y/Z: used both as Launch sim's spawn_x/y/z launch args and as
        # Reset standup's teleport target, so both agree on "home" for whatever
        # is currently selected -- editable, so a fall can be recovered from
        # without relaunching even at a custom, non-default position.
        ttk.Label(simulation, text='Spawn X/Y/Z', style='Card.TLabel').pack(side='left', padx=(8, 2))
        self.spawn_x = tk.StringVar()
        self.spawn_y = tk.StringVar()
        self.spawn_z = tk.StringVar()
        for var in (self.spawn_x, self.spawn_y, self.spawn_z):
            ttk.Entry(simulation, textvariable=var, width=6).pack(side='left', padx=1)
        self._apply_terrain_spawn_defaults()
        ttk.Button(simulation, text='Reset upright (sim-only)',
                  command=self.reset_standup).pack(side='left', padx=(8, 2))
        self.sim_status = tk.StringVar(value='Sim not running')
        ttk.Label(simulation, textvariable=self.sim_status, style='Card.TLabel').pack(side='left', padx=8)

        body = ttk.Frame(outer)
        body.pack(fill='both', expand=True)

        left = ttk.Frame(body)
        left.pack(side='left', fill='y', padx=(0, 10))

        drive = ttk.Frame(left, style='Card.TFrame', padding=14)
        drive.pack(fill='x')
        ttk.Label(drive, text='DRIVE  (hold / arrow keys)', style='Card.TLabel').pack(pady=(0, 8))
        pad = ttk.Frame(drive, style='Card.TFrame')
        pad.pack()
        self._dbtn(pad, '▲', 0, 1, 1., 0.)
        self._dbtn(pad, '◄', 1, 0, 0., 1.)
        ttk.Button(pad, text='■', style='Drive.TButton', width=3,
                  command=self.stop_drive).grid(row=1, column=1, padx=4, pady=4)
        self._dbtn(pad, '►', 1, 2, 0., -1.)
        self._dbtn(pad, '▼', 2, 1, -1., 0.)
        speedf = ttk.Frame(drive, style='Card.TFrame')
        speedf.pack(fill='x', pady=(10, 0))
        self.speed = tk.DoubleVar(value=.5)
        self.yaw = tk.DoubleVar(value=.3)
        ttk.Label(speedf, text='Speed (m/s)', style='Card.TLabel').grid(row=0, column=0, sticky='w')
        ttk.Spinbox(speedf, from_=0, to=1., increment=.02, textvariable=self.speed,
                   width=6).grid(row=0, column=1, padx=4)
        ttk.Label(speedf, text='Turn (rad/s)', style='Card.TLabel').grid(row=1, column=0, sticky='w')
        ttk.Spinbox(speedf, from_=0, to=.8, increment=.1, textvariable=self.yaw,
                   width=6).grid(row=1, column=1, padx=4)

        ttk.Separator(drive, orient='horizontal').pack(fill='x', pady=10)
        ttk.Label(drive, text='SHOVE  (click - disturbance test)', style='Card.TLabel').pack(pady=(0, 6))
        ppad = ttk.Frame(drive, style='Card.TFrame')
        ppad.pack()
        self._pbtn(ppad, '↑', 0, 1, 1., 0.)
        self._pbtn(ppad, '←', 1, 0, 0., 1.)
        self._pbtn(ppad, '→', 1, 2, 0., -1.)
        self._pbtn(ppad, '↓', 2, 1, -1., 0.)
        pushf = ttk.Frame(drive, style='Card.TFrame')
        pushf.pack(fill='x', pady=(8, 0))
        ttk.Label(pushf, text='impulse', style='Card.TLabel').pack(side='left')
        ttk.Scale(pushf, from_=.1, to=3., variable=self.push_mag,
                 orient='horizontal').pack(side='left', fill='x', expand=True, padx=6)

        right = ttk.Frame(body)
        right.pack(side='left', fill='both', expand=True)

        rob = ttk.Frame(right, style='Card.TFrame', padding=12)
        rob.pack(fill='x')
        ttk.Label(rob, text='ROBOT', style='Card.TLabel').pack(anchor='w')
        r1 = ttk.Frame(rob, style='Card.TFrame')
        r1.pack(fill='x', pady=(6, 4))
        for title, callback in [('Enable', self.enable), ('Disable', self.disable),
                                ('Clear E-stop', self.clear_estop),
                                ('Reset controller', self.reset)]:
            ttk.Button(r1, text=title, command=callback).pack(side='left', expand=True,
                                                              fill='x', padx=2)
        tk.Button(rob, text='EMERGENCY STOP', bg=BAD, fg='#180a0c', font=('', 10, 'bold'),
                 bd=0, command=self.estop).pack(fill='x', pady=(4, 0))

        jump_controls = ttk.Frame(right, style='Card.TFrame', padding=12)
        jump_controls.pack(fill='x', pady=(10, 0))
        jump_controls.columnconfigure(1, weight=1, minsize=0)
        self.jump_opt_in = tk.BooleanVar(value=self.experimental_jump)
        ttk.Checkbutton(jump_controls, text='Experimental jump (unstable landing)',
                        variable=self.jump_opt_in, command=self.change_jump_opt_in).grid(
                            row=0, column=0, columnspan=2, sticky='w')
        self.jump_button = ttk.Button(jump_controls, text='Jump once', command=self.jump_once)
        self.jump_button.grid(row=1, column=0, sticky='nw')
        self.jump_status_var = tk.StringVar(value=self.jump_text)
        jump_label = ttk.Label(jump_controls, textvariable=self.jump_status_var,
                               style='Card.TLabel', width=1, wraplength=380,
                               justify='left', anchor='nw')
        jump_label.grid(row=1, column=1, sticky='new', padx=6)
        jump_controls.bind('<Configure>', lambda event: self.resize_jump_status(
            event.width, self.jump_button, jump_label))

        state = ttk.Frame(right, style='Card.TFrame', padding=12)
        state.pack(fill='both', expand=True, pady=(10, 0))
        ttk.Label(state, text='STATE', style='Card.TLabel').pack(anchor='w')
        self.status = tk.StringVar(value=self.mode_text)
        self.telemetry = tk.StringVar(value=self.telemetry_text)
        self.motors = tk.StringVar(value=self.motor_text)
        self.reset_standup_status = tk.StringVar(value='')
        self.show_status = tk.StringVar(value=self.show_text)
        for var in (self.status, self.telemetry, self.motors,
                   self.reset_standup_status, self.show_status):
            ttk.Label(state, textvariable=var, style='Card.TLabel',
                     font=('monospace', 9), justify='left', wraplength=420).pack(
                         anchor='w', pady=(4, 0))

    def _dbtn(self, parent, text, r, c, vx, wz):
        b = ttk.Button(parent, text=text, style='Drive.TButton', width=3)
        b.grid(row=r, column=c, padx=4, pady=4)
        b.bind('<ButtonPress-1>', lambda _e: self.start_drive(vx, wz))
        b.bind('<ButtonRelease-1>', lambda _e: self.stop_drive())
        b.bind('<Leave>', lambda _e: self.stop_drive())

    def _pbtn(self, parent, text, r, c, fx, fy):
        b = ttk.Button(parent, text=text, style='Drive.TButton', width=3,
                       command=lambda: self._shove(fx, fy))
        b.grid(row=r, column=c, padx=4, pady=4)

    # ---------------- Pose tab ----------------
    def _tab_pose(self, curves):
        tab = ttk.Frame(self.nb, padding=14)
        self.nb.add(tab, text='Pose')

        modesf = ttk.Frame(tab, style='Card.TFrame', padding=10)
        modesf.pack(fill='x')
        self.workflow = tk.StringVar(value='Native joints')
        mode_names = ['Native joints', 'Squat', 'Roll', 'Wiggle']
        if self.deep_curves is not None:
            mode_names.append('Deep squat')
        if self.stand_curves is not None:
            mode_names.append('Standup')
        for label in mode_names:
            ttk.Radiobutton(modesf, text=label, value=label, variable=self.workflow,
                            command=self.change_workflow).pack(side='left', padx=4)
        self.experimental = tk.BooleanVar(value=False)
        ttk.Checkbutton(tab, text='Experimental motions', variable=self.experimental,
                        command=self.change_workflow).pack(anchor='w', pady=6)

        jointsf = ttk.Frame(tab, style='Card.TFrame', padding=12)
        jointsf.pack(fill='x', pady=(0, 8))
        jointsf.columnconfigure(1, weight=1)
        self.jvars = []
        self.joint_scales = []
        for i, name in enumerate(JOINTS):
            var = tk.DoubleVar(value=self.pose[i])
            self.jvars.append(var)
            var.trace_add('write', lambda *_: self.manual_joint_change())
            ttk.Label(jointsf, text=name.replace('cad_', '') + ' (rad)',
                     style='Card.TLabel').grid(row=i, column=0, sticky='w')
            lower, upper = curves['hip_bounds' if i % 2 == 0 else 'knee_bounds']
            scale = tk.Scale(jointsf, from_=lower, to=upper, resolution=.005,
                             orient='horizontal', variable=var, length=350,
                             bg=CARD, fg=FG, highlightthickness=0, troughcolor=BG)
            scale.grid(row=i, column=1, sticky='ew')
            self.joint_scales.append(scale)

        depthf = ttk.Frame(tab, style='Card.TFrame', padding=12)
        depthf.pack(fill='x', pady=(0, 8))
        self.depth = tk.DoubleVar(value=0.)
        self.depth.trace_add('write', lambda *_: self.depth_change())
        self.depth_label = ttk.Label(depthf, text='Squat / roll amplitude', style='Card.TLabel')
        self.depth_label.pack(anchor='w')
        self.depth_scale = tk.Scale(depthf, from_=-1, to=1, resolution=.01,
                                    orient='horizontal', variable=self.depth,
                                    bg=CARD, fg=FG, highlightthickness=0, troughcolor=BG)
        self.depth_scale.pack(fill='x')

        actions = ttk.Frame(tab, style='Card.TFrame', padding=10)
        actions.pack(fill='x')
        ttk.Button(actions, text='Neutral pose', command=self.neutral).pack(side='left')
        ttk.Button(actions, text='Stand up (experimental)', command=self.stand_up).pack(side='left', padx=8)
        ttk.Button(actions, text='Show', command=self.start_show).pack(side='left', padx=3)
        ttk.Button(actions, text='Stop show', command=self.stop_show).pack(side='left', padx=3)
        if self.deep_curves is not None:
            ttk.Button(actions, text='Hold squat', command=self.stop_deep).pack(side='left', padx=3)
            ttk.Button(actions, text='Deep squat', command=self.go_deep_squat).pack(side='left', padx=3)
        if self.stand_curves is not None:
            ttk.Button(actions, text='Standup', command=self.go_stand_tall).pack(side='left', padx=3)

    # ---------------- Telemetry tab ----------------
    def _tab_telemetry(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text='Telemetry')
        top = ttk.Frame(tab, style='Card.TFrame', padding=6)
        top.pack(fill='x')
        self.jtree = ttk.Treeview(top, columns=('p', 'v', 'e'),
                                  show='tree headings', height=6)
        self.jtree.heading('#0', text='joint')
        self.jtree.heading('p', text='position [rad]')
        self.jtree.heading('v', text='velocity [rad/s]')
        self.jtree.heading('e', text='effort [Nm]')
        self.jtree.column('#0', width=140)
        for c in ('p', 'v', 'e'):
            self.jtree.column(c, width=140, anchor='e')
        for j in ALL_JOINTS:
            self.jtree.insert('', 'end', iid=j, text=j, values=('-', '-', '-'))
        self.jtree.pack(fill='x')
        if HAVE_MPL:
            self.pitch_fig = Figure(figsize=(8, 2.6), dpi=96, facecolor=CARD)
            self.ax_pitch = self.pitch_fig.add_subplot(111)
            self._style_axes(self.ax_pitch, 'pitch [rad] / rate [rad/s]')
            self.l_pitch, = self.ax_pitch.plot([], [], color=GOOD, lw=1.4, label='pitch')
            self.l_pitch_rate, = self.ax_pitch.plot([], [], color=ACCENT, lw=0.9, label='pitch_rate')
            self.ax_pitch.legend(loc='upper left', fontsize=7, ncol=2,
                                 facecolor='#1b1e26', labelcolor=MUTED)
            self.pitch_fig.tight_layout()
            self.pitch_canvas = FigureCanvasTkAgg(self.pitch_fig, master=tab)
            self.pitch_canvas.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(8, 4))

            self.joint_fig = Figure(figsize=(8, 2.6), dpi=96, facecolor=CARD)
            self.ax_joint_p = self.joint_fig.add_subplot(211)
            self.ax_joint_v = self.joint_fig.add_subplot(212, sharex=self.ax_joint_p)
            self._style_axes(self.ax_joint_p, 'pos [rad]')
            self._style_axes(self.ax_joint_v, 'vel [rad/s]')
            cols = ['#4c8dff', '#3ecf8e', '#ffb454', '#c586ff', '#ff5c68', '#66d9e8']
            self.jl_p, self.jl_v = {}, {}
            for i, j in enumerate(ALL_JOINTS):
                self.jl_p[j], = self.ax_joint_p.plot([], [], color=cols[i], lw=1.0, label=j)
                self.jl_v[j], = self.ax_joint_v.plot([], [], color=cols[i], lw=1.0)
            self.ax_joint_p.legend(loc='upper left', fontsize=6, ncol=6,
                                   facecolor='#1b1e26', labelcolor=MUTED)
            self.joint_fig.tight_layout()
            self.joint_canvas = FigureCanvasTkAgg(self.joint_fig, master=tab)
            self.joint_canvas.get_tk_widget().pack(fill='both', expand=True, padx=8, pady=(4, 8))
        else:
            self.pitch_strip = StripChart(tab, [('pitch', GOOD), ('pitch_rate', ACCENT)],
                                          'pitch [rad] / rate [rad/s]', height=160)
            self.pitch_strip.pack(fill='both', expand=True, padx=8, pady=(8, 4))
            ttk.Label(tab, text='matplotlib unavailable (' + MPL_ERR[:80] +
                     ') - using canvas fallback', style='Muted.TLabel').pack()

    @staticmethod
    def _style_axes(ax, ylabel):
        ax.set_facecolor('#1b1e26')
        ax.tick_params(colors=MUTED, labelsize=8)
        ax.grid(True, color='#2b2f3a', lw=0.5)
        ax.set_ylabel(ylabel, color=MUTED, fontsize=8)
        for sp in ax.spines.values():
            sp.set_color('#3a3f4d')

    # ---------------- Log tab ----------------
    def _tab_log(self):
        tab = ttk.Frame(self.nb)
        self.nb.add(tab, text='Log')
        bar = ttk.Frame(tab, style='Card.TFrame', padding=8)
        bar.pack(fill='x')
        ttk.Label(bar, text='CSV file:', style='Card.TLabel').pack(side='left')
        self.csv_var = tk.StringVar(value=os.path.expanduser('~/tap_dashboard_log.csv'))
        ttk.Entry(bar, textvariable=self.csv_var, width=48).pack(side='left', padx=6)
        self.rec_btn = ttk.Button(bar, text='● Record', command=self.toggle_record)
        self.rec_btn.pack(side='left')
        self.rec_lbl = ttk.Label(bar, text='', style='Card.TLabel')
        self.rec_lbl.pack(side='left', padx=8)
        self.logbox = tk.Text(tab, bg=CARD, fg=MUTED, bd=0, font=('monospace', 9), wrap='word')
        self.logbox.pack(fill='both', expand=True, padx=8, pady=8)
        self.logbox.configure(state='disabled')

    def change_jump_opt_in(self):
        self.experimental_jump = bool(self.jump_opt_in.get())

    def on_terrain_changed(self, event=None):
        self._apply_terrain_spawn_defaults()

    def _apply_terrain_spawn_defaults(self):
        x, y, z = TERRAIN_SPAWN.get(self.sim_terrain.get(), TERRAIN_SPAWN['none'])
        self.spawn_x.set(f'{x:g}')
        self.spawn_y.set(f'{y:g}')
        self.spawn_z.set(f'{z:g}')

    def _read_spawn_position(self):
        """Parse the Spawn X/Y/Z fields; raises ValueError with a clear message
        on anything non-finite, shared by launch_sim() and reset_standup() so
        both fail the same way on a bad value instead of silently teleporting
        somewhere wrong."""
        values = []
        for label, var in (('X', self.spawn_x), ('Y', self.spawn_y), ('Z', self.spawn_z)):
            try:
                value = float(var.get())
            except ValueError:
                raise ValueError(f'spawn {label} is not a number')
            if not math.isfinite(value):
                raise ValueError(f'spawn {label} must be finite')
            values.append(value)
        return tuple(values)

    def refresh_controls(self, now):
        mode = self.workflow.get()
        manual = (mode == 'Native joints' and self.deep_trajectory is None
                  and not self.jump_locked and not self.pose_commands_blocked)
        deep = mode in ('Deep squat', 'Standup')
        self.depth_label.configure(text=f'{mode} depth' if deep else 'Squat / roll amplitude')
        # Tk Scale.configure writes its variable even for a state-only change.
        self.updating_controls = True
        try:
            for scale in self.joint_scales:
                state = 'normal' if manual else 'disabled'
                if str(scale.cget('state')) != state:
                    scale.configure(state=state)
            depth_state = 'normal' if (mode != 'Native joints' and not self.jump_locked
                                      and not self.pose_commands_blocked) else 'disabled'
            # Squat (unlike Roll) ignores the slider's sign entirely (see
            # target()'s curve selection below), so a -1..1 range let either
            # side of center produce the same squat - confusing and wastes
            # half the travel. One-sided 0..1 fixes that; Roll/Wiggle still
            # use sign to pick a side, so they keep the full -1..1 range.
            lower = 0 if (deep or mode == 'Squat') else -1
            if (str(self.depth_scale.cget('state')) != depth_state
                    or float(self.depth_scale.cget('from')) != lower):
                self.depth_scale.configure(from_=lower, state=depth_state)
        finally:
            self.updating_controls = False
        ready = (self.experimental_jump and self.experimental.get() and self.enabled
                 and not self.estopped and not self.jump_locked and not self.jump_fault
                 and self.jump_ready_status and now-self.jump_last_status <= 1.
                 and self.controller_fresh and now-self.last_mode <= 1.)
        self.jump_button.configure(state='normal' if ready else 'disabled')
        text = self.jump_text if self.experimental_jump else 'Jump locked: experimental opt-in off'
        if self.experimental_jump and now-self.jump_last_status > 1.:
            text += ' (status stale)'
        self.jump_status_var.set(text)

    @staticmethod
    def resize_jump_status(frame_width, button, label):
        # Text may add rows, but cannot widen the frame or shrink the command.
        label.configure(wraplength=max(1, frame_width - button.winfo_reqwidth() - 12))

    def on_jump_status(self, msg):
        """JumpController phase contract; enabled is feature config, not motors."""
        parts = msg.data.split('|')
        fields = dict(p.split('=', 1) for p in parts if '=' in p)
        phase = fields.get('phase', parts[0]).strip().lower()
        self.jump_last_status = time.monotonic()
        self.jump_status_sequence += 1
        self.jump_text = 'Jump: ' + phase[:40]
        self.jump_reason = fields.get('reason', '')[:40]
        self.jump_pose_pending = fields.get('pose_pending') == '1'
        if self.jump_pose_pending:
            self.jump_text += ' | pose pending'
        elif fields.get('ready') != '1' and self.jump_reason:
            self.jump_text += ' | ' + self.jump_reason
        inactive = phase in ('idle', 'complete') and fields.get('active') == '0'
        self.jump_ready_status = (inactive and fields.get('ready') == '1'
                                  and fields.get('enabled') == '1' and not self.jump_pose_pending)
        if phase in ('fault', 'failed', 'aborted', 'error'):
            self.pose_commands_blocked = True
            self.jump_fault = True
            self.jump_locked = True
            self.enabled = False
            self.drive = (0., 0.)
        elif inactive:
            reason = fields.get('reason', '')
            rejection = reason.startswith('not_ready_') or reason == 'disabled'
            control_eligible = (self.enabled and not self.estopped and self.controller_fresh
                                and self.jump_last_status-self.last_mode <= 1.)
            rejected_pending = (self.jump_locked and not self.jump_seen_active and rejection
                                and not self.jump_ready_status and control_eligible
                                and self.jump_status_sequence > self.jump_request_sequence)
            if self.jump_reset_pending:
                self.pose_commands_blocked = True
                self.jump_reset_pending = False
                self.jump_fault = False
                self.jump_locked = False
                self.jump_seen_active = False
                self.enabled = False
            elif not self.jump_fault and ((phase == 'complete' and self.jump_seen_active)
                                         or rejected_pending):
                if self.jump_seen_active:
                    self.pose_commands_blocked = True
                self.jump_locked = False
                self.jump_seen_active = False
                if rejected_pending:
                    self.jump_text = 'Jump rejected: ' + reason[:32]
        else:
            # Unknown phases fail closed, including when attaching mid-jump.
            self.jump_locked = True
            self.jump_seen_active = True
            self.pose_commands_blocked = True
            self.drive = (0., 0.)
            self.stop_show()
            self.stop_deep()

    def jump_once(self):
        now = time.monotonic()
        def neutral(pose, tolerance):
            return pose is not None and len(pose) == 4 and all(
                math.isfinite(q) and abs(q-r) <= tolerance
                for q, r in zip(pose, self.curves['rest_pose']))
        if (not self.experimental_jump or self.jump_locked or self.jump_fault
                or not self.experimental.get() or not self.enabled or self.estopped
                or not self.jump_ready_status or now-self.jump_last_status > 1.
                or not self.controller_fresh or now-self.last_mode > 1.
                or not self.operator_zero or self.drive != (0., 0.)
                or self.show is not None or self.deep_trajectory is not None
                or self.workflow.get() != 'Native joints'
                or not neutral(self.pose, .02)
                or not neutral([v.get() for v in self.jvars], .02)
                or not neutral(self.measured_pose, .08) or now-self.measured_wall > 1.):
            reason = ('pose pending' if self.jump_pose_pending else
                      'status stale' if now-self.jump_last_status > 1. else
                      self.jump_reason if not self.jump_ready_status and self.jump_reason else
                      'readiness gate')
            self.jump_text = 'Jump blocked: ' + reason
            return
        self.stop_drive()
        self.jump_locked = True
        self.jump_request_sequence = self.jump_status_sequence
        self.pose_commands_blocked = True
        self.jump_seen_active = False
        self.jump_ready_status = False
        self.jump_text = 'Jump requested'
        self.boolean('jump_request', False)
        self.boolean('jump_request', True)

    def on_mode(self, msg):
        parts = msg.data.split('|')
        self.mode_text = parts[0][:32]
        fields = dict(part.split('=', 1) for part in parts[1:] if '=' in part)
        self.controller_fresh = (fields.get('fresh') == '1' and fields.get('estop') == '0'
                                 and self.mode_text in ('Balance', 'Drive', 'PoseChange'))
        try:
            pitch, height = float(fields['pitch']), float(fields['pose_z'])
            self.last_pitch, self.last_pose_z = pitch, height
            self.deep_geometry_ready = (math.isfinite(pitch) and math.isfinite(height)
                                        and abs(pitch) <= .12 and height >= .115)
        except (KeyError, ValueError):
            self.last_pitch = self.last_pose_z = None
            self.deep_geometry_ready = False
        self.last_contact = fields.get('contact')
        try:
            bounds = tuple(float(fields[key]) for key in
                          ('pose_hip_max', 'pose_knee_min', 'pose_hip_min', 'pose_knee_max'))
            self.controller_pose_bounds = bounds if all(map(math.isfinite, bounds)) else None
        except (KeyError, ValueError):
            self.controller_pose_bounds = None
        try:
            commands = [float(fields[key]) for key in ('cmd_v', 'cmd_w')]
            operator_zero = all(math.isfinite(value) and value == 0. for value in commands)
        except (KeyError, ValueError):
            operator_zero = False
        self.operator_zero = operator_zero
        self.station_ready = (fields.get('station') == '1' and fields.get('fresh') == '1'
                              and fields.get('estop') == '0' and operator_zero
                              and self.mode_text in ('Balance', 'Drive', 'PoseChange'))
        if not self.station_ready:
            self.stop_show()
        def number(key, unit):
            try:
                value = float(fields[key])
                return f'{value:.3f} {unit}' if math.isfinite(value) else '--'
            except (KeyError, ValueError):
                return '--'
        fresh = {'1': 'fresh', '0': 'stale'}.get(fields.get('fresh'), '--')
        self.telemetry_text = (f'Pitch: {number("pitch", "rad")} | Ref: {number("pitch_ref", "rad")} | '
                               f'CoM: {number("wb_h", "m")}\n'
                               f'Sensors: {fresh} | Wheel speed: {number("wheel_vel", "m/s")}')
        if self.mode_text in ('Fallen', 'EmergencyStop', 'EStop', 'Disabled') or fields.get('estop') == '1':
            self.enabled = False
            self.stop_drive()
        if fields.get('estop') == '1':
            self.estopped = True
        self.last_mode = time.monotonic()
        self.feedback_grace_until = 0.
        if self.deep_trajectory is not None and not self.deep_ready(self.last_mode):
            self.stop_deep()
        # Telemetry tab plot history - additive, does not affect any of the
        # state/logic above.
        try:
            pitch_v, rate_v = float(fields['pitch']), float(fields.get('pitch_rate', 'nan'))
            if math.isfinite(pitch_v) and math.isfinite(rate_v):
                self._append_history(self.pitch_history, pitch_v, rate_v)
        except (KeyError, ValueError):
            pass
        # Log tab CSV recording - additive, reuses the fields already parsed above.
        if self.recording.get() and self.csv_writer is not None:
            self.csv_writer.writerow([
                time.time(), self.mode_text, fields.get('pitch', ''),
                fields.get('pitch_rate', ''), fields.get('cmd_v', ''),
                fields.get('cmd_w', ''), fields.get('wheel_vel', ''),
                fields.get('contact', ''), fields.get('fresh', '')])

    @staticmethod
    def _append_history(buf, *values):
        now = time.monotonic()
        buf.append((now,) + values)
        cutoff = now - HISTORY_S
        while buf and buf[0][0] < cutoff:
            buf.pop(0)

    def on_motors(self, msg):
        self.motor_text = 'Torque limits (Nm): ' + ', '.join(
            f'{n.replace("cad_", "")[:24]}={v:.2f}' for n, v in list(zip(msg.name, msg.effort))[:6])

    def on_clock(self, msg):
        value = msg.clock.sec + msg.clock.nanosec * 1e-9
        if not math.isfinite(value) or value < 0:
            self.stop_show()
            self.stop_deep()
            self.sim_time = None
            return
        if self.sim_time is not None:
            # Show continuity guard, not a general physical-time guarantee.
            gap = value - self.sim_time
            if gap < 0 or ((self.show is not None or self.deep_trajectory is not None)
                           and gap > .3 + 1e-9):
                self.stop_show()
                self.stop_deep()
        if self.sim_time is None or value != self.sim_time:
            self.clock_wall = time.monotonic()
        self.sim_time = value

    def on_joint_states(self, msg):
        # Telemetry tab: all six joints (incl. wheels), independent of the
        # 4-DOF pose-control validity check below - additive, does not
        # affect self.measured_pose or any pose-control logic.
        has_vel = len(msg.velocity) == len(msg.name)
        has_eff = len(msg.effort) == len(msg.name)
        by_name = {}
        for i, name in enumerate(msg.name):
            if name not in ALL_JOINTS:
                continue
            p = msg.position[i] if i < len(msg.position) else float('nan')
            v = msg.velocity[i] if has_vel else float('nan')
            e = msg.effort[i] if has_eff else float('nan')
            by_name[name] = (p, v, e)
        for name, (p, v, e) in by_name.items():
            if math.isfinite(p):
                self._append_history(self.joint_history[name], p,
                                     v if math.isfinite(v) else 0., e if math.isfinite(e) else 0.)
            if hasattr(self, 'jtree'):
                self.jtree.item(name, values=(
                    f'{p:+.4f}' if math.isfinite(p) else '-',
                    f'{v:+.3f}' if math.isfinite(v) else '-',
                    f'{e:+.3f}' if math.isfinite(e) else '-'))

        positions = dict(zip(msg.name, msg.position))
        valid = (len(msg.name) == len(msg.position) and len(set(msg.name)) == len(msg.name)
                 and all(name in positions and math.isfinite(positions[name]) for name in JOINTS))
        if not valid:
            self.measured_pose = None
            self.stop_show()
            self.stop_deep()
            return
        self.measured_pose = [positions[name] for name in JOINTS]
        self.measured_wall = time.monotonic()

    def show_ready(self, now):
        return (self.enabled and not self.estopped and self.experimental.get()
                and self.station_ready and now - self.last_mode <= 1.
                and self.sim_time is not None and now - self.clock_wall <= 1.
                and self.measured_pose is not None and now - self.measured_wall <= 1.
                and self.drive == (0., 0.))

    def start_show(self):
        if self.jump_locked or self.pose_commands_blocked:
            return
        now = time.monotonic()
        if self.deep_trajectory is not None:
            return
        if self.show is not None:
            return
        neutral = self.curves['rest_pose']
        def near(pose, tolerance):
            return len(pose) == 4 and all(math.isfinite(q) and abs(q-r) <= tolerance
                                          for q, r in zip(pose, neutral))
        measured_ok = (self.measured_pose is not None and now - self.measured_wall <= 1.
                       and near(self.measured_pose, .08))
        target_ok = (self.workflow.get() == 'Native joints'
                     and near([v.get() for v in self.jvars], .02) and near(self.pose, .02))
        if not self.show_ready(now) or not target_ok or not measured_ok:
            self.show_text = 'Show blocked: neutral, station and fresh feedback required'
            return
        self.show = ShowSequence(self.curves, amplitude=.25, transition_s=3., dwell_s=2.)
        self.show_epoch = self.sim_time
        self.show_tracking_since = None
        self.show_text = 'Show: initial_neutral'

    def stop_show(self):
        if self.show is None:
            return
        self.show = None
        self.show_tracking_since = None
        self.show_text = 'Show stopped'
        self.workflow.set('Native joints')
        self.depth.set(0.)
        for var, angle in zip(self.jvars, self.pose):
            var.set(angle)

    def show_target(self, now):
        if self.show is None:
            return None
        if not self.show_ready(now) or self.sim_time < self.show_epoch:
            self.stop_show()
            return list(self.pose)
        if max(abs(q-m) for q, m in zip(self.pose, self.measured_pose)) > .15:
            if self.show_tracking_since is None:
                self.show_tracking_since = self.sim_time
            elif self.sim_time - self.show_tracking_since >= .3 - 1e-9:
                self.stop_show()
                self.show_text = 'Show stopped: joint tracking'
                return list(self.pose)
        else:
            self.show_tracking_since = None
        result = self.show.evaluate(self.sim_time - self.show_epoch)
        self.pose = list(result.targets)
        self.show_text = 'Show: ' + result.stage
        if result.finished:
            self.stop_show()
            self.show_text = 'Show finished'
        return list(self.pose)

    def boolean(self, topic, value):
        self.pub[topic].publish(Bool(data=value))

    def enable(self):
        if self.estopped or self.jump_locked:
            return
        self.stop_drive()
        self.enabled = True
        self.feedback_grace_until = time.monotonic() + 2.
        self.boolean('enable', True)
        self._log('enable')

    def disable(self):
        self.stop_deep()
        self.stop_show()
        self.enabled = False
        self.feedback_grace_until = 0.
        self.stop_drive()
        self.boolean('enable', False)
        self._log('disable')

    def estop(self):
        self.estopped = True
        self.boolean('emergency_stop', True)
        self._log('EMERGENCY STOP')
        self.disable()

    def clear_estop(self):
        self.disable()
        self.boolean('emergency_stop', False)
        self.estopped = False
        self._log('clear e-stop')

    def reset(self):
        if self.jump_locked:
            if not self.jump_fault and self.enabled:
                return
            self.pose_commands_blocked = True
            self.disable()
            self.jump_reset_pending = True
            self.boolean('reset_controller', True)
            self._log('reset controller (jump)')
            return
        self.disable()
        self.pose_commands_blocked = True
        self.boolean('reset_controller', True)
        self._log('reset controller')

    def stand_up(self):
        if self.jump_locked or self.pose_commands_blocked:
            return
        if self.deep_trajectory is not None:
            self.stop_deep()
            return
        self.stop_deep()
        self.stop_show()
        if self.experimental.get() and not self.estopped:
            self.stop_drive()
            self.enabled = True
            self.feedback_grace_until = time.monotonic() + 2.
            self.boolean('stand_up', True)
            self._log('stand-up requested')

    def start_drive(self, forward, yaw):
        if self.jump_locked:
            return
        if self.deep_trajectory is not None:
            self.stop_deep()
            self.stop_drive()
            return
        self.stop_deep()
        self.stop_show()
        if not self.enabled or self.estopped:
            return
        try:
            speed, turn = float(self.speed.get()), float(self.yaw.get())
            if not all(math.isfinite(v) for v in (speed, turn)):
                return
            self.drive = (forward * max(0., min(1., speed)), yaw * max(0., min(.8, turn)))
        except (ValueError, tk.TclError):
            self.stop_drive()

    def stop_drive(self):
        self.drive = (0., 0.)
        if self.jump_locked:
            return
        self.pub['cmd_vel'].publish(Twist())

    # ---------------- arrow-key drive (additive to the D-pad buttons) ----------------
    def _bind_keys(self):
        keymap = {'Up': (1., 0.), 'Down': (-1., 0.), 'Left': (0., 1.), 'Right': (0., -1.)}
        for key, (forward, yaw) in keymap.items():
            self.root.bind(f'<KeyPress-{key}>',
                           lambda _e, f=forward, y=yaw: self._key_drive(f, y))
            self.root.bind(f'<KeyRelease-{key}>', lambda _e: self._key_release())
        self.root.bind('<space>', lambda _e: self.stop_drive())

    def _key_drive(self, forward, yaw):
        if self._key_release_job is not None:
            self.root.after_cancel(self._key_release_job)
            self._key_release_job = None
        self.start_drive(forward, yaw)

    def _key_release(self):
        # A held key auto-repeats press/release; delay the actual stop so a
        # normal inter-repeat gap doesn't chatter-stop the drive command.
        if self._key_release_job is not None:
            self.root.after_cancel(self._key_release_job)
        self._key_release_job = self.root.after(150, self.stop_drive)

    # ---------------- shove (simulation-only disturbance test) ----------------
    def _shove(self, fx, fy):
        if self.sim_process is None or self.sim_process.poll() is not None:
            self._log('shove ignored: sim not running')
            return
        impulse = float(self.push_mag.get())
        force = impulse / .001
        direction = {(1, 0): 'fwd', (-1, 0): 'back', (0, 1): 'left', (0, -1): 'right'}
        self._log(f'shove {direction.get((int(fx), int(fy)), "?")}  {impulse:.2f} N s')
        threading.Thread(target=self._shove_thread, args=(fx * force, fy * force),
                         daemon=True).start()

    def _shove_thread(self, force_x, force_y):
        try:
            gz_world.wrench('tap_robot', force_x, force_y)
        except Exception as exc:  # noqa: BLE001 - subprocess/CLI failure, reported not raised
            self.root.after(0, lambda: self._log(f'shove failed: {exc}'))

    # ---------------- Log tab ----------------
    def _log(self, message):
        if not hasattr(self, 'logbox'):
            return
        self.logbox.configure(state='normal')
        self.logbox.insert('end', f'{time.strftime("%H:%M:%S")}  {message}\n')
        self.logbox.see('end')
        self.logbox.configure(state='disabled')

    def toggle_record(self):
        if self.recording.get():
            self.recording.set(False)
            if self.csv_file is not None:
                self.csv_file.close()
            self.csv_file = None
            self.csv_writer = None
            self.rec_btn.configure(text='● Record')
            self.rec_lbl.configure(text='')
            self._log('CSV recording stopped')
            return
        path = self.csv_var.get()
        try:
            self.csv_file = open(path, 'w', newline='')
        except OSError as exc:
            self._log(f'CSV open failed: {exc}')
            return
        self.csv_writer = csv.writer(self.csv_file)
        self.csv_writer.writerow(['wall_time', 'mode', 'pitch', 'pitch_rate',
                                  'cmd_v', 'cmd_w', 'wheel_vel', 'contact', 'fresh'])
        self.recording.set(True)
        self.rec_btn.configure(text='■ Stop')
        self.rec_lbl.configure(text=f'recording -> {path}')
        self._log(f'CSV recording started: {path}')

    # ---------------- Telemetry tab plots ----------------
    def _redraw_plots(self):
        # Only draw what is actually on screen - a matplotlib canvas redraw
        # is the single most expensive thing this GUI does (see wheel_leg's
        # own _redraw_plots, the pattern this mirrors).
        try:
            if HAVE_MPL and self.nb.tab(self.nb.select(), 'text') == 'Telemetry':
                self._draw_pitch_plot()
                self._draw_joint_plots()
        except Exception:  # noqa: BLE001 - a plot glitch must not kill the GUI loop
            pass
        self.root.after(100, self._redraw_plots)

    def _draw_pitch_plot(self):
        if not self.pitch_history:
            return
        ts = [row[0] for row in self.pitch_history]
        t1 = ts[-1]
        self.l_pitch.set_data([t - t1 for t in ts], [row[1] for row in self.pitch_history])
        self.l_pitch_rate.set_data([t - t1 for t in ts], [row[2] for row in self.pitch_history])
        self.ax_pitch.set_xlim(-HISTORY_S, 0)
        values = [row[1] for row in self.pitch_history] + [row[2] for row in self.pitch_history]
        if values:
            lo, hi = min(values), max(values)
            pad = max(.05, (hi - lo) * .1)
            self.ax_pitch.set_ylim(lo - pad, hi + pad)
        self.pitch_canvas.draw_idle()

    def _draw_joint_plots(self):
        any_data = False
        for name in ALL_JOINTS:
            rows = self.joint_history.get(name, [])
            if not rows:
                continue
            any_data = True
            t1 = rows[-1][0]
            xs = [row[0] - t1 for row in rows]
            self.jl_p[name].set_data(xs, [row[1] for row in rows])
            self.jl_v[name].set_data(xs, [row[2] for row in rows])
        if not any_data:
            return
        self.ax_joint_p.set_xlim(-HISTORY_S, 0)
        self.ax_joint_v.set_xlim(-HISTORY_S, 0)
        self.ax_joint_p.relim()
        self.ax_joint_p.autoscale_view(scalex=False)
        self.ax_joint_v.relim()
        self.ax_joint_v.autoscale_view(scalex=False)
        self.joint_canvas.draw_idle()

    def neutral(self):
        if self.jump_locked:
            return
        if self.pose_commands_blocked:
            now = time.monotonic()
            measured = self.measured_pose
            bounds = [self.curves['hip_bounds'], self.curves['knee_bounds']] * 2
            if (measured is None or len(measured) != 4 or now-self.measured_wall > 1.
                    or any(not math.isfinite(q) or not lo <= q <= hi
                           for q, (lo, hi) in zip(measured, bounds))):
                self.show_text = 'Neutral blocked: measured pose'
                return
            # Reset owns measured posture, not the former GUI trajectory.
            self.pose = list(measured)
            self.deep_trajectory = None
            self.deep_returning = False
            self.deep_paused = False
            self.show = None
            self.pose_commands_blocked = False
        if self.deep_trajectory is not None:
            self.return_deep('Native joints')
            return
        self.stop_deep()
        self.stop_show()
        self.workflow.set('Native joints')
        self.depth.set(0.)
        for var, angle in zip(self.jvars, self.curves['rest_pose']):
            var.set(angle)

    def change_workflow(self):
        if self.jump_locked:
            return
        selected = self.workflow.get()
        if self.deep_trajectory is not None:
            self.return_deep(selected if selected not in ('Deep squat', 'Standup') else 'Native joints')
            return
        self.stop_deep()
        self.stop_show()
        self.workflow.set(selected)
        self.phase = 0.
        self.depth.set(0.)
        self.stop_drive()
        if not self.experimental.get():
            self.neutral()
        elif selected in ('Deep squat', 'Standup'):
            self.deep_mode = selected
            self.start_deep()

    def _deep_profile(self):
        return self.deep_curves if self.deep_mode == 'Deep squat' else self.stand_curves

    def stop_deep(self):
        if self.deep_trajectory is not None:
            self.deep_paused = True
            self.deep_tracking_since = None
            self.deep_text = f'{self.deep_mode} held'

    def return_deep(self, next_mode):
        self.stop_drive()
        self.deep_returning = True
        self.deep_next_mode = next_mode
        self.workflow.set(self.deep_mode)
        self.deep_paused = not self.deep_ready(time.monotonic())
        self.deep_last_sim_time = self.sim_time
        self.deep_tracking_since = None
        self.deep_text = f'{self.deep_mode} held' if self.deep_paused else f'{self.deep_mode} returning'

    def manual_joint_change(self):
        if self.updating_controls:
            return
        self.stop_show()
        self.stop_deep()

    def depth_change(self):
        if not self.updating_controls:
            self.stop_show()

    def start_deep(self):
        if self.jump_locked or self.pose_commands_blocked:
            return
        if self.deep_trajectory is not None:
            return
        profile = self._deep_profile()
        if profile is None:
            return
        neutral = self.curves['rest_pose']
        def near(values, tolerance):
            return values is not None and len(values) == 4 and all(
                math.isfinite(q) and abs(q-r) <= tolerance for q, r in zip(values, neutral))
        if (not self.deep_ready(time.monotonic()) or not near(self.pose, .02)
                or not near([v.get() for v in self.jvars], .02)
                or not near(self.measured_pose, .08)):
            self.deep_text = f'{self.deep_mode} blocked: neutral and profile required'
            return
        depth = max(abs(q[i]) for q in profile['table'] for i in (1, 3))
        minimum_time = (DEEP_SQUAT_FULL_DEPTH_MIN_TRANSITION_S
                        if depth >= DEEP_SQUAT_GEOMETRY_CEILING_RAD - 1e-9 else 0.)
        self.deep_trajectory = DeepSquatTrajectory(
            profile, joint_speed=.3, minimum_transition_s=minimum_time)
        self.deep_last_sim_time = self.sim_time
        self.deep_tracking_since = None
        self.deep_returning = False
        self.deep_paused = False
        self.deep_text = ''

    def go_deep_squat(self):
        """One-click: go straight to the full tested squat depth and hold -
        no slider drag and no automatic return afterward (only an explicit
        Neutral pose / workflow switch animates back)."""
        if self.deep_curves is None or self.jump_locked or self.pose_commands_blocked:
            return
        self.experimental.set(True)
        self.workflow.set('Deep squat')
        self.deep_mode = 'Deep squat'
        self.change_workflow()
        self.depth.set(1.)

    def go_stand_tall(self):
        """One-click: go straight to the tallest tested standup extension
        and hold - see go_deep_squat's docstring for the same no-auto-return
        behavior, mirrored here for the opposite (extension) direction."""
        if self.stand_curves is None or self.jump_locked or self.pose_commands_blocked:
            return
        self.experimental.set(True)
        self.workflow.set('Standup')
        self.deep_mode = 'Standup'
        self.change_workflow()
        self.depth.set(1.)

    def target(self, dt):
        mode = self.workflow.get()
        if self.deep_trajectory is not None:
            if self.deep_paused:
                return list(self.pose)
            if not self.deep_ready(time.monotonic()):
                self.stop_deep()
                return list(self.pose)
            sim_dt = self.sim_time - self.deep_last_sim_time
            self.deep_last_sim_time = self.sim_time
            if sim_dt < 0 or sim_dt > .3 + 1e-9:
                self.stop_deep()
                return list(self.pose)
            if max(abs(q-m) for q, m in zip(self.pose, self.measured_pose)) > .15:
                if self.deep_tracking_since is None:
                    self.deep_tracking_since = self.sim_time
                elif self.sim_time - self.deep_tracking_since >= .3 - 1e-9:
                    self.stop_deep()
                    self.deep_text = f'{self.deep_mode} held: joint tracking'
                    return list(self.pose)
            else:
                self.deep_tracking_since = None
            amount = 0. if self.deep_returning else abs(float(self.depth.get()))
            if not math.isfinite(amount):
                self.stop_deep()
                return list(self.pose)
            self.pose = self.deep_trajectory.advance(min(1., amount), sim_dt)
            if self.deep_returning and self.deep_trajectory.amplitude == 0.:
                self.deep_trajectory = None
                self.deep_returning = False
                self.deep_paused = False
                self.depth.set(0.)
                self.phase = 0.
                # Only neutral values are copied to the narrower manual scales.
                for var, angle in zip(self.jvars, self.pose):
                    var.set(angle)
                self.workflow.set(self.deep_next_mode)
                self.deep_text = f'{self.deep_mode} neutral'
            return list(self.pose)
        elif mode in ('Deep squat', 'Standup'):
            return list(self.pose)
        elif mode == 'Native joints':
            target = [v.get() for v in self.jvars]
            # Until explicitly opted in, keep manual motions close to neutral.
            if not self.experimental.get():
                target = [max(r - .15, min(r + .15, q))
                          for q, r in zip(target, self.curves['rest_pose'])]
        elif not self.experimental.get():
            target = self.curves['rest_pose']
        else:
            amount = self.depth.get()
            if mode == 'Wiggle':
                self.phase += dt * 2 * math.pi * .15
                amount *= math.sin(self.phase)
            if mode == 'Squat' and self.deep_curves is not None:
                # Reach the same tested full depth as the Deep squat button
                # (docs/DEEP_SQUAT.md), not the shallow -0.8 rad 'squat'
                # table other callers (Stationary Show, etc.) still use.
                # Safe under the plain per-tick rate limiter below: it caps
                # joint speed at the same 0.3 rad/s DEEP_SQUAT.md itself
                # validated against for this exact depth range.
                target = sample({'tables': {'deep': self.deep_curves['table']}}, 'deep', abs(amount))
            else:
                curve = 'squat' if mode == 'Squat' else ('roll_left' if amount >= 0 else 'roll_right')
                target = sample(self.curves, curve, abs(amount))
        # Rate limiting also prevents discontinuities when changing workflows.
        step = .3 * dt
        self.pose = [p + max(-step, min(step, q - p)) for p, q in zip(self.pose, target)]
        return self.pose

    def deep_ready(self, now):
        profile = self._deep_profile()
        if (profile is None or (not self.experimental.get() and not self.deep_returning)
                or not self.enabled
                or self.estopped or not self.controller_fresh or now - self.last_mode > 1.
                or not self.station_ready or not self.deep_geometry_ready
                or self.controller_pose_bounds is None or not self.operator_zero
                or self.drive != (0., 0.) or self.measured_pose is None
                or len(self.measured_pose) != 4 or not all(map(math.isfinite, self.measured_pose))
                or now - self.measured_wall > 1. or self.sim_time is None
                or now - self.clock_wall > 1.):
            return False
        hip_max, knee_min, hip_min, knee_max = self.controller_pose_bounds
        return all(hip_min <= q[0] <= hip_max and hip_min <= q[2] <= hip_max
                   and knee_min <= q[1] <= knee_max and knee_min <= q[3] <= knee_max
                   for q in profile['table'])

    # ---------------- simulation lifecycle (Launch sim / Stop sim) ----------------

    def toggle_sim(self):
        if self.sim_process is None:
            self.launch_sim()
        else:
            self.stop_sim()

    def launch_sim(self):
        if self.sim_process is not None:
            return
        # count_publishers, not get_topic_names_and_types: this node's own
        # /clock SUBSCRIPTION (created in __init__) makes /clock appear in
        # get_topic_names_and_types()'s graph listing unconditionally, even
        # with zero publishers -- that check always "found" /clock and
        # permanently blocked Launch sim on every attempt. count_publishers
        # only counts actual publishers, i.e. a real running sim.
        if self.node.count_publishers('/clock') > 0:
            self.sim_status.set('Launch blocked: a ROS graph with /clock already exists on '
                                'this domain; stop that TAP session before launching another')
            return
        # auto_enable:=true so activate_controllers publishes /enable=true
        # itself once controllers are ready -- launching from the GUI should
        # not need a separate manual Enable click afterward (see _tick_sim's
        # matching self.enabled sync on the "Running" milestone).
        extra_args = ['auto_enable:=true']
        if self.sim_terrain.get() != 'none':
            extra_args.append(f'terrain:={self.sim_terrain.get()}')
        profile_file = SIM_PROFILES.get(self.sim_profile.get())
        if profile_file is not None:
            config_path = Path(get_package_share_directory('tap_control')) / 'config' / profile_file
            extra_args.append(f'controller_config:={config_path}')
        try:
            sx, sy, sz = self._read_spawn_position()
        except ValueError as exc:
            self.sim_status.set(f'Launch blocked: {exc}')
            return
        extra_args += [f'spawn_x:={sx:g}', f'spawn_y:={sy:g}', f'spawn_z:={sz:g}']
        try:
            self.sim_process, self.sim_log_path, self.sim_stream = sim_process.launch_sim(extra_args)
        except Exception as exc:
            self.sim_status.set(f'Launch failed: {exc}')
            return
        self.sim_started = time.monotonic()
        self.sim_button.configure(text='Stop sim')
        self.sim_terrain_menu.configure(state='disabled')
        self.sim_profile_menu.configure(state='disabled')
        self.sim_status.set('Starting Gazebo...')

    def stop_sim(self):
        if self.sim_process is None:
            return
        try:
            sim_process.stop_sim(self.sim_process, self.sim_stream)
            self.sim_status.set('Sim stopped')
        except Exception as exc:
            self.sim_status.set(f'Stop sim: {exc}')
        finally:
            self.sim_process = None
            self.sim_stream = None
            self.sim_log_path = None
            self.sim_button.configure(text='Launch sim')
            self.sim_terrain_menu.configure(state='readonly')
            self.sim_profile_menu.configure(state='readonly')

    def _tick_sim(self, now):
        if self.sim_process is None:
            return
        if self.sim_process.poll() is not None:
            self.sim_status.set('Sim exited unexpectedly; see log/gui_sim_*/sim.log')
            self.sim_stream.close()
            self.sim_process = None
            self.sim_stream = None
            self.sim_log_path = None
            self.sim_button.configure(text='Launch sim')
            self.sim_terrain_menu.configure(state='readonly')
            self.sim_profile_menu.configure(state='readonly')
            return
        stage_text = sim_process.sim_stage(self.sim_log_path, now - self.sim_started)
        self.sim_status.set(stage_text)
        # Launch sim always requests auto_enable:=true (see launch_sim()), so
        # activate_controllers publishes /enable=true itself once controllers
        # are ready -- sync the GUI's own enabled/estopped bookkeeping to
        # match once that "Running" milestone is observed, so Drive and the
        # other enabled-gated controls work immediately without a redundant
        # manual Enable click.
        if not self.enabled and stage_text.startswith('Running'):
            self.enabled = True
            self.estopped = False
            self.feedback_grace_until = now + 2.

    # ---------------- reset standup (sim-only test convenience) ----------------

    def reset_standup(self):
        now = time.monotonic()
        if (not self.experimental.get() or self.jump_locked or self.deep_trajectory is not None
                or self.show is not None
                or self.reset_standup_stage not in (None, 'done', 'failed')
                or self.sim_time is None or now - self.clock_wall > 1.):
            self.reset_standup_status.set(
                'Reset upright blocked: enable Experimental motions, ensure a fresh sim '
                'clock, and no jump/show/deep-squat in progress')
            return
        try:
            # Read on the main thread (Tk widgets are not thread-safe); the
            # background phase below only touches the plain tuple result.
            self._reset_standup_target = self._read_spawn_position()
        except ValueError as exc:
            self.reset_standup_status.set(f'Reset upright blocked: {exc}')
            return
        self.stop_drive()
        self.stop_show()
        self.stop_deep()
        self.reset_standup_stage = 'teleporting'
        self.reset_standup_error = ''
        self.reset_standup_status.set(
            '[SIM-ONLY] reset upright: pause + teleport to spawn pose '
            '(test convenience only, not evidence of physical recovery)')
        threading.Thread(target=self._reset_standup_gz_phase, daemon=True).start()

    def _reset_standup_gz_phase(self):
        """Background thread: only blocking gz_world calls, one raw publish, and
        reads of plain attributes written by on_mode() on the main thread. Never
        calls rclpy.spin_once, touches Tk widgets, or calls enable()/reset()/
        neutral()/stand_up() — those stay main-thread-only, driven by
        _advance_reset_standup() from tick().

        A plain set_pose only moves the base link, so the legs keep whatever
        angles they had when the robot fell. An earlier version of this
        unpaused immediately after teleporting and left re-enabling the
        controller (reset_controller -> clear estop -> wait for a fresh
        neutral pose -> enable) to the main-thread stage machine below,
        driven one Tk tick at a time. Measured problem: while physics runs
        in real time across however many ticks that hand-off takes, the
        controller is still in EmergencyStop/Disabled (zero leg torque,
        see BalanceControllerNode's writeSafeStop), so an inherently
        unstable two-wheel balancer can tip over during the wait -- even
        when it was already standing fine and the "reset" should have been
        a no-op (measured: pitch grew from 0.02 to 0.5 rad and the sequence
        ended in EmergencyStop instead of confirmed Balance).

        The wheel_leg reference GUI avoids exactly this by never resuming
        real-time physics until AFTER it has already published
        reset_controller / cleared estop / enabled: it does all of that
        while the world is still paused (via bounded `multi_step` calls,
        each immediately re-paused), so the controller is already
        transitioning toward Balance before the base can fall under real
        gravity. This mirrors that: reset_controller, estop-clear and
        enable are published directly here (plain Bool publishes are
        thread-safe; this does not touch Tk) BEFORE unpausing, not after.

        Critically, `_advance_reset_standup` below must NOT then repeat the
        higher-level reset()/clear_estop() calls it used to run first: both
        call disable() internally, which publishes /enable=false and (per
        BalanceControllerNode's `enable` subscription) sets
        command_.request_disable=true, immediately re-entering EmergencyStop
        -- undoing the enable this phase just fought to establish before
        physics ever ran. Measured: without this fix the two phases visibly
        fought each other (Balance -> Disabled -> Balance -> EmergencyStop
        within ~400 ms, ending disabled) even though this phase's own
        enable had briefly succeeded. So the stage set here skips straight
        past that redundant reset/clear-estop/neutral bookkeeping to
        'confirming' -- see its handling below.
        """
        try:
            x, y, z = self._reset_standup_target
            gz_world.pause(True)
            self.pub['emergency_stop'].publish(Bool(data=True))
            gz_world.set_pose('tap_robot', x, y, z, w=1.)
            self.pub['reset_controller'].publish(Bool(data=True))
            self.pub['emergency_stop'].publish(Bool(data=False))
            self.pub['enable'].publish(Bool(data=True))
            gz_world.pause(False)
            self.reset_standup_stage = 'confirming_sync'
        except Exception as exc:
            try:
                gz_world.pause(False)
            except Exception:
                pass
            self.reset_standup_error = str(exc)
            self.reset_standup_stage = 'failed'

    def _advance_reset_standup(self, now):
        """Main-thread state machine, driven from tick(); only calls the
        dashboard's existing button methods, so it reuses their safety gates
        (pose_commands_blocked in particular) rather than bypassing them.
        """
        stage = self.reset_standup_stage
        if stage is None:
            return
        if stage == 'failed':
            self.reset_standup_status.set(f'[SIM-ONLY] reset upright FAILED: {self.reset_standup_error}')
            self.reset_standup_stage = None
        elif stage == 'confirming_sync':
            # The background gz phase already published reset_controller,
            # cleared estop and enabled directly, before unpausing (see
            # _reset_standup_gz_phase) -- teleporting always sets an upright
            # orientation, so the controller's EmergencyStop/Fallen modes
            # exit on that plain /enable=true regardless of whether this was
            # a trivial already-upright reset or a real fall. This stage only
            # syncs the GUI's own bookkeeping to match; it must NOT call
            # reset()/clear_estop() again, since both call disable()
            # internally (-> /enable=false -> command_.request_disable=true
            # -> immediate EmergencyStop), which measurably fought the
            # enable this sequence just fought to establish.
            self.enabled = True
            self.estopped = False
            self.feedback_grace_until = now + 2.
            self.reset_standup_confirm_start = None
            self.reset_standup_confirm_deadline = now + RESET_STANDUP_CONFIRM_TIMEOUT_S
            self.reset_standup_stage = 'confirming'
            self.reset_standup_status.set('[SIM-ONLY] reset upright: confirming balance...')
            self.reset_standup_status.set('[SIM-ONLY] reset upright: confirming balance...')
        elif stage == 'confirming':
            upright = (self.mode_text == 'Balance' and self.last_pitch is not None
                      and abs(self.last_pitch) < .2 and now - self.last_mode <= 1.)
            if upright:
                self.reset_standup_confirm_start = self.reset_standup_confirm_start or now
                if now - self.reset_standup_confirm_start >= RESET_STANDUP_CONFIRM_S:
                    self.reset_standup_status.set(
                        '[SIM-ONLY] reset upright done (simulation test convenience, '
                        'not evidence of physical recovery)')
                    self.reset_standup_stage = 'done'
            else:
                self.reset_standup_confirm_start = None
                if now > self.reset_standup_confirm_deadline:
                    self.reset_standup_error = f'timed out waiting for Balance mode (last: {self.mode_text})'
                    self.reset_standup_stage = 'failed'

    def tick(self):
        if not rclpy.ok():
            self.root.destroy()
            return
        # Drain bursts without letting sensor traffic monopolize the Tk thread.
        feedback_deadline = time.monotonic() + .004
        for _ in range(32):
            rclpy.spin_once(self.node, timeout_sec=0.)
            if time.monotonic() >= feedback_deadline:
                break
        now = time.monotonic()
        self._tick_sim(now)
        self._advance_reset_standup(now)
        elapsed = now - self.last_tick
        self.last_tick = now
        # No independent publisher thread: a frozen UI stops the keepalive.
        feedback_expired = now - self.last_mode > 1. and now >= self.feedback_grace_until
        if self.enabled and not self.jump_locked and (elapsed > .3 or feedback_expired):
            self.disable()
        dt = min(elapsed, .1)
        command = Twist()
        if self.enabled and not self.estopped:
            command.linear.x, command.angular.z = self.drive
        if not self.jump_locked:
            self.pub['cmd_vel'].publish(command)
        if self.enabled and not self.jump_locked and not self.pose_commands_blocked:
            show_pose = self.show_target(now)
            self.pub['target_pose'].publish(Float64MultiArray(
                data=show_pose if show_pose is not None else self.target(dt)))
        age = now - self.last_mode
        self.status.set(f'{self.mode_text} | {"E-STOP" if self.estopped else "Drive armed" if self.enabled else "Drive locked"}'
                        + (' | Feedback stale' if age > 1 else ''))
        self.motors.set(self.motor_text)
        self.telemetry.set(self.telemetry_text)
        self.show_status.set(self.deep_text if self.workflow.get() in ('Deep squat', 'Standup') else self.show_text)
        if hasattr(self, 'mode_pill'):
            cmap = {'Balance': GOOD, 'Drive': GOOD, 'PoseChange': WARN, 'Enabling': WARN,
                   'Fallen': BAD, 'EmergencyStop': BAD, 'Disabled': MUTED}
            running = age <= 1.
            if not running:
                self.mode_pill.configure(text=' offline ', bg='#333')
            else:
                self.mode_pill.configure(text=f' {self.mode_text} ',
                                         bg=cmap.get(self.mode_text, '#333'))
        self.refresh_controls(now)
        self.root.after(20, self.tick)

    def close(self):
        if not self.jump_locked:
            self.disable()
        if self.sim_process is not None:
            self.stop_sim()
        if self.csv_file is not None:
            self.csv_file.close()
        self.root.destroy()


def parse_options(argv=None):
    parser = argparse.ArgumentParser(description='TAP robot dashboard')
    # Both generated by default (2026-09-16): Deep squat is DEEP_SQUAT.md's
    # own fully tested/promoted candidate, not an unverified depth, so there
    # is no more reason to gate it behind a flag than Standup already isn't.
    # A one-click "Launch sim" + Deep squat/Standup workflow needs both
    # present without the user having to remember a CLI flag.
    parser.add_argument('--deep-squat-knee', type=float, default=DEEP_SQUAT_CANDIDATE_MAX_RAD)
    parser.add_argument('--no-deep-squat', action='store_true',
                        help='Disable the Deep squat button/mode entirely')
    parser.add_argument('--stand-tall-knee', type=float, default=STAND_TALL_KNEE_RAD)
    parser.add_argument('--no-stand-tall', action='store_true',
                        help='Disable the Standup button/mode entirely')
    parser.add_argument('--experimental-jump', action='store_true')
    options, ros_args = parser.parse_known_args(argv)
    if not options.no_deep_squat and (
            not math.isfinite(options.deep_squat_knee)
            or not 0 < options.deep_squat_knee <= DEEP_SQUAT_CANDIDATE_MAX_RAD):
        parser.error('--deep-squat-knee must be finite and in '
                     f'(0, {DEEP_SQUAT_CANDIDATE_MAX_RAD:g}] native CAD radians; '
                     'larger depths are not supported by the current GUI candidate profile')
    if not options.no_stand_tall and (
            not math.isfinite(options.stand_tall_knee) or options.stand_tall_knee <= 0):
        parser.error('--stand-tall-knee must be finite and positive')
    return options, ros_args


def main():
    options, ros_args = parse_options()
    root = tk.Tk()
    root.withdraw()
    try:
        # Regenerate from the installed description so a mass/mesh edit cannot
        # silently leave a stale table active in the dashboard.
        description = Path(get_package_share_directory('tap_robot_description'))
        curves = generate(description / 'description/cad_robot/robot.urdf.xacro')
        deep_curves = None
        if not options.no_deep_squat:
            deep_curves = generate_deep_squat(
                description / 'description/cad_robot/robot.urdf.xacro',
                knee_max=options.deep_squat_knee, hip_bounds=DEEP_SQUAT_GENERATION_HIP_BOUNDS)
        stand_curves = None
        if not options.no_stand_tall:
            stand_curves = generate_stand_tall(
                description / 'description/cad_robot/robot.urdf.xacro',
                knee_max=options.stand_tall_knee, hip_bounds=DEEP_SQUAT_GENERATION_HIP_BOUNDS)
        rclpy.init(args=ros_args)
        node = Node('tap_dashboard')
        Dashboard(root, node, curves, deep_curves=deep_curves, stand_curves=stand_curves,
                 experimental_jump=options.experimental_jump)
        root.deiconify()
        try:
            root.mainloop()
        finally:
            node.destroy_node()
            rclpy.shutdown()
    except Exception as exc:
        messagebox.showerror('TAP dashboard', str(exc), parent=root)
        raise


if __name__ == '__main__':
    main()
