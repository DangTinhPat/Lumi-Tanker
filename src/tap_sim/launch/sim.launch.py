from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sim = FindPackageShare('tap_sim')
    description = FindPackageShare('tap_robot_description')
    gui = LaunchConfiguration('gui')
    jump_model = LaunchConfiguration('jump_model')
    measure_contacts = LaunchConfiguration('measure_contacts')
    terrain = LaunchConfiguration('terrain')
    # jump_model takes priority: rough_training.sdf/ledge_test.sdf have no jump
    # contact sensors/plugin.
    world = PathJoinSubstitution([sim, 'worlds', PythonExpression([
        "'jump.sdf' if '", jump_model, "' == 'true' else "
        "('rough_training.sdf' if '", terrain, "' == 'rough' else "
        "('ledge_test.sdf' if '", terrain, "' == 'ledge' else 'empty.sdf'))"])])
    gui_config = PathJoinSubstitution([sim, 'config', 'gui.config'])
    xacro = PathJoinSubstitution([description, 'description', 'cad_robot', 'simulation.urdf.xacro'])
    base_controller_config = PathJoinSubstitution([
        FindPackageShare('tap_control'), 'config', 'balance_controller.yaml'])
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('ros_gz_sim'), '/launch/gz_sim.launch.py']),
        launch_arguments={'gz_args': PythonExpression([
            "'-v 3 -s ", world, "' if '", gui, "' == 'false' else '-v 3 ", world,
            " --gui-config ", gui_config, "'"])}.items())
    state = Node(package='robot_state_publisher', executable='robot_state_publisher',
                 parameters=[{'use_sim_time': True, 'robot_description': ParameterValue(
                     Command(['xacro ', xacro, ' jump_model:=', jump_model,
                              ' measure_contacts:=', measure_contacts,
                              ' jump_leg_speed:=', LaunchConfiguration('jump_leg_speed'),
                              ' jump_leg_velocity_damping:=', LaunchConfiguration('jump_leg_velocity_damping')]), value_type=str)}], output='screen')
    spawn = Node(package='ros_gz_sim', executable='create', output='screen',
                 arguments=['-world', 'empty', '-name', 'tap_robot', '-allow_renaming', 'false',
                            '-topic', '/robot_description',
                            '-x', LaunchConfiguration('spawn_x'), '-y', LaunchConfiguration('spawn_y'),
                            '-z', LaunchConfiguration('spawn_z')])
    controller = Node(package='tap_control', executable='balance_controller_node',
                      parameters=[base_controller_config, LaunchConfiguration('controller_config'),
                                  {'use_sim_time': True}], output='screen')
    activate = Node(package='tap_sim', executable='activate_controllers',
                    arguments=['--auto-enable', LaunchConfiguration('auto_enable')], output='screen')

    def after_spawn(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Robot spawn failed'))]
        return [controller, activate]

    def after_activation(event, context):
        if event.returncode != 0:
            return [EmitEvent(event=Shutdown(reason='Controller activation failed; simulation stopped'))]
        return []

    return LaunchDescription([
        DeclareLaunchArgument('gui', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('auto_enable', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('jump_model', default_value='false', choices=['true', 'false']),
        DeclareLaunchArgument('measure_contacts', default_value='false', choices=['true', 'false'],
                              description='Opt-in wheel contact sensors (observational telemetry '
                                          'only, see BalanceControllerNode status contact_l=/contact_r= '
                                          '- does not feed floor_contact or any control logic), '
                                          'decoupled from jump_model so it does not also swap in '
                                          'BoundedGazeboSystem/lower actuator limits. Requires world '
                                          'support for the gz-sim-contact-system plugin; only '
                                          'ledge_test.sdf (terrain:=ledge) has it so far.'),
        DeclareLaunchArgument('terrain', default_value='none', choices=['none', 'rough', 'ledge'],
                              description='rough selects worlds/rough_training.sdf (heightfield + '
                                          'step arenas ported from rl-basic); ledge selects '
                                          'worlds/ledge_test.sdf (one isolated 20cm ledge_pad on '
                                          'open ground_plane, nothing else, for landing/touchdown '
                                          'testing without rough_training.sdf\'s heightfield '
                                          'confound -- see docs/LEDGE_DRIVEOFF_LIMIT.md); ignored '
                                          'if jump_model=true'),
        DeclareLaunchArgument('jump_leg_speed', default_value='12.566370614359172',
                              description='Positive-work cutoff <=28.274333882308138 rad/s; graph taper above190rpm'),
        DeclareLaunchArgument('jump_leg_velocity_damping', default_value='0.0',
                              description='Bounded local leg motor D gain, Nm/(rad/s), not passive friction'),
        # terrain=rough default spawn: rough_arena_spawn_pad in rough_training.sdf,
        # a flat 20 cm step (top z=0.20, raised from an initial 5 cm at the user's
        # request) OUTSIDE the heightfield's own footprint, so the robot balances
        # on flat ground first, then can be driven off the edge for a step-down
        # test, or +X further onto the heightfield. At 20 cm the drop exceeds the
        # robot's own body height and leg stroke -- see the pad's own comment in
        # rough_training.sdf. 0.4372 = pad top (0.20) + the standard flat-ground
        # wheel-ground offset (0.2372, see src/tap_sim/CONTRACT.md). Explicit
        # spawn_x/y/z always override.
        DeclareLaunchArgument('spawn_x', default_value=PythonExpression(
            ["'-4.6' if '", LaunchConfiguration('terrain'), "' == 'rough' else '0.0'"])),
        DeclareLaunchArgument('spawn_y', default_value='0.0'),
        DeclareLaunchArgument('spawn_z', default_value=PythonExpression(
            ["'0.4372' if '", LaunchConfiguration('terrain'), "' in ('rough', 'ledge') else '0.2372'"])),
        DeclareLaunchArgument('controller_config', default_value=base_controller_config,
                              description='Parameter overrides layered over the base controller config'),
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', [PathJoinSubstitution([description, '..']), ':',
                               EnvironmentVariable('GZ_SIM_RESOURCE_PATH', default_value='')]),
        RegisterEventHandler(OnProcessExit(target_action=spawn, on_exit=after_spawn)),
        RegisterEventHandler(OnProcessExit(target_action=activate, on_exit=after_activation)),
        gazebo, state,
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'], output='screen'),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             arguments=['/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
                        '/model/tap_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                        '/world/empty/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'],
             remappings=[('/model/tap_robot/pose', '/tap_robot/pose')],
             output='screen'),
        # Bridges the sensor's own _raw gz topic (populated directly by
        # gz-sim-contact-system) rather than the tap_sim::ContactSnapshot
        # re-publish (libtap_contact_snapshot.so has been observed to
        # intermittently fail to load in this environment - "undefined
        # symbol: removexattrat" from a libsystemd version mismatch,
        # unrelated to this plugin's own code). Remapped to the plain
        # left_wheel_contacts/right_wheel_contacts names so
        # BalanceControllerNode's existing subscription (jump.left_contacts_topic
        # default) needs no change.
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             condition=IfCondition(PythonExpression([
                 "'", jump_model, "' == 'true' or '", measure_contacts, "' == 'true'"])),
             arguments=['/left_wheel_contacts_raw@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts',
                        '/right_wheel_contacts_raw@ros_gz_interfaces/msg/Contacts[gz.msgs.Contacts'],
             remappings=[('/left_wheel_contacts_raw', '/left_wheel_contacts'),
                         ('/right_wheel_contacts_raw', '/right_wheel_contacts')],
             output='screen'),
        spawn,
    ])
