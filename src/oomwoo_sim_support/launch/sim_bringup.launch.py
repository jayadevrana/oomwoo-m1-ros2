# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Headless OOMWOO simulation bringup shared by the regression harnesses.

Brings up, fully headless (no GUI, offscreen rendering — CI/Docker friendly):
  * Gazebo server (living_room world) with software rendering
  * robot_state_publisher (oomwoo_one URDF)
  * spawn of oomwoo_one
  * ros_gz bridges: sim sensors/actuators + ground-truth model poses
  * Nav2 localization (map_server + AMCL) on the saved living_room map
  * Nav2 navigation (planner/controller/bt_navigator/behaviors/waypoints)
  * ground_truth pose publisher + a seeded /initialpose

Coverage- and relocalization-specific nodes are added by the including launch.
Staggered TimerActions keep the emulated-CPU startup orderly; the application
nodes additionally wait on their own inputs, so exact timing is not critical.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')
    pkg_gazebo = get_package_share_directory('kaiaai_gazebo')
    pkg_oomwoo = get_package_share_directory('oomwoo_one')

    # coarser 200 Hz physics step (vs the stock 1 kHz) so the bridged /clock
    # is 5x lighter — critical for a stable sim clock under x86 emulation.
    # World + map default to the primitives test_room but are overridable so the
    # same harness can drive the stock living_room (or any other world/map pair).
    default_world = os.path.join(pkg_sim, 'worlds', 'test_room.world')
    default_map = os.path.join(pkg_sim, 'maps', 'test_room.yaml')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    xacro_file = os.path.join(pkg_oomwoo, 'urdf', 'robot.urdf.xacro')
    nav2_params = os.path.join(pkg_sim, 'config', 'nav2_params.yaml')
    bridge_sim = os.path.join(pkg_oomwoo, 'config', 'gz_bridge.yaml')

    x0 = LaunchConfiguration('x_pose')
    y0 = LaunchConfiguration('y_pose')
    yaw0 = LaunchConfiguration('yaw')

    args = [
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        # coverage needs the Nav2 nav servers; relocalization does not (it only
        # spins in place under AMCL), so it can bring up a much lighter stack.
        DeclareLaunchArgument('with_nav', default_value='true'),
    ]
    with_nav = IfCondition(LaunchConfiguration('with_nav'))

    # models + meshes resolvable by gz; offscreen software GL for emulation/CI
    set_env = [
        SetEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.pathsep.join([os.path.join(pkg_gazebo, 'models'), pkg_oomwoo])),
        SetEnvironmentVariable('LIBGL_ALWAYS_SOFTWARE', '1'),
        SetEnvironmentVariable('GALLIUM_DRIVER', 'llvmpipe'),
    ]

    gz_server = ExecuteProcess(
        cmd=['gz', 'sim', '-s', '-r', '--headless-rendering', '-v', '1', world],
        output='screen')

    robot_description = ParameterValue(
        Command(['xacro ', xacro_file]), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}])

    bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        parameters=[{'config_file': bridge_sim, 'use_sim_time': True}])

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-world', 'default', '-topic', 'robot_description',
                   '-name', 'oomwoo_one', '-x', x0, '-y', y0, '-z', '0.06',
                   '-Y', yaw0])

    # Explicit localization (map_server + AMCL + lifecycle) with an absolute
    # map path — avoids nav2_bringup's map-arg substitution quirk and keeps the
    # saved map self-contained inside this package.
    map_server = Node(
        package='nav2_map_server', executable='map_server', name='map_server',
        output='screen',
        parameters=[{'yaml_filename': map_yaml, 'use_sim_time': True,
                     'topic_name': 'map', 'frame_id': 'map'}])
    # Self-initialize AMCL at the known spawn pose. This uses the configured
    # pose directly (no stamped-message TF lookup), avoiding the "extrapolation
    # into the future" race that blocks localization under a slow sim clock.
    amcl = Node(
        package='nav2_amcl', executable='amcl', name='amcl', output='screen',
        parameters=[nav2_params, {
            'use_sim_time': True, 'set_initial_pose': True,
            # seed AMCL at the spawn pose (float-coerced from the launch args) so
            # a non-origin start (e.g. the clear cell in the cluttered living_room)
            # localizes immediately; test_room keeps its 0,0 default.
            'initial_pose.x': ParameterValue(x0, value_type=float),
            'initial_pose.y': ParameterValue(y0, value_type=float),
            'initial_pose.z': 0.0,
            'initial_pose.yaw': ParameterValue(yaw0, value_type=float),
            # update the filter a bit more often (vs 0.25 m / 0.2 rad) so a
            # kidnapped robot re-converges faster during the recovery drive.
            # recovery_alpha stays disabled (stock): continuous particle
            # injection keeps the published covariance permanently inflated,
            # which destroys the convergence signal both the recovery node and
            # the regression depend on. Wrong-mode escape comes from the
            # explicit global re-init + explore motion instead.
            'update_min_d': 0.15, 'update_min_a': 0.1,
            # SHARP measurement model for the noise-free sim LiDAR. The stock
            # z_hit 0.5 / z_rand 0.5 / 60-beam model is so permissive that a
            # mirrored symmetric hypothesis survives indefinitely after a global
            # re-init (observed: covariance trace pinned at ~6.5 for 45 s).
            # Weighting hits strongly and sampling more beams makes the ghost
            # mode collapse within a few updates.
            'z_hit': 0.95, 'z_rand': 0.05, 'max_beams': 120}])
    lifecycle_loc = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_localization', output='screen',
        parameters=[{'use_sim_time': True, 'autostart': True,
                     'bond_timeout': 0.0,
                     'node_names': ['map_server', 'amcl']}])
    localization = [map_server, amcl, lifecycle_loc]

    # Trimmed Nav2: only the servers coverage needs (planner, controller,
    # behaviors for recovery, bt_navigator). Dropping route/docking/collision/
    # smoother/velocity_smoother/waypoint servers keeps RAM+CPU low enough to
    # run on a 2-core / 3 GB machine. The controller publishes /cmd_vel directly
    # (no smoother chain to remap around).
    nav_common = {'use_sim_time': True}
    controller = Node(
        package='nav2_controller', executable='controller_server',
        name='controller_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    planner = Node(
        package='nav2_planner', executable='planner_server',
        name='planner_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    behavior = Node(
        package='nav2_behaviors', executable='behavior_server',
        name='behavior_server', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common])
    # the params yaml uses $(find-pkg-share ...) for the BT XML paths, which
    # only nav2_bringup's RewrittenYaml expands — passing the yaml raw leaves
    # the literal string and bt_navigator fails to activate. Override with
    # resolved absolute paths.
    bt_dir = os.path.join(
        get_package_share_directory('nav2_bt_navigator'), 'behavior_trees')
    bt_nav = Node(
        package='nav2_bt_navigator', executable='bt_navigator',
        name='bt_navigator', output='screen', condition=with_nav,
        parameters=[nav2_params, nav_common, {
            'default_nav_to_pose_bt_xml': os.path.join(
                bt_dir, 'navigate_to_pose_w_replanning_and_recovery.xml'),
            'default_nav_through_poses_bt_xml': os.path.join(
                bt_dir,
                'navigate_through_poses_w_replanning_and_recovery.xml')}])
    lifecycle_nav = Node(
        package='nav2_lifecycle_manager', executable='lifecycle_manager',
        name='lifecycle_manager_navigation', output='screen', condition=with_nav,
        parameters=[{'use_sim_time': True, 'autostart': True, 'bond_timeout': 0.0,
                     'node_names': ['controller_server', 'planner_server',
                                    'behavior_server', 'bt_navigator']}])
    navigation = [controller, planner, behavior, bt_nav, lifecycle_nav]

    ground_truth = Node(
        package='oomwoo_sim_support', executable='ground_truth', output='screen',
        parameters=[{'spawn_x': x0, 'spawn_y': y0, 'spawn_yaw': yaw0,
                     'use_sim_time': True}],
        remappings=[('odom', '/odom'), ('~/pose', '/ground_truth/pose')])

    # AMCL self-initializes at spawn (set_initial_pose); the standalone
    # initialpose_pub node remains available for bringups that need to seed a
    # pose over /initialpose instead.

    return LaunchDescription(args + set_env + [
        gz_server, rsp, bridge, ground_truth,
        TimerAction(period=10.0, actions=[spawn]),
        TimerAction(period=14.0, actions=localization + navigation),
    ])
