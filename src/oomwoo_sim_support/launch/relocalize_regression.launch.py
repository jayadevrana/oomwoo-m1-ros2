# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Kidnapped-robot relocalization bringup (headless, no Nav2 nav servers).

Brings up the sim + AMCL + ground truth (via sim_bringup with_nav:=false), plus
the kidnap_recovery node (detect lost -> AMCL global re-init -> spin to recover)
and the kidnap_injector (teleport the robot + signal). Much lighter than the
coverage stack since relocalization only spins in place under AMCL.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={'with_nav': 'false'}.items())

    kidnap_recovery = Node(
        package='oomwoo_nav_localize', executable='kidnap_recovery',
        output='screen',
        parameters=[{'use_sim_time': True,
                     'lost_cov_trace': 0.6, 'ok_cov_trace': 0.25,
                     'recovery_timeout_sec': 30.0, 'spin_speed': 0.9,
                     'drive_speed': 0.16, 'initial_spin_sec': 4.0}],
        remappings=[('amcl_pose', '/amcl_pose'),
                    ('kidnap_trigger', '/kidnap_trigger'),
                    ('cmd_vel', '/cmd_vel'),
                    ('reinitialize_global_localization',
                     '/reinitialize_global_localization')])

    kidnap_injector = Node(
        package='oomwoo_sim_support', executable='kidnap_injector',
        output='screen',
        parameters=[{'use_sim_time': True, 'robot_model_name': 'oomwoo_one',
                     'world_name': 'default', 'min_jump': 1.5, 'seed': 42}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose')])

    return LaunchDescription([
        base,
        # start recovery + injector once AMCL is up and localized
        TimerAction(period=20.0, actions=[kidnap_recovery, kidnap_injector]),
    ])
