# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Coverage-cleaning bringup: base sim + Nav2 + coverage planner + coverage meter.

Headless. The coverage_planner executes a boustrophedon sweep via Nav2; the
coverage_meter scores true coverage % and path efficiency against ground truth
and logs COVERAGE_REPORT lines the regression test asserts on.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')

    cleaning_radius = 0.20
    coverage_target = 0.90

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim_bringup.launch.py')))

    coverage_meter = Node(
        package='oomwoo_sim_support', executable='coverage_meter', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius, 'robot_radius': 0.30,
                     'coverage_target': coverage_target, 'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    coverage_planner = Node(
        package='oomwoo_coverage', executable='coverage_planner', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius, 'robot_radius': 0.30,
                     'coverage_target': coverage_target, 'row_overlap': 0.05, 'max_retries': 1,
                     'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('coverage_ratio', '/coverage_meter/ratio'),
                    ('covered_grid', '/coverage_meter/covered_grid'),
                    ('navigate_to_pose', '/navigate_to_pose')])

    return LaunchDescription([
        base,
        TimerAction(period=16.0, actions=[coverage_meter]),
        # start the sweep after localization + Nav2 have settled
        TimerAction(period=32.0, actions=[coverage_planner]),
    ])
