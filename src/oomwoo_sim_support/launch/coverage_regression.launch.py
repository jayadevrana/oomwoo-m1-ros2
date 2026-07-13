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
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    pkg_sim = get_package_share_directory('oomwoo_sim_support')

    cleaning_radius = 0.20
    coverage_target = 0.90

    # world/map/spawn default to the primitives test_room but are overridable so
    # the same regression drives the stock living_room (with collision proxies).
    default_world = os.path.join(pkg_sim, 'worlds', 'test_room.world')
    default_map = os.path.join(pkg_sim, 'maps', 'test_room.yaml')
    reg_args = [
        DeclareLaunchArgument('world', default_value=default_world),
        DeclareLaunchArgument('map', default_value=default_map),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0'),
        DeclareLaunchArgument('yaw', default_value='0.0'),
        # planning/serviceability clearance. 0.30 is roomy for the open
        # test_room; drop toward the true inscribed radius (0.1745) in cluttered
        # worlds so the sweep can go under furniture (e.g. between table legs).
        DeclareLaunchArgument('robot_radius', default_value='0.30'),
        # pinned so the regression gate is reproducible on any machine;
        # override robot_model:=<pkg> to run the suite against another vacuum.
        DeclareLaunchArgument('robot_model', default_value='oomwoo_one'),
    ]
    robot_radius = ParameterValue(
        LaunchConfiguration('robot_radius'), value_type=float)

    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_sim, 'launch', 'sim_bringup.launch.py')),
        launch_arguments={
            'world': LaunchConfiguration('world'),
            'map': LaunchConfiguration('map'),
            'x_pose': LaunchConfiguration('x_pose'),
            'y_pose': LaunchConfiguration('y_pose'),
            'yaw': LaunchConfiguration('yaw'),
            'robot_model': LaunchConfiguration('robot_model'),
        }.items())

    coverage_meter = Node(
        package='oomwoo_sim_support', executable='coverage_meter', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius, 'robot_radius': robot_radius,
                     'coverage_target': coverage_target, 'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('ground_truth/pose', '/ground_truth/pose'),
                    ('cleaning_active', '/coverage_planner/cleaning_active')])

    coverage_planner = Node(
        package='oomwoo_coverage', executable='coverage_planner', output='screen',
        parameters=[{'cleaning_radius': cleaning_radius, 'robot_radius': robot_radius,
                     'coverage_target': coverage_target, 'row_overlap': 0.05, 'max_retries': 1,
                     'use_sim_time': True}],
        remappings=[('map', '/map'),
                    ('coverage_ratio', '/coverage_meter/ratio'),
                    ('covered_grid', '/coverage_meter/covered_grid'),
                    ('navigate_to_pose', '/navigate_to_pose')])

    return LaunchDescription(reg_args + [
        base,
        TimerAction(period=16.0, actions=[coverage_meter]),
        # start the sweep after localization + Nav2 have settled
        TimerAction(period=32.0, actions=[coverage_planner]),
    ])
