#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Autonomous coverage-cleaning regression runner (headless CLI).

Observes the coverage_meter while the coverage_planner sweeps the map, then
scores the run against the cleaning-jobs acceptance metrics:

    * coverage   >= COVERAGE_TARGET   (default 0.90)
    * efficiency >= EFFICIENCY_TARGET (default 0.80)

The run ends when coverage reaches the target, or plateaus (no meaningful gain
for PLATEAU_S of sim time), or MAX_SIM_TIME is hit. Emits a machine-parseable
COVERAGE_SUMMARY log line, writes a JSON report, and exits 0 iff the run passes.
Intended to be launched alongside coverage_regression.launch.py.
"""

import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import Float32


class CoverageRunner(Node):
    def __init__(self) -> None:
        super().__init__('coverage_regression_runner')
        self.declare_parameter('coverage_target', 0.90)
        self.declare_parameter('efficiency_target', 0.80)
        self.declare_parameter('max_sim_time_s', 1800.0)
        self.declare_parameter('plateau_s', 180.0)
        self.declare_parameter('plateau_eps', 0.005)
        self.declare_parameter('report_path', '/root/coverage_report.json')

        self.cov_target = self.get_parameter('coverage_target').value
        self.eff_target = self.get_parameter('efficiency_target').value
        self.max_t = self.get_parameter('max_sim_time_s').value
        self.plateau_s = self.get_parameter('plateau_s').value
        self.plateau_eps = self.get_parameter('plateau_eps').value
        self.report_path = self.get_parameter('report_path').value

        self.coverage = 0.0
        self.efficiency = 0.0
        self.best = 0.0
        self.last_gain_sim_t = None

        self.create_subscription(Float32, '/coverage_meter/ratio', self._on_cov, 10)
        self.create_subscription(
            Float32, '/coverage_meter/efficiency', self._on_eff, 10)

    def _on_cov(self, msg):
        self.coverage = float(msg.data)

    def _on_eff(self, msg):
        self.efficiency = float(msg.data)

    def _sim_now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def run(self) -> int:
        self.get_logger().info(
            f'watching coverage; target={self.cov_target:.0%} '
            f'efficiency>={self.eff_target:.0%}')
        # wait for the clock + first coverage message
        t_wall = time.time()
        while rclpy.ok() and self.coverage == 0.0 and time.time() - t_wall < 300:
            rclpy.spin_once(self, timeout_sec=0.2)
        start_sim = self._sim_now()
        self.last_gain_sim_t = start_sim

        reason = 'max_time'
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.5)
            sim_t = self._sim_now() - start_sim
            if self.coverage > self.best + self.plateau_eps:
                self.best = self.coverage
                self.last_gain_sim_t = self._sim_now()
            if self.coverage >= self.cov_target:
                reason = 'target_reached'
                break
            if self._sim_now() - self.last_gain_sim_t >= self.plateau_s:
                reason = 'plateau'
                break
            if sim_t >= self.max_t:
                reason = 'max_time'
                break

        result = {
            'coverage': round(self.coverage, 4),
            'coverage_target': self.cov_target,
            'efficiency': round(self.efficiency, 4),
            'efficiency_target': self.eff_target,
            'end_reason': reason,
            'pass': bool(self.coverage >= self.cov_target
                         and self.efficiency >= self.eff_target),
        }
        try:
            with open(self.report_path, 'w') as f:
                json.dump(result, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'could not write report: {e}')
        self.get_logger().info(
            f'COVERAGE_SUMMARY coverage={result["coverage"]:.4f} '
            f'efficiency={result["efficiency"]:.4f} reason={reason} '
            f'pass={result["pass"]}')
        return 0 if result['pass'] else 1


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageRunner()
    code = 1
    try:
        code = node.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(code)


if __name__ == '__main__':
    main()
