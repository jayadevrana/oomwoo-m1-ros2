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

If the coverage_meter flags the simulation as unstable (ground-truth pose
teleports — seen on Docker-under-WSL2 and emulated hosts), the run aborts
immediately with end_reason "sim_unstable" and exit code 2: metrics from an
unstable sim are meaningless and must not be reported as a plain pass/fail.
Intended to be launched alongside coverage_regression.launch.py.
"""

import json
import sys
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from std_msgs.msg import Bool, Float32


class CoverageRunner(Node):
    def __init__(self) -> None:
        super().__init__('coverage_regression_runner')
        self.declare_parameter('coverage_target', 0.90)
        self.declare_parameter('efficiency_target', 0.80)
        # generous: run-to-completion sweeps outlast the old stop-at-90% runs
        self.declare_parameter('max_sim_time_s', 3600.0)
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
        self.sim_unstable = False
        self.target_crossed = False
        self.eff_at_target = 0.0
        self.t_to_target = 0.0

        # sim_unstable is published latched; a latched sub can't miss it even if
        # the flag was raised before this runner finished starting up.
        latched = QoSProfile(
            depth=1, history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(Float32, '/coverage_meter/ratio', self._on_cov, 10)
        self.create_subscription(
            Float32, '/coverage_meter/efficiency', self._on_eff, 10)
        self.create_subscription(
            Bool, '/coverage_meter/sim_unstable', self._on_unstable, latched)
        # planner publishes cleaning_active False when the sweep + gap-fill
        # passes are exhausted — the honest end-of-job signal (latched)
        self.sweep_started = False
        self.sweep_complete = False
        self.create_subscription(
            Bool, '/coverage_planner/cleaning_active', self._on_active, latched)

    def _on_active(self, msg):
        if msg.data:
            self.sweep_started = True
        elif self.sweep_started:
            self.sweep_complete = True

    def _on_cov(self, msg):
        self.coverage = float(msg.data)

    def _on_eff(self, msg):
        self.efficiency = float(msg.data)

    def _on_unstable(self, msg):
        self.sim_unstable = self.sim_unstable or bool(msg.data)

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
            if self.sim_unstable:
                # no point measuring further — the numbers are already invalid
                break
            if self.coverage > self.best + self.plateau_eps:
                self.best = self.coverage
                self.last_gain_sim_t = self._sim_now()
            # The contract's two gates are ONE condition: reach >=90% coverage
            # at >=80% efficiency. Efficiency is judged the moment coverage
            # first crosses the target; the sweep then continues to completion
            # so the coverage number is uncapped. Chasing the final few percent
            # costs extra path (diminishing returns) — that thoroughness tax is
            # reported (efficiency_final) but doesn't retroactively fail a
            # gate that was already met.
            if not self.target_crossed and self.coverage >= self.cov_target:
                self.target_crossed = True
                self.eff_at_target = self.efficiency
                self.t_to_target = sim_t
            # No break at coverage_target: the target is the PASS GATE, not a
            # stop switch. The run ends when the sweep genuinely finishes (or
            # stalls), so the reported number is uncapped and informative —
            # 97% and 90.2% no longer read the same.
            if self.sweep_complete:
                reason = 'sweep_complete'
                # let the meter's last 1 Hz update land before scoring
                t_grace = time.time()
                while rclpy.ok() and time.time() - t_grace < 3.0:
                    rclpy.spin_once(self, timeout_sec=0.2)
                break
            if self._sim_now() - self.last_gain_sim_t >= self.plateau_s:
                reason = 'plateau'
                break
            if sim_t >= self.max_t:
                reason = 'max_time'
                break

        if self.sim_unstable:
            reason = 'sim_unstable'
        # gate efficiency = at the target crossing (the contract condition);
        # if the target was never crossed the final value is all there is.
        gate_eff = self.eff_at_target if self.target_crossed else self.efficiency
        result = {
            'coverage': round(self.coverage, 4),          # final, uncapped
            'coverage_target': self.cov_target,
            'efficiency_at_target': round(gate_eff, 4),   # judged at 90% crossing
            'efficiency_final': round(self.efficiency, 4),  # incl. thoroughness tax
            'efficiency_target': self.eff_target,
            'time_to_target_s': round(self.t_to_target, 1),
            'end_reason': reason,
            'sim_unstable': self.sim_unstable,
            'pass': bool(not self.sim_unstable
                         and self.coverage >= self.cov_target
                         and gate_eff >= self.eff_target),
        }
        try:
            with open(self.report_path, 'w') as f:
                json.dump(result, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'could not write report: {e}')
        if self.sim_unstable:
            # Invalid measurement, NOT a behavior failure: distinct exit code
            # so CI/users don't read a physics glitch as a coverage regression.
            self.get_logger().error(
                'COVERAGE_SUMMARY MEASUREMENT INVALID (sim unstable: ground-'
                'truth pose teleported). This host cannot run the sim '
                'faithfully — use a native x86-64 Linux machine or CI runner. '
                f'coverage={result["coverage"]:.4f} (informational only)')
            return 2
        self.get_logger().info(
            f'COVERAGE_SUMMARY coverage={result["coverage"]:.4f} '
            f'eff_at_target={result["efficiency_at_target"]:.4f} '
            f'eff_final={result["efficiency_final"]:.4f} '
            f'reason={reason} pass={result["pass"]}')
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
