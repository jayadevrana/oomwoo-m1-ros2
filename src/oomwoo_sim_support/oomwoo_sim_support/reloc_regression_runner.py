#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Autonomous kidnapped-robot relocalization regression runner (headless CLI).

Runs N kidnap trials against the running relocalize stack and scores each against
the acceptance metrics from the nav-localize RFC:

    * relocalize within RECOVERY_TIME_S (default 30 s)
    * final AMCL pose within SUCCESS_DIST_M (default 2 m) of the true teleport pose
    * success rate over N trials >= SUCCESS_RATE (default 0.90)

For each trial it: calls the injector's ~/kidnap service (teleport + trigger),
reads the true post-teleport pose the injector reports on ~/target_pose, waits
for the recovery node to reach RELOCALIZED (or times out), then compares the
converged /amcl_pose to truth. Emits machine-parseable RELOC_RESULT / RELOC_SUMMARY
log lines and writes a JSON report. Exit code 0 iff the suite passes.
"""

import json
import math
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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from std_srvs.srv import Trigger


def _amcl_qos() -> QoSProfile:
    return QoSProfile(
        depth=5, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE)


class RelocRunner(Node):
    def __init__(self) -> None:
        super().__init__('reloc_regression_runner')
        self.declare_parameter('num_trials', 10)
        self.declare_parameter('recovery_time_s', 30.0)
        self.declare_parameter('success_dist_m', 2.0)
        self.declare_parameter('success_rate', 0.90)
        self.declare_parameter('per_trial_timeout_s', 40.0)
        self.declare_parameter('settle_s', 8.0)
        self.declare_parameter('report_path', '/root/reloc_report.json')

        self.n = int(self.get_parameter('num_trials').value)
        self.reco_time = self.get_parameter('recovery_time_s').value
        self.dist_ok = self.get_parameter('success_dist_m').value
        self.rate_ok = self.get_parameter('success_rate').value
        self.trial_timeout = self.get_parameter('per_trial_timeout_s').value
        self.settle_s = self.get_parameter('settle_s').value
        self.report_path = self.get_parameter('report_path').value

        self.amcl = None            # latest (x, y, trace)
        self.target = None          # latest injector target pose (x, y)
        self.create_subscription(
            PoseWithCovarianceStamped, '/amcl_pose', self._on_amcl, _amcl_qos())
        self.create_subscription(
            PoseStamped, '/kidnap_injector/target_pose', self._on_target, 10)
        self.kidnap_cli = self.create_client(
            Trigger, '/kidnap_injector/kidnap')

    def _on_amcl(self, msg):
        p = msg.pose.pose.position
        c = msg.pose.covariance
        self.amcl = (p.x, p.y, float(c[0] + c[7] + c[35]))

    def _on_target(self, msg):
        self.target = (msg.pose.position.x, msg.pose.position.y)

    # --- helpers ------------------------------------------------------------
    def _spin(self, dt):
        rclpy.spin_once(self, timeout_sec=dt)

    def _wait(self, pred, timeout):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self._spin(0.1)
            if pred():
                return True
        return False

    def run(self) -> int:
        self.get_logger().info('waiting for /amcl_pose + kidnap service...')
        if not self._wait(lambda: self.amcl is not None
                          and self.kidnap_cli.service_is_ready(), 120.0):
            self.get_logger().error('stack not ready (no amcl_pose / kidnap srv)')
            return 2
        self.get_logger().info(f'stack ready; running {self.n} kidnap trials')
        self._wait(lambda: False, self.settle_s)   # let initial localization settle

        results = []
        for i in range(self.n):
            results.append(self._trial(i))

        passed = sum(1 for r in results if r['success'])
        rate = passed / len(results) if results else 0.0
        times = [r['time_s'] for r in results if r['success']]
        summary = {
            'trials': len(results), 'passed': passed, 'success_rate': rate,
            'success_rate_target': self.rate_ok,
            'mean_reloc_time_s': (sum(times) / len(times)) if times else None,
            'max_reloc_time_s': max(times) if times else None,
            'suite_pass': rate >= self.rate_ok, 'results': results,
        }
        try:
            with open(self.report_path, 'w') as f:
                json.dump(summary, f, indent=2)
        except OSError as e:
            self.get_logger().warn(f'could not write report: {e}')
        self.get_logger().info(
            f'RELOC_SUMMARY passed={passed}/{len(results)} '
            f'success_rate={rate:.2f} target={self.rate_ok:.2f} '
            f'mean_t={summary["mean_reloc_time_s"]} '
            f'suite_pass={summary["suite_pass"]}')
        return 0 if summary['suite_pass'] else 1

    def _trial(self, i: int) -> dict:
        self.target = None
        # 1) kidnap
        fut = self.kidnap_cli.call_async(Trigger.Request())
        self._wait(lambda: fut.done(), 10.0)
        if not (fut.done() and fut.result() and fut.result().success):
            self.get_logger().warn(f'trial {i}: kidnap call failed')
            return {'trial': i, 'success': False, 'time_s': None,
                    'err_m': None, 'reason': 'kidnap_call_failed'}
        self._wait(lambda: self.target is not None, 5.0)
        target = self.target
        # score recovery time on the SIM clock (the RFC's 30 s budget) — wall
        # clock would unfairly penalize hosts running the sim below realtime
        t0_sim = self.get_clock().now()

        # 2) wait for re-convergence: amcl trace low AND close to truth
        def converged():
            if self.amcl is None or target is None:
                return False
            err = math.hypot(self.amcl[0] - target[0], self.amcl[1] - target[1])
            return self.amcl[2] <= 0.5 and err <= self.dist_ok
        ok = self._wait(converged, self.trial_timeout)
        dt = (self.get_clock().now() - t0_sim).nanoseconds * 1e-9
        err = (math.hypot(self.amcl[0] - target[0], self.amcl[1] - target[1])
               if (self.amcl and target) else None)
        # success requires convergence within the RFC time budget too
        success = bool(ok and dt <= self.reco_time and err is not None
                       and err <= self.dist_ok)
        self.get_logger().info(
            f'RELOC_RESULT trial={i} success={success} time={dt:.1f}s '
            f'err={err:.2f}m target=({target[0]:.2f},{target[1]:.2f})'
            if err is not None else
            f'RELOC_RESULT trial={i} success=False (no pose)')
        return {'trial': i, 'success': success, 'time_s': round(dt, 1),
                'err_m': round(err, 2) if err is not None else None,
                'target': target}


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RelocRunner()
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
