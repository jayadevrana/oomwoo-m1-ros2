#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Kidnapped-robot detection and relocalization for OOMWOO.

Behavior: on a saved map with Nav2 + AMCL running, detect when localization
confidence collapses (the robot was picked up and moved, or AMCL diverged),
then actively relocalize: scatter AMCL particles globally and rotate in place to
gather scans until the pose re-converges. Report a clear success/failure signal.

Success (per nav-localize RFC): re-converge within 30 s and ≤ 2 m of truth,
≥ 90 % of the time. This node owns detection + the motion recovery; AMCL owns
the particle filter.

Interfaces:
  sub   /amcl_pose        geometry_msgs/PoseWithCovarianceStamped
  sub   /kidnap_trigger   std_msgs/Empty   (optional external "you were moved")
  srv c /reinitialize_global_localization  std_srvs/Empty  (AMCL)
  pub   /cmd_vel          geometry_msgs/Twist   (exclusive while recovering)
  pub   ~/localization_status  oomwoo status (published as diagnostic string+bool)

/cmd_vel arbitration: while state == RECOVERING this node is the *only* velocity
source. The relocalize launch does not run a Nav2 goal concurrently; if it did,
integrators must gate Nav2 on ~/recovering.
"""

import math
from enum import Enum
from typing import Optional

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Empty, Bool, Float32, String
from std_srvs.srv import Empty as EmptySrv


class State(Enum):
    TRACKING = 0      # localized, confidence ok
    RECOVERING = 1    # lost -> actively relocalizing
    FAILED = 2        # gave up -> hand off to dock-cycle fallback


def amcl_qos() -> QoSProfile:
    return QoSProfile(
        depth=5,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


def sensor_qos() -> QoSProfile:
    return QoSProfile(
        depth=5,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.BEST_EFFORT,
        durability=QoSDurabilityPolicy.VOLATILE,
    )


class KidnapRecovery(Node):
    def __init__(self) -> None:
        super().__init__('kidnap_recovery')

        # --- parameters ---------------------------------------------------
        # Confidence proxy: trace of the x,y,yaw covariance from /amcl_pose.
        self.declare_parameter('lost_cov_trace', 0.6)     # enter recovery above
        # AMCL global re-init scatters particles over the whole map; a converged
        # trace in this room settles ~0.1-0.3, so 0.25 is a safe "converged" gate
        # (still << the 2 m accuracy target).
        self.declare_parameter('ok_cov_trace', 0.25)      # converged below
        self.declare_parameter('converge_hold_sec', 1.0)  # stay converged this long
        self.declare_parameter('recovery_timeout_sec', 30.0)
        self.declare_parameter('spin_speed', 0.9)         # rad/s in-place
        self.declare_parameter('drive_speed', 0.16)       # m/s while exploring
        self.declare_parameter('front_clear_m', 0.45)     # obstacle stop distance
        self.declare_parameter('initial_spin_sec', 4.0)   # spin first to see 360
        self.declare_parameter('settle_after_trigger_sec', 0.5)

        self.lost_trace = self.get_parameter('lost_cov_trace').value
        self.ok_trace = self.get_parameter('ok_cov_trace').value
        self.hold_sec = self.get_parameter('converge_hold_sec').value
        self.timeout_sec = self.get_parameter('recovery_timeout_sec').value
        self.spin_speed = self.get_parameter('spin_speed').value
        self.drive_speed = self.get_parameter('drive_speed').value
        self.front_clear_m = self.get_parameter('front_clear_m').value
        self.initial_spin_sec = self.get_parameter('initial_spin_sec').value

        # --- state --------------------------------------------------------
        self.state = State.TRACKING
        self.last_trace: Optional[float] = None
        self.recover_start: Optional[rclpy.time.Time] = None
        self.converged_since: Optional[rclpy.time.Time] = None
        self.reinit_sent = False
        self.front_clear = True

        # --- ROS plumbing -------------------------------------------------
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, amcl_qos())
        self.create_subscription(
            Empty, 'kidnap_trigger', self._on_trigger, 10)
        self.create_subscription(LaserScan, 'scan', self._on_scan, sensor_qos())

        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.recovering_pub = self.create_publisher(Bool, '~/recovering', 10)
        self.conf_pub = self.create_publisher(Float32, '~/confidence', 10)
        self.status_pub = self.create_publisher(String, '~/localization_status', 10)

        self.reinit_cli = self.create_client(
            EmptySrv, 'reinitialize_global_localization')

        self.create_timer(0.1, self._tick)  # 10 Hz control
        self.get_logger().info('kidnap_recovery up; tracking localization')

    # ------------------------------------------------------------- inputs
    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        cov = msg.pose.covariance
        # covariance is row-major 6x6: xx=0, yy=7, yaw yaw=35
        self.last_trace = float(cov[0] + cov[7] + cov[35])

    def _on_trigger(self, _msg: Empty) -> None:
        # external "you were picked up / moved" signal (e.g. from sim harness
        # or a future pickup sensor). Force recovery regardless of covariance.
        if self.state != State.RECOVERING:
            self.get_logger().warn('kidnap_trigger received -> entering recovery')
            self._enter_recovery()

    def _on_scan(self, msg: LaserScan) -> None:
        # front clearance = min range within +/-25 deg of straight ahead
        n = len(msg.ranges)
        if n == 0:
            return
        span = max(1, int((25.0 * math.pi / 180.0) / msg.angle_increment))
        window = list(msg.ranges[:span]) + list(msg.ranges[-span:])
        valid = [r for r in window if msg.range_min < r < msg.range_max]
        self.front_clear = (min(valid) > self.front_clear_m) if valid else True

    # -------------------------------------------------------------- logic
    def _enter_recovery(self) -> None:
        self.state = State.RECOVERING
        self.recover_start = self.get_clock().now()
        self.converged_since = None
        self.reinit_sent = False

    def _tick(self) -> None:
        now = self.get_clock().now()
        conf = 0.0 if self.last_trace is None else \
            float(max(0.0, 1.0 - self.last_trace / self.lost_trace))
        self.conf_pub.publish(Float32(data=conf))

        if self.state == State.TRACKING:
            self.recovering_pub.publish(Bool(data=False))
            if self.last_trace is not None and self.last_trace >= self.lost_trace:
                self.get_logger().warn(
                    f'localization lost (cov trace {self.last_trace:.2f}) -> recovery')
                self._enter_recovery()
            else:
                self._publish_status('LOCALIZED', True)

        elif self.state == State.RECOVERING:
            self.recovering_pub.publish(Bool(data=True))
            self._do_recovery(now)

        elif self.state == State.FAILED:
            self.recovering_pub.publish(Bool(data=False))
            self._stop()

    def _do_recovery(self, now) -> None:
        # 1) scatter particles globally once AMCL client is available
        if not self.reinit_sent:
            if self.reinit_cli.service_is_ready():
                self.reinit_cli.call_async(EmptySrv.Request())
                self.reinit_sent = True
                self.get_logger().info('AMCL global re-init requested')
            elif not self.reinit_cli.wait_for_service(timeout_sec=0.0):
                pass  # keep trying next tick

        # 2) actively gather scans: spin in place first (see 360 from the drop
        #    point), then explore (drive + obstacle-avoid) so AMCL can resolve
        #    position, not just heading — pure rotation can't disambiguate where
        #    in the room the robot is.
        since_start = (now - self.recover_start).nanoseconds * 1e-9
        if since_start < self.initial_spin_sec:
            self._spin()
        else:
            self._explore()

        # 3) converged? trace below ok for hold_sec continuously
        trace = self.last_trace if self.last_trace is not None else math.inf
        if trace <= self.ok_trace:
            if self.converged_since is None:
                self.converged_since = now
            elif (now - self.converged_since).nanoseconds * 1e-9 >= self.hold_sec:
                elapsed = (now - self.recover_start).nanoseconds * 1e-9
                self._stop()
                self.state = State.TRACKING
                self.get_logger().info(
                    f'RELOCALIZED in {elapsed:.1f}s (cov trace {trace:.3f})')
                self._publish_status('RELOCALIZED', True)
                return
        else:
            self.converged_since = None

        # 4) timeout -> fail, hand off to dock-cycle find-the-dock fallback
        if (now - self.recover_start).nanoseconds * 1e-9 >= self.timeout_sec:
            self._stop()
            self.state = State.FAILED
            self.get_logger().error(
                'relocalization FAILED (timeout) -> dock-cycle fallback')
            self._publish_status('LOCALIZATION_LOST', False)

    # --------------------------------------------------------------- motion
    def _spin(self) -> None:
        t = Twist()
        t.angular.z = self.spin_speed
        self.cmd_pub.publish(t)

    def _explore(self) -> None:
        # reactive wander: drive forward while the way ahead is clear, otherwise
        # rotate to find a new heading. Moves the robot around the room so AMCL
        # gathers scans from different positions and the filter can converge.
        t = Twist()
        if self.front_clear:
            t.linear.x = self.drive_speed
        else:
            t.angular.z = self.spin_speed
        self.cmd_pub.publish(t)

    def _stop(self) -> None:
        self.cmd_pub.publish(Twist())

    def _publish_status(self, reason_code: str, recoverable: bool) -> None:
        # SOFTWARE_INTERFACES.md status shape, serialized until the project
        # picks a status message type.
        self.status_pub.publish(String(
            data=f'state={self.state.name.lower()};reason_code={reason_code};'
                 f'recoverable={recoverable};source=nav-localize'))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = KidnapRecovery()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
