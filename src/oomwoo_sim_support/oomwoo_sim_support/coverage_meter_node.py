#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Measure true coverage % and path efficiency for the coverage behavior.

Coverage and efficiency are computed from the robot's *ground-truth* pose
(see ground_truth_node), never from the planner's own belief:

  coverage   = (reachable free cells swept by the cleaning disk) / (reachable
               free cells).  "Reachable" = free cells flood-filled from the
               robot's start cell, so sealed-off voids never inflate the score.
  efficiency = ideal_path_len / actual_path_len, where
               ideal_path_len = reachable_area / swath_width  (the length of a
               perfect gap-free boustrophedon).  At constant speed this equals
               time efficiency; reported alongside sim time.

  sub  /map            nav_msgs/OccupancyGrid   (transient_local)
  sub  /ground_truth/pose  geometry_msgs/PoseStamped
  pub  ~/ratio         std_msgs/Float32
  pub  ~/efficiency    std_msgs/Float32
Emits a machine-parseable ``COVERAGE_REPORT ...`` log line every second.
"""

import math
from typing import Optional

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Float32

FREE = 0
OCC_THRESH = 50


def latched_qos() -> QoSProfile:
    return QoSProfile(
        depth=1, history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)


class CoverageMeter(Node):
    def __init__(self) -> None:
        super().__init__('coverage_meter')
        self.declare_parameter('cleaning_radius', 0.16)
        self.declare_parameter('coverage_target', 0.90)
        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.coverage_target = self.get_parameter('coverage_target').value

        self.info = None
        self.free: Optional[np.ndarray] = None       # bool[H,W]
        self.reachable: Optional[np.ndarray] = None   # bool[H,W]
        self.covered: Optional[np.ndarray] = None     # bool[H,W]
        self.total_reachable = 0

        self.last_xy: Optional[tuple] = None
        self.path_len = 0.0
        self.t_start: Optional[rclpy.time.Time] = None
        self.t_target: Optional[float] = None         # sim sec to reach target
        self.target_hit = False

        self.create_subscription(OccupancyGrid, 'map', self._on_map, latched_qos())
        self.create_subscription(
            PoseStamped, 'ground_truth/pose', self._on_pose, 20)
        self.ratio_pub = self.create_publisher(Float32, '~/ratio', 10)
        self.eff_pub = self.create_publisher(Float32, '~/efficiency', 10)
        self.create_timer(1.0, self._report)
        self.get_logger().info('coverage_meter up; waiting for /map + truth')

    # --------------------------------------------------------------- map
    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.info is not None:
            return
        self.info = msg.info
        h, w = msg.info.height, msg.info.width
        grid = np.asarray(msg.data, dtype=np.int16).reshape(h, w)
        self.free = (grid >= 0) & (grid < OCC_THRESH)
        self.covered = np.zeros_like(self.free, dtype=bool)
        self.get_logger().info(
            f'map {w}x{h} @ {msg.info.resolution:.3f}m, {int(self.free.sum())} free cells '
            '(reachable set computed at first truth pose)')

    def _ensure_reachable(self, cx: int, cy: int) -> None:
        if self.reachable is not None:
            return
        start = _nearest_free(self.free, cx, cy)
        if start is None:
            return
        self.reachable = _flood_fill(self.free, start)
        self.total_reachable = int(self.reachable.sum())
        self.get_logger().info(
            f'reachable free cells from start {start}: {self.total_reachable}')

    # -------------------------------------------------------------- pose
    def _on_pose(self, msg: PoseStamped) -> None:
        if self.info is None:
            return
        res = self.info.resolution
        cx = int((msg.pose.position.x - self.info.origin.position.x) / res)
        cy = int((msg.pose.position.y - self.info.origin.position.y) / res)
        self._ensure_reachable(cx, cy)
        if self.reachable is None:
            return
        if self.t_start is None:
            self.t_start = self.get_clock().now()

        rad = max(1, int(round(self.cleaning_radius / res)))
        _stamp_disk(self.covered, self.reachable, cx, cy, rad)

        xy = (msg.pose.position.x, msg.pose.position.y)
        if self.last_xy is not None:
            self.path_len += math.hypot(xy[0] - self.last_xy[0],
                                        xy[1] - self.last_xy[1])
        self.last_xy = xy

    # ------------------------------------------------------------ report
    def _ratio(self) -> float:
        if self.total_reachable == 0 or self.covered is None:
            return 0.0
        return float((self.covered & self.reachable).sum()) / self.total_reachable

    def _ideal_path_len(self) -> float:
        area = self.total_reachable * (self.info.resolution ** 2)
        return area / (2.0 * self.cleaning_radius)

    def _efficiency(self) -> float:
        # Only meaningful once the robot has actually driven a bit; before that
        # a near-zero denominator would explode the ratio.
        if self.path_len < 0.5:
            return 0.0
        return self._ideal_path_len() / self.path_len

    def _report(self) -> None:
        if self.info is None or self.total_reachable == 0:
            return
        ratio = self._ratio()
        eff = self._efficiency()
        self.ratio_pub.publish(Float32(data=float(ratio)))
        self.eff_pub.publish(Float32(data=float(min(eff, 1.0))))

        sim_t = 0.0
        if self.t_start is not None:
            sim_t = (self.get_clock().now() - self.t_start).nanoseconds * 1e-9
        if not self.target_hit and ratio >= self.coverage_target:
            self.target_hit = True
            self.t_target = sim_t

        self.get_logger().info(
            f'COVERAGE_REPORT coverage={ratio:.4f} efficiency={eff:.4f} '
            f'path_m={self.path_len:.2f} ideal_m={self._ideal_path_len():.2f} '
            f'reachable_cells={self.total_reachable} sim_t={sim_t:.1f} '
            f'target_hit={self.target_hit} t_target={self.t_target}')


# ------------------------------------------------------------ numpy helpers
def _nearest_free(free, cx, cy, max_r=20):
    h, w = free.shape
    if 0 <= cy < h and 0 <= cx < w and free[cy, cx]:
        return (cy, cx)
    for r in range(1, max_r):
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        sub = free[y0:y1, x0:x1]
        if sub.any():
            ys, xs = np.where(sub)
            return (y0 + int(ys[0]), x0 + int(xs[0]))
    return None


def _flood_fill(free, start):
    """4-connected flood fill of the free region containing `start`."""
    h, w = free.shape
    out = np.zeros_like(free, dtype=bool)
    stack = [start]
    out[start] = True
    while stack:
        y, x = stack.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and free[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True
                stack.append((ny, nx))
    return out


def _stamp_disk(covered, mask, cx, cy, rad):
    h, w = covered.shape
    y0, y1 = max(0, cy - rad), min(h, cy + rad + 1)
    x0, x1 = max(0, cx - rad), min(w, cx + rad + 1)
    if y0 >= y1 or x0 >= x1:
        return
    ys = np.arange(y0, y1)[:, None]
    xs = np.arange(x0, x1)[None, :]
    disk = (ys - cy) ** 2 + (xs - cx) ** 2 <= rad * rad
    covered[y0:y1, x0:x1] |= disk & mask[y0:y1, x0:x1]


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoverageMeter()
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
