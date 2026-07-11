#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Boustrophedon coverage planner for the OOMWOO robot vacuum.

Behavior "regular / auto cleaning": given a saved occupancy map, plan a
back-and-forth (boustrophedon) sweep that covers the entire reachable floor,
respecting keep-out zones, and execute it through Nav2's
``NavigateThroughPoses`` action. Nav2 handles obstacle-aware routing between the
sweep waypoints; this node owns the *what and where to clean* decision.

Interfaces (per docs/SOFTWARE_INTERFACES.md):
  subscribes  /map              nav_msgs/OccupancyGrid   (transient_local)
  subscribes  /keepout_filter_mask nav_msgs/OccupancyGrid (optional, latched)
  action clnt /navigate_through_poses nav2_msgs/NavigateThroughPoses
  publishes   ~/coverage_grid   nav_msgs/OccupancyGrid   (covered cells, viz)
  publishes   ~/coverage_ratio  std_msgs/Float32         (0..1, monitoring)

Coverage is *measured*, not assumed: a disk of ``cleaning_radius`` swept along
the robot's actual pose (map->base_footprint from TF) marks free cells covered.
This is the same honest metric the regression harness asserts against.
"""

import math
from typing import List, Optional, Tuple

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateThroughPoses
from std_msgs.msg import Float32

# OccupancyGrid cell conventions
FREE = 0
UNKNOWN = -1
# cells with occupancy >= OCC_THRESH are treated as obstacle
OCC_THRESH = 50


def latched_qos() -> QoSProfile:
    """QoS matching a transient-local map publisher (map_server / SLAM)."""
    return QoSProfile(
        depth=1,
        history=QoSHistoryPolicy.KEEP_LAST,
        reliability=QoSReliabilityPolicy.RELIABLE,
        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    )


class CoveragePlanner(Node):
    def __init__(self) -> None:
        super().__init__('coverage_planner')

        # --- parameters ---------------------------------------------------
        self.declare_parameter('cleaning_radius', 0.16)   # m, effective clean swath / 2
        self.declare_parameter('row_overlap', 0.10)       # fraction of swath overlapped
        self.declare_parameter('robot_radius', 0.17)      # m, for obstacle inflation
        self.declare_parameter('coverage_target', 0.90)   # stop when reached
        self.declare_parameter('sweep_axis', 'x')         # 'x' = horizontal rows
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('min_segment_len', 0.20)   # m, drop slivers
        self.declare_parameter('goal_settle_sec', 0.0)    # optional dwell per goal

        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.row_overlap = self.get_parameter('row_overlap').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.coverage_target = self.get_parameter('coverage_target').value
        self.sweep_axis = self.get_parameter('sweep_axis').value
        self.global_frame = self.get_parameter('global_frame').value
        self.base_frame = self.get_parameter('robot_base_frame').value
        self.min_segment_len = self.get_parameter('min_segment_len').value

        # --- state --------------------------------------------------------
        self.map_msg: Optional[OccupancyGrid] = None
        self.free_mask: Optional[np.ndarray] = None       # bool[H,W], inflated-free
        self.keepout: Optional[np.ndarray] = None         # bool[H,W], True = no-go
        self.total_free_cells = 0
        self.robot_xy = None            # latest robot pose (map frame), for seeding
        self.ext_ratio = 0.0            # coverage %, from the coverage_meter
        self.plan_started = False       # a goal is in flight or accepted
        self.finished = False
        self.cached_poses = None        # boustrophedon plan, computed once
        self.last_attempt = None        # time of last goal send (for retry)
        self.retry_period = 5.0         # s between goal retries on rejection
        self.goal_handle = None

        # --- ROS plumbing -------------------------------------------------
        # Coverage % comes from the coverage_meter (ground-truth based), so this
        # node needs no TF listener — which also avoids flooding the graph with
        # TF_OLD_DATA warnings under a slow/jumpy sim clock.
        self.create_subscription(
            OccupancyGrid, 'map', self._on_map, latched_qos())
        self.create_subscription(
            OccupancyGrid, 'keepout_filter_mask', self._on_keepout, latched_qos())
        self.create_subscription(
            Float32, 'coverage_ratio', self._on_ratio, 10)
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl, 10)

        self.nav_client = ActionClient(
            self, NavigateThroughPoses, 'navigate_through_poses')

        # 5 Hz coverage accounting; planning kicks off once the map arrives
        self.create_timer(0.2, self._tick)
        self.get_logger().info('coverage_planner up; waiting for /map')

    # ---------------------------------------------------------------- maps
    def _on_map(self, msg: OccupancyGrid) -> None:
        if self.map_msg is not None:
            return
        self.map_msg = msg
        self._build_masks()
        self.get_logger().info(
            f'map {msg.info.width}x{msg.info.height} @ {msg.info.resolution:.3f} m, '
            f'{self.total_free_cells} reachable free cells')

    def _on_keepout(self, msg: OccupancyGrid) -> None:
        # keepout filter mask uses the same grid geometry; occupied => no-go
        data = np.asarray(msg.data, dtype=np.int16).reshape(
            msg.info.height, msg.info.width)
        self.keepout = data >= OCC_THRESH
        if self.map_msg is not None:
            self._build_masks()

    def _build_masks(self) -> None:
        info = self.map_msg.info
        h, w = info.height, info.width
        grid = np.asarray(self.map_msg.data, dtype=np.int16).reshape(h, w)

        obstacle = grid >= OCC_THRESH
        unknown = grid == UNKNOWN
        # inflate obstacles + unknown by robot radius so the center path is safe
        infl = max(1, int(round(self.robot_radius / info.resolution)))
        blocked = _dilate(obstacle | unknown, infl)
        free = (grid == FREE) & ~blocked

        if self.keepout is not None and self.keepout.shape == free.shape:
            free &= ~self.keepout

        self.free_mask = free
        self.total_free_cells = int(free.sum())

    def _on_ratio(self, msg: Float32) -> None:
        self.ext_ratio = float(msg.data)

    def _on_amcl(self, msg: PoseWithCovarianceStamped) -> None:
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    # ------------------------------------------------------------- planning
    def _plan_waypoints(self) -> List[PoseStamped]:
        """Boustrophedon sweep over the robot's *reachable* inflated-free area.

        Waypoints are restricted to the free-space connected component that
        contains the robot (flood fill), so the planner is never handed a pose
        stranded behind a wall or in the map-border inflation — the failure that
        aborts a NavigateThroughPoses goal. Rows are emitted starting from the
        robot's row outward to minimize the initial transit.
        """
        info = self.map_msg.info
        res = info.resolution
        ox, oy = info.origin.position.x, info.origin.position.y
        h, w = self.free_mask.shape

        # seed the reachable set from the robot's current cell
        rcx = int((self.robot_xy[0] - ox) / res)
        rcy = int((self.robot_xy[1] - oy) / res)
        seed = _nearest_true(self.free_mask, rcx, rcy)
        if seed is None:
            self.get_logger().warn('robot not on reachable free space')
            return []
        reachable = _flood_fill(self.free_mask, seed)

        swath = 2.0 * self.cleaning_radius
        step_m = max(res, swath * (1.0 - self.row_overlap))
        step = max(1, int(round(step_m / res)))
        min_seg_cells = max(1, int(round(self.min_segment_len / res)))

        # rows from the robot's row outward: down first, then up
        robot_row = seed[0]
        rows_down = list(range(robot_row, -1, -step))
        rows_up = list(range(robot_row + step, h, step))
        ordered_rows = rows_down + rows_up

        waypoints: List[Tuple[float, float]] = []
        for i, row in enumerate(ordered_rows):
            cols = np.where(reachable[row])[0]
            if cols.size == 0:
                continue
            for seg in _contiguous_runs(cols):
                if seg[-1] - seg[0] + 1 < min_seg_cells:
                    continue
                a, b = int(seg[0]), int(seg[-1])
                y = oy + (row + 0.5) * res
                xa = ox + (a + 0.5) * res
                xb = ox + (b + 0.5) * res
                if i % 2 == 0:                 # serpentine
                    waypoints.append((xa, y))
                    waypoints.append((xb, y))
                else:
                    waypoints.append((xb, y))
                    waypoints.append((xa, y))

        poses: List[PoseStamped] = []
        for (x, y) in waypoints:
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            poses.append(p)
        return poses

    def _start_plan(self) -> None:
        # Nav2 lifecycle activation can lag well behind process start under a
        # slow/emulated sim, so retry until the action server accepts the goal.
        self.last_attempt = self.get_clock().now()
        if self.robot_xy is None:
            self.get_logger().info('waiting for robot pose (amcl)...')
            return
        if self.cached_poses is None:
            poses = self._plan_waypoints()
            if not poses:
                self.get_logger().warn('no waypoints yet; will retry')
                return
            self.cached_poses = poses
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.0)
            self.get_logger().info('waiting for navigate_through_poses server...')
            return
        now = self.get_clock().now().to_msg()
        for p in self.cached_poses:
            p.header.stamp = now
        goal = NavigateThroughPoses.Goal()
        goal.poses = self.cached_poses
        self.get_logger().info(
            f'sending coverage plan: {len(self.cached_poses)} waypoints')
        self.plan_started = True
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _on_goal_response(self, fut) -> None:
        handle = fut.result()
        if not handle.accepted:
            # server was not active yet — fall back to retry in _tick
            self.get_logger().warn('coverage goal rejected; will retry')
            self.plan_started = False
            return
        self.goal_handle = handle
        self.get_logger().info('coverage goal accepted; executing sweep')
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, _fut) -> None:
        self.goal_handle = None
        if self.finished:
            return
        # Nav2 finished the waypoint list; if coverage is short, replan the gaps
        ratio = self._ratio()
        self.get_logger().info(f'coverage run ended; coverage={ratio:.1%}')
        if ratio >= self.coverage_target:
            self.finished = True
        else:
            self.plan_started = False   # allow a fresh sweep attempt

    def _elapsed(self, t) -> float:
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    # -------------------------------------------------------------- ticking
    def _tick(self) -> None:
        if self.map_msg is None or self.total_free_cells == 0:
            return
        ratio = self.ext_ratio

        if ratio >= self.coverage_target and not self.finished:
            self.get_logger().info(
                f'coverage target {self.coverage_target:.0%} reached '
                f'({ratio:.1%}); stopping')
            self.finished = True
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
        elif not self.plan_started and not self.finished:
            if self.last_attempt is None or self._elapsed(self.last_attempt) >= \
                    self.retry_period:
                self._start_plan()


# ------------------------------------------------------------- numpy helpers
def _dilate(mask: np.ndarray, radius: int) -> np.ndarray:
    """Binary dilation by a square structuring element of given radius."""
    if radius <= 0:
        return mask.copy()
    out = mask.copy()
    for _ in range(radius):
        shifted = out.copy()
        shifted[1:, :] |= out[:-1, :]
        shifted[:-1, :] |= out[1:, :]
        shifted[:, 1:] |= out[:, :-1]
        shifted[:, :-1] |= out[:, 1:]
        out = shifted
    return out


def _contiguous_runs(cols: np.ndarray) -> List[np.ndarray]:
    """Split a sorted 1-D index array into contiguous runs."""
    if cols.size == 0:
        return []
    breaks = np.where(np.diff(cols) > 1)[0] + 1
    return np.split(cols, breaks)


def _nearest_true(mask: np.ndarray, cx: int, cy: int, max_r: int = 25):
    """Nearest True cell (row, col) to (cy, cx) within max_r, else None."""
    h, w = mask.shape
    if 0 <= cy < h and 0 <= cx < w and mask[cy, cx]:
        return (cy, cx)
    for r in range(1, max_r):
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        sub = mask[y0:y1, x0:x1]
        if sub.any():
            ys, xs = np.where(sub)
            return (y0 + int(ys[0]), x0 + int(xs[0]))
    return None


def _flood_fill(mask: np.ndarray, start) -> np.ndarray:
    """4-connected flood fill of the True region containing `start`."""
    h, w = mask.shape
    out = np.zeros_like(mask, dtype=bool)
    stack = [start]
    out[start] = True
    while stack:
        y, x = stack.pop()
        for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            ny, nx = y + dy, x + dx
            if 0 <= ny < h and 0 <= nx < w and mask[ny, nx] and not out[ny, nx]:
                out[ny, nx] = True
                stack.append((ny, nx))
    return out


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CoveragePlanner()
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
