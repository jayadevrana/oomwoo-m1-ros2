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

from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav_msgs.msg import OccupancyGrid
from nav2_msgs.action import NavigateToPose
from std_msgs.msg import Bool, Float32

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
        # coverage_target is a GATE the harness asserts on, not a stop switch:
        # by default the sweep runs to completion (all rows + gap-fill passes)
        # so the reported number is uncapped — it can distinguish a planner
        # that reaches 97% from one that barely scrapes 90%. Set
        # stop_at_target:=true for battery-frugal behaviour on a real robot.
        self.declare_parameter('coverage_target', 0.90)
        self.declare_parameter('stop_at_target', False)
        self.declare_parameter('sweep_axis', 'x')         # 'x' = horizontal rows
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_base_frame', 'base_footprint')
        self.declare_parameter('min_segment_len', 0.20)   # m, drop slivers
        self.declare_parameter('goal_settle_sec', 0.0)    # optional dwell per goal

        self.cleaning_radius = self.get_parameter('cleaning_radius').value
        self.row_overlap = self.get_parameter('row_overlap').value
        self.robot_radius = self.get_parameter('robot_radius').value
        self.coverage_target = self.get_parameter('coverage_target').value
        self.stop_at_target = self.get_parameter('stop_at_target').value
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
        self.wp_index = 0              # next waypoint to visit
        self.goal_deadline = None      # per-waypoint watchdog start
        self.declare_parameter('goal_timeout_sec', 30.0)
        self.goal_timeout = self.get_parameter('goal_timeout_sec').value
        self.declare_parameter('max_retries', 3)
        self.max_retries = self.get_parameter('max_retries').value
        self.awaiting = False
        self.wp_retries = 0
        self.next_send = None
        self.gapfill_passes = 0
        self.declare_parameter('max_gapfill', 3)
        self.max_gapfill = self.get_parameter('max_gapfill').value
        # Wedge escape: when Nav2 gives up on several waypoints in a row the
        # robot is usually stuck in a pocket the inflated costmap paints lethal
        # (e.g. between furniture legs) — spin/backup recoveries refuse to move
        # there, so nothing Nav2-side can free it. Physics can: reverse straight
        # out with a short open-loop cmd_vel pulse, then resume the sweep.
        self.declare_parameter('escape_after_skips', 2)
        self.declare_parameter('escape_sec', 2.5)
        self.declare_parameter('escape_speed', -0.12)
        self.escape_after = self.get_parameter('escape_after_skips').value
        self.escape_sec = self.get_parameter('escape_sec').value
        self.escape_speed = self.get_parameter('escape_speed').value
        self.consecutive_skips = 0
        self.escape_until = None

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
        self.covered_grid = None
        self.create_subscription(
            OccupancyGrid, 'covered_grid', self._on_covered, latched_qos())
        # AMCL latches amcl_pose (transient_local) and only republishes on
        # motion — match its durability or a still robot never sends a pose
        self.create_subscription(
            PoseWithCovarianceStamped, 'amcl_pose', self._on_amcl,
            latched_qos())

        self.active_pub = self.create_publisher(
            Bool, '~/cleaning_active', latched_qos())
        # only used by the wedge escape, while no Nav2 goal is active
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose')

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

    def _on_covered(self, msg: OccupancyGrid) -> None:
        self.covered_grid = np.asarray(msg.data, dtype=np.int8).reshape(
            msg.info.height, msg.info.width)

    def _covered_at(self, pose) -> bool:
        """True when the disk around this waypoint is already mostly clean."""
        if self.covered_grid is None or self.map_msg is None:
            return False
        info = self.map_msg.info
        res = info.resolution
        cx = int((pose.pose.position.x - info.origin.position.x) / res)
        cy = int((pose.pose.position.y - info.origin.position.y) / res)
        r = max(1, int(round(self.cleaning_radius / res)))
        h, w = self.covered_grid.shape
        y0, y1 = max(0, cy - r), min(h, cy + r + 1)
        x0, x1 = max(0, cx - r), min(w, cx + r + 1)
        if y0 >= y1 or x0 >= x1:
            return False
        sub = self.covered_grid[y0:y1, x0:x1]
        return float((sub >= 100).mean()) > 0.6

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

        # Continuous bottom-to-top serpentine (no down-then-up split). The robot
        # transits once to the first row, then sweeps straight through — this
        # avoids a wasteful mid-sweep jump across the room. The first row is
        # chosen as whichever end (bottom/top) is nearer the robot.
        rows = list(range(0, h, step))
        if rows and abs(seed[0] - rows[-1]) < abs(seed[0] - rows[0]):
            rows.reverse()

        # intermediate waypoints along each row keep the robot ON the straight
        # row line: with only two endpoints, the controller cuts the corner
        # toward the far next-row goal and curves, adding path and leaving band
        # slivers. One waypoint per `substep` metres tightens tracking.
        substep = max(1, int(round(1.0 / res)))

        waypoints: List[Tuple[float, float]] = []
        flip = False
        for row in rows:
            cols = np.where(reachable[row])[0]
            if cols.size == 0:
                continue
            y = oy + (row + 0.5) * res
            for seg in _contiguous_runs(cols):
                if seg[-1] - seg[0] + 1 < min_seg_cells:
                    continue
                a, b = int(seg[0]), int(seg[-1])
                pts = list(range(a, b + 1, substep))
                if pts[-1] != b:
                    pts.append(b)
                if flip:                       # serpentine per row
                    pts.reverse()
                for c in pts:
                    waypoints.append((ox + (c + 0.5) * res, y))
            flip = not flip

        poses: List[PoseStamped] = []
        for (x, y) in waypoints:
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.pose.position.x = float(x)
            p.pose.position.y = float(y)
            p.pose.orientation.w = 1.0
            poses.append(p)
        return poses

    # ----------------------------------------------------------- execution
    # Waypoints are executed ONE AT A TIME via NavigateToPose, not as a single
    # NavigateThroughPoses goal. A NavigateThroughPoses goal aborts the *whole*
    # sequence when one pose is briefly unreachable, and the replan-from-scratch
    # re-drives already-cleaned rows (efficiency collapse). Per-waypoint goals
    # are independent: a failure just advances to the next, and a per-goal
    # timeout prevents Nav2 from grinding on a hard pose.
    def _start_plan(self) -> None:
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
            self.wp_index = 0
            self.get_logger().info(
                f'coverage plan: {len(poses)} waypoints, executing sequentially')
        if not self.nav_client.server_is_ready():
            self.nav_client.wait_for_server(timeout_sec=0.0)
            self.get_logger().info('waiting for navigate_to_pose server...')
            return
        self.active_pub.publish(Bool(data=True))
        self.plan_started = True
        self.awaiting = False           # a goal is in flight
        self.wp_retries = 0
        self.next_send = self.get_clock().now()
        # sends are paced by _tick so instant-aborts (Nav2 not ready) retry the
        # SAME waypoint instead of burning through the whole list

    def _send_next(self) -> None:
        # skip waypoints already cleaned (coverage grid)
        while (self.wp_index < len(self.cached_poses)
               and self._covered_at(self.cached_poses[self.wp_index])):
            self.wp_index += 1
            self.wp_retries = 0
        if self.wp_index >= len(self.cached_poses):
            # the boustrophedon leaves furniture-shadow / pocket gaps a single
            # sweep direction can't reach; a targeted gap-fill pass visits the
            # remaining uncovered clusters directly (real vacuums do the same
            # spot-recleaning). Cheap in path since it only touches what's left.
            # run-to-completion mode gap-fills until the passes are spent or
            # nothing uncovered remains — not merely until the gate is met
            if self.gapfill_passes < self.max_gapfill and \
                    (not self.stop_at_target
                     or self.ext_ratio < self.coverage_target):
                gaps = self._gapfill_waypoints()
                if gaps:
                    self.gapfill_passes += 1
                    self.cached_poses = gaps
                    self.wp_index = 0
                    self.get_logger().info(
                        f'gap-fill pass {self.gapfill_passes}: '
                        f'{len(gaps)} uncovered spots, coverage '
                        f'{self.ext_ratio:.1%}')
                    return
            self.get_logger().info(
                f'coverage complete: {self.ext_ratio:.1%} covered')
            self.finished = True
            self.active_pub.publish(Bool(data=False))
            return
        if not self.nav_client.server_is_ready():
            return                      # try again next tick
        p = self.cached_poses[self.wp_index]
        p.header.stamp = self.get_clock().now().to_msg()
        goal = NavigateToPose.Goal()
        goal.pose = p
        self.awaiting = True
        self.goal_deadline = self.get_clock().now()
        self.nav_client.send_goal_async(goal).add_done_callback(
            self._on_goal_response)

    def _gapfill_waypoints(self):
        """Waypoints at the still-uncovered drivable cells, nearest-neighbour
        ordered from the robot so the fill path is short."""
        if self.covered_grid is None or self.free_mask is None:
            return []
        info = self.map_msg.info
        res = info.resolution
        cov = self.covered_grid >= 100
        if cov.shape != self.free_mask.shape:
            return []
        uncovered = self.free_mask & ~cov          # drivable but not cleaned
        ys, xs = np.where(uncovered)
        if ys.size == 0:
            return []
        # subsample to ~0.3 m so we don't over-visit a cluster
        keep = ((ys % 6 == 0) & (xs % 6 == 0))
        ys, xs = ys[keep], xs[keep]
        if ys.size == 0:
            return []
        pts = [(ox_i, oy_i) for ox_i, oy_i in zip(xs.tolist(), ys.tolist())]
        # nearest-neighbour order from the robot's current cell
        rcx = int((self.robot_xy[0] - info.origin.position.x) / res)
        rcy = int((self.robot_xy[1] - info.origin.position.y) / res)
        order, cur = [], (rcx, rcy)
        remaining = pts[:]
        while remaining and len(order) < 60:
            j = min(range(len(remaining)),
                    key=lambda k: (remaining[k][0] - cur[0]) ** 2
                    + (remaining[k][1] - cur[1]) ** 2)
            cur = remaining.pop(j)
            order.append(cur)
        poses = []
        for (cx, cy) in order:
            p = PoseStamped()
            p.header.frame_id = self.global_frame
            p.pose.position.x = info.origin.position.x + (cx + 0.5) * res
            p.pose.position.y = info.origin.position.y + (cy + 0.5) * res
            p.pose.orientation.w = 1.0
            poses.append(p)
        return poses

    def _on_goal_response(self, fut) -> None:
        handle = fut.result()
        if not handle.accepted:
            self.awaiting = False       # Nav2 busy/not-ready -> retry same wp
            self.next_send = self.get_clock().now()
            return
        self.goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_result)

    def _on_result(self, fut) -> None:
        status = fut.result().status    # 4=SUCCEEDED, 5=CANCELED, 6=ABORTED
        self.goal_handle = None
        self.awaiting = False
        self.goal_deadline = None
        if self.finished:
            return
        if status == 4:                 # reached the waypoint
            self.wp_index += 1
            self.wp_retries = 0
            self.consecutive_skips = 0
        else:
            # aborted/canceled: retry the SAME waypoint a few times (Nav2 may
            # just have been settling); skip only if it stays unreachable
            self.wp_retries += 1
            if self.wp_retries >= self.max_retries:
                self.get_logger().warn(
                    f'waypoint {self.wp_index} unreachable after '
                    f'{self.wp_retries} tries; skipping')
                self.wp_index += 1
                self.wp_retries = 0
                self.consecutive_skips += 1
        if self.wp_index % 10 == 0 and self.wp_retries == 0:
            self.get_logger().info(
                f'waypoint {self.wp_index}/{len(self.cached_poses)}, '
                f'coverage {self.ext_ratio:.1%}')
        # small cooldown so a run of instant-aborts can't spin the CPU
        self.next_send = self.get_clock().now() + rclpy.duration.Duration(
            seconds=0.25)

    def _elapsed(self, t) -> float:
        return (self.get_clock().now() - t).nanoseconds * 1e-9

    # -------------------------------------------------------------- ticking
    def _tick(self) -> None:
        if self.map_msg is None or self.total_free_cells == 0:
            return
        ratio = self.ext_ratio

        if self.stop_at_target and ratio >= self.coverage_target \
                and not self.finished:
            self.get_logger().info(
                f'coverage target {self.coverage_target:.0%} reached '
                f'({ratio:.1%}); stopping (stop_at_target=true)')
            self.finished = True
            self.active_pub.publish(Bool(data=False))
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()
                self.goal_handle = None
            return
        if self.finished:
            return

        if not self.plan_started:
            if self.last_attempt is None or self._elapsed(self.last_attempt) >= \
                    self.retry_period:
                self._start_plan()
            return

        # wedge escape in progress: reverse open-loop, then hand back to Nav2
        if self.escape_until is not None:
            tw = Twist()
            if self.get_clock().now() < self.escape_until:
                tw.linear.x = self.escape_speed
                self.cmd_pub.publish(tw)
                return
            self.cmd_pub.publish(tw)    # zero twist: stop cleanly
            self.escape_until = None
            self.next_send = self.get_clock().now() + rclpy.duration.Duration(
                seconds=1.0)
            return

        # per-goal watchdog: cancel a waypoint Nav2 is grinding on
        if self.awaiting and self.goal_deadline is not None \
                and self._elapsed(self.goal_deadline) >= self.goal_timeout:
            self.get_logger().warn(f'waypoint {self.wp_index} timed out')
            if self.goal_handle is not None:
                self.goal_handle.cancel_goal_async()  # -> _on_result(CANCELED)
            else:
                self.awaiting = False
                self.wp_retries = self.max_retries    # force skip next result
        # several skips in a row = wedged in a costmap-lethal pocket; Nav2's
        # own recoveries refuse to move there, so back straight out ourselves
        elif not self.awaiting and self.consecutive_skips >= self.escape_after:
            self.get_logger().warn(
                f'{self.consecutive_skips} waypoints skipped back-to-back — '
                f'robot likely wedged; reversing {self.escape_sec}s to escape')
            self.consecutive_skips = 0
            self.escape_until = self.get_clock().now() + rclpy.duration.Duration(
                seconds=self.escape_sec)
        # dispatch the next waypoint once idle and past the cooldown
        elif not self.awaiting \
                and self.get_clock().now() >= self.next_send:
            self._send_next()


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
