#!/usr/bin/env python3
# Copyright 2026 Jayadev Rana
# SPDX-License-Identifier: Apache-2.0
"""Publish the robot's ground-truth pose for honest coverage measurement.

The regression harness must not grade a module against that module's own belief.
In this Gazebo setup the ``gz-sim-odometry-publisher`` produces *noise-free*
odometry derived from the model's true world pose, so ``/odom`` is effectively
ground truth. The odom frame is pinned to the robot's spawn pose, which for the
living_room bringup coincides with the SLAM map origin, so odom xy == map xy
(a constant spawn offset can be supplied if the robot starts elsewhere).

This node relabels ``/odom`` into a clean map-frame ``PoseStamped`` that the
coverage meter integrates. (The relocalization test does not use this: after a
teleport, odometry does not jump, so that test scores AMCL against the
injector's known teleport target instead.)

  sub  /odom          nav_msgs/Odometry
  pub  ~/pose         geometry_msgs/PoseStamped   (map frame)
  pub  ~/yaw          std_msgs/Float32
"""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class GroundTruth(Node):
    def __init__(self) -> None:
        super().__init__('ground_truth')
        self.declare_parameter('map_frame', 'map')
        # constant map<-odom offset = robot spawn pose in the map frame
        self.declare_parameter('spawn_x', 0.0)
        self.declare_parameter('spawn_y', 0.0)
        self.declare_parameter('spawn_yaw', 0.0)

        self.map_frame = self.get_parameter('map_frame').value
        self.sx = self.get_parameter('spawn_x').value
        self.sy = self.get_parameter('spawn_y').value
        self.syaw = self.get_parameter('spawn_yaw').value

        self.pose_pub = self.create_publisher(PoseStamped, '~/pose', 10)
        self.yaw_pub = self.create_publisher(Float32, '~/yaw', 10)
        self.create_subscription(Odometry, 'odom', self._on_odom, 20)
        self.get_logger().info('ground_truth up; relabeling /odom as map-frame truth')

    def _on_odom(self, msg: Odometry) -> None:
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        oyaw = _yaw_from_quat(q.x, q.y, q.z, q.w)

        # map pose = spawn offset composed with odom pose
        c, s = math.cos(self.syaw), math.sin(self.syaw)
        mx = c * p.x - s * p.y + self.sx
        my = s * p.x + c * p.y + self.sy
        myaw = oyaw + self.syaw

        out = PoseStamped()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.map_frame
        out.pose.position.x = mx
        out.pose.position.y = my
        out.pose.orientation.z = math.sin(myaw / 2.0)
        out.pose.orientation.w = math.cos(myaw / 2.0)
        self.pose_pub.publish(out)
        self.yaw_pub.publish(Float32(data=float(myaw)))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GroundTruth()
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
