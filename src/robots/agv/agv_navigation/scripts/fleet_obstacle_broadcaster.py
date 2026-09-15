#!/usr/bin/env python3
"""Publishes every robot's position as a cylindrical obstacle into every
OTHER robot's costmaps, so each robot's Nav2 planner/controller actually
plans around the rest of the fleet instead of only reacting to whatever
its own lidar happens to see of them.

Why this exists: this fleet has no shared TF tree (docs/agv-fleet/
04-known-issues-and-next-steps.md, "No fleet-wide TF view") - each robot's
tf lives entirely on its own /<namespace>/tf topic, so one robot's Nav2
stack has no built-in way to ask "where is the other robot right now."
This node is the aggregator/relay that doc flags as the missing piece: it
listens to every fleet robot's own /<namespace>/tf (+ tf_static) into a
private tf2 buffer per robot, looks up that robot's live map -> base_link
pose, and republishes a filled disk of points at that (x, y) - the
robot's cylindrical footprint - onto every OTHER robot's own
/<namespace>/fleet_obstacles topic. nav2_params.yaml's
fleet_obstacle_layer (an ObstacleLayer instance added to both
local_costmap and global_costmap) subscribes to that and marks/clears it
exactly like a real sensor observation, closing the loop from "informed
where the other robot is" to "actually avoided."

The published frame is "map": the one frame every robot's own AMCL agrees
on numerically (single shared map_server), even though each robot's tf
tree is otherwise disjoint from the others'. So no cross-robot tf lookup
is ever needed - only tf *within* each source robot's own tree (to get
its pose) and *within* each target robot's own tree (already true of
anything its costmap consumes, including this).
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header
from tf2_msgs.msg import TFMessage
from tf2_ros import Buffer, ExtrapolationException, LookupException

# Mirrors tf2_ros.TransformListener's own defaults (transform_listener.py) -
# tf2_ros hardcodes absolute "/tf"/"/tf_static" internally with no way to
# point it at a namespaced topic, which is why this node subscribes by hand
# instead of using TransformListener (one instance per source robot,
# feeding a private tf2_ros.Buffer via set_transform()/set_transform_static()).
DYNAMIC_TF_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.VOLATILE, history=HistoryPolicy.KEEP_LAST)
STATIC_TF_QOS = QoSProfile(depth=100, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


def disk_points(radius: float, spacing: float, z: float) -> List[Tuple[float, float, float]]:
    """A filled grid of local (x, y, z) points covering a disk of `radius`
    - the cylinder ObstacleLayer marks as an obstacle. Filled, not just a
    ring, so the whole footprint reads as occupied at the costmap's own
    resolution rather than leaving its center passable.
    """
    points = []
    steps = max(1, int(math.ceil(radius / spacing)))
    for i in range(-steps, steps + 1):
        for j in range(-steps, steps + 1):
            x, y = i * spacing, j * spacing
            if x * x + y * y <= radius * radius:
                points.append((x, y, z))
    return points


@dataclass
class RobotTf:
    namespace: str
    buffer: Buffer = field(default_factory=lambda: Buffer(cache_time=Duration(seconds=5)))


class FleetObstacleBroadcaster(Node):

    def __init__(self) -> None:
        super().__init__("fleet_obstacle_broadcaster")

        self.declare_parameter("robots", ["agv_1", "agv_2"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("robot_radius", 0.22)
        self.declare_parameter("point_spacing", 0.04)
        self.declare_parameter("cloud_height", 0.3)
        self.declare_parameter("publish_rate_hz", 5.0)
        self.declare_parameter("stale_timeout_sec", 1.0)

        robots = [str(ns) for ns in self.get_parameter("robots").value]
        if not robots:
            raise RuntimeError("fleet_obstacle_broadcaster: 'robots' parameter is empty")

        self.map_frame = self.get_parameter("map_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.stale_timeout = Duration(seconds=float(self.get_parameter("stale_timeout_sec").value))
        self.template_points = disk_points(
            float(self.get_parameter("robot_radius").value),
            float(self.get_parameter("point_spacing").value),
            float(self.get_parameter("cloud_height").value),
        )

        self.robot_tfs: Dict[str, RobotTf] = {ns: RobotTf(namespace=ns) for ns in robots}
        self.fleet_obstacle_pubs = {ns: self.create_publisher(PointCloud2, f"/{ns}/fleet_obstacles", 5) for ns in robots}

        for ns, rtf in self.robot_tfs.items():
            self.create_subscription(TFMessage, f"/{ns}/tf", self._make_tf_callback(rtf, static=False), DYNAMIC_TF_QOS)
            self.create_subscription(TFMessage, f"/{ns}/tf_static", self._make_tf_callback(rtf, static=True), STATIC_TF_QOS)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(f"Broadcasting fleet obstacles for: {', '.join(robots)}")

    def _make_tf_callback(self, rtf: RobotTf, *, static: bool):
        def callback(msg: TFMessage) -> None:
            for transform in msg.transforms:
                if static:
                    rtf.buffer.set_transform_static(transform, f"{rtf.namespace}_tf")
                else:
                    rtf.buffer.set_transform(transform, f"{rtf.namespace}_tf")
        return callback

    def _lookup_position(self, rtf: RobotTf) -> Optional[Tuple[float, float]]:
        try:
            transform = rtf.buffer.lookup_transform(self.map_frame, self.base_frame, Time())
        except (LookupException, ExtrapolationException) as exc:
            self.get_logger().warning(
                f"[{rtf.namespace}] no {self.map_frame}->{self.base_frame} tf yet: {exc}", throttle_duration_sec=5.0
            )
            return None

        age = self.get_clock().now() - Time.from_msg(transform.header.stamp)
        if age > self.stale_timeout:
            self.get_logger().warning(
                f"[{rtf.namespace}] pose is {age.nanoseconds / 1e9:.1f}s stale, skipping", throttle_duration_sec=5.0
            )
            return None

        return transform.transform.translation.x, transform.transform.translation.y

    def _tick(self) -> None:
        positions = {ns: self._lookup_position(rtf) for ns, rtf in self.robot_tfs.items()}
        stamp = self.get_clock().now().to_msg()

        for ns, pub in self.fleet_obstacle_pubs.items():
            points = [
                (x + dx, y + dy, dz)
                for other_ns, position in positions.items() if other_ns != ns and position is not None
                for x, y in (position,)
                for dx, dy, dz in self.template_points
            ]
            header = Header(stamp=stamp, frame_id=self.map_frame)
            pub.publish(create_cloud_xyz32(header, points))


def main() -> None:
    rclpy.init()
    node = FleetObstacleBroadcaster()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
