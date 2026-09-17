#!/usr/bin/env python3
"""Sends each available robot to a random office, one at a time.

Room locations aren't hand-typed: they're read straight out of the
generated office_floor model (workcell_description/scripts/generator/) by
finding every room's ceiling-light `<include>`. room_compositions.py
places each room's CeilingLight at that room's local origin (0, 0, ...),
so once the room's own door_side rotation is applied, the light's final
world (x, y, yaw) in the generated SDF is exactly the room's center and
door-facing direction -- no separate copy of the generator's layout math
needed here. Re-running generate.py (a different seed, room count,
meeting/break room slot, ...) is picked up on the next launch with no
changes to this node.

Only offices ("*_office_{n,s}_{i}") are delivery targets -- the meeting
room and break room (kitchenette) are filtered out in __init__, since
they aren't desks anyone is delivering to.

Each namespace in the `robots` parameter gets its own NavigateToPose
action client and is dispatched independently: a robot only receives a
new random goal once its current one has finished (succeeded, aborted, or
canceled) and a short random dwell has passed, so an in-progress delivery
is never preempted by a freshly-picked goal.
"""

import math
import random
import re
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

Room = Tuple[float, float, float]  # world-frame x, y, yaw

INCLUDE_RE = re.compile(r"<include>(.*?)</include>", re.S)
NAME_RE = re.compile(r"<name>([^<]+)</name>")
URI_RE = re.compile(r"<uri>([^<]+)</uri>")
POSE_RE = re.compile(r"<pose[^>]*>([^<]+)</pose>")


def parse_rooms(model_sdf_path: Path) -> Dict[str, Room]:
    """Every room's (x, y, yaw) in the generated office_floor model, keyed
    by room id (e.g. "floor_0_office_n_0", "floor_0_meeting_room").

    A room has no SDF element of its own -- only its furniture does -- so
    this keys off the one piece every room (and only a room) has exactly
    one of: its CeilingLight include.
    """
    text = model_sdf_path.read_text()
    rooms: Dict[str, Room] = {}
    for block in INCLUDE_RE.findall(text):
        uri = URI_RE.search(block)
        name = NAME_RE.search(block)
        pose = POSE_RE.search(block)
        if not (uri and name and pose):
            continue
        if "ceiling_light" not in uri.group(1) or not name.group(1).endswith("_light"):
            continue
        room_id = name.group(1)[: -len("_light")]
        x, y, _z, _roll, _pitch, yaw = (float(v) for v in pose.group(1).split())
        rooms[room_id] = (x, y, yaw)
    return rooms


def yaw_to_quaternion(yaw: float) -> Tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))


@dataclass
class RobotState:
    namespace: str
    client: ActionClient
    busy: bool = False
    current_room: Optional[str] = None
    next_eligible: float = 0.0
    goal_handle: object = None
    server_seen: bool = False


class RoomGoalDistributor(Node):

    def __init__(self) -> None:
        super().__init__("office_distributor")

        self.declare_parameter("robots", ["agv_1"])
        self.declare_parameter("map_frame", "map")
        self.declare_parameter("model_package", "workcell_description")
        self.declare_parameter("model_relative_path", "models/office_floor/model.sdf")
        self.declare_parameter("min_dwell_sec", 3.0)
        self.declare_parameter("max_dwell_sec", 8.0)
        self.declare_parameter("dispatch_period_sec", 1.0)
        self.declare_parameter("random_seed", 0)  # 0 = unseeded (time-based)

        robots = [str(ns) for ns in self.get_parameter("robots").value]
        self.map_frame = self.get_parameter("map_frame").value
        model_package = self.get_parameter("model_package").value
        model_relative_path = self.get_parameter("model_relative_path").value
        self.min_dwell = float(self.get_parameter("min_dwell_sec").value)
        self.max_dwell = float(self.get_parameter("max_dwell_sec").value)
        dispatch_period = float(self.get_parameter("dispatch_period_sec").value)
        seed = int(self.get_parameter("random_seed").value)

        if not robots:
            raise RuntimeError("office_distributor: 'robots' parameter is empty -- nothing to dispatch to")

        model_sdf_path = Path(get_package_share_directory(model_package)) / model_relative_path
        all_rooms = parse_rooms(model_sdf_path)
        # Delivery goals only -- the meeting room ("*_meeting_room") and
        # break room ("*_break_room", the kitchenette) aren't desks to
        # deliver to, so only offices ("*_office_{n,s}_{i}", per
        # floors.py's naming) are dispatchable.
        self.rooms = {room_id: pose for room_id, pose in all_rooms.items() if "_office_" in room_id}
        if not self.rooms:
            raise RuntimeError(f"office_distributor: no office rooms found in {model_sdf_path}")
        self.get_logger().info(
            f"Loaded {len(self.rooms)} office rooms (of {len(all_rooms)} total rooms) from {model_sdf_path}"
        )

        self.rng = random.Random(seed) if seed else random.Random()

        self.robot_states: Dict[str, RobotState] = {
            ns: RobotState(namespace=ns, client=ActionClient(self, NavigateToPose, f"/{ns}/navigate_to_pose"))
            for ns in robots
        }
        for ns in robots:
            self.get_logger().info(f"[{ns}] dispatching against /{ns}/navigate_to_pose")

        self.timer = self.create_timer(dispatch_period, self._tick)

    # ── room selection ──────────────────────────────────────────────────
    def _pick_room(self, robot: RobotState) -> str:
        # Avoid immediately re-sending a robot to the room it just left.
        candidates = [r for r in self.rooms if r != robot.current_room]
        return self.rng.choice(candidates or list(self.rooms))

    # ── dispatch loop ───────────────────────────────────────────────────
    def _tick(self) -> None:
        now = time.monotonic()
        for robot in self.robot_states.values():
            if robot.busy or now < robot.next_eligible:
                continue
            if not robot.client.server_is_ready():
                continue
            if not robot.server_seen:
                robot.server_seen = True
                self.get_logger().info(f"[{robot.namespace}] navigate_to_pose action server is up")
            self._send_goal(robot)

    def _send_goal(self, robot: RobotState) -> None:
        room_id = self._pick_room(robot)
        x, y, yaw = self.rooms[room_id]

        goal_pose = PoseStamped()
        goal_pose.header.frame_id = self.map_frame
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = x
        goal_pose.pose.position.y = y
        qx, qy, qz, qw = yaw_to_quaternion(yaw)
        goal_pose.pose.orientation.x = qx
        goal_pose.pose.orientation.y = qy
        goal_pose.pose.orientation.z = qz
        goal_pose.pose.orientation.w = qw

        robot.busy = True
        robot.current_room = room_id
        self.get_logger().info(f"[{robot.namespace}] -> {room_id} ({x:.2f}, {y:.2f})")

        send_future = robot.client.send_goal_async(NavigateToPose.Goal(pose=goal_pose))
        send_future.add_done_callback(lambda f, r=robot: self._on_goal_response(r, f))

    def _on_goal_response(self, robot: RobotState, future) -> None:
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warning(f"[{robot.namespace}] goal to {robot.current_room} was rejected")
            robot.busy = False
            robot.next_eligible = time.monotonic() + self.rng.uniform(self.min_dwell, self.max_dwell)
            return

        robot.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(lambda f, r=robot: self._on_result(r, f))

    def _on_result(self, robot: RobotState, future) -> None:
        status = future.result().status
        reached = status == GoalStatus.STATUS_SUCCEEDED
        if reached:
            self.get_logger().info(f"[{robot.namespace}] reached {robot.current_room} (status={status})")
        else:
            self.get_logger().warning(f"[{robot.namespace}] did not reach {robot.current_room} (status={status})")

        robot.busy = False
        robot.goal_handle = None
        robot.next_eligible = time.monotonic() + self.rng.uniform(self.min_dwell, self.max_dwell)

    def destroy_node(self) -> None:
        for robot in self.robot_states.values():
            if robot.busy and robot.goal_handle is not None:
                robot.goal_handle.cancel_goal_async()
        super().destroy_node()


def main() -> None:
    rclpy.init()
    node = RoomGoalDistributor()
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
