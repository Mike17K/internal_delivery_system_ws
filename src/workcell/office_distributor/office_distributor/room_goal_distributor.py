#!/usr/bin/env python3
"""Periodically submits a "go deliver to a random office" task to Open-RMF.

Used to call Nav2's NavigateToPose directly, with its own per-robot
ActionClient/busy-tracking state machine. That was the wrong layer for
this: deciding *which* robot goes *where* and *when* is task allocation,
Open-RMF Core's job (docs/open-rmf/02-openrmf-nav2-logic-operation.md's
"Layered Responsibility Model" - "If you find yourself trying to solve a
scheduling/allocation problem inside a Nav2 plugin ... that's a sign it's
in the wrong layer"), not something to reimplement ad hoc here. It's also
why every robot RMF already controls (agv_navigation's agv_fleet_adapter)
had to be manually excluded from this node's own `robots` list, to avoid
the two systems fighting over the same navigate_to_pose action server.

Now this node only ever picks a random office and submits a `patrol`
dispatch request (task_api_requests, the same mechanism
agv_fleet_adapter/submit_patrol_task.py uses) with that one place and
rounds=1 - the simplest task shape that means "go here, once". Open-RMF's
own task dispatcher decides which idle robot in the fleet actually takes
it, handles traffic scheduling against every other robot's task, and
tracks completion - none of that is this node's job anymore. That also
means no `robots` parameter: the fleet is whatever
agv_fleet_adapter/config/fleet_config.yaml says it is, and this node
doesn't need to know.

Room locations still aren't hand-typed: they're read straight out of the
generated office_floor model (workcell_description/scripts/generator/) by
finding every room's ceiling-light `<include>` - same technique
workcell_description/scripts/generator/rmf_building_map.py uses to build
the nav graph these room names are waypoints in.
"""

import json
import random
import re
import uuid
from pathlib import Path
from typing import Dict, Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from rmf_task_msgs.msg import ApiRequest, ApiResponse

Room = Tuple[float, float, float]  # world-frame x, y, yaw (yaw unused - RMF places are graph waypoints, not raw poses)

INCLUDE_RE = re.compile(r"<include>(.*?)</include>", re.S)
NAME_RE = re.compile(r"<name>([^<]+)</name>")
URI_RE = re.compile(r"<uri>([^<]+)</uri>")
POSE_RE = re.compile(r"<pose[^>]*>([^<]+)</pose>")

TRANSIENT_LOCAL_QOS = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL, history=HistoryPolicy.KEEP_LAST)


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


class RoomTaskDistributor(Node):

    def __init__(self) -> None:
        super().__init__("office_distributor")

        self.declare_parameter("model_package", "workcell_description")
        self.declare_parameter("model_relative_path", "models/office_floor/model.sdf")
        self.declare_parameter("dispatch_period_sec", 6.0)
        self.declare_parameter("random_seed", 0)  # 0 = unseeded (time-based)
        self.declare_parameter("requester", "office_distributor")

        model_package = self.get_parameter("model_package").value
        model_relative_path = self.get_parameter("model_relative_path").value
        dispatch_period = float(self.get_parameter("dispatch_period_sec").value)
        seed = int(self.get_parameter("random_seed").value)
        self.requester = self.get_parameter("requester").value

        model_sdf_path = Path(get_package_share_directory(model_package)) / model_relative_path
        all_rooms = parse_rooms(model_sdf_path)
        # Delivery goals only -- the meeting room ("*_meeting_room") and
        # break room ("*_break_room", the kitchenette) aren't desks to
        # deliver to, so only offices ("*_office_{n,s}_{i}", per
        # floors.py's naming) are dispatchable.
        self.rooms = [room_id for room_id in all_rooms if "_office_" in room_id]
        if not self.rooms:
            raise RuntimeError(f"office_distributor: no office rooms found in {model_sdf_path}")
        self.get_logger().info(
            f"Loaded {len(self.rooms)} office rooms (of {len(all_rooms)} total rooms) from {model_sdf_path}"
        )

        self.rng = random.Random(seed) if seed else random.Random()
        self.last_room: Optional[str] = None

        self.request_pub = self.create_publisher(ApiRequest, "task_api_requests", TRANSIENT_LOCAL_QOS)
        self.create_subscription(ApiResponse, "task_api_responses", self._on_response, TRANSIENT_LOCAL_QOS)

        self.timer = self.create_timer(dispatch_period, self._tick)
        self.get_logger().info(f"Submitting a delivery task to Open-RMF every {dispatch_period:.1f}s")

    def _pick_room(self) -> str:
        # Avoid immediately re-submitting the room just visited.
        candidates = [r for r in self.rooms if r != self.last_room]
        room = self.rng.choice(candidates or self.rooms)
        self.last_room = room
        return room

    def _tick(self) -> None:
        room_id = self._pick_room()
        now = self.get_clock().now().to_msg()
        start_time_millis = now.sec * 1000 + round(now.nanosec / 1e6)

        payload = {
            "type": "dispatch_task_request",
            "request": {
                "unix_millis_request_time": start_time_millis,
                "unix_millis_earliest_start_time": start_time_millis,
                "requester": self.requester,
                # "patrol" with one place and rounds=1 is just "go here,
                # once" - the same category submit_patrol_task.py already
                # proved works end-to-end, rather than a second, unverified
                # task category.
                "category": "patrol",
                "description": {"places": [room_id], "rounds": 1},
            },
        }

        msg = ApiRequest()
        msg.request_id = f"office_distributor_{uuid.uuid4()}"
        msg.json_msg = json.dumps(payload)
        self.request_pub.publish(msg)
        self.get_logger().info(f"submitted delivery task -> {room_id}")

    def _on_response(self, response_msg: ApiResponse) -> None:
        if response_msg.request_id.startswith("office_distributor_"):
            self.get_logger().info(f"dispatcher response: {response_msg.json_msg}")


def main() -> None:
    rclpy.init()
    node = RoomTaskDistributor()
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
