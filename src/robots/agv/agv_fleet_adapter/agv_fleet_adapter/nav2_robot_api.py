#!/usr/bin/env python3
"""Nav2RobotAPI: the robot-side half of the fleet adapter.

Open-RMF's own demos (open-rmf/rmf_demos, rmf_demos_fleet_adapter/
RobotClientAPI.py) implement this exact contract - navigate/stop/get_data
- as an HTTP client hitting their own REST "fleet manager", which in turn
drives a *simulated* teleporting robot (rmf_fleet_msgs/PathRequest), not
Nav2. That whole REST/fleet-manager layer only exists because a demo (or
a real vendor robot) might not expose direct ROS 2 control. We already
have real ROS 2 access to every AGV's own Nav2 stack, so this class talks
to it directly: no HTTP, no separate fleet-manager process.

MAP_NAME is hardcoded "L1" - office_world.building.yaml only has the one
level, matching workcell_description/maps/office_world.building.yaml's
`levels: L1:` key and the runtime nav graph's own `levels: L1:` key.
"""

import math
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Union

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

MAP_NAME = "L1"

# nav2_amcl publishes amcl_pose with TRANSIENT_LOCAL durability (confirmed
# live via `ros2 topic info /agv_1/amcl_pose -v`) - it latches its last
# message for late-joining subscribers, exactly like /map or /tf_static.
# amcl only re-publishes on the initial pose set and then again only once
# the robot moves past update_min_d/update_min_a (nav2_params.yaml) - it
# does NOT tick continuously. A plain depth-int subscription here defaults
# to VOLATILE, which is QoS-*compatible* with a transient_local publisher
# but does NOT get the already-latched message - only ones sent after the
# subscription exists. Since this node starts after amcl's one-time
# initial publish, that meant last_pose stayed None forever: the robot
# never registered with RMF, so nothing could ever be commanded to move
# in the first place. Requesting TRANSIENT_LOCAL here is what actually
# fixes it - confirmed live: `ros2 topic echo /agv_1/amcl_pose --once`
# returns the latched pose instantly, exactly this same mechanism.
AMCL_POSE_QOS = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def yaw_to_quaternion(yaw: float):
    return 0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class RobotUpdateData:
    """Matches rmf_demos_fleet_adapter's RobotClientAPI.RobotUpdateData
    contract: .position/.map/.battery_soc read directly,
    .is_command_completed(cmd_id) checked by RobotAdapter.update()."""

    def __init__(self, robot_name: str, position: List[float], last_completed_cmd_id: Optional[int]):
        self.robot_name = robot_name
        self.position = position
        self.map = MAP_NAME
        self.battery_soc = 1.0  # no real battery model this slice - see fleet_config.yaml
        self._last_completed_cmd_id = last_completed_cmd_id

    def is_command_completed(self, cmd_id: int) -> bool:
        return self._last_completed_cmd_id == cmd_id


@dataclass
class _RobotChannel:
    client: ActionClient
    issued_cmd_id: Optional[int] = None
    accepted: Optional[bool] = None
    goal_handle: object = None
    last_completed_cmd_id: Optional[int] = None
    last_pose: Optional[PoseWithCovarianceStamped] = None


class Nav2RobotAPI:

    def __init__(self, node: Node, robot_names: List[str]) -> None:
        self.node = node
        self.robots: Dict[str, _RobotChannel] = {
            name: _RobotChannel(client=ActionClient(node, NavigateToPose, f"/{name}/navigate_to_pose"))
            for name in robot_names
        }
        for name, channel in self.robots.items():
            node.create_subscription(
                PoseWithCovarianceStamped, f"/{name}/amcl_pose",
                lambda msg, ch=channel: setattr(ch, "last_pose", msg), AMCL_POSE_QOS,
            )

    # ── RobotClientAPI contract ──────────────────────────────────────────
    def navigate(self, robot_name: str, cmd_id: int, position, map_name: str, speed_limit: float = 0.0) -> bool:
        channel = self.robots[robot_name]
        if channel.issued_cmd_id != cmd_id:
            # A new command - (re)send exactly once; retries of the same
            # cmd_id just poll the outcome below instead of resending.
            channel.issued_cmd_id = cmd_id
            channel.accepted = None
            channel.goal_handle = None

            goal = NavigateToPose.Goal()
            goal.pose.header.frame_id = "map"  # this robot's own Nav2 global frame - see MAP_NAME's own docstring
            goal.pose.header.stamp = self.node.get_clock().now().to_msg()
            goal.pose.pose.position.x = float(position[0])
            goal.pose.pose.position.y = float(position[1])
            qx, qy, qz, qw = yaw_to_quaternion(float(position[2]))
            goal.pose.pose.orientation.x = qx
            goal.pose.pose.orientation.y = qy
            goal.pose.pose.orientation.z = qz
            goal.pose.pose.orientation.w = qw

            send_future = channel.client.send_goal_async(goal)
            send_future.add_done_callback(
                lambda f, ch=channel, cid=cmd_id: self._on_goal_response(ch, cid, f)
            )
            return False

        return bool(channel.accepted)

    def stop(self, robot_name: str, cmd_id: int) -> bool:
        channel = self.robots[robot_name]
        if channel.goal_handle is not None:
            channel.goal_handle.cancel_goal_async()
        channel.last_completed_cmd_id = cmd_id
        return True

    def get_data(self, robot_name: Optional[str] = None) -> Union[RobotUpdateData, List[RobotUpdateData], None]:
        if robot_name is not None:
            return self._robot_update_data(robot_name)
        data = [self._robot_update_data(name) for name in self.robots]
        return [d for d in data if d is not None]

    # ── internals ─────────────────────────────────────────────────────────
    def _robot_update_data(self, robot_name: str) -> Optional[RobotUpdateData]:
        channel = self.robots[robot_name]
        if channel.last_pose is None:
            return None
        pose = channel.last_pose.pose.pose
        yaw = yaw_from_quaternion(pose.orientation)
        return RobotUpdateData(
            robot_name, [pose.position.x, pose.position.y, yaw], channel.last_completed_cmd_id
        )

    def _on_goal_response(self, channel: _RobotChannel, cmd_id: int, future) -> None:
        if channel.issued_cmd_id != cmd_id:
            return  # superseded by a newer command already
        goal_handle = future.result()
        channel.accepted = goal_handle.accepted
        if goal_handle.accepted:
            channel.goal_handle = goal_handle
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda f, ch=channel, cid=cmd_id: self._on_result(ch, cid, f)
            )

    def _on_result(self, channel: _RobotChannel, cmd_id: int, future) -> None:
        # Completed regardless of the actual Nav2 outcome (succeeded,
        # aborted, canceled) - RMF needs to know the command has finished
        # either way, not get stuck waiting forever on a failed goal.
        if channel.issued_cmd_id == cmd_id:
            channel.last_completed_cmd_id = cmd_id
