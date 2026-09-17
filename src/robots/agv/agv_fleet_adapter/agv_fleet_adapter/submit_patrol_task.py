#!/usr/bin/env python3
"""Submit an Open-RMF "patrol" (loop) task, for testing agv_fleet_adapter
without depending on the external open-rmf/rmf_demos clone -
rmf_demos_tasks' dispatch_patrol.py only needs rclpy + rmf_task_msgs
(already installed with the rest of the RMF apt packages), so this is a
trimmed equivalent that stays inside this workspace.

    ros2 run agv_fleet_adapter submit_patrol_task -p floor_0_office_n_0 floor_0_office_n_1 -n 3 --use_sim_time
"""

import argparse
import json
import sys
import uuid

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import QoSDurabilityPolicy as Durability
from rclpy.qos import QoSHistoryPolicy as History
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy as Reliability
from rmf_task_msgs.msg import ApiRequest, ApiResponse


def main(argv=sys.argv) -> None:
    rclpy.init(args=argv)
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(prog="submit_patrol_task", description="Submit an Open-RMF patrol/loop task")
    parser.add_argument("-p", "--places", required=True, nargs="+", type=str, help="Nav-graph waypoint names to patrol between")
    parser.add_argument("-n", "--rounds", type=int, default=1, help="Number of loops to perform")
    parser.add_argument("--use_sim_time", action="store_true", help="Use sim time")
    args = parser.parse_args(args_without_ros[1:])

    node = Node("submit_patrol_task")
    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

    transient_qos = QoSProfile(history=History.KEEP_LAST, depth=1, reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL)
    request_pub = node.create_publisher(ApiRequest, "task_api_requests", transient_qos)

    now = node.get_clock().now().to_msg()
    start_time_millis = now.sec * 1000 + round(now.nanosec / 1e6)
    payload = {
        "type": "dispatch_task_request",
        "request": {
            "unix_millis_request_time": start_time_millis,
            "unix_millis_earliest_start_time": start_time_millis,
            "requester": "agv_fleet_adapter",
            "category": "patrol",
            "description": {"places": args.places, "rounds": args.rounds},
        },
    }

    msg = ApiRequest()
    msg.request_id = f"patrol_{uuid.uuid4()}"
    msg.json_msg = json.dumps(payload)

    got_response = False

    def on_response(response_msg: ApiResponse) -> None:
        nonlocal got_response
        if response_msg.request_id == msg.request_id:
            got_response = True
            node.get_logger().info(f"response: {response_msg.json_msg}")

    node.create_subscription(ApiResponse, "task_api_responses", on_response, QoSProfile(history=History.KEEP_LAST, depth=10, reliability=Reliability.RELIABLE, durability=Durability.TRANSIENT_LOCAL))

    node.get_logger().info(f"submitting: {json.dumps(payload, indent=2)}")
    request_pub.publish(msg)

    deadline = node.get_clock().now() + rclpy.duration.Duration(seconds=5.0)
    while rclpy.ok() and not got_response and node.get_clock().now() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)

    if not got_response:
        node.get_logger().warning("no response from rmf_task_dispatcher within 5s - is rmf.launch.py running?")

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
