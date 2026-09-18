#!/usr/bin/env python3
"""Open-RMF fleet adapter for the AGV fleet.

Adapted from open-rmf/rmf_demos' rmf_demos_fleet_adapter/fleet_adapter.py -
same rmf_easy (rmf_adapter.easy_full_control)/Adapter/RobotCallbacks/
add_easy_fleet skeleton, verified against this machine's actually-installed
rmf_fleet_adapter_python (2.7.2) via `help()` on each class, not just
copied from the demo's usage. The one real difference: RobotClientAPI
there is an HTTP client hitting the demo's own REST "fleet manager" (which
drives a simulated teleporting robot); Nav2RobotAPI (nav2_robot_api.py)
talks directly to each robot's real Nav2 NavigateToPose action instead, so
there's no REST layer and no `fleet_manager:` config block.

This is the first-slice adapter (see docs/open-rmf/ and the approved plan):
navigate/stop only, no teleop/clean/docking/lane-closure handling - those
are real rmf_demos_fleet_adapter features, left out here because nothing
in this fleet's `loop` task capability needs them yet.
"""

import argparse
import asyncio
import datetime
import sys
import threading
import time

import numpy as np
import rclpy
import rclpy.node
import rmf_adapter
from rclpy.parameter import Parameter
from rmf_adapter import Adapter
import rmf_adapter.easy_full_control as rmf_easy

from .nav2_robot_api import Nav2RobotAPI

# How often each robot's state is pushed up to RMF - doesn't need to be
# fast, RMF isn't doing moment-to-moment collision avoidance (Nav2 still
# does that locally, see fleet_obstacle_broadcaster).
UPDATE_PERIOD_SEC = 0.5


def parallel(f):
    def run_in_parallel(*args, **kwargs):
        return asyncio.get_event_loop().run_in_executor(None, f, *args, **kwargs)
    return run_in_parallel


class RobotAdapter:
    def __init__(self, name, configuration, node, api: Nav2RobotAPI, fleet_handle):
        self.name = name
        self.execution = None
        self.cmd_id = 0
        self.update_handle = None
        self.configuration = configuration
        self.node = node
        self.api = api
        self.fleet_handle = fleet_handle
        self.issue_cmd_thread = None
        self.cancel_cmd_event = threading.Event()

    def update(self, state, data) -> None:
        activity_identifier = None
        if self.execution is not None:
            if data.is_command_completed(self.cmd_id):
                self.execution.finished()
                self.execution = None
            else:
                activity_identifier = self.execution.identifier
        self.update_handle.update(state, activity_identifier)

    def make_callbacks(self):
        return rmf_easy.RobotCallbacks(
            lambda destination, execution: self.navigate(destination, execution),
            lambda activity: self.stop(activity),
            lambda category, description, execution: self.execute_action(category, description, execution),
        )

    def navigate(self, destination, execution) -> None:
        self.cmd_id += 1
        self.execution = execution
        self.node.get_logger().info(
            f"[{self.name}] navigating to {destination.name or list(destination.xy)} on map [{destination.map}]"
        )
        self.attempt_cmd_until_success(
            cmd=self.api.navigate,
            args=(self.name, self.cmd_id, destination.position, destination.map, destination.speed_limit),
        )

    def stop(self, activity) -> None:
        if self.execution is not None and self.execution.identifier.is_same(activity):
            self.execution = None
            self.attempt_cmd_until_success(cmd=self.api.stop, args=(self.name, self.cmd_id))

    def execute_action(self, category, description, execution) -> None:
        # fleet_config.yaml's task_capabilities only enables `loop` for
        # this first slice - nothing should ever request a custom action,
        # but RobotCallbacks requires some callable regardless.
        self.node.get_logger().warning(f"[{self.name}] unexpected action request: {category} - finishing as a no-op")
        execution.finished()

    def attempt_cmd_until_success(self, cmd, args) -> None:
        self.cancel_cmd_attempt()

        def loop():
            while not cmd(*args):
                if self.cancel_cmd_event.wait(1.0):
                    break

        self.issue_cmd_thread = threading.Thread(target=loop, daemon=True)
        self.issue_cmd_thread.start()

    def cancel_cmd_attempt(self) -> None:
        if self.issue_cmd_thread is not None:
            self.cancel_cmd_event.set()
            if self.issue_cmd_thread.is_alive():
                self.issue_cmd_thread.join()
        self.cancel_cmd_event.clear()
        self.issue_cmd_thread = None


@parallel
def update_robot(robot: RobotAdapter) -> None:
    data = robot.api.get_data(robot.name)
    if data is None:
        return

    state = rmf_easy.RobotState(data.map, np.array(data.position, dtype=float), data.battery_soc)

    if robot.update_handle is None:
        robot.update_handle = robot.fleet_handle.add_robot(
            robot.name, state, robot.configuration, robot.make_callbacks()
        )
        robot.node.get_logger().info(f"[{robot.name}] registered with RMF at {list(data.position)} on map [{data.map}]")
        return

    robot.update(state, data)


def main(argv=sys.argv) -> None:
    rclpy.init(args=argv)
    rmf_adapter.init_rclcpp()
    args_without_ros = rclpy.utilities.remove_ros_args(argv)

    parser = argparse.ArgumentParser(prog="agv_fleet_adapter", description="Configure and spin up the AGV fleet adapter")
    parser.add_argument("-c", "--config_file", type=str, required=True, help="Path to fleet_config.yaml")
    parser.add_argument("-n", "--nav_graph", type=str, required=True, help="Path to the nav_graph yaml (e.g. nav_graphs/0.yaml)")
    parser.add_argument("-sim", "--use_sim_time", action="store_true", help="Use sim time")
    args = parser.parse_args(args_without_ros[1:])

    fleet_config = rmf_easy.FleetConfiguration.from_config_files(args.config_file, args.nav_graph)
    if not fleet_config:
        raise RuntimeError(f"Failed to parse fleet config file [{args.config_file}]")

    fleet_name = fleet_config.fleet_name
    node = rclpy.node.Node(f"{fleet_name}_command_handle")
    # wait_time defaults to None, which does not retry - confirmed live:
    # rmf.launch.py starts every RMF-core process at once, and Adapter.make()
    # died immediately with "is rmf_traffic_schedule running?" while
    # rmf_traffic_schedule's own log showed it was still mid-startup, not
    # actually down. 30s gives it a real window to come up instead of a
    # single immediate check.
    adapter = Adapter.make(f"{fleet_name}_fleet_adapter", wait_time=datetime.timedelta(seconds=30))
    if not adapter:
        raise RuntimeError("Unable to initialize fleet adapter - rmf_traffic_schedule did not become available within 30s")

    if args.use_sim_time:
        node.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])
        adapter.node.use_sim_time()

    adapter.start()
    time.sleep(1.0)

    fleet_handle = adapter.add_easy_fleet(fleet_config)
    api = Nav2RobotAPI(node, list(fleet_config.known_robots))

    robots = {
        robot_name: RobotAdapter(robot_name, fleet_config.get_known_robot_configuration(robot_name), node, api, fleet_handle)
        for robot_name in fleet_config.known_robots
    }
    node.get_logger().info(f"Fleet [{fleet_name}] tracking robots: {list(robots)}")

    def update_loop() -> None:
        asyncio.set_event_loop(asyncio.new_event_loop())
        while rclpy.ok():
            jobs = [update_robot(robot) for robot in robots.values()]
            asyncio.get_event_loop().run_until_complete(asyncio.wait(jobs))
            time.sleep(UPDATE_PERIOD_SEC)

    update_thread = threading.Thread(target=update_loop, daemon=True)
    update_thread.start()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
