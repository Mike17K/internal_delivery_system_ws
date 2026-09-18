"""Open-RMF core services + the AGV fleet adapter, for the fleet's
currently-active robots (agv_1/2/3 - see agv_fleet_adapter/config/
fleet_config.yaml and workcell_bringup/launch/workcell.launch.py's
robots_config). Kept separate from workcell.launch.py rather than folded
in: this is still an opt-in addition, not yet something every normal
launch should bring up.

Node list is a direct port of open-rmf/rmf_demos'
rmf_demos/launch/common.launch.xml (door/lift supervisors omitted - no
doors/lifts in this building) plus our own agv_fleet_adapter in place of
rmf_demos_fleet_adapter's REST-backed one.

    ros2 launch workcell_bringup rmf.launch.py

Run this AFTER every robot's own Nav2 stack is up (workcell.launch.py) -
the fleet adapter needs /agv_N/navigate_to_pose and /agv_N/amcl_pose to
already exist for each robot in fleet_config.yaml.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    args.append(DeclareLaunchArgument(
        "building_map",
        default_value=os.path.join(get_package_share_directory("workcell_description"), "maps", "office_world.building.yaml"),
        description="Path to the hand-authored Open-RMF building map (scripts/generator/rmf_building_map.py)",
    ))
    args.append(DeclareLaunchArgument(
        "nav_graph",
        default_value=os.path.join(get_package_share_directory("workcell_description"), "maps", "nav_graphs", "0.yaml"),
        description="Runtime nav graph generated from building_map at build time",
    ))
    args.append(DeclareLaunchArgument(
        "fleet_config",
        default_value=os.path.join(get_package_share_directory("agv_fleet_adapter"), "config", "fleet_config.yaml"),
        description="agv_fleet_adapter's fleet_config.yaml",
    ))
    args.append(DeclareLaunchArgument("bidding_time_window", default_value="2.0", description="rmf_task_dispatcher's task-bidding time window, seconds"))
    return args


def generate_launch_description() -> LaunchDescription:
    use_sim_time = LaunchConfiguration("use_sim_time")

    traffic_schedule = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_schedule",
        name="rmf_traffic_schedule_primary",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    traffic_blockade = Node(
        package="rmf_traffic_ros2",
        executable="rmf_traffic_blockade",
        output="both",
        parameters=[{"use_sim_time": use_sim_time}],
    )
    # building_map_server takes the map path as a positional CLI arg, not
    # a ROS param - matches common.launch.xml's `args="$(var config_file)"`.
    building_map_server = Node(
        package="rmf_building_map_tools",
        executable="building_map_server",
        arguments=[LaunchConfiguration("building_map")],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    task_dispatcher = Node(
        package="rmf_task_ros2",
        executable="rmf_task_dispatcher",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "bidding_time_window": LaunchConfiguration("bidding_time_window"),
            "use_unique_hex_string_with_task_id": True,
            "server_uri": "",
        }],
    )
    fleet_adapter = Node(
        package="agv_fleet_adapter",
        executable="fleet_adapter",
        output="screen",
        emulate_tty=True,
        # argparse-based (main()'s own -c/-n/-sim), not ROS params - "-sim"
        # is unconditional since this project always runs with sim time by
        # default anyway.
        arguments=["-c", LaunchConfiguration("fleet_config"), "-n", LaunchConfiguration("nav_graph"), "-sim"],
    )

    return LaunchDescription([
        *get_launch_arguments(),
        traffic_schedule,
        traffic_blockade,
        building_map_server,
        task_dispatcher,
        # Confirmed live: Adapter.make() (fleet_adapter.py) failed
        # immediately - "is rmf_traffic_schedule running?" - when started
        # at the same instant as rmf_traffic_schedule itself, which was
        # still mid-startup, not actually down. fleet_adapter.py's own
        # Adapter.make() now also passes wait_time=30s as a first line of
        # defense; this stagger is belt-and-suspenders on top of that,
        # matching how every other multi-process race in this project
        # (odom bootstrap, spawner lock contention, ...) ended up needing
        # both a retry/timeout AND a launch-level head start.
        TimerAction(period=3.0, actions=[fleet_adapter]),
    ])
