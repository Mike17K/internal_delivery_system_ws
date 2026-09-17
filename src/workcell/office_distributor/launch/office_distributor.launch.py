"""Randomly sends each robot in `robots` to a room from the generated
office scene, one goal at a time per robot.

    ros2 launch office_distributor office_distributor.launch.py robots:=agv_1
    ros2 launch office_distributor office_distributor.launch.py robots:=agv_1,agv_2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("robots", default_value="agv_1", description="Comma-separated robot namespaces to dispatch goals to (e.g. agv_1,agv_2)"))
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    args.append(DeclareLaunchArgument("map_frame", default_value="map", description="Frame goals are expressed in"))
    args.append(DeclareLaunchArgument("min_dwell_sec", default_value="3.0", description="Minimum wait after reaching a room before the next goal"))
    args.append(DeclareLaunchArgument("max_dwell_sec", default_value="8.0", description="Maximum wait after reaching a room before the next goal"))
    args.append(DeclareLaunchArgument("random_seed", default_value="0", description="Seed for reproducible goal picking; 0 = unseeded"))
    return args


def launch_setup(context):
    robots = [ns.strip() for ns in LaunchConfiguration("robots").perform(context).split(",") if ns.strip()]

    node = Node(
        package="office_distributor",
        executable="room_goal_distributor",
        name="office_distributor",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "robots": robots,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "map_frame": LaunchConfiguration("map_frame"),
            "min_dwell_sec": LaunchConfiguration("min_dwell_sec"),
            "max_dwell_sec": LaunchConfiguration("max_dwell_sec"),
            "random_seed": LaunchConfiguration("random_seed"),
        }],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([*get_launch_arguments(), OpaqueFunction(function=launch_setup)])
