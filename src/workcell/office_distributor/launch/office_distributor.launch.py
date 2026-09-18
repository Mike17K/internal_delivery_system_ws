"""Periodically submits a "go deliver to a random office" task to
Open-RMF's task dispatcher - see room_goal_distributor.py for why this
calls RMF instead of commanding a robot directly. Requires
workcell_bringup/launch/rmf.launch.py (Open-RMF core + agv_fleet_adapter)
to already be running - this node only ever publishes task requests, it
does not know or care which robots exist.

    ros2 launch office_distributor office_distributor.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    args.append(DeclareLaunchArgument("dispatch_period_sec", default_value="6.0", description="Seconds between submitted delivery tasks"))
    args.append(DeclareLaunchArgument("random_seed", default_value="0", description="Seed for reproducible room picking; 0 = unseeded"))
    return args


def generate_launch_description() -> LaunchDescription:
    node = Node(
        package="office_distributor",
        executable="room_goal_distributor",
        name="office_distributor",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "dispatch_period_sec": LaunchConfiguration("dispatch_period_sec"),
            "random_seed": LaunchConfiguration("random_seed"),
        }],
    )
    return LaunchDescription([*get_launch_arguments(), node])
