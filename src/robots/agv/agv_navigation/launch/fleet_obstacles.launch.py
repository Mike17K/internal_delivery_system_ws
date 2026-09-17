"""Makes every robot in `robots` see every OTHER robot as a cylindrical
obstacle in its own costmaps - see scripts/fleet_obstacle_broadcaster.py
for why/how. workcell_bringup/launch/workcell.launch.py already includes
this for the fleet as configured there; this file is for launching it
standalone (e.g. against an already-running fleet) or with a different
robot set.

    ros2 launch agv_navigation fleet_obstacles.launch.py robots:=agv_1,agv_2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("robots", default_value="agv_1,agv_2", description="Comma-separated fleet robot namespaces"))
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    args.append(DeclareLaunchArgument("robot_radius", default_value="0.45", description="Cylinder radius (m) each robot is represented as in another robot's costmap - padded well past the AGV's own ~0.22m footprint half-diagonal so a moving robot is avoided with real margin instead of right up to contact"))
    args.append(DeclareLaunchArgument("publish_rate_hz", default_value="5.0", description="Obstacle cloud publish rate - matches local_costmap's update_frequency"))
    return args


def launch_setup(context):
    robots = [ns.strip() for ns in LaunchConfiguration("robots").perform(context).split(",") if ns.strip()]

    node = Node(
        package="agv_navigation",
        executable="fleet_obstacle_broadcaster.py",
        name="fleet_obstacle_broadcaster",
        output="screen",
        emulate_tty=True,
        parameters=[{
            "robots": robots,
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "robot_radius": LaunchConfiguration("robot_radius"),
            "publish_rate_hz": LaunchConfiguration("publish_rate_hz"),
        }],
    )
    return [node]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([*get_launch_arguments(), OpaqueFunction(function=launch_setup)])
