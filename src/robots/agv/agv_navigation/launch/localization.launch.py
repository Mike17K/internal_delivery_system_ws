from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Fleet-wide, launched ONCE (not per-robot) from workcell_bringup/launch/
# workcell.launch.py, only when slam:=false. Every robot's AMCL (see
# agv_navigation/launch/navigation.launch.py) localizes against this single
# shared, unnamespaced /map topic - the fleet has one map, not one per robot.


def generate_launch_description():
    map_yaml_file = LaunchConfiguration("map")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")

    sim_time_param = {"use_sim_time": use_sim_time}

    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{"yaml_filename": map_yaml_file}, sim_time_param],
    )

    lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        output="screen",
        parameters=[{"autostart": autostart, "node_names": ["map_server"]}, sim_time_param],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Full path to the map yaml file to load (required)"),
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"),
            DeclareLaunchArgument("autostart", default_value="true", description="Automatically bring map_server to the active state"),
            map_server_node,
            lifecycle_manager_node,
        ]
    )
