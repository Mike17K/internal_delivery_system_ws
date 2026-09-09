import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_workcell_description = get_package_share_directory("workcell_description")

    # model:// URIs (office_floor and all the furniture/fixture models it
    # includes) are resolved by searching GZ_SIM_RESOURCE_PATH for a
    # directory named after the model that contains a model.config.
    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_workcell_description, "models"),
    )

    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_workcell_description, "worlds", "office_world.sdf"]),
        description="Gazebo world file to load.",
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),
    )

    return LaunchDescription([
        world_arg,
        set_gz_resource_path,
        gz_sim,
    ])
