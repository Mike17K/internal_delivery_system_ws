import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction, AppendEnvironmentVariable, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node


def generate_launch_description():
    ld = LaunchDescription()

    # 1. Εντοπισμός Πακέτων για το Gazebo Global Environment
    pkg_ros_gz_sim = get_package_share_directory("ros_gz_sim")
    pkg_workcell_bringup = get_package_share_directory("workcell_bringup")
    pkg_workcell_description = get_package_share_directory("workcell_description")

    # 2. Global Gazebo Resource Paths (GZ_SIM_RESOURCE_PATH)
    # model:// URIs (office_floor and all the furniture/fixture models it
    # includes) are resolved by searching GZ_SIM_RESOURCE_PATH for a
    # directory named after the model that contains a model.config.
    set_gz_resource_path = AppendEnvironmentVariable(
        "GZ_SIM_RESOURCE_PATH",
        os.path.join(pkg_workcell_description, "models"),
    )
    ld.add_action(set_gz_resource_path)

    # 3. Global Launch Arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        "use_fake_hardware",
        default_value="true",
        description="True for mock components (RViz only). False for Gazebo or Real Hardware.",
    )

    sim_gazebo_arg = DeclareLaunchArgument(
        "sim_gazebo",
        default_value="false",
        description="True to launch Gazebo Simulator.",
    )
    world_arg = DeclareLaunchArgument(
        "world",
        default_value=PathJoinSubstitution([pkg_workcell_description, "worlds", "office_world.sdf"]), # workcell_world.sdf
        description="Gazebo world file to load",
    )
    slam_arg = DeclareLaunchArgument(
        "slam",
        default_value="true",
        description="true: every robot runs SLAM Toolbox to build/extend a map (drive one robot at a time to map). "
        "false: fleet-wide shared map_server + per-robot AMCL against a saved map (requires 'map' to be set).",
    )
    map_arg = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Full path to a saved map yaml file - required when slam:=false, ignored when slam:=true",
    )

    # 4. Εκκίνηση Global Gazebo Instance
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_ros_gz_sim, "launch", "gz_sim.launch.py")),
        launch_arguments={"gz_args": ["-r ", LaunchConfiguration("world")]}.items(),  # trailing space
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(gazebo)

    # 5. Global Clock Bridge (ROS 2 <-> Gazebo time synchronization)
    bridge_params = os.path.join(pkg_workcell_bringup, "config", "gz_bridge.yaml")
    clock_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="clock_bridge",
        output="screen",
        parameters=[{"use_sim_time": True}],
        arguments=["--ros-args", "-p", f"config_file:={bridge_params}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )
    ld.add_action(clock_bridge)

    # 6. Global Static Transform Publisher για το World Frame
    world_node = Node(package="tf2_ros", executable="static_transform_publisher", arguments=["0", "0", "0", "0", "0", "0", "world", "map"])
    ld.add_action(world_node)

    # 6b. Fleet-wide shared map_server (once, not per-robot) - only when
    # localizing against a saved map. In slam:=true mode each robot's own
    # SLAM Toolbox instance (see agv_navigation/launch/navigation.launch.py)
    # publishes /map itself instead, so this and that are mutually exclusive.
    pkg_agv_navigation_share = get_package_share_directory("agv_navigation")
    localization_launch_path = os.path.join(pkg_agv_navigation_share, "launch", "localization.launch.py")
    nav_launch_path = os.path.join(pkg_agv_navigation_share, "launch", "navigation.launch.py")

    localization = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(localization_launch_path),
        launch_arguments={
            "map": LaunchConfiguration("map"),
            "use_sim_time": LaunchConfiguration("sim_gazebo"),
            "autostart": "true",
        }.items(),
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )
    ld.add_action(localization)

    # 7. Ορισμός των Ρομπότ στην Κυψέλη Εργασίας
    robots_config = [
        {"name": "agv_1", "xyz": "0.0 0.0 0.0", "rpy": "0.0 0.0 0.0"},
        {"name": "agv_2", "xyz": "1.0 0.0 0.0", "rpy": "0.0 0.0 0.0"},
    ]

    pkg_agv_bringup_share = get_package_share_directory("agv_bringup")
    agv_launch_path = os.path.join(pkg_agv_bringup_share, "launch", "bringup.launch.py")

    # 8. Loop που καλεί το ανεξάρτητο bringup + navigation stack του κάθε ρομπότ
    for i, robot in enumerate(robots_config):
        x, y, z = robot["xyz"].split()
        roll, pitch, yaw = robot["rpy"].split()

        robot_stack = GroupAction(
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(agv_launch_path),
                    launch_arguments={
                        "xyz": robot["xyz"],
                        "rpy": robot["rpy"],
                        "sim_gazebo": LaunchConfiguration("sim_gazebo"),
                        "use_fake_hardware": LaunchConfiguration("use_fake_hardware"),
                        "namespace": robot["name"],
                    }.items(),
                ),
            ]
        )
        # Stagger each robot by 0.5s to avoid simultaneous Gazebo spawn requests
        ld.add_action(TimerAction(period=float(i) * 0.5, actions=[robot_stack]))

        if i == 0:
            # Mapping (slam:=true) is a single-robot activity - running two
            # simultaneous slam_toolbox instances doesn't build one merged
            # map, it's just two independent mappers each thinking they own
            # the "map" frame, competing for CPU for no benefit (confirmed
            # live: with both robots mapping, agv_1's own slam_toolbox missed
            # its lifecycle bond heartbeat and failed to bring up at all).
            # Only the first fleet robot gets SLAM; it always gets a
            # navigation stack, in whichever mode the fleet is in.
            nav_slam_arg = LaunchConfiguration("slam")
            nav_condition = None
        else:
            # Other robots only get a navigation stack once there's an actual
            # map to localize against (slam:=false) - while mapping, they sit
            # idle in Gazebo without a Nav2 stack rather than each spinning up
            # a redundant, competing SLAM instance.
            nav_slam_arg = "false"
            nav_condition = UnlessCondition(LaunchConfiguration("slam"))

        navigation_include = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav_launch_path),
            launch_arguments={
                "namespace": robot["name"],
                "use_sim_time": LaunchConfiguration("sim_gazebo"),
                "autostart": "true",
                "slam": nav_slam_arg,
                "initial_pose_x": x,
                "initial_pose_y": y,
                "initial_pose_yaw": yaw,
            }.items(),
            condition=nav_condition,
        )
        navigation_stack = GroupAction(actions=[navigation_include])
        # Give each robot's own bringup (spawn + controllers) a head start
        # before its Nav2 stack comes up and starts looking for it.
        ld.add_action(TimerAction(period=float(i) * 0.5 + 3.0, actions=[navigation_stack]))

    return LaunchDescription([use_fake_hardware_arg, sim_gazebo_arg, world_arg, slam_arg, map_arg, ld])
