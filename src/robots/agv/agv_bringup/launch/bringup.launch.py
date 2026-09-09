import os
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import UnlessCondition, IfCondition
from launch_ros.parameter_descriptions import ParameterFile
from launch_ros.actions import Node
from launch_param_builder import ParameterBuilder


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("use_fake_hardware", default_value="true", description="Use mock_components/GenericSystem (true) or real hardware drivers (false)"))
    args.append(DeclareLaunchArgument("sim_gazebo", default_value="false", description="Switch to true if launching inside a Gazebo Simulation environment"))
    args.append(DeclareLaunchArgument("xyz", default_value="0.0 0.0 0.0", description="Robot spawn position (x y z, space-separated) - passed to the spawner, not baked into the model"))
    args.append(DeclareLaunchArgument("rpy", default_value="0.0 0.0 0.0", description="Robot spawn orientation (roll pitch yaw, space-separated) - passed to the spawner, not baked into the model"))
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for the robot tf frames, topics and nodes"))
    args.append(DeclareLaunchArgument("tf_prefix", default_value="", description="Prefix for all TF frames after namespace is applied"))
    return args


_param_file_refs: list[Any] = []


def _make_param_file(path, context):
    pf = ParameterFile(path, allow_substs=True)
    _param_file_refs.append(pf)  # prevent garbage collection / temp-file deletion
    return pf.evaluate(context)


def launch_setup(context):
    pkg_bringup = get_package_share_directory("agv_bringup")

    # ── Runtime values ───────────────────────────────────────────────────────
    use_fake_hardware = LaunchConfiguration("use_fake_hardware").perform(context)
    sim_gazebo = LaunchConfiguration("sim_gazebo").perform(context)
    xyz = LaunchConfiguration("xyz").perform(context)
    rpy = LaunchConfiguration("rpy").perform(context)
    namespace = LaunchConfiguration("namespace").perform(context)
    tf_prefix = LaunchConfiguration("tf_prefix").perform(context)

    spawn_x, spawn_y, spawn_z = xyz.split()
    spawn_roll, spawn_pitch, spawn_yaw = rpy.split()

    # ── Controllers YAML (namespace-substituted) ─────────────────────────────────
    controllers_file_path = _make_param_file(os.path.join(pkg_bringup, "config", "controllers.yaml"), context)

    robot_desc = (
        ParameterBuilder("agv_description")
        .xacro_parameter(
            "robot_description",
            "urdf/agv.urdf.xacro",
            mappings={
                "sim_gazebo": sim_gazebo,
                "use_fake_hardware": use_fake_hardware,
                "simulation_controllers": str(controllers_file_path),
                "namespace": namespace,
                "tf_prefix": tf_prefix,
            },
        )
        .to_dict()
    )

    gz_bridge_yaml_path = _make_param_file(os.path.join(pkg_bringup, "config", "gz_bridge.yaml"), context)

    sim_time_param = {"use_sim_time": LaunchConfiguration("sim_gazebo")}

    # ── 1. Robot State Publisher ─────────────────────────────────────────────
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            sim_time_param,
        ],
    )

    # ── 2. Standalone Controller Manager (real hardware only) ─────────────────
    controller_manager_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="screen",
        namespace=namespace,
        parameters=[
            robot_desc,
            controllers_file_path,
            sim_time_param,
        ],
        condition=UnlessCondition(LaunchConfiguration("sim_gazebo")),
        remappings=[("/robot_description", f"{namespace}/robot_description")],
    )

    # ── 3. Gazebo Spawner ────────────────────────────────────────────────────
    gazebo_spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        output="screen",
        namespace=namespace,
        arguments=[
            "-topic",
            "robot_description",
            "-name",
            namespace,
            "-x", spawn_x,
            "-y", spawn_y,
            "-z", spawn_z,
            "-R", spawn_roll,
            "-P", spawn_pitch,
            "-Y", spawn_yaw,
        ],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 4. Pose/TF Bridge ─────────────────────────────────────────────────────
    gz_default_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="agv_bridge",
        output="screen",
        parameters=[sim_time_param],
        namespace=namespace,
        arguments=["--ros-args", "-p", f"config_file:={gz_bridge_yaml_path}"],
        condition=IfCondition(LaunchConfiguration("sim_gazebo")),
    )

    # ── 5. Controller Spawner ─────────────────────────────────────────────────
    default_controllers_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "joint_state_broadcaster",
            "diff_drive_controller",
            "--controller-manager",
            f"/{namespace}/controller_manager",
            "--controller-manager-timeout",
            "30",
        ],
        parameters=[sim_time_param],
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        gazebo_spawn_robot,
        gz_default_bridge,
        TimerAction(
            period=4.0,
            actions=[default_controllers_spawner],
        ),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
