import os
from typing import Any
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, TimerAction
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
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Namespace for this robot's nodes and topics, including its own /<namespace>/tf"))
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
            },
        )
        .to_dict()
    )

    gz_bridge_yaml_path = _make_param_file(os.path.join(pkg_bringup, "config", "gz_bridge.yaml"), context)

    sim_time_param = {"use_sim_time": LaunchConfiguration("sim_gazebo")}

    # tf2_ros hardcodes an absolute "/tf"/"/tf_static" internally, which a
    # Node's own `namespace=` does NOT touch (an already-absolute topic name
    # is never re-namespaced) - this explicit remap to the relative "tf"/
    # "tf_static" is what actually makes this robot's transforms land on its
    # own /<namespace>/tf instead of the global /tf. Applied to every node
    # here that publishes or looks up transforms.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

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
        remappings=tf_remappings,
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
        # NOTE: deliberately NO tf remap here. controller_manager itself never
        # publishes tf - only the *controllers* it loads do, and a remap on
        # this process does not reach them (controller_manager says so itself:
        # "The use of remapping arguments to the controller_manager node is
        # deprecated. Please use the 'controller ros args' argument of the
        # spawner..."). The controllers' tf remap lives on the spawner below,
        # which is the only place that actually works - in this mode AND in
        # sim_gazebo mode, where this node doesn't even run.
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

    # ── 5. Controller Spawners ────────────────────────────────────────────────
    # Two spawner invocations rather than one, because only
    # diff_drive_controller needs --controller-ros-args and those args are
    # applied to *every* controller named in a single spawner call.
    controller_manager_args = [
        "--controller-manager",
        f"/{namespace}/controller_manager",
        "--controller-manager-timeout",
        "30",
    ]

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=["joint_state_broadcaster", *controller_manager_args],
        parameters=[sim_time_param],
    )

    # One of three fixes for the "Invalid frame ID 'odom' ... frame does not
    # exist" bug - the one that gets the transform onto the right TOPIC. The
    # other two are the odometry bootstrap below (which gets the controller to
    # publish anything at all) and tf_frame_prefix_enable in controllers.yaml
    # (which gets the frame NAMES right). All three are required; each alone
    # produced the identical error message.
    #
    # A controller is not a process - controller_manager instantiates it as a
    # node *inside its own process*, so no launch-file `remappings=` and no
    # gz_ros2_control `<ros><remapping>` (which only configure the
    # controller_manager process) ever reach diff_drive_controller's internal
    # TransformBroadcaster. That broadcaster hardcodes an absolute "/tf", so
    # anything it published would land on the GLOBAL /tf while every Nav2 node
    # for this robot listens on /<namespace>/tf.
    #
    # `--controller-ros-args` is the supported way in: the spawner sets it as
    # the controller's `node_options_args` parameter on controller_manager
    # *before* loading it, so the args are applied when the controller node is
    # constructed. Works identically in sim_gazebo mode (where
    # controller_manager lives inside the Gazebo process) and on real hardware
    # - the args travel with the controller, not with the process.
    #
    # Remap targets are RELATIVE ("tf", not "/agv_1/tf"): they resolve against
    # the controller node's own namespace, which controller_manager sets from
    # its own. That keeps this correct for an empty namespace too.
    #
    # This fixes which TOPIC the transform lands on. It does not affect the
    # FRAME NAMES in it - those needed a separate fix (see
    # agv_bringup/config/controllers.yaml's tf_frame_prefix_enable), and
    # getting one right without the other still produces Nav2's
    # "Invalid frame ID 'odom'".
    #   /tf, /tf_static -> the odom -> base_link transform, on this robot's topic
    #   ~/odom          -> /<ns>/odom, which nav2_params.yaml's `odom_topic: odom`
    #                      resolves to (default was /<ns>/diff_drive_controller/odom)
    #   ~/cmd_vel       -> /<ns>/cmd_vel, what velocity_smoother publishes
    #                      (default was /<ns>/diff_drive_controller/cmd_vel)
    diff_drive_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        namespace=namespace,
        arguments=[
            "diff_drive_controller",
            *controller_manager_args,
            "--controller-ros-args",
            "-r /tf:=tf -r /tf_static:=tf_static -r ~/odom:=odom -r ~/cmd_vel:=cmd_vel",
        ],
        parameters=[sim_time_param],
    )

    # ── 6. Odometry bootstrap ─────────────────────────────────────────────────
    # Breaks a genuine deadlock in diff_drive_controller 4.x, and it is not
    # optional - without it the robot has no "odom" frame, ever. Confirmed
    # live: with this in place odom -> base_link appears on /<namespace>/tf at
    # ~50 Hz; without it, nothing was published on any odometry topic at all.
    #
    # update_and_write_commands() begins with an isfinite() check on
    # reference_interfaces_[0..1] and returns early when either is NaN
    # (confirmed by disassembly: libdiff_drive_controller.so, the andpd/ucomisd
    # against +INF at the top of that symbol). reset_buffers() NaN-fills those
    # on activation, and only an incoming velocity command makes them finite.
    # Everything else in that function - Odometry::update, the /odom publisher
    # AND the odom -> base_link tf broadcast - sits *after* that early return.
    #
    # So a freshly activated controller publishes no odometry of any kind,
    # which deadlocks the stack: Nav2 won't issue a velocity command until its
    # costmaps can resolve "odom", and "odom" doesn't exist until a velocity
    # command arrives. One zero command is enough to break it permanently -
    # nothing puts the reference interfaces back to NaN except a
    # deactivate/activate cycle (a cmd_vel timeout writes finite zeros, and
    # reset_buffers is only reached from the lifecycle callbacks).
    #
    # `-w 1` waits for diff_drive_controller's own subscription to match
    # before publishing, so this is robust against spawner/discovery timing
    # rather than relying on the TimerAction period being long enough.
    # TwistStamped, not Twist: see controllers.yaml on the removal of
    # use_stamped_vel. A zero header stamp is fine - the controller replaces it
    # with the current time (and says so, once).
    odom_bootstrap = ExecuteProcess(
        cmd=[
            "ros2", "topic", "pub",
            "-w", "1",
            "--times", "3",
            "--rate", "2",
            f"/{namespace}/cmd_vel",
            "geometry_msgs/msg/TwistStamped",
            "{}",
        ],
        output="screen",
        name="odom_bootstrap",
    )

    return [
        robot_state_publisher,
        controller_manager_node,
        gazebo_spawn_robot,
        gz_default_bridge,
        TimerAction(
            period=4.0,
            actions=[joint_state_broadcaster_spawner, diff_drive_controller_spawner],
        ),
        TimerAction(period=6.0, actions=[odom_bootstrap]),
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
