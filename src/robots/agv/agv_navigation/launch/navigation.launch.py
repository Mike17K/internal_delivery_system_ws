import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("namespace", default_value="", description="Robot namespace (e.g. agv_1) - one instance of this launch file per fleet member"))
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    args.append(DeclareLaunchArgument("autostart", default_value="true", description="Automatically bring the Nav2 lifecycle nodes to the active state"))
    args.append(DeclareLaunchArgument("slam", default_value="true", description="true: run SLAM Toolbox to build/extend a map. false: localize with AMCL against the fleet-wide shared map (see localization.launch.py)"))
    args.append(DeclareLaunchArgument("params_file", default_value=os.path.join(get_package_share_directory("agv_navigation"), "config", "nav2_params.yaml"), description="Nav2 params template"))
    args.append(DeclareLaunchArgument("slam_params_file", default_value=os.path.join(get_package_share_directory("agv_navigation"), "config", "mapper_params_online_async.yaml"), description="SLAM Toolbox params template"))
    args.append(DeclareLaunchArgument("initial_pose_x", default_value="0.0", description="AMCL initial pose X (ignored when slam:=true)"))
    args.append(DeclareLaunchArgument("initial_pose_y", default_value="0.0", description="AMCL initial pose Y (ignored when slam:=true)"))
    args.append(DeclareLaunchArgument("initial_pose_yaw", default_value="0.0", description="AMCL initial pose yaw (ignored when slam:=true)"))
    return args


def launch_setup(context):
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    ns = namespace.perform(context)
    slam_enabled = LaunchConfiguration("slam").perform(context).lower() == "true"

    # tf2_ros hardcodes an absolute "/tf"/"/tf_static" internally, which a
    # Node's own `namespace=` does NOT touch (an already-absolute topic name
    # is never re-namespaced) - this explicit remap to the relative "tf"/
    # "tf_static" is what actually makes this robot's Nav2 stack publish/
    # look up transforms on its own /<namespace>/tf instead of the global
    # /tf, matching agv_bringup/launch/bringup.launch.py's robot_state_
    # publisher/controller_manager remap and nav2_bringup's own default
    # multi-robot behavior. Applied to every node here that touches tf -
    # not the lifecycle managers, which don't.
    tf_remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # ── Resolve params files ─────────────────────────────────────────────────
    # Two things happen here, composed the same way nav2_bringup itself does:
    #   1. RewrittenYaml's root_key re-keys the template's top-level node
    #      sections (amcl:, controller_server:, ...) so they match this
    #      robot's actual namespaced node names (e.g. /agv_1/amcl).
    #   2. ParameterFile(allow_substs=True) resolves the $(var namespace)/
    #      $(var initial_pose_*) tokens embedded in the template's
    #      initial-pose values against this launch context's
    #      LaunchConfigurations - the same mechanism agv_bringup/launch/
    #      bringup.launch.py already uses for controllers.yaml, kept
    #      consistent rather than introducing a second templating idiom.
    #      (Frame ids in nav2_params.yaml are bare now - base_link/odom -
    #      so they don't need per-robot substitution at all anymore.)
    # autostart isn't a key in nav2_params.yaml itself - it's only meaningful
    # to the lifecycle managers below, which get it directly via their own
    # inline parameters dict.
    # The map topic is the one thing that genuinely differs between the two
    # modes, so it can't be a fixed value in the template:
    #   slam:=true  - this robot's own slam_toolbox is the map authority and,
    #                 being namespaced, publishes /<namespace>/map. A relative
    #                 "map" resolves there (nav2_costmap_2d joins a relative
    #                 topic with the parent namespace, not the costmap's own
    #                 sub-namespace), so global_costmap's static layer follows
    #                 the live map as it's built.
    #   slam:=false - the ONE fleet-wide map_server (localization.launch.py)
    #                 publishes an unnamespaced /map that every robot's AMCL
    #                 and static layer share.
    # Leaving this at "/map" unconditionally was a real bug: while mapping,
    # nothing publishes /map at all, so global_costmap waited forever for a
    # map that was sitting on /<namespace>/map the whole time - the planner
    # could never produce a path. param_rewrites matches on leaf key name at
    # any depth, so this covers both amcl's map_topic and the static layer's.
    param_rewrites = {
        "use_sim_time": use_sim_time,
        "map_topic": "map" if slam_enabled else "/map",
    }

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("params_file"),
            root_key=namespace,
            param_rewrites=param_rewrites,
            convert_types=True,
        ),
        allow_substs=True,
    )

    configured_slam_params = ParameterFile(
        RewrittenYaml(
            source_file=LaunchConfiguration("slam_params_file"),
            root_key=namespace,
            param_rewrites={"use_sim_time": use_sim_time},
            convert_types=True,
        ),
        allow_substs=True,
    )

    sim_time_param = {"use_sim_time": use_sim_time}

    # nav2_lifecycle_manager's default bond_timeout (4.0s) is tuned for a
    # single robot's nav stack with hardware acceleration. Under two full
    # fleet-robot stacks plus lidar gpu_lidar raycasting on software
    # rendering, a lifecycle node can miss that window even though it's not
    # actually stuck - observed live: agv_1's own slam_toolbox failed
    # bringup ("unable to be reached after 4.00s by bond") purely from
    # system load, not a real hang. A more generous timeout trades a bit of
    # "detect a truly dead node" responsiveness for not treating "briefly
    # busy" as "dead" - the right trade for this environment.
    bond_timeout_param = {"bond_timeout": 10.0}

    # slam_toolbox gets the bond disabled outright (0.0), not just a longer
    # timeout. 4.0s failed, then 10.0s failed the same way on the next run:
    # "Server slam_toolbox was unable to be reached after 10.00s by bond ...
    # Failed to bring up all requested nodes. Aborting bringup." - while
    # slam_toolbox itself was demonstrably alive and processing scans
    # afterwards. The bond isn't created until slam_toolbox's on_activate
    # returns, and under llvmpipe software rendering with two robots'
    # gpu_lidar raycasting that takes longer than any timeout worth setting.
    # The cost is losing automatic detection of a genuinely dead
    # slam_toolbox; the alternative is a manager that aborts the whole
    # bringup on a node that is merely slow to start, which is strictly
    # worse. The navigation manager keeps its bond (10.0s above) - those
    # nodes activate fast and the monitoring is worth having.
    slam_bond_timeout_param = {"bond_timeout": 0.0}

    # ── Localization: SLAM Toolbox (mapping) XOR AMCL (against the shared map) ──
    slam_toolbox_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        namespace=namespace,
        output="screen",
        parameters=[configured_slam_params, sim_time_param],
        # slam_toolbox is internally inconsistent about its own namespace: it
        # PUBLISHES the occupancy grid on a hardcoded absolute "/map" (which
        # `namespace=` cannot touch, since an already-absolute name is never
        # re-namespaced), while the nav2 MapSaver it owns for `use_map_saver`
        # SUBSCRIBES to a relative "map" - i.e. /<namespace>/map. Left alone,
        # those two never meet, and it fails in two places at once:
        #   1. /agv_1/slam_toolbox/save_map returns 255
        #      (RESULT_UNDEFINED_FAILURE) forever. The saver is waiting for a
        #      map on /agv_1/map that nothing publishes - it is NOT a bad
        #      output path, which is the obvious first guess and is wrong.
        #   2. global_costmap's static layer stays empty for the whole
        #      mapping run, since navigation.launch.py points map_topic at
        #      this robot's own /<namespace>/map (see the map_topic rewrite
        #      above).
        # Remapping the publisher down into the namespace fixes both and is
        # what the rest of this stack already assumes. Do not "simplify" it
        # back to the global /map: that is map_server's topic under
        # slam:=false, and two robots mapping at once would collide on it.
        remappings=tf_remappings + [("/map", "map"), ("/map_metadata", "map_metadata")],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    lifecycle_manager_slam = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_slam",
        namespace=namespace,
        output="screen",
        parameters=[{"autostart": autostart, "node_names": ["slam_toolbox"]}, sim_time_param, slam_bond_timeout_param],
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    amcl_node = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        namespace=namespace,
        output="screen",
        parameters=[configured_params, sim_time_param],
        remappings=tf_remappings,
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )

    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        namespace=namespace,
        output="screen",
        parameters=[{"autostart": autostart, "node_names": ["amcl"]}, sim_time_param, bond_timeout_param],
        condition=UnlessCondition(LaunchConfiguration("slam")),
    )

    # ── Core navigation stack (always on, regardless of slam mode) ──────────
    navigation_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "waypoint_follower",
        "velocity_smoother",
    ]

    navigation_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            # Raw output goes to cmd_vel_nav; velocity_smoother (below)
            # consumes that and republishes the smoothed result as cmd_vel,
            # which is what diff_drive_controller actually subscribes to.
            remappings=[("cmd_vel", "cmd_vel_nav"), *tf_remappings],
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_smoother",
            executable="smoother_server",
            name="smoother_server",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_waypoint_follower",
            executable="waypoint_follower",
            name="waypoint_follower",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=tf_remappings,
        ),
        Node(
            package="nav2_velocity_smoother",
            executable="velocity_smoother",
            name="velocity_smoother",
            namespace=namespace,
            output="screen",
            parameters=[configured_params, sim_time_param],
            remappings=[("cmd_vel", "cmd_vel_nav"), ("cmd_vel_smoothed", "cmd_vel"), *tf_remappings],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=namespace,
            output="screen",
            parameters=[{"autostart": autostart, "node_names": navigation_lifecycle_nodes}, sim_time_param, bond_timeout_param],
        ),
    ]

    return [
        slam_toolbox_node,
        lifecycle_manager_slam,
        amcl_node,
        lifecycle_manager_localization,
        *navigation_nodes,
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            *get_launch_arguments(),
            OpaqueFunction(function=launch_setup),
        ]
    )
