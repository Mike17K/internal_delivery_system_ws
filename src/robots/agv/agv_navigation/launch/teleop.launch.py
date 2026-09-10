"""Standalone teleop UI for one AGV.

Deliberately NOT wired into navigation.launch.py or workcell.launch.py:
driving by hand is something you opt into for map-building or debugging, and
a manual command stream fighting Nav2's for the same cmd_vel topic is exactly
the situation to keep opt-in. Cancel any active Nav2 goal before driving.

    ros2 launch agv_navigation teleop.launch.py namespace:=agv_1
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_launch_arguments() -> list[DeclareLaunchArgument]:
    args = []
    args.append(DeclareLaunchArgument("namespace", default_value="agv_1", description="Robot to drive (e.g. agv_1)"))
    args.append(DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation (Gazebo) clock"))
    # cmd_vel_nav, not cmd_vel: this goes through nav2's velocity_smoother,
    # so hand-driving obeys the same accel limits as autonomous motion and
    # can't step-command the wheels. Set to "cmd_vel" to bypass the smoother
    # and talk to diff_drive_controller directly.
    args.append(DeclareLaunchArgument("cmd_vel_topic", default_value="cmd_vel_nav", description="Relative topic to publish TwistStamped on (cmd_vel_nav = via velocity_smoother, cmd_vel = direct to diff_drive_controller)"))
    # 20 Hz = 50 ms between commands, comfortably inside
    # diff_drive_controller's cmd_vel_timeout (0.25 s) - see
    # agv_bringup/config/controllers.yaml for why that timeout exists.
    args.append(DeclareLaunchArgument("publish_rate", default_value="20.0", description="Command publish rate in Hz - must stay well above 1/cmd_vel_timeout"))
    args.append(DeclareLaunchArgument("max_linear", default_value="0.5", description="Linear velocity at 100% on the speed slider - keep in step with velocity_smoother's max_velocity"))
    args.append(DeclareLaunchArgument("max_angular", default_value="1.5", description="Angular velocity at 100% on the turn slider"))
    return args


def generate_launch_description() -> LaunchDescription:
    teleop_node = Node(
        package="agv_navigation",
        executable="teleop_ui.py",
        name="teleop_ui",
        namespace=LaunchConfiguration("namespace"),
        output="screen",
        # emulate_tty keeps the node's log lines (including the resolved
        # topic it prints on startup) unbuffered in the launching terminal.
        emulate_tty=True,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "cmd_vel_topic": LaunchConfiguration("cmd_vel_topic"),
            "publish_rate": LaunchConfiguration("publish_rate"),
            "max_linear": LaunchConfiguration("max_linear"),
            "max_angular": LaunchConfiguration("max_angular"),
        }],
    )

    return LaunchDescription([*get_launch_arguments(), teleop_node])
