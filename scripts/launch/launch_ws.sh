#!/bin/bash

ROS_DOMAIN_ID=40
ROS_DISTRO="jazzy"
WS=${PWD}
DEFAULT_DELAY=0.05
DEFAULT_LONG_DELAY=0.2
# Η εντολή που προετοιμάζει κάθε νέο terminal panel
GLOBAL_CMD="cd $WS && source /opt/ros/$ROS_DISTRO/setup.bash && source $WS/install/setup.bash && source $WS/.venv/bin/activate && export PYTHONPATH=\$PYTHONPATH:$WS/external && export ROS_DOMAIN_ID=$ROS_DOMAIN_ID"
LAYOUT_NAME="GazeboLayout"
TERMINATOR_CONFIG="$WS/scripts/config/terminator_config"

source $WS/scripts/utils.sh

open_terminator

# Grid is 3 columns x 2 rows (see scripts/config/terminator_config's
# GazeboLayout): col1 = sim/nav bringup + teleop, col2 = office task
# distributor + rviz, col3 = Open-RMF core + task submission.
#
#   [ T0 Gazebo+Nav2* ] [ T2 office_distributor* ] [ T4 RMF core+adapter* ]
#   [ T1 teleop        ] [ T3 rviz                ] [ T5 RMF task submit   ]
#
# * = required for the fleet to actually do continuous work: T0 up first,
#     then T4 (Open-RMF core + agv_fleet_adapter), then T2 to start the
#     steady stream of random delivery tasks. T5 is a one-off manual test,
#     not needed once T2 is running. T1/T3 are for general fleet
#     operation, optional either way.
#
# office_distributor now submits tasks to Open-RMF's dispatcher instead of
# commanding a robot directly (room_goal_distributor.py) - it doesn't know
# or care which robots exist, so it's safe to run alongside any fleet size
# without the two systems fighting over an action server, unlike before.
#
# Fleet is agv_1/agv_2/agv_3 (workcell.launch.py's robots_config), all
# registered with Open-RMF (agv_fleet_adapter/config/fleet_config.yaml).

# Προετοιμασία: σιγουρεύουμε ότι είμαστε στο πάνω-αριστερό panel (T0),
# όσα Alt+Left παραπάνω χρειαστούν είναι no-ops στο άκρο του grid.
move_up
move_left
move_left

# configuration broadcasting - στέλνει το GLOBAL_CMD σε όλα τα 6 panels
echo "Enabling broadcasting for all panels..."
broadcast_on
paste_cmd "$GLOBAL_CMD && clear"
enter
broadcast_off

# --- T0 (πάνω-αριστερά): Gazebo + Nav2 bringup, όλο το fleet ---
echo "Configuring T0: Gazebo + Nav2 bringup..."
paste_cmd "source install/setup.bash && ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false slam:=false map:=$WS/src/robots/agv/agv_navigation/maps/office_world.yaml"

# --- T2 (πάνω-μέση): office_distributor ---
# Submits tasks to Open-RMF (T4) rather than commanding a robot directly -
# start this AFTER T4 is up, or its early requests just get dropped with
# nobody subscribed on task_api_requests yet.
move_right
echo "Configuring T2: office_distributor..."
paste_cmd 'source install/setup.bash && ros2 launch office_distributor office_distributor.launch.py'

# --- T4 (πάνω-δεξιά): Open-RMF core + agv_fleet_adapter (agv_1/2/3) ---
move_right
echo "Configuring T4: Open-RMF core + fleet adapter..."
paste_cmd 'source install/setup.bash && ros2 launch workcell_bringup rmf.launch.py'

# --- T5 (κάτω-δεξιά): RMF task submission - ready to fire once T0/T4 are up ---
# agv_fleet_adapter/submit_patrol_task.py - our own script, entirely inside
# this workspace (no external open-rmf/rmf_demos clone needed).
move_down
echo "Configuring T5: RMF task submission..."
paste_cmd 'source install/setup.bash && ros2 run agv_fleet_adapter submit_patrol_task -p floor_0_office_n_0 floor_0_office_n_1 -n 3 --use_sim_time'

# --- T3 (κάτω-μέση): RViz ---
move_left
echo "Configuring T3: RViz..."
paste_cmd 'source install/setup.bash && ros2 run rviz2 rviz2 -d scripts/config/rviz_slam.rviz --ros-args -r /tf:=/agv_1/tf -r /tf_static:=/agv_1/tf_static -r /goal_pose:=/agv_1/goal_pose -r /initialpose:=/agv_1/initialpose -p use_sim_time:=True'

# --- T1 (κάτω-αριστερά): AGV teleop ---
move_left
echo "Configuring T1: AGV teleop..."
paste_cmd 'source install/setup.bash && ros2 launch agv_navigation teleop.launch.py namespace:=agv_1'
