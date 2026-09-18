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
# GazeboLayout). T0 (sim/nav bringup), T4 (Open-RMF core + agv_fleet_
# adapter), T2 (office_distributor - submits tasks to T4's dispatcher,
# start after T4 or its early requests get dropped with nobody subscribed
# yet), and T5 (Open-RMF visualization, also watching T4) all get a
# command pasted. T1/T3 still get GLOBAL_CMD sourced via the broadcast
# below, so they're ready to use, but nothing is pasted into them - paste
# whatever you actually need there yourself (teleop, plain RViz, a one-off
# task submission via `ros2 run agv_fleet_adapter submit_patrol_task`, ...).
#
#   [ T0 Gazebo+Nav2 ] [ T2 office_distributor ] [ T4 RMF core+adapter ]
#   [ T1 (empty)      ] [ T3 (empty)            ] [ T5 RMF visualization ]
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
move_right
echo "Configuring T2: office_distributor..."
paste_cmd 'source install/setup.bash && ros2 launch office_distributor office_distributor.launch.py'

# --- T4 (πάνω-δεξιά): Open-RMF core + agv_fleet_adapter (agv_1/2/3) ---
move_right
echo "Configuring T4: Open-RMF core + fleet adapter..."
paste_cmd 'source install/setup.bash && ros2 launch workcell_bringup rmf.launch.py'

# --- T5 (κάτω-δεξιά): Open-RMF visualization (RViz + schedule/navgraph/fleet-state plugins) ---
# Start after T4 - it's watching the same schedule/fleet state T4 publishes.
move_down
echo "Configuring T5: Open-RMF visualization..."
paste_cmd 'source install/setup.bash && ros2 launch rmf_visualization visualization.launch.xml map_name:=L1 use_sim_time:=true'
