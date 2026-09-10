# Open-RMF + Nav2 Fleet Integration Guide

Target: multi-robot **simulation** fleet, Nav2 per-robot navigation, Open-RMF for
fleet-level orchestration, traffic management, and task dispatch.

---

## 1. Architecture Overview

```
                        ┌─────────────────────────────┐
                        │        Open-RMF Core         │
                        │  (rmf_core / rmf-web / rmf_  │
                        │   traffic_editor building)   │
                        │                              │
                        │  - Traffic Schedule           │
                        │  - Task Dispatcher            │
                        │  - Building/Nav Graph          │
                        │  - Fleet State Aggregator      │
                        └──────────────┬───────────────┘
                                       │ (RMF free_fleet / fleet_adapter API)
              ┌────────────────────────┼────────────────────────┐
              │                        │                        │
      ┌───────▼───────┐        ┌───────▼───────┐        ┌───────▼───────┐
      │ Fleet Adapter  │        │ Fleet Adapter  │        │ Fleet Adapter  │
      │  (robot1)      │        │  (robot2)      │        │  (robotN)      │
      └───────┬───────┘        └───────┬───────┘        └───────┬───────┘
              │ /robot1/*              │ /robot2/*              │ /robotN/*
      ┌───────▼───────┐        ┌───────▼───────┐        ┌───────▼───────┐
      │  Nav2 Stack    │        │  Nav2 Stack    │        │  Nav2 Stack    │
      │  (namespaced)  │        │  (namespaced)  │        │  (namespaced)  │
      └───────┬───────┘        └───────┬───────┘        └───────┬───────┘
              │                        │                        │
              └────────────────────────┼────────────────────────┘
                                       │
                              ┌────────▼────────┐
                              │  Gazebo (single  │
                              │  shared world)    │
                              └───────────────────┘
```

Key principle: **Open-RMF never talks to Nav2 directly.** It talks to a
**Fleet Adapter**, one per fleet (a fleet = a group of robots of the same
type/capability), which translates RMF's abstract commands (`navigate to
waypoint X`, `dock`, `pause`) into Nav2 action calls for each robot in that
fleet.

---

## 2. Repository / Package Layout

```
fleet_ws/
  src/
    rmf/                       # cloned Open-RMF core repos
      rmf_core
      rmf_traffic
      rmf_traffic_editor
      rmf_fleet_adapter
      rmf_task
      rmf-web                  # optional dashboard
    nav2_fleet_bringup/        # your custom package
      launch/
        fleet_bringup.launch.py
        single_robot_nav2.launch.py
        gazebo_world.launch.py
      config/
        nav2_params_robot_template.yaml
        robots.yaml             # fleet roster: names, poses, models
      maps/
        warehouse.yaml
        warehouse.pgm
      urdf/
        diff_drive_robot.urdf.xacro
    my_fleet_adapter/          # RMF fleet adapter for your robot type
      my_fleet_adapter/
        fleet_adapter.py
        robot_client_api.py    # bridges RMF <-> Nav2 actions
      config.yaml               # fleet config for RMF (nav graph, params)
```

---

## 3. Building Blocks

### 3.1 Open-RMF Core (install)

```bash
# Ubuntu 22.04 / ROS 2 Humble assumed
sudo apt install ros-humble-rmf-* -y
# or build from source for latest fleet_adapter API
mkdir -p ~/fleet_ws/src && cd ~/fleet_ws/src
git clone https://github.com/open-rmf/rmf.git
vcs import < rmf/rmf.repos
cd ~/fleet_ws && rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

Core packages you'll actually touch:
- `rmf_traffic` — the deconfliction/scheduling engine
- `rmf_fleet_adapter` — base classes + `full_control` template for building
  your adapter
- `rmf_traffic_editor` (or the newer `traffic_editor` GUI) — build the **nav
  graph** (waypoints, lanes, lifts, doors) overlaid on your map
- `rmf-web` — optional web dashboard for fleet visualization/task submission

### 3.2 Nav Graph vs Nav2 Costmap

This is the biggest conceptual gap to close: RMF plans over a **topological
graph** (discrete waypoints + lanes), not a costmap. Nav2 plans continuously
over the costmap **between two graph waypoints**.

- Use `rmf_traffic_editor` (or its GUI, `traffic-editor`) to draw the graph
  on top of the same map image (`.pgm`/`.yaml`) used by Nav2's `map_server`.
- Export as `nav_graphs/L1.yaml` — this is what the fleet adapter loads to
  know legal waypoints/lanes per robot.
- **The map used by RMF's graph and the map used by each robot's Nav2
  `map_server` must be the same map, in the same frame.** Any offset here
  causes silent waypoint-to-pose mismatches.

### 3.3 Per-Robot Nav2 Bringup (namespaced)

`config/nav2_params_robot_template.yaml` — one shared template, with
per-robot substitutions applied at launch time (namespace, initial pose).

```yaml
# excerpt — this is the pattern from the single-robot config, just namespaced
bt_navigator:
  ros__parameters:
    use_sim_time: True
    global_frame: map
    robot_base_frame: <robot_ns>/base_link
    odom_topic: <robot_ns>/odom

controller_server:
  ros__parameters:
    use_sim_time: True
    controller_frequency: 20.0
    controller_plugins: ["FollowPath"]
    FollowPath:
      plugin: "nav2_mppi_controller::MPPIController"
      # ... critics etc

local_costmap:
  local_costmap:
    ros__parameters:
      use_sim_time: True
      global_frame: <robot_ns>/odom
      robot_base_frame: <robot_ns>/base_link
      plugins: ["obstacle_layer", "inflation_layer", "fleet_obstacle_layer"]
      fleet_obstacle_layer:
        plugin: "nav2_costmap_2d::ObstacleLayer"
        observation_sources: other_robots
        other_robots:
          topic: /fleet/robot_footprints
          data_type: "PointCloud2"
```

`fleet_bringup.launch.py` loops over `robots.yaml`:

```python
for robot in load_yaml("robots.yaml")["robots"]:
    ns = robot["name"]
    actions += [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource("nav2_bringup/launch/bringup_launch.py"),
            launch_arguments={
                "namespace": ns,
                "use_namespace": "True",
                "map": "maps/warehouse.yaml",
                "params_file": generate_params_for(robot, template),
                "use_sim_time": "True",
                "autostart": "True",
            }.items(),
        ),
        Node(
            package="gazebo_ros", executable="spawn_entity.py",
            arguments=["-entity", ns, "-x", str(robot["x"]), "-y", str(robot["y"]),
                       "-topic", f"/{ns}/robot_description"],
        ),
    ]
```

Each robot gets its **own lifecycle manager**, own `map_server` instance (or
a shared one if you namespace the service calls — shared is fine, map is
static), own costmaps, own BT navigator.

### 3.4 TF Strategy

Shared global `map` frame, per-robot `odom`/`base_link` beneath it:

```
map -> robot1/odom -> robot1/base_link -> robot1/<sensors>
map -> robot2/odom -> robot2/base_link -> robot2/<sensors>
```

Each robot's AMCL (or the localization you use) publishes
`map -> robotN/odom` into a **namespaced TF tree**, but the `map` frame
itself is common — do **not** namespace `map`. This is what lets RMF reason
about all robots in one coordinate system and lets cross-robot costmap
obstacle marking work.

### 3.5 Fleet Adapter (the RMF <-> Nav2 bridge)

This is the piece you write. Minimum responsibilities:

```python
class RobotClientAPI:
    def navigate(self, robot_name, pose, map_name) -> bool:
        # calls Nav2 NavigateToPose action for /robot_name
        ...
    def stop(self, robot_name):
        # cancel current Nav2 goal
        ...
    def position(self, robot_name) -> (x, y, yaw):
        # read from /robot_name/amcl_pose or TF
        ...
    def battery_soc(self, robot_name) -> float:
        # simulated battery topic
        ...
    def dock(self, robot_name, dock_name):
        # for charging/pickup stations — custom behavior
        ...
```

`config.yaml` for the adapter declares the fleet:

```yaml
rmf_fleet:
  name: "amr_fleet"
  limits:
    linear: [0.5, 0.75]     # [nominal, max] m/s
    angular: [0.6, 1.0]
  profile:
    footprint: 0.3
    vicinity: 0.5
  reversible: true
  battery_system:
    voltage: 24.0
    capacity: 40.0
    charging_current: 26.4
  recharge_threshold: 0.2
  recharge_soc: 1.0
  publish_fleet_state: 10.0
  account_for_battery_drain: true
  robots:
    robot1:
      charger: "charger_wp_1"
    robot2:
      charger: "charger_wp_2"
```

Launch the adapter per fleet (not per robot):

```bash
ros2 run my_fleet_adapter fleet_adapter -c config.yaml -n nav_graphs/L1.yaml
```

---

## 4. Startup Sequence (simulation)

1. `ros2 launch nav2_fleet_bringup gazebo_world.launch.py` — spawns the
   shared Gazebo world (empty of robots).
2. `ros2 launch nav2_fleet_bringup fleet_bringup.launch.py` — spawns all
   robots + brings up namespaced Nav2 stacks, waits for each to reach
   `active` lifecycle state.
3. `ros2 run my_fleet_adapter fleet_adapter -c config.yaml -n nav_graphs/L1.yaml`
   — one process, registers all robots in `robots.yaml`/`config.yaml` with
   RMF core.
4. `ros2 launch rmf_traffic ...` (or however your RMF core services are
   launched) — traffic schedule node, task dispatcher.
5. (Optional) `ros2 launch rmf_visualization rmf_visualization.launch.xml`
   or `rmf-web` dashboard for visual confirmation.
6. Submit a task via `rmf_task` CLI, `rmf-web`, or a ROS 2 service call to
   confirm end-to-end wiring before scaling to N robots.

---

## 5. DDS / Discovery Notes for Simulation Scale

- Single machine, many nodes (N robots × ~6 Nav2 nodes each + RMF core) —
  default multicast discovery gets noisy fast.
- Use **Cyclone DDS**: `export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`
- Either `ROS_LOCALHOST_ONLY=1` for a fully single-machine sim, or a
  **discovery server** (`fastdds discovery-server` / Cyclone's equivalent)
  if you're distributing robots across multiple sim machines.
- All nodes: `use_sim_time: True`, synced to Gazebo's `/clock`.

---

## 6. Validation Checklist

- [ ] Each robot's `map -> robotN/odom -> robotN/base_link` TF chain is
      publishing and consistent with the shared `map` frame.
- [ ] Nav graph waypoints in `nav_graphs/L1.yaml` line up spatially with the
      Nav2 map — spot check by overlaying in RViz.
- [ ] Each robot's Nav2 lifecycle nodes reach `active` independently.
- [ ] Fleet adapter can command a single robot to a single waypoint (test
      in isolation before enabling the full fleet).
- [ ] Cross-robot obstacle layer: robot1's local costmap shows robot2 as an
      obstacle when nearby.
- [ ] RMF traffic schedule rejects/reroutes a second robot when its
      requested lane conflicts with an already-scheduled one.

See `02-openrmf-nav2-logic-operation.md` for how task allocation, traffic
scheduling, and deconfliction actually behave at runtime.
