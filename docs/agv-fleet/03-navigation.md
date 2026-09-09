# `agv_navigation`

Nav2 bringup for the fleet: one shared map server + per-robot, namespaced
SLAM/AMCL + navigation stack. Built to plug into the Open-RMF architecture
described in [`docs/open-rmf/`](../open-rmf/) later — see that folder for
the target end-state (fleet adapter, traffic scheduling, task dispatch);
this package is the Nav2 layer underneath it, not RMF itself.

**Nav2 is not installed in this workspace's base image** — only
`nav2_common`, `nav2_costmap_2d`, `nav2_msgs`, `nav2_util` are present.
Everything else (`nav2_bringup`, `nav2_amcl`, `slam_toolbox`, ...) is
declared in `agv_navigation/package.xml` so `make rosdeps` pulls it in via
apt (`ros-jazzy-nav2-*`, `ros-jazzy-slam-toolbox`), but **this has not
actually been installed or run** in the environment this was built in — see
`04-known-issues-and-next-steps.md`.

## Architecture

```
                  workcell.launch.py
                         │
        ┌─────────────────┴─────────────────┐
        │ (slam:=false only, once)           │ (per robot, looped)
   localization.launch.py              navigation.launch.py
        │                                     │
   map_server ─── /map (shared,      slam:=true → slam_toolbox   ┐
   unnamespaced)   topic              slam:=false → amcl         │ mutually
        │                                     │                   exclusive
        │                          controller_server, planner_server,
        │                          smoother_server, behavior_server,
        │                          bt_navigator, waypoint_follower,
        │                          velocity_smoother (always on)
        │                                     │
        └──────────── shared /map topic ──────┘
                (global_costmap's static_layer +
                 amcl both subscribe to it, absolute "/map")
```

One `map_server` for the whole fleet (map is static — every robot shares
it). Everything else is per-robot: own AMCL *or* own SLAM instance
(never both at once — they'd fight over publishing `map -> odom`), own
costmaps, own planner/controller/BT navigator, own lifecycle manager(s).

## TF strategy — matches vanilla Nav2 (this repo didn't always)

Standard Nav2 multi-robot convention: **separate `/<namespace>/tf` topic
per robot**, **bare frame names** (`base_link`, `odom`) everywhere. See
`02-bringup-and-simulation.md`'s "Fleet TF strategy" for the full
reasoning (N² traffic problem, config reuse, native namespacing) — this
repo originally did the opposite (one shared `/tf`, prefixed frame IDs) and
was refactored to the standard convention. What that means concretely in
this package:

- `nav2_params.yaml`'s frame-id parameters (`base_frame_id`,
  `robot_base_frame`, etc.) are bare `base_link`/`odom` — the same file
  works unmodified for every robot in the fleet.
- `global_frame` is bare `map` everywhere — the one frame actually shared
  by the whole fleet.
- `navigation.launch.py` **does** remap `/tf`/`/tf_static` → `tf`/
  `tf_static` on every node that touches transforms (AMCL, SLAM Toolbox,
  controller/planner/smoother/behavior servers, BT navigator, waypoint
  follower, velocity smoother) — this is the part that actually makes
  per-robot topics happen; a node's own `namespace=` alone does **not**
  affect `tf2_ros`'s hardcoded absolute `/tf`.

## Parameter templating: two substitution passes, composed

`nav2_params.yaml` and `mapper_params_online_async.yaml` use two different,
composed mechanisms — both already proven elsewhere in this repo, not new
machinery invented for this file:

1. **`$(var initial_pose_*)` tokens**, embedded directly in
   `amcl.initial_pose`'s x/y/yaw values — resolved by
   `ParameterFile(..., allow_substs=True)`, the exact same mechanism
   `agv_bringup/config/controllers.yaml` already uses. This is the one
   thing in `nav2_params.yaml` that's genuinely different per robot (each
   spawns at a different point); frame ids are bare now and need no
   per-robot substitution at all.
2. **`nav2_common.launch.RewrittenYaml`'s `root_key`**, wrapping the whole
   resolved param tree under the namespace key (`{"agv_1": {...}}`) so
   that the file's bare top-level keys (`amcl:`, `controller_server:`, ...)
   correctly match nodes actually running as `/agv_1/amcl`,
   `/agv_1/controller_server`, etc. **This was verified by reading
   `RewrittenYaml`'s actual source** (`/opt/ros/jazzy/lib/python3.12/
   site-packages/nav2_common/launch/rewritten_yaml.py`) rather than assumed
   — `root_key` does a single dict-wrap (`data = {root_key: data}`), and
   `param_rewrites` does a global, leaf-key-name-based substitution (every
   occurrence of that key anywhere in the tree gets the same value) — fine
   for `use_sim_time` (same value everywhere), which is why that one goes
   through `param_rewrites` rather than a `$(var ...)` token.

Composition order in `navigation.launch.py`:
`ParameterFile(RewrittenYaml(source_file=params_file, root_key=namespace, param_rewrites={"use_sim_time": ...}, convert_types=True), allow_substs=True)`
— matches `nav2_bringup`'s own established code pattern for this exact
call, just pointed at a params file with the extra `$(var ...)` tokens.

## Why not just include `nav2_bringup`'s own launch files

Now that the TF convention matches `nav2_bringup`'s own default (separate
`/<namespace>/tf` per robot, bare frame ids), its bundled
`navigation_launch.py`/`localization_launch.py` would likely work
unmodified — that's a real, available simplification if this package ever
needs trimming down. `agv_navigation/launch/navigation.launch.py` still
defines each node (`controller_server`, `planner_server`,
`smoother_server`, `behavior_server`, `bt_navigator`, `waypoint_follower`,
`velocity_smoother`) directly instead, for two reasons that are independent
of the TF convention: (1) `nav2_bringup`'s `localization_launch.py` bundles
`map_server` *and* AMCL together as one unit, but this fleet needs them
split — one shared `map_server` (`localization.launch.py`), many per-robot
AMCLs (`navigation.launch.py`) — which their bundled file doesn't support
directly; (2) explicit node definitions make the `cmd_vel`/`cmd_vel_nav`
velocity-smoother chaining (below) and the tf remap itself visible and
auditable in one place, rather than buried inside someone else's launch
file. Either reason alone would justify keeping this hand-rolled; it's not
a leftover of the old TF convention.

## `velocity_smoother` chaining

`controller_server`'s raw output is remapped `cmd_vel → cmd_vel_nav`;
`velocity_smoother` consumes `cmd_vel_nav` and republishes the smoothed
result as `cmd_vel` (its default output topic, unremapped) — which is what
`diff_drive_controller` actually subscribes to. Without this, both nodes
would publish directly onto the same `cmd_vel` topic.

## SLAM vs. AMCL (`slam` launch arg)

- **`slam:=true`** (default — no map exists for `office_world` yet):
  `slam_toolbox`'s `async_slam_toolbox_node`, namespaced, using
  `config/mapper_params_online_async.yaml`. Its own map frame is left as
  bare `map` (single-mapping-robot assumption — two simultaneous SLAM
  instances would each think they own the `map` frame, competing for CPU
  with no benefit, not producing one merged map). This is now **enforced at
  the launch level**, not just documented: `workcell_bringup/launch/
  workcell.launch.py` only gives the *first* robot in `robots_config` a
  navigation stack while `slam:=true` — other robots stay spawned and
  controllable in Gazebo but without a Nav2 stack until `slam:=false` gives
  them an actual map to localize against. (This was originally just a
  documented assumption; a live run showed two robots each spinning up
  their own `slam_toolbox` simultaneously, with one of them failing bringup
  under the resulting load — see `04-known-issues-and-next-steps.md`.)
- **`slam:=false`**: `nav2_amcl`'s `amcl` node, namespaced, subscribing to
  the shared `/map` topic (absolute path — the leading `/` is what makes it
  bypass the node's own namespace).

`map_topic` is therefore **not a fixed value** in `nav2_params.yaml`:
`navigation.launch.py` rewrites it per mode — `"/map"` under `slam:=false`
(the one fleet-wide `map_server`), relative `"map"` under `slam:=true` (this
robot's own namespaced `slam_toolbox`, publishing `/<namespace>/map`). It
was hardcoded to `/map` at first, which meant `global_costmap`'s static
layer waited forever for a topic nobody published while mapping — see
`04-known-issues-and-next-steps.md`.

**`slam_toolbox` runs with its lifecycle bond disabled**
(`bond_timeout: 0.0` on `lifecycle_manager_slam` only). It does not create
its bond until `on_activate` returns, and under software rendering with two
robots' `gpu_lidar` raycasting that outran both 4.0s and 10.0s timeouts —
`lifecycle_manager_slam` then aborted the entire bringup for a node that
was demonstrably alive and processing scans. The cost is losing automatic
detection of a genuinely dead `slam_toolbox`; `lifecycle_manager_navigation`
keeps its 10.0s bond, since those nodes activate quickly.

To build the first map: launch with `slam:=true`, drive `agv_1` around
(e.g. `teleop_twist_keyboard --stamped` publishing to `/agv_1/cmd_vel`, or
`/agv_1/cmd_vel_nav` if going through the smoother — note `--stamped`:
`diff_drive_controller` 4.x only accepts `TwistStamped`), then save it —
`ros2 run nav2_map_server map_saver_cli -f <path>` or SLAM Toolbox's own
save-map service. Then relaunch with `slam:=false map:=<path>.yaml`.

## Controller/planner choice

`FollowPath` → `nav2_regulated_pure_pursuit_controller` (dependency-light,
well-supported default for a small diff-drive base). `GridBased` →
`nav2_navfn_planner` (simple, reliable, no extra dependencies). Both are
config-only swaps in `nav2_params.yaml` if something more advanced is
needed later — e.g. `nav2_mppi_controller::MPPIController`, as sketched in
the Open-RMF integration guide for cross-robot-aware planning.

## Footprint and velocity limits

`local_costmap`/`global_costmap` footprint:
`[[0.176, 0.126], [0.176, -0.126], [-0.176, -0.126], [-0.176, 0.126]]` —
AGVv3's actual `base_link` box (`0.35177 × 0.25126`), halved and rounded.
`velocity_smoother`'s `max_velocity`/`max_accel` match
`diff_drive_controller`'s command range. **Neither is derived
automatically** — if the robot's body dimensions or speed limits change in
`agv_description`/`agv_bringup`, these need updating by hand to match.

## Package layout

```
agv_navigation/
  config/
    nav2_params.yaml                  per-robot Nav2 template (AMCL, costmaps, planner, controller, BT, ...)
    mapper_params_online_async.yaml   SLAM Toolbox template
  launch/
    localization.launch.py            fleet-wide map_server + lifecycle manager (once)
    navigation.launch.py              per-robot: SLAM xor AMCL + nav stack (looped by workcell.launch.py)
  maps/                                empty - saved maps go here
```
