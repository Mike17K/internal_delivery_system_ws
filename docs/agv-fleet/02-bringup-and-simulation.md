# `agv_bringup` + `workcell_bringup`

How one AGV gets from URDF to a controllable, moving entity in Gazebo, and
how `workcell_bringup` loops that over a fleet.

## `agv_bringup/launch/bringup.launch.py` — one robot

Args: `use_fake_hardware` (default `true`), `sim_gazebo` (default `false`),
`xyz` / `rpy` (spawn pose, space-separated strings — passed to the spawner,
**not** baked into the URDF), `namespace` (default `""`).

Nodes, in order:

1. **`robot_state_publisher`** — publishes the URDF-derived transforms.
   Namespaced via `namespace=`, *and* explicitly remapped
   (`[("/tf", "tf"), ("/tf_static", "tf_static")]`) — see "Fleet TF
   strategy" below for why the remap is the part that actually matters.
2. **`ros2_control_node`** (`controller_manager`) — only when `sim_gazebo`
   is false (real-hardware / mock path; in sim, `gz_ros2_control`'s plugin
   inside gz-sim itself plays this role instead). Same tf remap applied.
3. **`ros_gz_sim create`** — spawns the robot into the already-running
   Gazebo world via `-topic robot_description -name <namespace>`, with pose
   given as `-x -y -z -R -P -Y` (parsed from the `xyz`/`rpy` args).
4. **`ros_gz_bridge parameter_bridge`** — bridges pose/tf out of Gazebo
   (see `config/gz_bridge.yaml` below).
5. **`joint_state_broadcaster` + `diff_drive_controller` spawner** — via
   `controller_manager/spawner`, delayed 4s (`TimerAction`) to let the
   controller manager come up first.

## `agv_bringup/config/controllers.yaml`

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]
    wheel_separation: 0.28      # from AGVv3's actual joint geometry
    wheel_radius: 0.02011
    base_frame_id: "base_link"   # bare - see "Fleet TF strategy" below
    odom_frame_id: "odom"
    enable_odom_tf: false        # PosePublisher owns tf instead - see below
```

Joint names and frame ids are bare (`wheel_left_joint`, `base_link`, `odom`)
— not prefixed with the namespace. This `diff_drive_controller` instance
already runs scoped to one robot (the whole file is wrapped in
`/$(var namespace):`, a *node-parameter* scoping concern, unrelated to frame
naming), so there's no ambiguity a bare name could cause.

`wheel_separation`/`wheel_radius` are real, geometry-derived values (from
`agv_macro.xacro`'s joint origins and cylinder radius) — not placeholders.
Earlier robot revisions (AGV, AGV_S) had zeroed-out or ambiguous wheel
offsets in their raw export, so these numbers were estimated/guessed at the
time; they're exact now that AGVv3 provides real joint origins. **If the
robot model's wheel geometry changes again, this file needs updating to
match** — nothing computes it automatically.

`enable_odom_tf: false` is deliberate: `diff_drive_controller` *could*
publish `odom -> base_link`, but the Gazebo `PosePublisher` plugin (in
`agv_macro.xacro`'s `<gazebo>` block) already publishes the whole tf tree
from the simulator's ground-truth poses. Having both would mean two
competing broadcasters for the same frames.

## `agv_bringup/config/gz_bridge.yaml`

Bridges exactly two things out of Gazebo, both namespace-templated via
`$(var namespace)` — **on both sides**:

```yaml
- ros_topic_name: "/$(var namespace)/pose"
  gz_topic_name: "/model/$(var namespace)/pose"
  ...
- ros_topic_name: "/$(var namespace)/tf"
  gz_topic_name: "/$(var namespace)/tf"
  ...
```

The Gazebo `PosePublisher` plugin (in `agv_macro.xacro`) publishes poses
using the URDF's bare link names (`base_link`, `wheel_left_link`, ...); this
bridge lands them on `agv_1`'s *own* `/agv_1/tf` topic, not a shared global
one — that per-robot topic is what disambiguates `agv_1`'s `base_link` from
`agv_2`'s, not the frame name text.

## Fleet TF strategy — read this before touching anything TF-related

**Separate `/<namespace>/tf` topic per robot, bare frame IDs** (`base_link`,
`odom`, ...) — the standard ROS 2/Nav2 multi-robot convention. This
workspace originally did the opposite (one shared `/tf` topic with
namespace-prefixed frame IDs like `agv_1/base_link`) and was refactored to
this convention deliberately, for reasons worth keeping in mind before ever
reverting:

- **Avoids the N² problem**: with one shared `/tf` topic, every robot's
  Nav2 stack receives and has to parse every *other* robot's high-rate
  local TF traffic (`base_link` ↔ wheel joints, etc.) just to find its own.
  Separate topics mean each robot's stack only ever sees its own transforms.
- **Config reuses across every robot unmodified**: `nav2_params.yaml` and
  `controllers.yaml` use plain `base_link`/`odom` throughout — nothing in
  either file needs to differ per robot (except genuinely per-robot
  *values* like initial pose), because frame names never encode identity.
- **Matches how `gz_ros2_control`/`ros_gz_bridge` already namespace
  everything else**: ordinary topics (`cmd_vel`, `odom`, `scan`, ...) were
  already automatically namespaced by each node's own `namespace=` — frame
  prefixing was the one place this repo didn't follow that same pattern.

**The mechanism, not just the convention**: `tf2_ros::TransformBroadcaster`/
`TransformListener` hardcode an *absolute* `/tf` and `/tf_static` topic
internally — a node's own `namespace=` does **not** touch an already-
absolute topic name. Getting a per-robot `/agv_1/tf` requires an *explicit*
remap on every tf-touching node:
```python
remappings=[("/tf", "tf"), ("/tf_static", "tf_static")]
```
This is applied in `agv_bringup/launch/bringup.launch.py` (robot_state_publisher,
controller_manager) and `agv_navigation/launch/navigation.launch.py` (AMCL/
SLAM Toolbox and the whole navigation stack) — miss this remap on any one
node and that node silently falls back to the global `/tf`, which is exactly
the kind of subtle multi-robot bug this convention is otherwise supposed to
prevent.

**What this does *not* give you for free**: a single RViz session showing
every robot in one unified tree. Each robot's `map -> odom -> base_link`
chain lives entirely on that robot's own topic; nothing merges them. A
fleet-wide view (or letting one robot's costmap see another robot as an
obstacle) needs an explicit aggregator/relay node reading each robot's tf
and republishing centrally — not implemented here, see
`04-known-issues-and-next-steps.md`.

## `workcell_bringup/launch/workcell.launch.py` — the fleet

Top-level entry point. Args: `use_fake_hardware`, `sim_gazebo`, `world`
(default `office_world.sdf`), `slam` (default `true`), `map` (default `""`,
required when `slam:=false`).

1. Sets `GZ_SIM_RESOURCE_PATH` to `workcell_description/models` — needed
   for the office furniture's `model://` mesh references. (AGV's own mesh
   does *not* need this — see `01-robot-description.md`'s `$(find ...)`
   explanation.)
2. Starts Gazebo (`ros_gz_sim`'s `gz_sim.launch.py`) with the chosen world,
   only if `sim_gazebo:=true`.
3. Clock bridge (`/clock`, Gazebo → ROS), and a static `world -> map`
   transform (published unconditionally, not gated on `sim_gazebo`).
4. **Fleet-wide `map_server`** (via `agv_navigation/launch/localization.launch.py`),
   included once — only when `slam:=false`. Every robot's AMCL localizes
   against this one shared map; there is no per-robot map.
5. **Per-robot loop** over `robots_config` (currently `agv_1` at
   `(0,0,0)`, `agv_2` at `(1,0,0)`): for each, includes
   `agv_bringup/launch/bringup.launch.py` (staggered `0.5s × i` to avoid
   simultaneous spawn requests) and, 3s after that,
   `agv_navigation/launch/navigation.launch.py` (giving the robot's own
   spawn/controllers time to come up first).

**Adding a third robot**: append one entry to `robots_config` — name +
spawn pose. Nothing else needs to change; bringup and navigation are both
already parameterized by `namespace` with no hardcoded robot count anywhere
in the loop.

## Debugging notes worth keeping

- **Don't build from two different absolute paths for the same
  `build`/`install`/`log` tree.** This workspace is bind-mounted into a
  devcontainer at a different path (`/workspaces/isaac_ros-dev`) than its
  host path. Running `colcon build` from the host path against an
  `install`/`build` directory that was generated from inside the container
  (or vice versa) fails with a `CMakeCache.txt` path mismatch — and
  `rm -rf build install log` to "fix" it destroys the other environment's
  real build artifacts (recoverable via `make` there, but avoidable:
  always build from the same environment/path you're going to launch from).
- **`libEGL warning: failed to open /dev/dri/renderD128: Permission
  denied`** in the `gz sim` log is a software-rendering fallback (no GPU
  passthrough in that environment), not a functional error — office world
  furniture and the AGV both render fine under it.
