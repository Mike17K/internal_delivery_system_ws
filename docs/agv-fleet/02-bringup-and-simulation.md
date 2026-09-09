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
   is false (real-hardware / mock path). Deliberately gets **no** tf remap:
   `controller_manager` never publishes tf, only the controllers it loads
   do, and a remap on this process does not reach them (see item 5). **In
   `sim_gazebo` mode this node never runs at all** — `gz_ros2_control`'s
   plugin (in `agv_macro.xacro`'s `<gazebo>` block) creates its own
   `controller_manager` *inside the Gazebo process itself* instead.
3. **`ros_gz_sim create`** — spawns the robot into the already-running
   Gazebo world via `-topic robot_description -name <namespace>`, with pose
   given as `-x -y -z -R -P -Y` (parsed from the `xyz`/`rpy` args).
4. **`ros_gz_bridge parameter_bridge`** — bridges pose and the lidar scan out of Gazebo
   (see `config/gz_bridge.yaml` below).
5. **Two `controller_manager/spawner` invocations** — one for
   `joint_state_broadcaster`, one for `diff_drive_controller` — both
   delayed 4s (`TimerAction`) to let the controller manager come up first.
   They're split because only `diff_drive_controller` needs
   `--controller-ros-args`, and those args apply to every controller named
   in a single spawner call:

   ```python
   "--controller-ros-args",
   "-r /tf:=tf -r /tf_static:=tf_static -r ~/odom:=odom -r ~/cmd_vel:=cmd_vel",
   ```

   **This is the single most important line in the bringup**, and the
   reason it has to be here rather than in a `remappings=` list is worth
   internalizing: a controller is *not a process*. `controller_manager`
   builds `diff_drive_controller` as a node inside its own process, and
   that node creates its own `TransformBroadcaster` (hardcoded absolute
   `/tf`). Remaps given to the `controller_manager` process — from a launch
   file or from the gz plugin's `<ros><remapping>` — configure that
   process's node, never the controller nodes it constructs later. The
   spawner instead writes these args to the controller's
   `node_options_args` parameter *before* loading it, so they're applied at
   controller-node construction. It works the same in `sim_gazebo` mode and
   on real hardware, because the args travel with the controller. The four
   remaps do:

   | Remap | Fixes |
   | --- | --- |
   | `/tf`, `/tf_static` | `odom -> base_link` lands on `/<ns>/tf`, not the global `/tf` |
   | `~/odom` | `/<ns>/odom`, which `nav2_params.yaml`'s `odom_topic: odom` resolves to (default: `/<ns>/diff_drive_controller/odom`) |
   | `~/cmd_vel` | `/<ns>/cmd_vel`, what `velocity_smoother` publishes (default: `/<ns>/diff_drive_controller/cmd_vel`) |

6. **`odom_bootstrap`** (`ExecuteProcess`) — publishes three zero
   `TwistStamped` messages to `/<namespace>/cmd_vel`. This looks like a
   hack and is not: `diff_drive_controller` 4.x early-returns out of
   `update_and_write_commands` while its reference interfaces are NaN
   (which they are from activation until the first command), and the
   `/odom` publisher *and* the `odom -> base_link` tf broadcast both sit
   after that early return. Nav2 won't command the base until it can
   resolve `odom`; the controller won't produce `odom` until commanded.
   One finite command breaks the deadlock permanently. Full evidence,
   including the disassembly, is in
   `04-known-issues-and-next-steps.md`. **Delete this and the robot has no
   `odom` frame, ever.**

## `agv_bringup/config/controllers.yaml`

```yaml
diff_drive_controller:
  ros__parameters:
    left_wheel_names: ["wheel_left_joint"]
    right_wheel_names: ["wheel_right_joint"]
    wheel_separation: 0.28      # from AGVv3's actual joint geometry
    wheel_radius: 0.02011
    base_frame_id: "/base_link"      # see "Frame ids" below
    odom_frame_id: "/odom"
    tf_frame_prefix_enable: false     # ditto - required
    enable_odom_tf: true
    cmd_vel_timeout: 0.5
```

There is deliberately **no `use_stamped_vel`** — it was removed from
`diff_drive_controller` 4.x, which now accepts `geometry_msgs/TwistStamped`
on `~/cmd_vel` and nothing else. Nav2 defaults to unstamped `Twist`, so
`nav2_params.yaml` sets `enable_stamped_cmd_vel: true` on
`controller_server`, `behavior_server` and `velocity_smoother`. The two
must agree, or `cmd_vel` connects by topic name and silently never
delivers a message. The controller side is confirmed — `odom_bootstrap`
(item 6) publishes `TwistStamped` and the controller acts on it; Nav2's own
output hasn't been exercised by a navigation goal yet.

Joint names are bare (`wheel_left_joint`) — not prefixed with the
namespace. This `diff_drive_controller` instance already runs scoped to one
robot (the whole file is wrapped in `/$(var namespace):`, a
*node-parameter* scoping concern, unrelated to frame naming), so there's no
ambiguity a bare name could cause.

### Frame ids — `tf_frame_prefix_enable: false` is required

The effective frame names are bare (`base_link`, `odom`), matching
`robot_state_publisher` and `nav2_params.yaml`, per the fleet TF strategy
below. Getting them that way takes two settings, both non-obvious:

- **`tf_frame_prefix_enable: false`.** `diff_drive_controller` defaults
  this to `true`, and with `tf_frame_prefix` empty it falls back to the
  controller node's namespace. Left at the default it published
  `agv_1/odom -> agv_1/base_link`, while `robot_state_publisher` (its own
  `frame_prefix` defaults to `""`) published
  `base_link -> {wheel_*, caster_*, lidar_link}` — two disjoint trees on
  the same topic, and no frame named plain `odom` anywhere.
- **The leading `/`** on `"/base_link"` / `"/odom"`. `tf2` strips a leading
  slash from frame ids on receipt (`BufferCore::setTransform`), so these
  arrive as `base_link` / `odom`, and the values remain correct even if
  some prefix were concatenated in front of them again.

This exact pair is what was verified working. Bare names should behave
identically with the prefix disabled, but that variant wasn't tested —
re-run `view_frames` (below) if you change either.

`wheel_separation`/`wheel_radius` are real, geometry-derived values (from
`agv_macro.xacro`'s joint origins and cylinder radius) — not placeholders.
Earlier robot revisions (AGV, AGV_S) had zeroed-out or ambiguous wheel
offsets in their raw export, so these numbers were estimated/guessed at the
time; they're exact now that AGVv3 provides real joint origins. **If the
robot model's wheel geometry changes again, this file needs updating to
match** — nothing computes it automatically.

**`enable_odom_tf: true` — necessary, but not sufficient by itself.**
`diff_drive_controller` is the standard, correct source for `odom ->
base_link` (integrated from commanded wheel motion) — what Nav2/AMCL/SLAM
expect. `PosePublisher`'s tf output is not bridged into ROS (see
`gz_bridge.yaml` below) to avoid it also claiming a conflicting
(ground-truth-relative) parent for `base_link`.

This flag is one of **three** things that must all be right before a usable
`odom -> base_link` exists. Each one missing on its own produced the
byte-identical `Invalid frame ID "odom"` from Nav2, which is why it took
seven runs to unpick:

1. **The topic.** The tf broadcaster this flag enables hardcodes an
   absolute `/tf`; only the spawner's `--controller-ros-args` can redirect
   it onto `/<namespace>/tf` (item 5 above).
2. **Publishing at all.** The controller must have received at least one
   velocity command, or it early-returns before computing any odometry
   (item 6 above).
3. **The frame names.** `tf_frame_prefix_enable: false` — see "Frame ids"
   above.

The full history, with the disassembly and `view_frames` output that
settled it, is in `04-known-issues-and-next-steps.md`. Check all three
before touching anything else.

**The diagnostic that actually distinguishes them:**

```bash
ros2 run tf2_tools view_frames --ros-args -r tf:=/agv_1/tf -r tf_static:=/agv_1/tf_static
```

Nothing on the topic → cause 1 or 2. Two disconnected trees → cause 3.
`ros2 topic echo /agv_1/tf --once` shows frame names but makes a
disconnected tree look like a perfectly reasonable transform.

## `agv_bringup/config/gz_bridge.yaml`

Bridges two things out of Gazebo (plus the lidar's scan — see
`01-robot-description.md`), namespace-templated via `$(var namespace)`:

```yaml
- ros_topic_name: "/$(var namespace)/pose"
  gz_topic_name: "/model/$(var namespace)/pose"
  ...
```

`PosePublisher`'s tf output is deliberately **not** bridged (it used to be,
as `/$(var namespace)/tf` — removed). With `diff_drive_controller` now
publishing `odom -> base_link` and `robot_state_publisher` publishing
`base_link -> {wheels, lidar_link, casters}`, bridging `PosePublisher`'s tf
too would mean two sources independently claiming a (different) parent for
`base_link` — an active conflict, not just redundant data. The `/pose`
bridge above is unaffected (an independent gz topic) — ground-truth pose
data is still available there for anything that wants it later (RMF fleet
state, monitoring, etc.), it's just no longer part of the tf tree Nav2
relies on.

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
This is applied in `agv_bringup/launch/bringup.launch.py`
(`robot_state_publisher`) and `agv_navigation/launch/navigation.launch.py`
(AMCL / SLAM Toolbox and the whole navigation stack) — miss this remap on
any one node and that node silently falls back to the global `/tf`, which
is exactly the kind of subtle multi-robot bug this convention is otherwise
supposed to prevent.

**Two exceptions, both learned the hard way:**

- **`ros2_control` controllers don't take it this way.** A controller is a
  node built *inside* `controller_manager`'s process, so a `remappings=`
  on the process never reaches it. It goes on the spawner instead, as
  `--controller-ros-args "-r /tf:=tf ..."`. `controller_manager` itself
  gets no tf remap at all — it publishes no tf.
- **Getting the topic right does not get the frame names right.**
  `diff_drive_controller` prefixes frame ids with its namespace by default,
  which yields a correctly-namespaced topic carrying frames
  (`agv_1/odom`, `agv_1/base_link`) that no other publisher on that topic
  agrees with. See "Frame ids" above.

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
