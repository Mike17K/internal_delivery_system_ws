# Known Issues & Next Steps

Honest record of what's incomplete, unverified, or assumed — read this
before trusting the rest of this folder as "done and working."

Nav2 has now been launched seven times against live Gazebo output, each run
surfacing real bugs that got fixed before the next: `bt_navigator`
segfaulting, redundant SLAM instances, a lidar scan `frame_id` that matched
nothing, and — the long one — a missing `odom` frame that took until run 7
and turned out to be **three** independent bugs stacked on each other, each
producing the byte-identical `Invalid frame ID "odom"` from Nav2:

1. a process-vs-node remapping distinction in `ros2_control`, so the
   transform would have gone to the global `/tf`;
2. a NaN early-return in `diff_drive_controller`, so no odometry was being
   published on any topic at all;
3. `tf_frame_prefix_enable` defaulting to `true`, so the frames that *did*
   finally appear were named `agv_1/odom` → `agv_1/base_link` and formed a
   tree completely disjoint from `robot_state_publisher`'s.

**Confirmed working on live runs**: the lidar `frame_id`
(`gz_frame_id` → `lidar_link`), the spawner's controller-scoped tf remap,
the odometry bootstrap (`odom -> base_link` now publishes at ~50 Hz on
`/agv_1/tf`), the `TwistStamped` command contract, and the per-mode `map`
topic. **Still unverified**: the `slam_toolbox` bond change, and whether
`slam_toolbox` now accepts scans with a connected TF tree. Full end-to-end
navigation (an actual `NavigateToPose` goal succeeding) has not yet been
demonstrated.

## No fleet-wide TF view

Each robot's TF lives entirely on its own `/<namespace>/tf` topic (see
`02-bringup-and-simulation.md`'s "Fleet TF strategy") — nothing merges
`agv_1`'s and `agv_2`'s trees into one. Practical consequences: a single
RViz session can't show both robots' full TF trees at once without adding
both topics manually and accepting they don't share a common displayed
root beyond the static `world -> map`; and no robot's costmap currently
sees any *other* robot as an obstacle (the "fleet_obstacle_layer" sketched
in the Open-RMF integration guide, publishing each robot's footprint to a
shared topic other robots' costmaps subscribe to, is not implemented).
Fixing either needs an explicit relay/aggregator node reading each robot's
`map -> odom -> base_link` off its own namespaced tf and republishing it
somewhere shared — a real, separate piece of work, not a side effect of
anything currently in `agv_navigation`.

## `odom` frame didn't exist at all — seven runs, three stacked causes

**Run 2** turned up a fundamental bug: `local_costmap` logged, forever,
`Timed out waiting for transform from base_link to odom ... Invalid frame
ID "odom" ... frame does not exist`. Not a timing issue — the frame
genuinely did not exist anywhere in tf. Root cause #1:
`agv_bringup/config/controllers.yaml` had `enable_odom_tf: false`, on the
documented (at the time) theory that the Gazebo `PosePublisher` plugin
covered tf instead. That reasoning was wrong — `PosePublisher` mirrors
ground-truth simulator poses; it has no concept of "odometry." **Fix
attempt 1**: `enable_odom_tf: true`, and stopped bridging `PosePublisher`'s
tf into ROS at all (it would otherwise claim a conflicting parent for
`base_link` once `diff_drive_controller` owned `odom -> base_link`).

**Run 3, with fix attempt 1 in place, showed the exact same error.** This
exposed root cause #2: in `sim_gazebo` mode, `controller_manager` (and
`diff_drive_controller`) don't run as a normal launched ROS node at all —
they run **inside the Gazebo process itself**, loaded by the
`gz_ros2_control-system` plugin (confirmed by the log: those lines are
prefixed `[gazebo-1]`, i.e. gzserver's own stdout). The tf remap applied to
every *properly launched* node never reached this plugin-internal instance.
`diff_drive_controller` *was* publishing `odom -> base_link` the whole
time, just onto the **global** `/tf`. **Fix attempt 2**: added
`<remapping>/tf:=tf</remapping>` directly to the plugin's own `<ros>`
block in `agv_macro.xacro`.

**Run 4, with fix attempt 2 in place, showed the exact same error yet
again.** The user also captured `ros2 node info /agv_1/controller_server`
independently, and the run 4 log itself was the tell: the remap *was*
visibly present in the controller's own launch arguments
(`--remap /tf:=tf --remap /tf_static:=tf_static`), but
`controller_manager` also logged `The use of remapping arguments to the
controller_manager node is deprecated. Please use the
'--controller-ros-args' argument of the spawner to pass remapping
arguments to the controller node.`

**Fix attempt 3** — a small `odom_to_tf.py` relay node that re-published
the `Odometry` topic as tf — was written and then **rejected**: it
sidesteps the problem instead of fixing it, and adds a node to every robot
in the fleet forever to work around a two-line config gap.

**The actual fix (attempt 4)** is the thing `controller_manager`'s own log
message had been saying all along. Root cause #3 is a *process vs. node*
distinction: a controller is not a process. `controller_manager`
instantiates `diff_drive_controller` as a node **inside its own process**,
and that node builds its own `tf2_ros::TransformBroadcaster` — which
hardcodes an absolute `/tf`. Remap arguments given to the
`controller_manager` process (whether from a launch file's `remappings=`
or from the gz plugin's `<ros><remapping>`) configure that process's node,
not the controller nodes it constructs later. Nothing at the process level
can reach them.

`--controller-ros-args` is the supported way in. The spawner writes the
args to the controller's `node_options_args` parameter on
`controller_manager` *before* issuing `load_controller`, so they are
applied when the controller node is constructed. In
`agv_bringup/launch/bringup.launch.py`:

```python
arguments=[
    "diff_drive_controller",
    *controller_manager_args,
    "--controller-ros-args",
    "-r /tf:=tf -r /tf_static:=tf_static -r ~/odom:=odom -r ~/cmd_vel:=cmd_vel",
]
```

Because the args travel with the *controller*, this works identically in
`sim_gazebo` mode and on real hardware — the plugin-internal
`controller_manager` is no longer something a fix has to reach. Three
consequences, all applied:

- The `<remapping>` pair was removed from `agv_macro.xacro`'s `<ros>`
  block (ineffective, and the source of the deprecation warning), with a
  "do not add this back" comment in its place.
- The tf remap was removed from the standalone `controller_manager_node`
  in `bringup.launch.py` for the same reason — `controller_manager` itself
  never publishes tf.
- The spawner was **split into two** `Node` actions, because
  `--controller-ros-args` applies to every controller named in a single
  spawner call and only `diff_drive_controller` should get these.

### Run 5: remap confirmed applied, `odom` still missing — second cause

Run 5 showed the remap arriving exactly as intended:

```
[agv_1.spawner_diff_drive_controller]: Setting controller param
  "node_options_args" to "['-r', '/tf:=tf', '-r', '/tf_static:=tf_static',
  '-r', '~/odom:=odom', '-r', '~/cmd_vel:=cmd_vel']" for diff_drive_controller
[agv_1.controller_manager]: Controller 'diff_drive_controller' node arguments:
  ... --ros-args -r /tf:=tf -r /tf_static:=tf_static -r ~/odom:=odom -r ~/cmd_vel:=cmd_vel
```

…and `Invalid frame ID "odom"` regardless. The remapping work was correct
and necessary, but it was never the last blocker. **Root cause #4**, found
by disassembling the installed controller rather than reasoning about
config:

`DiffDriveController::update_and_write_commands` opens with an
`std::isfinite` check on `reference_interfaces_[0..1]` and returns
`return_type::OK` immediately when either is NaN. In
`libdiff_drive_controller.so` at the top of that symbol:

```asm
mov  0x118(%r14),%rax    ; reference_interfaces_.data()
movsd (%rax),%xmm0       ; [0] linear
movsd 0x8(%rax),%xmm1    ; [1] angular
andpd %xmm3,%xmm0        ; fabs
ucomisd %xmm0,%xmm2      ; vs +INF  -> isfinite
jb    6f6f3              ; not finite ->
6f6f3: xor %ebx,%ebx     ; return_type::OK
       ret               ; <- before ANY odometry work
```

`reset_buffers()` NaN-fills those interfaces on activation (verified: it
stores a `.rodata` NaN across `this+0x118..0x120`), and only an incoming
velocity command makes them finite. `Odometry::update`, the `/odom`
publisher **and** the `odom -> base_link` broadcaster all sit *after* that
early return. So a freshly activated `diff_drive_controller` publishes no
odometry at all, on any topic — which is why every remapping fix in runs
2-5 was aiming at a transform that was never being broadcast in the first
place. `[agv_2.diff_drive_controller]: Command message contains NaNs. Not
updating reference interfaces.` was the same fact, visible in the log the
whole time.

**This is a bootstrap deadlock**: Nav2 issues no velocity command until its
costmaps resolve `odom`, and `odom` doesn't exist until a velocity command
arrives.

**Fix**: `bringup.launch.py` publishes three zero `TwistStamped` messages
to `/<namespace>/cmd_vel` at bringup (`odom_bootstrap`, an
`ExecuteProcess` running `ros2 topic pub -w 1 --times 3 --rate 2`). One
finite command is enough, permanently — nothing returns the reference
interfaces to NaN except a deactivate/activate cycle (a `cmd_vel` timeout
writes finite zeros, and `reset_buffers` is reachable only from the
lifecycle callbacks). `-w 1` waits for the controller's own subscription to
match, so it doesn't depend on the `TimerAction` period being generous
enough.

**Confirmed on run 6**: with the bootstrap in place, `/agv_1/tf` finally
carried a transform. But the frame names in it were wrong, which was the
third and last cause — see the next section.

### Run 6/7: the frames were prefixed — third cause

Run 6 produced this on `/agv_1/tf`:

```yaml
frame_id: agv_1/odom
child_frame_id: agv_1/base_link
```

and `view_frames` on the same topic showed two **disjoint** trees:

```
agv_1/base_link:  parent: 'agv_1/odom'   rate: 50.249   <- diff_drive_controller
wheel_left_link:  parent: 'base_link'    rate: 20.308   <- robot_state_publisher
wheel_right_link: parent: 'base_link'    rate: 20.308
caster_back_link: parent: 'base_link'
caster_front_link:parent: 'base_link'
lidar_link:       parent: 'base_link'
```

`agv_1/base_link` and `base_link` are different frames, so nothing
connected, and no frame named plain `odom` existed — Nav2's error message
was literally true the whole time.

**Root cause #5**: `diff_drive_controller` defaults
`tf_frame_prefix_enable` to `true`, and when `tf_frame_prefix` is empty it
falls back to the controller node's namespace. So `base_frame_id:
"base_link"` was silently published as `agv_1/base_link`, while
`robot_state_publisher` (whose own `frame_prefix` defaults to `""`) used
the bare names. This repo's whole TF convention is *topic* namespacing with
bare frame names (see `02-bringup-and-simulation.md`), so the prefixing has
to be off.

**Fix**, in `agv_bringup/config/controllers.yaml`:

```yaml
base_frame_id: "/base_link"
odom_frame_id: "/odom"
tf_frame_prefix_enable: false
```

`tf_frame_prefix_enable: false` is what removes the `agv_1/` prefix. The
leading `/` on the two values is belt-and-braces: `tf2` strips a leading
slash from frame ids on receipt (`BufferCore::setTransform`), so they
arrive as `base_link`/`odom` regardless, and the values stay correct even
if a prefix were ever concatenated in front of them again. This exact
combination is the one verified working; bare names are expected to behave
identically with the prefix disabled, but that variant was not the one
tested — don't "simplify" it without re-running `view_frames`.

**Diagnostic worth keeping.** The single command that made this obvious
after four runs of guessing:

```bash
ros2 run tf2_tools view_frames --ros-args -r tf:=/agv_1/tf -r tf_static:=/agv_1/tf_static
```

`ros2 topic echo /agv_1/tf --once` shows the frame names too, but
`view_frames` is what shows two *disconnected* trees as such rather than as
one plausible-looking transform.

## Lidar scan `frame_id` — fixed, confirmed on run 5

`agv_description`'s `agv_macro.xacro` has a `gpu_lidar` sensor on
`lidar_link`, bridged to ROS as `/<namespace>/scan`. The bridge itself
works exactly as designed (confirmed on all four runs: `Creating GZ->ROS
Bridge: [/model/agv_1/scan (gz.msgs.LaserScan) -> /agv_1/scan
(sensor_msgs/msg/LaserScan)]`), but `slam_toolbox` logged, on **all four
live runs**, the identical `Message Filter dropping message: frame
'agv_1/base_link/lidar' ... queue is full`, forever. That frame matches
nothing in the published TF tree.

**Cause**: gz-sim stamps sensor messages with the sensor's *scoped entity
name* — `<model>/<link>/<sensor>` — not with the URDF link name. There was
never any chance of `agv_1/base_link/lidar` matching a
`robot_state_publisher` frame.

`<disableFixedJointLumping>true</disableFixedJointLumping>` was added
between runs 1 and 2, on the theory that URDF→SDF conversion was merging
`lidar_link` into `base_link`, and had no observed effect — for two
independent reasons, both now understood:

1. It was placed on the **link** (`<gazebo reference="lidar_link">`).
   sdformat's URDF converter reads it off the **joint**. It is now also on
   `lidar_joint`, which is where it actually takes effect.
2. Even with lumping correctly disabled, the scoped name would only have
   become `agv_1/lidar_link/lidar` — still not `lidar_link`. The frame name
   was never really about lumping at all.

**Fix**: `<gz_frame_id>lidar_link</gz_frame_id>` inside the `<sensor>`,
which overrides the stamp outright with the real URDF link name.
`disableFixedJointLumping` is kept (now on the joint) for the separate,
genuine benefit of the SDF keeping a real `lidar_link` to hang the sensor
off, rather than a sensor re-parented onto `base_link` with a baked-in
offset.

**Confirmed on run 5**: the frame is now `lidar_link`, exactly as
intended —

```
[agv_1.slam_toolbox]: Message Filter dropping message: frame 'lidar_link'
  at time 2.400 for reason 'discarding message because the queue is full'
```

— and the remaining drop is a *different* problem: `slam_toolbox` can
resolve the frame name now, but not the `lidar_link -> odom` chain, because
`odom` still didn't exist on that run (see above). gz-sim also logs
`XML Element[gz_frame_id], child of element[sensor], not defined in SDF.
Copying[gz_frame_id] as children of [sensor]` — that warning is expected
and harmless; it is how `gz_frame_id` is delivered to the sensor system,
and the correct frame in the output proves it was read.

Separately, still open regardless of the above: `gpu_lidar` uses the
Sensors system's render pipeline, which has been running under software
rendering (llvmpipe, no GPU passthrough) throughout this project. Worth
confirming raycasting isn't unusably slow under that fallback, especially
at fleet scale, before relying on it — real GPU passthrough or the
CPU-only `lidar` sensor type (no render dependency) are the two ways out if
it is.

## `slam_toolbox` lifecycle bond — bond disabled after two failures

On two separate runs `lifecycle_manager_slam` gave up on `slam_toolbox`:

```
[agv_1.lifecycle_manager_slam]: Server slam_toolbox was unable to be reached
  after 10.00s by bond. This server may be misconfigured.
[agv_1.lifecycle_manager_slam]: Failed to bring up all requested nodes.
  Aborting bringup.
```

`bond_timeout` had already been raised 4.0 → 10.0 for this exact symptom
and it failed identically at the new value — while `slam_toolbox` was
demonstrably alive and processing scans seconds later. The bond isn't
created until `slam_toolbox`'s `on_activate` returns, and under llvmpipe
software rendering with two robots' `gpu_lidar` raycasting that takes
longer than any timeout worth setting.

`lifecycle_manager_slam` now gets `bond_timeout: 0.0`, which disables the
bond rather than lengthening it. The cost is losing automatic detection of
a genuinely dead `slam_toolbox`; the alternative is a manager that aborts
the entire bringup because a node was slow to start, which is strictly
worse. `lifecycle_manager_navigation` keeps its 10.0s bond — those nodes
activate quickly and the monitoring is worth having there.

## Three more wiring bugs found by inspection

Found while auditing the whole launch/param chain for the `odom` bug —
none of these had surfaced in a log yet, because the missing `odom` frame
was stopping the stack before any of them could matter.

**`cmd_vel` never would have connected.** `diff_drive_controller` 4.x
accepts only `geometry_msgs/TwistStamped` on `~/cmd_vel`; Nav2 (Jazzy)
publishes unstamped `Twist` by default. A type mismatch like this doesn't
error — the publisher and subscriber simply never match, and the robot sits
still. Fixed with `enable_stamped_cmd_vel: true` on `controller_server`,
`behavior_server` and `velocity_smoother` in `nav2_params.yaml`, paired
with the `~/cmd_vel` remap on the spawner. **Confirmed**: the
`odom_bootstrap` action publishes `TwistStamped` to `/<ns>/cmd_vel` and the
controller acted on it (odometry started flowing), so the type contract on
the controller side is settled. What is still unverified is Nav2's *own*
output — `enable_stamped_cmd_vel: true` was set on `controller_server`,
`behavior_server` and `velocity_smoother` but no navigation goal has been
run yet. `ros2 topic info /agv_1/cmd_vel -v` during an active goal is the
check; every endpoint must read `geometry_msgs/msg/TwistStamped`.

**`odom` topic name mismatch.** `nav2_params.yaml` uses `odom_topic: odom`
(→ `/agv_1/odom`), but `diff_drive_controller` publishes on `~/odom` (→
`/agv_1/diff_drive_controller/odom`). Fixed by the `~/odom:=odom` remap on
the spawner.

**`map_topic: /map` was wrong in `slam:=true` mode.** Under `slam:=true`
there is no `map_server`; each robot's own namespaced `slam_toolbox` is the
map authority and publishes `/<namespace>/map`. `global_costmap`'s static
layer, hardcoded to `/map`, would have waited forever for a topic nobody
publishes — so the global costmap would stay empty and the planner could
never produce a path. `navigation.launch.py` now rewrites `map_topic` per
mode (`"map"` when `slam`, `"/map"` otherwise) via `RewrittenYaml`'s
`param_rewrites`, which matches on leaf key name at any depth and so covers
both `amcl`'s and the static layer's copy. (A relative `map` resolves to
`/agv_1/map`, not `/agv_1/global_costmap/map`, because `nav2_costmap_2d`
joins relative topics with the *parent* namespace.) **Confirmed on a live
run** under `slam:=true`: `global_costmap` logs `Subscribing to the map
topic (map) with transient local durability`.

## Known cosmetic/inert issues, not fixed

**The `world -> map` static transform goes to the global `/tf_static`.**
`workcell_bringup/launch/workcell.launch.py`'s `static_transform_publisher`
has no namespace and no tf remap, so it publishes onto the global
`/tf_static` that no namespaced node listens to. Nothing consumes a `world`
frame today (each robot's tree is rooted at `map`), so this is inert rather
than broken — but it is also not doing what it looks like it's doing. It
also runs unconditionally, without `use_sim_time`, so its stamps are wall
time. Either drop it or publish one per robot namespace with the tf remap;
left as-is for now to keep the fix set focused.

## Runs 1-2: `bt_navigator` segfault and redundant SLAM

Run 1 (2 robots, `slam:=true`) surfaced two bugs, both fixed before run 2:

- **`bt_navigator` segfaulted on both robots** immediately on first
  configure: `Exception: ID [ComputePathToPose] already registered`. Cause:
  `nav2_params.yaml`'s `bt_navigator` section had an explicit 49-entry
  `plugin_lib_names` list, copied from an older nav2 reference config where
  it was required. Current `nav2_bt_navigator` (Jazzy) auto-registers its
  own core BT node set internally; the explicit list collided with that.
  **Fixed** by removing `plugin_lib_names` entirely. **Confirmed fixed on
  run 2** — both navigators (`navigate_to_pose`, `navigate_through_poses`)
  configured successfully, no crash.
- **Both robots ran their own `slam_toolbox` simultaneously** under
  `slam:=true`, contrary to `mapper_params_online_async.yaml`'s own
  documented single-robot-mapping assumption. **Fixed** in
  `workcell_bringup/launch/workcell.launch.py`: only the first robot gets a
  navigation stack while `slam:=true`. **Confirmed fixed on run 2** — only
  `agv_1` spun up a nav stack; `agv_2` stayed a plain Gazebo/`agv_bringup`
  robot with no Nav2 nodes at all.

**But run 2 still showed the exact same bond-timeout failure** —
`lifecycle_manager_slam: Server slam_toolbox was unable to be reached after
10.00s by bond` — with only *one* `slam_toolbox` instance running and the
raised `bond_timeout: 10.0` in effect. So the redundant-instance theory was
at best a partial explanation. Current best hypothesis, tying this directly
to the `odom`-frame bug above: `slam_toolbox`'s scan message filter was
continuously backlogged waiting for a transform to `odom` that could never
resolve (the "queue is full" spam), and if that backlog was monopolizing
`slam_toolbox`'s executor thread, the bond heartbeat callback (processed on
the same executor) would starve too — a downstream symptom of the same root
cause, not an independent scaling problem. **Re-test this specifically
after the `odom` fix** before concluding `bond_timeout: 10.0` needs to go
even higher, or that there's a genuine, separate resource-contention issue.

`nav2_bringup`, `nav2_amcl`, `slam_toolbox`, and friends are still not
installed in the environment this was debugged in (confirmed via
`ros2 pkg list` — only `nav2_common`/`nav2_costmap_2d`/`nav2_msgs`/
`nav2_util` are present); the *user's* devcontainer has them, which is how
these bugs actually surfaced. This workspace's own history (the AGV
mesh-loading saga in `01-robot-description.md`) is a good reminder that
"should work per the docs" and "actually works in this specific
environment" aren't the same thing.

## Wheel texture unverified

`AGVv3Weel.png` is wired into the wheel material via `<texture>`. Cylinder
primitives get renderer-provided UVs (unlike the body's STL mesh, which
structurally cannot be textured — see `01-robot-description.md`), so this
*should* work, but it has not been visually confirmed. If it just shows the
flat fallback color, that's expected-until-verified, not necessarily a bug.

## `vision` package points at a robot that no longer exists

`src/vision/config/nvblox_topics.yaml` still references
`robot_1/camera/depth`, `robot_1/camera/color`, `robot_1/pose` — leftover
from the deleted `group_a` arm's Orbbec camera. The AGV has no camera, so
`nvblox.launch.py` would currently run against topics nobody publishes.
Not touched as part of the AGV/Nav2 work since it's a separate concern, but
flagged here since it's a real, currently-broken piece of the workspace
that a casual `ros2 launch vision nvblox.launch.py` would not obviously
explain.

## `base_link` inertial origin was corrected without asking

For AGVv3, `base_link`'s inertial `<origin>` was changed from the source
export's `0 0 0` to `0 0 0.06` (matching the body's actual geometric
center — `0 0 0` sits entirely outside the body's own volume, which isn't
physically possible). This is very likely correct, but it's a judgment
call made without confirming against the actual Blender model's computed
center of mass — if a future export's body shape changes significantly
(off-center mass distribution, e.g. a battery mounted to one side), this
assumption should be revisited rather than blindly recentered again.

## Open-RMF integration not started

Everything in `agv_navigation` was built to plug into the architecture in
[`docs/open-rmf/`](../open-rmf/) — shared `map` frame, per-robot namespaced
Nav2 stack exposing the standard `NavigateToPose` action — but no RMF code
exists yet: no `rmf_fleet_adapter`-derived package, no nav graph, no
`rmf_traffic`/task-dispatch launch. That's cloning external repos and
writing a new Nav2↔RMF bridge package — a separate, substantial piece of
work, not a natural extension of the changes documented here.

## No map exists for `office_world` yet

`agv_navigation/maps/` is empty. `slam:=true` is the default specifically
because of this — see `03-navigation.md`'s SLAM/AMCL section for the
build-then-switch workflow. Until a map is saved, `slam:=false` will fail
(`map` arg required, `map_server` has nothing to load).

## Things confirmed working (for contrast)

Verified against live `ros2 launch workcell_bringup workcell.launch.py`
output, not just written and hoped:

- The AGV spawns and renders correctly (mesh, wheels, casters) and drives
  under `diff_drive_controller` without tipping or destabilising the
  physics solver.
- Both robots spawn, each with its own `controller_manager` inside the
  Gazebo process, and both controllers configure and activate.
- The lidar bridges to `/<ns>/scan` and stamps frame `lidar_link`.
- `odom -> base_link` publishes at ~50 Hz on `/agv_1/tf`, with bare frame
  names, forming one connected tree with `robot_state_publisher`'s links.
- `diff_drive_controller` accepts `TwistStamped` on `/<ns>/cmd_vel`.
- `global_costmap`'s static layer subscribes to the right map topic in
  `slam:=true` mode.
- `bt_navigator` configures both navigators without the plugin-registration
  segfault, and only one `slam_toolbox` runs.

## Next verification step

The remaining unknown is whether the stack now actually navigates. In
order:

1. `ros2 run tf2_tools view_frames --ros-args -r tf:=/agv_1/tf -r tf_static:=/agv_1/tf_static`
   — expect one tree, `map -> odom -> base_link -> {wheels, casters, lidar_link}`.
   The `map -> odom` link comes from `slam_toolbox`, so its absence points
   at SLAM, not at the odometry chain.
2. `ros2 topic hz /agv_1/scan` and check `slam_toolbox` has stopped logging
   `Message Filter dropping message` — with a connected tree it should now
   accept scans and start publishing `/agv_1/map`.
3. Send a goal and watch `/agv_1/cmd_vel` for `TwistStamped` traffic, which
   is what closes out the last unverified item above.

## Build hygiene — this workspace has bitten twice

`build/`, `install/` and `log/` are shared between the host
(`~/Desktop/projects/robotics/internal_delivery_system_ws`) and the
devcontainer (`/workspaces/isaac_ros-dev`), which bind-mount the same
directory at two different absolute paths. `colcon --symlink-install` bakes
the *building* environment's absolute path into `install/**` symlinks and
`build/*/CMakeCache.txt`, so building from one side silently breaks the
other:

- Build on the host, then launch in the container → every
  `install/*/share/*/...` symlink points at `/home/kaipis/...`, which does
  not exist there. Observed symptom: `Package 'workcell_bringup' not found`
  even though `install/workcell_bringup/` is plainly present.
- Build in the container, then build on the host → CMake aborts on the
  `CMakeCache.txt` path mismatch. Deleting `build/ install/ log/` to "fix"
  it destroys the other environment's real artifacts.

**Pick one side and build there consistently.** If both are genuinely
needed, give each its own tree via `colcon build --build-base build_host
--install-base install_host` (or the container equivalent) instead of
sharing one.
