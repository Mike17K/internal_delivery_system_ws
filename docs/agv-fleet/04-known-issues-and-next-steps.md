# Known Issues & Next Steps

Honest record of what's incomplete, unverified, or assumed — read this
before trusting the rest of this folder as "done and working." Nothing
past the robot spawning and moving under `diff_drive_controller` has been
confirmed running end-to-end; the debugging in `01`/`02` was done live
against user-reported Gazebo output, but Nav2 itself has never been
launched.

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

## No sensor on the AGV — Nav2's obstacle data has no source

`agv_description`'s `agv_macro.xacro` has **no lidar, camera, or any other
sensor**. But `agv_navigation/config/nav2_params.yaml` configures
`local_costmap`/`global_costmap`'s `obstacle_layer` with
`observation_sources: scan` (`data_type: LaserScan`), and AMCL uses
`laser_model_type: likelihood_field` against a `scan_topic: scan` that also
doesn't exist. **Practical effect**: right now, AMCL would only ever
correct against odometry (no real sensor update), and the costmaps' dynamic
obstacle layer would just never receive data — only the static map layer
(in `slam:=false` mode) would have any content. SLAM Toolbox in
`slam:=true` mode is in the same position: no scan topic to actually build
a map from.

**This needs a sensor added to `agv_description` before Nav2 can do
anything meaningful** — a 2D lidar via a `<gazebo>` sensor block (matching
the `<gazebo>`-plugin pattern already used for `gz_ros2_control` and
`PosePublisher` in `agv_macro.xacro`) is the natural next step, bridged
through `agv_bringup/config/gz_bridge.yaml` the same way pose/tf already
are.

## Nav2 has never actually been run

`nav2_bringup`, `nav2_amcl`, `slam_toolbox`, and friends are not installed
in the environment this was built in (confirmed via `ros2 pkg list` —
only `nav2_common`/`nav2_costmap_2d`/`nav2_msgs`/`nav2_util` are present).
The params schema, plugin names, and launch wiring in `agv_navigation` are
written from Nav2 Jazzy's known reference structure and one verified read
of `RewrittenYaml`'s actual source, but the package has never been built or
launched. Expect a debugging pass — this workspace's own history (the AGV
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

To be clear about what *has* actually been debugged against live Gazebo
output, not just written and hoped: the AGV spawns, renders correctly
(mesh, wheels, casters), and drives under `diff_drive_controller` without
tipping or crashing the physics solver — all confirmed via iterative
back-and-forth against real `ros2 launch workcell_bringup workcell.launch.py`
output. That's the solid foundation the Nav2 layer sits on top of.
