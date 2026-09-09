# `agv_description`

The robot model: `urdf/agv_macro.xacro` (the actual link/joint/material
definitions, wrapped in a namespaced macro) + `urdf/agv.urdf.xacro` (the
top-level file that instantiates it, mirroring `group_a`'s old
`group_a_macro.xacro` / `group_a.urdf.xacro` split).

## Where the geometry comes from

Three successive Blender/Phobos exports live under `src/robots/blender_files/`:
`AGV` → `AGV_S` → `AGVv3` (current). Each export is a raw, unedited URDF dump
plus mesh/texture assets; `agv_macro.xacro` is a **hand-maintained
reintegration** of that data, not a straight copy — every export needed at
least one correction (see below), so treat a new export as "new data to
integrate," not "new file to swap in."

Current source: `src/robots/blender_files/AGVv3/urdf/AGVv3.urdf`, mesh
`AGVv3/meshes/stl/Cube.026.stl`, textures `BodyTexture.png` +
`AGVv3Weel.png` (both at `src/robots/blender_files/`, one level above the
per-version folders since they're not version-specific).

## Link/joint naming (stable across all three export versions)

| URDF-facing name | AGVv3 source name | Notes |
| --- | --- | --- |
| `base_link` | `MainBody_link` | root link, no parent joint (see below) |
| `wheel_left_link` / `wheel_right_link` | `LWeel_link` / `RWeel_link` | driven, continuous joints |
| `wheel_left_joint` / `wheel_right_joint` | `LWeel_link_joint` / `RWeel_link_joint` | in `ros2_control` |
| `caster_front_link` / `caster_back_link` | *(not in any export - added by hand)* | fixed, frictionless |

Names are **bare** — never prefixed with the robot's namespace (no
`agv_1/base_link`). Multi-robot disambiguation is entirely a runtime
concern, handled by each robot's nodes running in their own ROS namespace
and publishing to their own `/<namespace>/tf` topic (see
`02-bringup-and-simulation.md`'s TF strategy section) — not by baking a
prefix into link/joint names at the URDF level. `${namespace}` still exists
as a macro parameter, but it's used for exactly one thing now: telling
`gz_ros2_control`'s plugin which ROS namespace its own nodes should live in
(see `agv_macro.xacro`'s `<gazebo>` block) — never to build a frame or link
name. Names were kept stable across the AGV → AGV_S → AGVv3 transitions so
that `agv_bringup`/`agv_navigation` never needed to change when the mesh
did — only `agv_description`'s internals move.

## `base_link` is the URDF root — no `world` link

Earlier revision welded `base_link` to a `<link name="world"/>` via a
`fixed` joint (mirroring `group_a`'s pattern for its stationary arm). For a
*mobile* robot this is wrong on two counts: a `fixed` joint gives zero DOF,
so the chassis would be nailed in place regardless of wheel motion; and a
model link literally named `world` collides semantically with gz-sim's
actual global `world` frame, which is very likely why that model never
appeared in the Entity Tree even though it spawned successfully server-side.
Current design: `base_link` has no parent joint at all; the spawn pose is
given to the spawner (`ros_gz_sim create -x -y -z -R -P -Y`) in
`agv_bringup/launch/bringup.launch.py`, not baked into the model.

## Wheels

Radius `0.02011 m`, length `0.014 m` (visual) — collision uses slightly
different, source-provided per-wheel values (`0.02023`/`0.01349` and
`0.02025`/`0.01351`). Joint origins: `y=±0.14, z=0.02` (wheel separation
`0.28 m`). Each joint's `rpy` (`±90°` about X) rotates the cylinder's local
Z spin axis into world Y — verified analytically, not assumed.

**Two-wheel tipping fix**: a 2-wheel diff-drive robot has no fore/aft
support and pitches/bounces on its axle in simulation. Two extra contact
points were added — `caster_front_link` / `caster_back_link`, spheres at
`x=±0.15, y=0, z=0.02` (same ground-contact height as the drive wheels, so
the chassis sits level), `fixed` joints (not actuated, not in
`ros2_control`), friction zeroed via `<gazebo reference="..."><collision><surface><friction><ode><mu>0.0</mu><mu2>0.0</mu2></ode></friction></surface></collision></gazebo>`
so they only provide vertical support and never resist rolling or turning.

## Lidar

`lidar_link`, mounted centered on `base_link` via a `fixed` joint at
`x=0, y=0, z=0.13` (body top is at `z=0.11`, so `0.02 m` standoff). A
`gpu_lidar` sensor (gz-sim's standard 2D/3D lidar type — despite the name,
it runs on the render pipeline, not a requirement for real GPU hardware; see
`04-known-issues-and-next-steps.md` for the "is this fast enough under
software rendering" caveat):

```xml
<gazebo reference="lidar_link">
  <disableFixedJointLumping>true</disableFixedJointLumping>
  <sensor name="lidar" type="gpu_lidar">
    <topic>/model/${namespace}/scan</topic>
    <lidar>
      <scan><horizontal><samples>360</samples>
        <min_angle>-3.14159265</min_angle><max_angle>3.14159265</max_angle>
      </horizontal></scan>
      <range><min>0.12</min><max>10.0</max><resolution>0.01</resolution></range>
    </lidar>
  </sensor>
</gazebo>
```

**`<disableFixedJointLumping>` is required, not decorative.** Without it,
URDF→SDF conversion merges `lidar_link` (attached via a `fixed` joint) into
`base_link` as a standard optimization, and the sensor's auto-generated
frame reflects *that* merge instead of `lidar_link`. This wasn't caught
until a live run: the bridge worked correctly, but `slam_toolbox` logged,
forever, `Message Filter dropping message: frame '.../base_link/lidar' ...
queue is full` — that frame matched nothing in the actually-published TF
tree, so every scan was silently unusable. Same fix applied to the casters
(`caster_front_link`/`caster_back_link`) for consistency, though nothing
currently depends on their own tf frame.

`<topic>` is set to an **explicit absolute path**
(`/model/<namespace>/scan`) rather than left to gz-sim's default
sensor-topic naming convention — same reasoning as the pose bridge
entries: an absolute path is unambiguous, whereas trusting a specific
version's default topic-scoping behavior is exactly the category of thing
that silently broke the mesh loading (see below) earlier in this project.
Bridged to ROS as `/<namespace>/scan` in `agv_bringup/config/gz_bridge.yaml`:

```yaml
- ros_topic_name: "/$(var namespace)/scan"
  gz_topic_name: "/model/$(var namespace)/scan"
  ros_type_name: "sensor_msgs/msg/LaserScan"
  gz_type_name: "gz.msgs.LaserScan"
  direction: GZ_TO_ROS
```

`agv_navigation/config/nav2_params.yaml` and `mapper_params_online_async.yaml`
needed **zero changes** for this — their `scan_topic`/`observation_sources`
were already set to the relative `scan`, anticipating a sensor that didn't
exist yet. A relative `scan`, inside a node running namespaced as `agv_1`,
resolves to `/agv_1/scan` via ROS 2's ordinary topic namespacing — this is
an ordinary sensor topic, not `tf`, so unlike the tf remap
(`02-bringup-and-simulation.md`) no special remapping was needed here.

360 samples over a full 360° FOV, `0.12–10.0 m` range, `10 Hz` — reasonable
defaults for a small-footprint 2D lidar (RPLidar-class), not measured
against any real hardware spec. `lidar_link`'s own mass/inertia
(`0.05 kg`, solid-cylinder tensor for `r=0.03, L=0.04`) are real, non-zero
values — never leave a link's inertia at zero (see the "Known-wrong data"
section below for why that specifically crashes Gazebo's physics solver).

## Body mesh

`base_link`'s visual is `Cube.026.stl` (28 triangles, verified clean —
0 degenerate triangles, 0 zero-normal facets), referenced as:

```xml
<mesh filename="$(find agv_description)/meshes/stl/Cube.026.stl" scale="1 1 1" />
```

**Why `$(find ...)` and not `package://` or `model://`**: this took three
attempts to get right. `package://agv_description/...` (the "correct"/
idiomatic URDF way) silently failed — the model's entire scene registration
in gz-sim's Entity Tree was dropped, with no error anywhere in the logs,
even though the physics/ros2_control side worked fine (confirmed via
`/agv_1/pose` publishing valid data while a *fresh* `gz sim -g` client still
didn't show the model). Switching to `model://agv_description/...` +
wiring `GZ_SIM_RESOURCE_PATH` to include the package's install share dir
(matching how `workcell_description`'s furniture models resolve) had the
*identical* failure. Only `$(find agv_description)/...` — a **xacro-time**
substitution that resolves to a plain absolute filesystem path before
gz-sim ever sees the URDF, bypassing gz-sim's own runtime URI resolver
entirely — worked. Root cause of the URI-scheme failures was never
identified; ruling out mesh-content problems came first (see below), and
once those were fixed, switching to `$(find ...)` was the pragmatic
decisive fix rather than continuing to debug the URI resolver blind.

**Degenerate triangles**: the AGV_S-era mesh (`Cube.stl`, since replaced)
had 8 of 36 triangles with zero area and zero-length normals — a Blender
export artifact. This is a very plausible reason a mesh visual could break
a renderer's scene-graph construction (zero-length normals commonly cause
divide-by-zero in tangent-space/bounding-volume computation). The AGVv3
mesh (`Cube.026.stl`) was verified clean from the start — 0 degenerate
triangles.

## Textures

`meshes/textures/BodyTexture.png` and `AGVv3Weel.png`, wired into materials
via the same `$(find agv_description)/...` pattern as the mesh:

```xml
<material name="Body">
  <texture filename="$(find agv_description)/meshes/textures/BodyTexture.png" />
  <color rgba="0.80000 0.80000 0.80000 1.00000" />
</material>
```

**Hard limitation, not a bug**: `BodyTexture.png` cannot actually render on
`base_link`'s mesh — **STL carries no UV-coordinate data**, so there is no
way to map a 2D image onto it in *any* tool, gz-sim or otherwise. It's
wired in anyway (harmless no-op, falls back to the flat `<color>`) in case
the mesh format changes later (e.g. to glTF/DAE with real UVs).
`AGVv3Weel.png` has a real chance of working since the wheels are
`<cylinder>` primitives, which do get renderer-provided UVs — **this has
not been visually confirmed**; it may just show the flat color.

## Known-wrong data in the source exports, corrected by hand

Every export needed at least one manual fix — worth knowing before trusting
raw values from a *new* export without re-checking them:

- **AGV_S wheel visual double-rotation** (fixed by rewriting AGVv3 from
  scratch, not directly relevant to current AGVv3 code, but a pattern to
  watch for): the wheel visual's `<origin rpy>` was the exact negation of
  the wheel joint's own `<origin rpy>` — the two canceled to identity,
  leaving the cylinder aligned to world Z (standing upright) instead of
  world Y (lying on its side, rolling). AGVv3's export doesn't have this bug
  (its visual/collision origins are already near-identity).
- **`base_link` inertial origin** (AGVv3, still present): source gives
  `0 0 0`, but the body spans Z `0.01..0.11` (mesh-verified) — an origin at
  `0 0 0` sits entirely outside the body's own volume, not a physically
  possible center of mass. Corrected to `0 0 0.06`, matching the body's
  actual geometric center (which is also where the collision box's origin
  already correctly sat — only inertial had the stale value).
- **Wheel inertia tensor axis** (AGVv3): source declares the "along-axis"
  (larger) moment on `iyy`, but the wheels' visual/collision origins are
  identity, meaning the cylinder's axis is the link's own local Z (URDF's
  `<cylinder>` convention) — the joint's rotation alone aligns that to
  world Y. The "along-axis" value belongs on `izz`, not `iyy`. Recomputed
  directly from `r=0.02011, L=0.014, m=0.5`
  (`Izz=0.5·m·r²`, `Ixx=Iyy=m·(3r²+L²)/12`).
- **Original AGV wheel inertia** (superseded, historical): the very first
  export had an all-zero inertia tensor for both wheels — a singular/
  invalid matrix that made Gazebo's physics solver produce NaNs and reset/
  crash the model. This is the kind of thing worth checking first if a
  *future* export crashes gz-sim outright (as opposed to just rendering
  wrong): zero or missing inertia values are a common raw-export defect.

## Package layout

```
agv_description/
  urdf/
    agv_macro.xacro    the actual robot: links, joints, materials, ros2_control, gazebo plugins
    agv.urdf.xacro      top-level: declares args, includes the macro, instantiates it
  meshes/
    stl/Cube.026.stl
    textures/BodyTexture.png, AGVv3Weel.png
  package.xml, CMakeLists.txt
```

`CMakeLists.txt` installs `urdf/` and `meshes/` recursively — a new mesh or
texture dropped under `meshes/` needs no build-file change, just a rebuild.
