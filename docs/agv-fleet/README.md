# AGV Fleet — Current Setup

Documents what actually exists in this workspace today: a differential-drive
AGV (description + bringup + Nav2), spawned in an office world, scalable to
a fleet of N robots. This supersedes the old `group_a` (lift-mounted UR arm)
/ `planning_bringup` (cuMotion) setup, which was legacy and has been removed
— see the note in the main [README.md](../../README.md).

This folder is a factual record of the current state, not a plan. For where
this is headed next (Open-RMF fleet orchestration on top of this Nav2 layer),
see [docs/open-rmf/](../open-rmf/) — that's the target architecture; this
folder is the Nav2 layer it will eventually plug into.

## Contents

| File | Covers |
| --- | --- |
| [01-robot-description.md](01-robot-description.md) | `agv_description` — URDF/xacro, mesh/texture history, the bugs found and fixed along the way |
| [02-bringup-and-simulation.md](02-bringup-and-simulation.md) | `agv_bringup` + `workcell_bringup` — spawn flow, ros2_control, TF/bridge wiring |
| [03-navigation.md](03-navigation.md) | `agv_navigation` — Nav2 architecture, fleet TF strategy, SLAM/AMCL toggle |
| [04-known-issues-and-next-steps.md](04-known-issues-and-next-steps.md) | What's not done, what's unverified, what to check first when something breaks |

## Package map

```
src/
  robots/
    agv/
      agv_description/     URDF/xacro, meshes, textures - the robot model
      agv_bringup/          robot_state_publisher, ros2_control, Gazebo spawn, gz bridge (pose/tf/scan)
      agv_navigation/        Nav2 params + launch (SLAM/AMCL, per-robot nav stack)
    blender_files/
      AGV/ AGV_S/ AGVv3/     successive Blender/Phobos exports - AGVv3 is current
  workcell/
    workcell_description/    office_world.sdf + furniture models
    workcell_bringup/         top-level launch: Gazebo, fleet loop, shared map_server
  vision/                     nvblox (currently has no camera input to consume - see 04)
```

## Quickstart

```bash
# Build a map first (no map exists for office_world yet)
ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false slam:=true
# ... drive agv_1 around, then save the map (see 03-navigation.md) ...

# Normal fleet operation against the saved map
ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false \
  slam:=false map:=/path/to/office_world.yaml
```

## High-level architecture

```
                    workcell_bringup/workcell.launch.py
                                   │
        ┌──────────────────────────┼──────────────────────────┐
        │                          │                           │
   Gazebo (office_world.sdf)   map_server (shared,        per-robot loop
   + clock bridge              slam:=false only)          (robots_config)
        │                          │                           │
        │                          │              ┌────────────┴────────────┐
        │                          │              │                         │
        │                          │        agv_bringup             agv_navigation
        │                          │     (spawn, ros2_control,    (SLAM xor AMCL +
        │                          │      diff_drive_controller,   controller/planner/
        │                          │      gz_bridge: pose+tf)      bt_navigator/costmaps,
        │                          │                                 namespaced)
        └──────────────────────────┴───────────────────────────────────────┘
                    each robot's own /<namespace>/tf topic
              (frame ids bare: base_link, odom, ... - disambiguated by
               topic namespace, not by prefixing the frame name text)
```

Two robots (`agv_1`, `agv_2`) are configured in `workcell_bringup/launch/workcell.launch.py`'s
`robots_config` list today; adding a third is a one-line addition to that list — everything
downstream (bringup, controllers, Nav2) is already namespaced per-robot with no hardcoded count.
