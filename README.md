<img src="docs/shared_nvblox.png">

# Internal Delivery System Workspace

ROS 2 (Jazzy) workspace for a namespaced fleet of differential-drive AGVs
delivering inside a Gazebo office world, navigating with Nav2 + SLAM Toolbox,
alongside a shared [nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html)
reconstruction.

All development happens inside a container built via the
[Isaac ROS CLI](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html) —
there is no host ROS install. Design rationale (why nvblox runs in static TSDF
mode, why the workspace lives inside the container) is in
[docs/STRUCTURAL_DESISIONS.md](docs/STRUCTURAL_DESISIONS.md).

The AGV fleet setup (robot description, bringup, Nav2 — what's done, what's
not, why some decisions were made) is documented in
[docs/agv-fleet/](docs/agv-fleet/README.md). The target fleet-orchestration
architecture on top of it (Open-RMF) is sketched in
[docs/open-rmf/](docs/open-rmf/01-openrmf-nav2-integration.md).

## Quickstart

```bash
git clone https://github.com/Mike17K/ros2_cuda_robotic_ws.git
bash scripts/setup_host.sh
bash scripts/build_docker_image.sh
```

The build takes ~1.5h and leaves you in a shell inside the container as
`admin`. If not, run:

```bash
bash scripts/shell.sh
```

In the container:

```bash
make
source install/setup.bash
```

Then, from a new terminal, open the Terminator layout with the commands ready
to run (leave it alone until it finishes setting up):

```bash
bash scripts/launch/launch_ws.sh
```

## Layout

| Path                     | What                                                                                                                    |
| ------------------------ | ----------------------------------------------------------------------------------------------------------------------- |
| `src/workcell`           | Gazebo world (`office_world.sdf`) + shared workcell description                                                         |
| `src/robots/agv`         | Description + bringup + Nav2 for the AGV differential-drive base (see [docs/agv-fleet/](docs/agv-fleet/README.md))       |
| `src/vision`             | nvblox launch/config                                                                                                    |
| `Dockerfile.cumotion_ws` | Layer added on top of the Isaac ROS base image                                                                          |
| `scripts/`               | Entry points, see below                                                                                                 |

## Prerequisites (host)

- [NVIDIA Container Toolkit](https://nvidia-isaac-ros.github.io/getting_started/index.html) + `isaac-ros-cli` — see `scripts/setup_host.sh`
- `docker login nvcr.io` with an [NGC API key](https://org.ngc.nvidia.com/account/api-keys) (username: `$oauthtoken`)
- `isaac_ros_common` pinned to the `3.2-15` release

## Entry points (`scripts/`)

| Script                  | Purpose                                                             |
| ----------------------- | ------------------------------------------------------------------- |
| `build_docker_image.sh` | Builds/activates the container (`isaac-ros activate --build-local`) |
| `shell.sh`              | Opens a shell in the running container                              |
| `entrypoint.sh`         | Container entrypoint, runs `make`                                   |
| `setup_workspace.sh`    | First-boot dependency install inside the container                  |
| `setup_host.sh`         | One-off host setup (NVIDIA container toolkit + isaac-ros-cli)        |
| `launch/launch_ws.sh`   | Opens a Terminator layout with workcell / nvblox panels             |

## Build & run (inside the container)

```bash
make                      # colcon build
make rosdeps              # install rosdep dependencies
make builds n=<package>   # build a single package
make clean                # wipe build/ install/
```

Then, e.g.:

```bash
ros2 launch workcell_bringup workcell.launch.py sim_gazebo:=true use_fake_hardware:=false
ros2 launch vision nvblox.launch.py
```

## Mapping the world (SLAM)

`office_world` has no map, so `workcell.launch.py` defaults to `slam:=true`.
`slam_toolbox` maps automatically from `/agv_1/scan` + odom — no start command.
Details: [docs/agv-fleet/03-navigation.md](docs/agv-fleet/03-navigation.md).

**1. Drive around** (loop back to your start — that triggers loop closure):

```bash
ros2 launch agv_navigation teleop.launch.py namespace:=agv_1
```

Small Tk window: arrows/WASD (hold), `qezc` diagonals, speed sliders, space =
E-stop. Publishes `TwistStamped` at 20 Hz while a key is held and zeros on
release. Don't use `teleop_twist_keyboard` — it sends one message per keypress,
so `cmd_vel_timeout` stops the robot between keys. Scripted alternative:

```bash
ros2 topic pub -r 20 /agv_1/cmd_vel_nav geometry_msgs/msg/TwistStamped \
  "{header: {frame_id: base_link}, twist: {linear: {x: 0.2}, angular: {z: 0.0}}}"
```

**2. Watch the map build, live, in RViz.** The remap is mandatory — this fleet
puts tf on per-robot topics (`/agv_1/tf`), so a plain `rviz2` finds no
transforms at all and every display errors out:

```bash
ros2 run rviz2 rviz2 --ros-args -r /tf:=/agv_1/tf -r /tf_static:=/agv_1/tf_static
```

Set **Fixed Frame** to `map`, then `Add` → `By topic`:

| Display       | Topic                                      | Note                                                          |
| ------------- | ------------------------------------------ | ------------------------------------------------------------- |
| `Map`         | `/agv_1/map`                               | if it only shows on `/map`, your session predates the remap in `navigation.launch.py`      |
| `LaserScan`   | `/agv_1/scan`                              | the live scan being matched in                                |
| `RobotModel`  | `/agv_1/robot_description`                 | set *Description Topic*, not the default parameter source      |
| `MarkerArray` | `/agv_1/slam_toolbox/graph_visualization`  | the pose graph — nodes snap into place when a loop closes      |

The map grows every `map_update_interval` (5.0 s), not per scan, so give it a
few seconds after each stretch of driving. `File → Save Config As` to avoid
redoing this.

**3. Save** — both, into `src/` (symlink-install, stays in git):

```bash
# .pgm + .yaml — what map_server/AMCL load
ros2 service call /agv_1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'src/robots/agv/agv_navigation/maps/office_world'}}"
# .posegraph + .data — the only way to resume mapping later
ros2 service call /agv_1/slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph \
  "{filename: 'src/robots/agv/agv_navigation/maps/office_world'}"
```

Paths are resolved by `slam_toolbox`, which runs **inside the container** with
its cwd at `/workspaces/isaac_ros-dev` — so relative paths like the above land
in the repo, and a host path (`/home/<you>/...`) silently fails. `result: 0` is
success; `255` means it could not write, `1` means it never got a map.

If `save_map` returns `255`, the map topic is the likely cause, not the path
(see the `/map` remap comment in `navigation.launch.py`). This always works and
saves from wherever you run it:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f src/robots/agv/agv_navigation/maps/office_world \
  --ros-args -r map:=/agv_1/map
```

**4. Switch over** (also when `agv_2` gets its Nav2 stack):

```bash
ros2 launch workcell_bringup workcell.launch.py \
  slam:=false map:=src/robots/agv/agv_navigation/maps/office_world.yaml
```

Gotchas: check `/agv_1/odom` against the Gazebo GUI first — bad odom fails
silently and ruins the map. Watch `position`, not `velocity`, in
`/agv_1/joint_states`.

## References

- [Isaac ROS dev environment](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html)
- [Isaac ROS getting started](https://nvidia-isaac-ros.github.io/getting_started/index.html)
- [Isaac ROS nvblox quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart)
- [nvblox](https://nvidia-isaac.github.io/nvblox/v0.0.10/index.html)
- [Nav2](https://docs.nav2.org/)
- [SLAM Toolbox](https://github.com/SteveMacenski/slam_toolbox)
- [direnv](https://direnv.net/)
