<img src="docs/shared_nvblox.png">

# Diffusion Robot Test Workspace

ROS 2 (Jazzy) workspace for multi-arm manipulation on lift-mounted UR arms ("group_a": Ewellix lift + UR arm + Orbbec camera), planned with [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html) against a shared [nvblox](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html) reconstruction, simulated in Gazebo. Longer-term goal is a diffusion-policy planner — see [docs/DIFFUSION_MODEL_IDEA.md](docs/DIFFUSION_MODEL_IDEA.md).

All development happens inside a container built via the [Isaac ROS CLI](https://nvidia-isaac-ros.github.io/concepts/dev_env/index.html) — there is no host ROS install. Design rationale (why nvblox runs in static TSDF mode, why the workspace lives inside the container) is in [docs/STRUCTURAL_DESISIONS.md](docs/STRUCTURAL_DESISIONS.md).

The current AGV fleet setup (robot description, bringup, Nav2 — what's done, what's not, why some decisions were made) is documented in [docs/agv-fleet/](docs/agv-fleet/README.md). The target fleet-orchestration architecture on top of it (Open-RMF) is sketched in [docs/open-rmf/](docs/open-rmf/01-openrmf-nav2-integration.md).

## Quickstart

```bash
git clone --recurse-submodules https://github.com/Mike17K/ros2_cuda_robotic_ws.git
bash scripts/setup_host.sh
bash scripts/build_docker_image.sh
```

after the build that takes about 1.5h in my pc the layered docker image will be created, and already you should be in a shell inside the container with the name admin if not just run

```bash
bash scripts/shell.sh
```

in the container run

```bash
make
source install/setup.bash
```

after you can open a new terminal and just run the bellow and keep away from pressing any buttons for a while until the setup is finished

```bash
bash scripts/launch/launch_ws.sh
```

this will open the terminator with the commands ready to run

## Layout

| Path                          | What                                                                                   |
| ----------------------------- | -------------------------------------------------------------------------------------- |
| `src/workcell`                | Gazebo world (office_world.sdf) + shared workcell description                          |
| `src/robots/agv`              | Robot description + bringup + Nav2 for the AGV differential-drive base (see [docs/agv-fleet/](docs/agv-fleet/README.md)) |
| `src/vision`                  | nvblox launch/config                                                                   |
| `src/isaac_ros_cumotion_fork` | Submodule, [Mike17K/isaac_ros_cumotion](https://github.com/Mike17K/isaac_ros_cumotion) |
| `Dockerfile.cumotion_ws`      | Layer added on top of the Isaac ROS base image                                         |
| `scripts/`                    | Entry points, see below                                                                |

> `src/robots/group_a` (lift-mounted UR arm) and `src/planning_bringup` (cuMotion arm planning) were legacy and have been removed.

## Prerequisites (host)

- [NVIDIA Container Toolkit](https://nvidia-isaac-ros.github.io/getting_started/index.html) + `isaac-ros-cli` — see `scripts/setup_host.sh`
- `docker login nvcr.io` with an [NGC API key](https://org.ngc.nvidia.com/account/api-keys) (username: `$oauthtoken`)
- `isaac_ros_common` pinned to the `3.2-15` release

## Entry points (`scripts/`)

| Script                  | Purpose                                                                           |
| ----------------------- | --------------------------------------------------------------------------------- |
| `build_docker_image.sh` | Builds/activates the container (`isaac-ros activate --build-local`)               |
| `shell.sh`              | Opens a shell in the running container                                            |
| `entrypoint.sh`         | Container entrypoint, runs `make`                                                 |
| `setup_workspace.sh`    | First-boot dependency install inside the container                                |
| `setup_host.sh`         | One-off host setup (NVIDIA container toolkit + isaac-ros-cli)                     |
| `launch/launch_ws.sh`   | Opens a Terminator layout and launches workcell / cuMotion / RViz / nvblox panels |

## Build & run (inside the container)

```bash
make            # colcon build
make rosdeps    # install rosdep dependencies
make builds n=<package>   # build a single package
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

**2. Save** — both, into `src/` (symlink-install, stays in git):

```bash
# .pgm + .yaml — what map_server/AMCL load
ros2 service call /agv_1/slam_toolbox/save_map slam_toolbox/srv/SaveMap \
  "{name: {data: 'src/robots/agv/agv_navigation/maps/office_world'}}"
# .posegraph — the only way to resume mapping later
ros2 service call /agv_1/slam_toolbox/serialize_map slam_toolbox/srv/SerializeMap \
  "{filename: 'src/robots/agv/agv_navigation/maps/office_world'}"
```

**3. Switch over** (also when `agv_2` gets its Nav2 stack):

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
- [Isaac ROS cuMotion](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_cumotion/isaac_ros_cumotion/index.html)
- [Isaac ROS nvblox quickstart](https://nvidia-isaac-ros.github.io/repositories_and_packages/isaac_ros_nvblox/isaac_ros_nvblox/index.html#quickstart)
- [nvblox](https://nvidia-isaac.github.io/nvblox/v0.0.10/index.html)
- [curobo](https://nvlabs.github.io/curobo/latest/getting-started/installation.html)
- [Isaac Sim quick install](https://docs.isaacsim.omniverse.nvidia.com/latest/installation/quick-install.html#isaac-sim-quick-install)
- [Isaac Sim robot config generator (Lula)](https://docs.isaacsim.omniverse.nvidia.com/6.0.1/robot_setup_tutorials/tutorial_generate_robot_config.html)
- [direnv](https://direnv.net/)
