# Open-RMF: Installation, Operations Guide & Review

## Table of Contents

1. [Overview](#overview)
2. [Installation](#installation)
3. [System Architecture](#system-architecture)
4. [Configuration Files](#configuration-files)
5. [Operations Guide](#operations-guide)
6. [Integrating a Real Fleet](#integrating-a-real-fleet)
7. [Troubleshooting](#troubleshooting)
8. [Review & Assessment](#review--assessment)

---

## Overview

Open-RMF (Robotics Middleware Framework) is an open-source platform for coordinating multiple robot fleets — potentially from different vendors — operating in a shared space. It handles traffic negotiation, task dispatch, and integration with building infrastructure (doors, lifts, etc.) on top of ROS 2.

Supported ROS 2 distributions: **Humble**, **Jazzy**, **Kilted**, **Rolling** (Ubuntu primarily, with limited RHEL/Fedora RPM support, amd64/aarch64).

---

## Installation

### 1. Prerequisites

Install ROS 2 for your chosen distro (binary debs recommended), then set up common tooling:

```bash
sudo apt update && sudo apt install ros-dev-tools -y

# rosdep
sudo rosdep init   # only if first time using rosdep
rosdep update

# colcon mixin (for optimized release builds)
colcon mixin add default https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml
colcon mixin update default
```

### 2. Binary Installation (recommended for first-time use)

```bash
sudo apt update && sudo apt install ros-<distro>-rmf-dev
```

Replace `<distro>` with `humble`, `jazzy`, `kilted`, or `rolling`. This installs the core RMF packages but **not** `rmf_demos` (not released as a binary), so build demos from source into a workspace:

```bash
mkdir -p ~/rmf_ws/src
cd ~/rmf_ws/src

# Find the matching rmf_demos tag for your installed binaries in the
# rmf.repos file on the <distro>-release branch of open-rmf/rmf
git clone https://github.com/open-rmf/rmf_demos.git -b <matching-tag>

cd ~/rmf_ws
colcon build
source install/setup.bash
```

### 3. Building from Source (for development / contributing)

```bash
# Remove existing binaries for this distro first to avoid conflicts
sudo apt purge ros-<distro>-rmf* && sudo apt autoremove

mkdir -p ~/rmf_ws/src
cd ~/rmf_ws
wget https://raw.githubusercontent.com/open-rmf/rmf/main/rmf.repos
vcs import src < rmf.repos

sudo apt update
rosdep update
source /opt/ros/<distro>/setup.bash
rosdep install --from-paths src --ignore-src --rosdistro $ROS_DISTRO -y

# Recommended toolchain
sudo apt install clang clang-tools lldb lld libstdc++-12-dev

export CXX=clang++
export CC=clang
colcon build --mixin release lld
source install/setup.bash
```

Use the `main` branch of `rmf.repos` for the latest development version, or `<distro>-release` to match the latest binaries for that distro.

### 4. Verify Installation

```bash
ros2 pkg list | grep rmf
```

Expect to see packages such as `rmf_traffic`, `rmf_fleet_adapter`, `rmf_task`, `rmf_fleet_msgs`.

### 5. Docker Option

Nightly containerized builds are available for testing/underlay use:

```bash
docker pull ghcr.io/open-rmf/rmf/rmf_demos:<distro>-rmf-latest
docker run -it --network host ghcr.io/open-rmf/rmf/rmf_demos:<distro>-rmf-latest \
  bash -c "export ROS_DOMAIN_ID=9; ros2 launch rmf_demos_gz office.launch.xml headless:=1"
```

> Nightly images track `main`/`rolling` and can break; pin a specific tag/hash for anything resembling production use.

---

## System Architecture

| Component                        | Role                                                                                  |
| -------------------------------- | ------------------------------------------------------------------------------------- |
| `rmf_traffic_schedule`           | Central node tracking every robot's itinerary, resolving conflicts before they happen |
| Fleet adapter (per fleet)        | Bridges RMF's task/traffic system to a fleet's native API or nav stack                |
| `rmf_building_map_server`        | Serves the semantic map: lanes, waypoints, doors, lifts                               |
| Task dispatcher                  | Accepts task requests (loop, delivery, clean, custom) and assigns robots              |
| rmf-web (API server + dashboard) | REST/WebSocket API and browser UI for monitoring and dispatch                         |

---

## Configuration Files

Open-RMF deployments are driven by several distinct YAML files. Knowing which one controls what saves a lot of debugging time.

### 1. `building.yaml` — the site map

Created visually with [`rmf_traffic_editor`](https://github.com/open-rmf/rmf_traffic_editor): floors, walls, lanes, waypoints, doors, lifts. This is the source-of-truth map and is not consumed directly by the fleet adapter — it's compiled into a nav graph first:

```bash
ros2 run rmf_building_map_tools building_map_generator nav \
  <path/to/building.yaml> <output_nav_graphs_dir>
```

### 2. `nav_graph.yaml` (often `0.yaml`, `1.yaml`, etc. per floor)

The generated graph of waypoints and lanes that the fleet adapter and traffic schedule actually reason about. Passed to a fleet adapter with `-n`:

```bash
ros2 run fleet_adapter_template fleet_adapter -c config.yaml -n 0.yaml
```

> Common gotcha: `parse_graph` doesn't do shell tilde-expansion — use an absolute path (`/home/you/...`), not `~/...`, on the command line.

### 3. `config.yaml` — the fleet adapter's main config

This is the one you'll edit most. It has three core sections:

```yaml
rmf_fleet:
  name: "my_fleet"
  limits:
    linear: [0.5, 0.75] # [nominal, max] velocity
    angular: [0.6, 1.0]
  profile: # footprint for collision checking
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

fleet_manager:
  prefix: "http://127.0.0.1:22011" # or a websocket URI
  user: "some_user"
  password: "some_password"

robots:
  robot_1:
    charger: "charger_waypoint_name"

reference_coordinates:
  rmf: [[0.95, -3.25], [2.25, -3.25], [2.25, -1.95]] # points from traffic-editor
  robot: [[0.0, 0.0], [1.24, -0.12], [1.57, 0.95]] # same points in the robot's own coordinate frame
```

- **`rmf_fleet`** — physical/behavioral parameters of the robots (speed limits, footprint, battery model).
- **`fleet_manager`** — how to reach the fleet's own API/manager (REST prefix, websocket URI, credentials).
- **`reference_coordinates`** — pairs of matching points in the RMF map frame vs. the robot's native frame, used to compute the coordinate transform between the two. Get these by reading the same physical points off the traffic-editor map and off the robot's own odometry/map — mismatched or too-few point pairs is the most common cause of robots navigating to the wrong location.

### 4. `dock_summary.yaml` (optional)

Maps named docking waypoints to a specific sequence of docking motions, for fleets that need custom docking behavior (e.g. backing into a charger). Passed with `-d`:

```bash
ros2 run fleet_adapter_r3 fleet_adapter_r3 -c config.yaml -n nav_graph.yaml -d dock_summary.yaml
```

### 5. Task-related config (demos)

`rmf_demos` ships per-world task/port configuration under `rmf_demos/rmf_demos/config/` — this is where each demo fleet manager's REST port and available task types are defined, useful as a reference when wiring up your own fleet manager.

### 6. `rmf.repos`

Not a runtime config — this is the manifest used at build time (`vcs import src < rmf.repos`) to pull the correct set of package versions for a given ROS 2 distro/release, as covered in [Installation](#installation).

### Quick reference: which file, which purpose

| File                | Produced by              | Consumed by                     | Purpose                                                      |
| ------------------- | ------------------------ | ------------------------------- | ------------------------------------------------------------ |
| `building.yaml`     | Traffic Editor (manual)  | `building_map_generator`        | Source map: floors, lanes, doors, lifts                      |
| `nav_graph.yaml`    | `building_map_generator` | Fleet adapter, traffic schedule | Routable graph robots plan over                              |
| `config.yaml`       | You (manual)             | Fleet adapter                   | Robot params, fleet manager connection, coordinate transform |
| `dock_summary.yaml` | You (manual, optional)   | Fleet adapter                   | Custom docking sequences                                     |
| `rmf.repos`         | Open-RMF maintainers     | `vcs import`                    | Build-time package manifest                                  |

---

## Operations Guide

### Starting a Simulation Deployment

```bash
source ~/rmf_ws/install/setup.bash
ros2 launch rmf_demos_gz office.launch.xml   # or hotel.launch.xml, airport_terminal.launch.xml
```

### Submitting Tasks

Via CLI:

```bash
ros2 run rmf_demos_tasks dispatch_patrol -p pantry -n 1 --use_sim_time
ros2 run rmf_demos_tasks dispatch_delivery -p pickup -d dropoff --use_sim_time
```

Or through the rmf-web dashboard's task submission panel.

### Monitoring via rmf-web

```bash
docker run --network host -it --rm \
  -e ROS_DOMAIN_ID=<id> \
  -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
  ghcr.io/open-rmf/rmf-web/api-server:<distro>-nightly
```

Run the pnpm-based dashboard frontend to see live robot positions, task queues, and fleet states, and to submit/cancel tasks from the browser.

### Day-2 Operations Checklist

- **Pause/e-stop a fleet:** use the dispatcher's fleet-level pause command, or call the fleet adapter's pause/resume service directly.
- **Close a lane / reroute:** issue a lane closure request; RMF's negotiation will replan affected itineraries automatically.
- **Check for schedule conflicts:** inspect `rmf_traffic_schedule` logs or the dashboard's traffic view.
- **Add/remove robots at runtime:** depends on the fleet adapter — some support dynamic registration; others require a restart with an updated robot list.
- **Rolling updates:** avoid restarting the traffic schedule node while robots are mid-task; drain or pause fleets first.

---

## Integrating a Real Fleet

1. Start from [`fleet_adapter_template`](https://github.com/open-rmf/fleet_adapter_template) rather than writing an adapter from scratch.
2. If the robot exposes a standard ROS 2 nav stack, [`free_fleet`](https://github.com/open-rmf/free_fleet) can integrate it with minimal custom code.
3. Otherwise, implement the `RobotCommandHandle` interface: report position/battery, accept path/pause/resume commands, against the robot's native API.
4. Register the fleet in the fleet adapter's config YAML (fleet name, nav graph, robot list, charger waypoints).
5. Check [`awesome_adapters`](https://github.com/open-rmf/awesome_adapters) — a commercial or community adapter may already exist for your robot.

---

## Troubleshooting

| Symptom                                  | Likely Cause                                                                           |
| ---------------------------------------- | -------------------------------------------------------------------------------------- |
| Fleet adapter not receiving tasks        | `ROS_DOMAIN_ID` mismatch across nodes/containers                                       |
| Robots stuck "negotiating"               | Disconnected lanes or missing waypoints in the nav graph                               |
| Build fails after installing from source | Old `ros-<distro>-rmf*` debs not purged before source build                            |
| Two workspaces conflict                  | More than one `setup.bash` sourced; only source one underlay/workspace chain at a time |

---

## Review & Assessment

**Strengths**

- Vendor-agnostic: designed from the ground up to coordinate heterogeneous fleets (different robot vendors, different nav stacks) in one shared space.
- Traffic negotiation is a genuinely hard problem it solves well — deadlock avoidance, lane reservations, and door/lift coordination are handled centrally rather than per-robot.
- Backed by Open Robotics/OSRF with an active GitHub presence, ROS 2 alignment, and a real adopter ecosystem (documented in `awesome_adapters`).
- Simulation-first workflow (`rmf_demos` + Gazebo) makes it possible to validate a deployment before touching real hardware.

**Limitations / things to weigh before adopting**

- Operational complexity is real: multiple moving parts (traffic schedule, per-fleet adapters, map server, dashboard) mean a non-trivial ops burden, especially for small deployments.
- `rmf_demos` isn't distributed as binaries, so even a "binary install" ends up requiring a source build step for anything beyond the bare core.
- Integration effort for a new fleet is nontrivial unless a community adapter already exists; writing a `RobotCommandHandle` from scratch requires decent familiarity with both RMF's task model and the target robot's API.
- Documentation is spread across many repos (`rmf`, `rmf_demos`, `rmf-web`, `rmf_ros2`, fleet-adapter repos); there isn't always one canonical source of truth per topic.
- As with most actively developed open-source robotics middleware, expect breaking changes between ROS distros and `main`/nightly builds — pin specific versions for anything production-facing.

**Bottom line:** Open-RMF is a strong fit if you're coordinating multiple robot fleets (possibly multi-vendor) sharing physical infrastructure like elevators and doors, and you're willing to invest in the adapter integration and operational tooling. For a single robot or single-vendor fleet with no shared infrastructure to coordinate, it's likely more machinery than the problem calls for.
