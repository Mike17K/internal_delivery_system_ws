# Open-RMF + Nav2 Fleet: Logic & Operation

Companion to `01-openrmf-nav2-integration.md`. This document covers **what
happens at runtime** — task lifecycle, traffic deconfliction, negotiation,
failure handling, and how Nav2-level events surface up to RMF.

---

## 1. Layered Responsibility Model

| Layer | Owns | Does NOT own |
|---|---|---|
| **Nav2 (per robot)** | Local path following, obstacle avoidance in its own costmap, recovery behaviors (spin, backup, wait), footprint-level collision avoidance | Knowledge of other robots' intents, global schedule, task assignment |
| **Fleet Adapter (per fleet)** | Translating RMF waypoint commands into Nav2 goals, reporting robot state (pose, battery, task status) back to RMF, robot-specific behaviors (docking, custom actions) | Traffic scheduling, task assignment logic |
| **RMF Core** | Task allocation across fleets, traffic schedule (who occupies which lane/waypoint when), negotiation between conflicting robots, lift/door coordination | Low-level path following |

This separation is the whole point: Nav2 never needs to know a fleet exists,
and RMF never needs to know what a costmap is. The fleet adapter is the only
component that speaks both languages.

---

## 2. Task Lifecycle

```
 Task submitted (rmf-web / CLI / API)
        │
        ▼
 Task Dispatcher — matches task to an eligible fleet
   (based on fleet capability, battery, availability)
        │
        ▼
 Task decomposed into a sequence of Activities
   e.g. "Delivery" -> [GoToPlace(pickup), Dock, Wait, GoToPlace(dropoff), Dock]
        │
        ▼
 Fleet Adapter receives Activity requests one at a time
        │
        ▼
 For each GoToPlace: Fleet Adapter asks Traffic Schedule
   "can robot1 traverse lane A->B starting at time T?"
        │
   ┌────┴────┐
   │ granted │ denied → adapter holds robot, retries / requests
   └────┬────┘          alternate route from traffic negotiation
        ▼
 Fleet Adapter calls Nav2 NavigateToPose (or NavigateThroughPoses)
   for the waypoint's map pose
        │
        ▼
 Nav2 executes locally (planner + controller + recovery as needed)
        │
        ▼
 Fleet Adapter polls/receives Nav2 action feedback & result
        │
        ▼
 On success: Fleet Adapter reports waypoint reached to Traffic Schedule
   (releases the lane reservation), advances to next Activity
        │
        ▼
 Task marked complete when final Activity finishes
```

Key point: **Nav2 only ever knows about one waypoint at a time.** RMF issues
`NavigateToPose` goals one leg at a time (or a short queued sequence via
`NavigateThroughPoses` if the adapter chooses to batch a straight run), not
the whole task.

---

## 3. Traffic Scheduling & Deconfliction

RMF's `rmf_traffic` maintains a **space-time schedule**: each robot's
planned trajectory is a reserved block of (lane/waypoint, time window).
Before a robot starts a leg, its fleet adapter requests a schedule slot.

### 3.1 Conflict types RMF resolves centrally (not via Nav2)

- **Head-on conflict** on a single-lane corridor — RMF either sequences
  the two robots (one waits) or reroutes one via an alternate lane if the
  graph has one.
- **Intersection conflict** — RMF treats intersections as mutually
  exclusive resources; only one robot's reserved time-slot can occupy the
  intersection at a time.
- **Waypoint contention** (e.g. shared docking/charging spot) — treated as
  a resource lock in the schedule, not a costmap-level avoidance problem.

### 3.2 Negotiation

When two robots' desired schedules conflict, RMF runs a **negotiation**:
each robot's adapter proposes alternative itineraries (delay, reroute), and
the negotiation picks the combination with lowest total cost. This happens
**before** either robot starts moving on the contested segment — it is not
a reactive/runtime avoidance mechanism.

### 3.3 What Nav2 still handles locally

Even with RMF scheduling lanes, Nav2's local costmap obstacle layer (see
integration doc §3.3) is still needed for:
- Dynamic obstacles RMF doesn't know about (people, forklifts, unscheduled
  objects in simulation)
- Fine-grained spacing between two robots RMF has sequenced onto the same
  lane at different times but that are still transiently close
- Any error between planned schedule and actual execution (a robot running
  slightly behind schedule due to a Nav2 recovery behavior)

So: **RMF prevents lane/intersection-level conflicts before they happen;
Nav2 prevents collisions from whatever slips through.** Both layers are
required — RMF alone doesn't replace local obstacle avoidance, and Nav2
alone (independent per-robot) doesn't replace global traffic scheduling.

---

## 4. Lift / Door Coordination

If your building graph includes lifts/doors (`rmf_traffic_editor` supports
modeling these):
- Fleet adapter requests lift/door state changes through RMF's
  `AdapterDoorState` / `AdapterLiftState` interfaces, not directly.
- RMF core sequences multiple fleets' access to the same lift — this is
  effectively the same resource-lock pattern as intersections, extended to
  physical infrastructure.
- Nav2 has zero awareness of doors/lifts; the fleet adapter simply doesn't
  issue the next `NavigateToPose` goal until RMF confirms the door/lift is
  in the required state.

---

## 5. Failure & Recovery Propagation

| Failure | Detected by | Propagates to |
|---|---|---|
| Local obstacle blocks path briefly | Nav2 recovery behaviors (wait, spin, backup) | Nav2 handles silently; only surfaces to RMF if it exceeds a timeout |
| `FollowPath`/planner repeated failure | Nav2 BT navigator returns FAILURE | Fleet adapter reports task/activity failure to RMF; RMF may replan, reassign task, or flag for human intervention |
| Battery below threshold mid-task | Fleet adapter (via `battery_soc` in RobotClientAPI) | RMF task dispatcher inserts a charging task, may reassign current task to another robot if fleet allows |
| Robot goes fully unresponsive (adapter loses heartbeat) | Fleet adapter watchdog | RMF marks robot offline, excludes it from future task allocation, may need to manually reassign in-flight task |
| Traffic schedule conflict robot can't resolve (deadlock) | RMF negotiation failure | RMF logs/flags; typically needs graph review (missing alternate lane) rather than a runtime fix |

Design implication: your `RobotClientAPI.navigate()` implementation should
**not silently retry forever** on Nav2 failure — bubble the actual
`NavigateToPose` result (including recoveries exhausted / aborted) up so
RMF's task layer can make a reassignment decision instead of a robot idling
indefinitely mid-corridor.

---

## 6. State Reporting Loop

Continuous, independent of task execution:

```
Nav2 (per robot)                Fleet Adapter                RMF Core
  amcl_pose  ──────────────────▶ position()
  battery topic (sim) ─────────▶ battery_soc()
  BT navigator status  ────────▶ activity progress
                                        │
                                        ▼  publish_fleet_state (e.g. every 1-10s)
                                RobotState msg ─────────────▶ Fleet State Aggregator
                                                                     │
                                                                     ▼
                                                          rmf-web dashboard / task
                                                          dispatcher availability
```

This loop is what lets the task dispatcher know which robots are idle,
mid-task, charging, or offline when a new task is submitted — it runs
regardless of whether any task is currently active.

---

## 7. Practical Simulation Debugging Order

When something doesn't work end-to-end, check in this order (cheapest to
verify first):

1. **Single robot, no RMF** — send a raw `NavigateToPose` to one namespaced
   Nav2 stack. Confirms Nav2 bringup itself is correct.
2. **Fleet adapter, single robot, no traffic conflicts** — command one
   waypoint through the adapter. Confirms the RMF↔Nav2 translation layer.
3. **Two robots, non-conflicting routes** — confirms fleet state reporting
   and independent task execution.
4. **Two robots, deliberately conflicting routes** (same lane, opposite
   direction) — confirms traffic scheduling/negotiation is actually
   engaging, not just relying on local costmap avoidance.
5. **Full fleet + task dispatcher submitting concurrent tasks** — the real
   stress test; failures here are almost always graph design (missing
   passing lanes, bad waypoint placement) rather than Nav2 tuning.

---

## 8. Summary: Division of Concerns

- **Nav2** = "how do I get from A to B without hitting anything right now."
- **Fleet Adapter** = "translate RMF's request into a Nav2 goal, and Nav2's
  result into an RMF-understood outcome."
- **RMF Core** = "who should do this task, and who gets to be where, when,
  so nobody needs to fight it out reactively."

If you find yourself trying to solve a scheduling/allocation problem inside
a Nav2 plugin, or a local-obstacle problem inside RMF's traffic schedule,
that's a sign it's in the wrong layer.
