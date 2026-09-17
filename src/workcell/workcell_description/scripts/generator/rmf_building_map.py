#!/usr/bin/env python3
"""Generate maps/office_world.building.yaml: the Open-RMF topological nav
graph (named waypoints + lanes) for office_world, for
rmf_building_map_tools' `building_map_generator nav` to convert into the
runtime nav graph the RMF fleet adapter actually loads.

Room waypoints are read straight out of the generated office_floor model
the same way office_distributor/room_goal_distributor.py's parse_rooms()
does (every room's ceiling-light `<include>` gives its world x/y/yaw) --
duplicated here rather than imported, since importing across packages
would make workcell_description's build depend on office_distributor's,
backwards from the real relationship (office_distributor already depends
on workcell_description's generated scene). Re-running generate.py (a
different seed, room count, ...) is picked up here too, on the next run of
this script -- no coordinates are hand-typed.

Corridor waypoints are NOT derivable from room data alone (the elevator/
stair bay column has no room either side), so office_width/
n_offices_per_side are hardcoded here matching floors.py's FloorLayout
defaults -- same tradeoff generate.py itself makes for its own layout
constants.

Re-run after changing anything below, or after office_floor/model.sdf is
regenerated with a different layout:

    python3 scripts/generator/rmf_building_map.py
"""

import os
import re
from pathlib import Path

OFFICE_WIDTH = 4.0          # floors.py FloorLayout.office_width default
N_OFFICES_PER_SIDE = 6      # floors.py FloorLayout.n_offices_per_side default
ROOM_ROW_Y = 3.2            # abs(y) of a room's own waypoint (matches parsed room data)

# Real Gazebo spawn points (workcell_bringup/launch/workcell.launch.py
# robots_config's active - i.e. uncommented - entries) - none of these are
# one of the office-column corridor waypoints, so each gets its own
# vertex, linked into the corridor chain as that robot's charger stub.
ROBOT_SPAWNS = {
    "agv_1": (0.0, 0.0),
    "agv_2": (1.5, 0.0),
    "agv_3": (3.0, 0.0),
}

INCLUDE_RE = re.compile(r"<include>(.*?)</include>", re.S)
NAME_RE = re.compile(r"<name>([^<]+)</name>")
URI_RE = re.compile(r"<uri>([^<]+)</uri>")
POSE_RE = re.compile(r"<pose[^>]*>([^<]+)</pose>")


def parse_rooms(model_sdf_path: Path) -> dict:
    """Same technique as office_distributor/room_goal_distributor.py's
    parse_rooms(): every room's (x, y, yaw), keyed by room id, read off its
    CeilingLight include (the one thing every room, and only a room, has
    exactly one of)."""
    text = model_sdf_path.read_text()
    rooms = {}
    for block in INCLUDE_RE.findall(text):
        uri = URI_RE.search(block)
        name = NAME_RE.search(block)
        pose = POSE_RE.search(block)
        if not (uri and name and pose):
            continue
        if "ceiling_light" not in uri.group(1) or not name.group(1).endswith("_light"):
            continue
        room_id = name.group(1)[: -len("_light")]
        x, y, _z, _roll, _pitch, yaw = (float(v) for v in pose.group(1).split())
        rooms[room_id] = (x, y, yaw)
    return rooms


# building_map/param_value.py's ParamValue: every flag value must be
# encoded as [type_code, value] - STRING=1, INT=2, DOUBLE=3, BOOL=4 - not
# a bare scalar, or Building.parse_yaml crashes trying to subscript a bool.
def yaml_param(value) -> str:
    if isinstance(value, bool):
        return f"[4, {'true' if value else 'false'}]"
    if isinstance(value, int):
        return f"[2, {value}]"
    if isinstance(value, float):
        return f"[3, {value:.4f}]"
    return f"[1, {value}]"


def vertex_line(x: float, y: float, name: str, flags: dict) -> str:
    flags_str = ", ".join(f"{k}: {yaml_param(v)}" for k, v in flags.items())
    return f"      - [{x:.4f}, {y:.4f}, 0, {name}, {{{flags_str}}}]"


def lane_line(a: int, b: int, *, bidirectional: bool = True) -> str:
    # graph_idx is required by Level.generate_nav_graph (KeyError otherwise)
    # - 0 is the only graph, matching every lane in rmf_demos' own example.
    return f"      - [{a}, {b}, {{bidirectional: {yaml_param(bidirectional)}, graph_idx: {yaml_param(0)}}}]"


def build(rooms: dict) -> str:
    corridor_xs = sorted({round((i + 0.5) * OFFICE_WIDTH, 4) for i in range(N_OFFICES_PER_SIDE)})

    vertices = []   # list of (x, y, name, flags)
    index = {}       # name -> vertex index (name is "" for unnamed corridor points, so key by (x, y) instead)

    def add_vertex(x: float, y: float, name: str = "", **flags) -> int:
        idx = len(vertices)
        vertices.append((x, y, name, flags))
        index[(round(x, 4), round(y, 4))] = idx
        return idx

    # Charger/spawn stub per robot, linked into the corridor chain below.
    charger_idx = {}
    for robot_name, (x, y) in ROBOT_SPAWNS.items():
        charger_idx[robot_name] = add_vertex(
            x, y, f"{robot_name}_charger",
            is_charger=True, is_holding_point=True, is_parking_spot=True,
            spawn_robot_name=robot_name, spawn_robot_type="Open-RMF/AGV",
        )

    corridor_idx = {}
    for x in corridor_xs:
        corridor_idx[x] = add_vertex(x, 0.0)

    room_idx = {}
    for room_id, (x, y, _yaw) in sorted(rooms.items()):
        room_idx[room_id] = add_vertex(x, y, room_id)

    lanes = []
    # Each charger -> its nearest corridor waypoint.
    for robot_name, (x, y) in ROBOT_SPAWNS.items():
        nearest_x = min(corridor_xs, key=lambda cx: abs(cx - x))
        lanes.append((charger_idx[robot_name], corridor_idx[nearest_x]))
    # Corridor spine, west to east.
    for a, b in zip(corridor_xs, corridor_xs[1:]):
        lanes.append((corridor_idx[a], corridor_idx[b]))
    # Each room <-> its own column's corridor waypoint.
    for room_id, (x, y, _yaw) in rooms.items():
        nearest_x = min(corridor_xs, key=lambda cx: abs(cx - x))
        lanes.append((room_idx[room_id], corridor_idx[nearest_x]))

    vertex_lines = "\n".join(vertex_line(x, y, name or '""', flags) for x, y, name, flags in vertices)
    lane_lines = "\n".join(lane_line(a, b) for a, b in lanes)

    return f"""name: office_world
coordinate_system: cartesian_meters
crowd_sim:
  agent_groups: []
  agent_profiles: []
  enable: 0
  goal_sets: []
  model_types: []
  obstacle_set: {{class: 1, file_name: "", type: nav_mesh}}
  states: []
  transitions: []
  update_time_step: 0.1
graphs: {{}}
levels:
  L1:
    elevation: 0
    vertices:
{vertex_lines}
    lanes:
{lane_lines}
lifts: {{}}
"""


if __name__ == "__main__":
    model_sdf_path = Path(os.path.dirname(__file__)) / ".." / ".." / "models" / "office_floor" / "model.sdf"
    model_sdf_path = model_sdf_path.resolve()
    rooms = parse_rooms(model_sdf_path)
    if not rooms:
        raise SystemExit(f"no rooms found in {model_sdf_path} - run generate.py first")

    out_path = Path(os.path.dirname(__file__)) / ".." / ".." / "maps" / "office_world.building.yaml"
    out_path = out_path.resolve()
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(build(rooms))
    print(f"wrote {out_path} ({len(rooms)} rooms)")
