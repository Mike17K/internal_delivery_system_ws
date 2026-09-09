#!/usr/bin/env python3
"""Generate models/office_floor/model.sdf: a small campus of separate
office floors placed side by side along y (not stacked into a
multi-story building) -- except TOWER_FLOOR_INDEX, which is a
TOWER_LEVELS-story building instead, its levels stacked directly above
each other in z, reachable by a staircase and an elevator.

Each floor comes from floors.build_floor(), which assembles one floor's
building shell (via models/basic_structures.py + models/shapes.py) and
furniture (via room_compositions.py + models/models.py) into a single
Group, parametrically (FloorLayout) and with a little per-floor
randomization (which offices get a plant/cabinet, which slot hosts the
meeting/break room, a small color tint) driven by a seeded random.Random
so re-running this script is reproducible.

For the tower floor, build_floor() is called once per level, each at
z = level * FLOOR_TO_FLOOR_HEIGHT, and every level's staircase flight
uses the exact same shape/size/offset -- the module is never mirrored,
resized, or otherwise varied floor to floor:

- Only level 0 builds the elevator itself (build_elevator=True), sized
  to reach the *top* of the whole stack -- it doesn't shift position, so
  one tall shaft serves every level's identical doorway.
- Every level except the topmost builds a new staircase flight
  (build_staircase=True) departing upward from the middle slot's own
  doorway, in the same shape every time -- so its landing (offset by the
  fixed STAIR_OFFSET) is *not* directly above where it started.
- Every level except the base one has an arriving flight from below
  (has_incoming_staircase=True), needing its own doorway at that offset
  position -- a middle level generally has *two* stair doorways (its own
  departure at the base position, an arrival at the offset one); floors.py
  cuts both into that level's wall as needed.
- Only level 0 has has_main_entrance=True (a front door and lobby); a
  level with nothing below it has no street-level entrance of its own.

Every floor is resolved into one flat list of world-posed models; at
render time these split into raw shapes (Box/Window -- embedded inline in
the structural <link>) and catalog assets (everything else -- emitted as
<include>s), purely by type, since floors.build_floor() freely mixes both
kinds under one Group tree.

Re-run this script after changing anything below:

    python3 scripts/generator/generate.py
"""

import os
import random

import floors
from models.base import Group
from models.shapes import Box, Window

N_FLOORS = 1
FLOOR_GAP = 6.0                 # clear space between adjacent floors' outer walls
RANDOM_SEED = 1000              # base seed; floor i uses RANDOM_SEED + i, so re-runs are reproducible
TOWER_FLOOR_INDEX = 0           # which floor is the multi-story tower
TOWER_LEVELS = 1                # how many stories the tower has
FLOOR_TO_FLOOR_HEIGHT = 3.0     # rise from one level's walking surface to the next


def build_campus(
    n_floors: int = N_FLOORS,
    gap: float = FLOOR_GAP,
    seed: int = RANDOM_SEED,
    tower_floor_index: int = TOWER_FLOOR_INDEX,
    tower_levels: int = TOWER_LEVELS,
    floor_to_floor_height: float = FLOOR_TO_FLOOR_HEIGHT,
) -> list:
    root = Group(id="office_campus")

    y_offset = 0.0
    for i in range(n_floors):
        layout = floors.FloorLayout()
        is_tower = (i == tower_floor_index)
        depth = floors.footprint_depth(layout)

        if not is_tower:
            rng = random.Random(seed + i)
            floor_group, _ = floors.build_floor(f"floor_{i}", layout, rng)
            root.add_group(Group(id=f"floor_{i}_site", groups=[floor_group], pose=(0.0, y_offset, 0.0, 0, 0, 0)))
        else:
            # Levels sit directly above each other (same x/y footprint,
            # offset only in z), not beside the other floors along y.
            elevator_shaft_height = (tower_levels - 1) * floor_to_floor_height
            for level in range(tower_levels):
                level_rng = random.Random(seed + i + level * 500)
                level_name = f"floor_{i}" if level == 0 else f"floor_{i}_L{level}"
                level_group, _ = floors.build_floor(
                    level_name, layout, level_rng,
                    include_vertical_circulation=True,
                    floor_to_floor_height=floor_to_floor_height,
                    build_elevator=(level == 0),
                    build_staircase=(level < tower_levels - 1),
                    has_incoming_staircase=(level > 0),
                    elevator_shaft_height=elevator_shaft_height,
                    has_main_entrance=(level == 0),
                )
                root.add_group(Group(
                    id=f"{level_name}_site", groups=[level_group],
                    pose=(0.0, y_offset, level * floor_to_floor_height, 0, 0, 0),
                ))

        y_offset += depth + gap

    return root.resolve()


def render() -> str:
    resolved = build_campus()

    # floors.build_floor() mixes raw structural shapes (Box/Window --
    # inline <collision>/<visual> geometry) and catalog furniture models
    # (<include>s) under one Group tree; split them back out here purely
    # by type, since that's the only thing that determines where each one
    # goes in the final SDF.
    link_body = "".join(m.to_sdf() for m in resolved if isinstance(m, (Box, Window)))
    includes_body = "".join(m.to_sdf() for m in resolved if not isinstance(m, (Box, Window)))

    return f"""<?xml version="1.0"?>

<!-- AUTO-GENERATED by scripts/generator/generate.py — do not hand-edit. -->

<sdf version="1.10">

  <model name="office_floor">

    <static>true</static>

    <link name="structure_link">
{link_body}
    </link>
{includes_body}
  </model>

</sdf>
"""


if __name__ == "__main__":
    out_path = os.path.join(os.path.dirname(__file__), "..", "..", "models", "office_floor", "model.sdf")
    out_path = os.path.normpath(out_path)
    with open(out_path, "w") as f:
        f.write(render())
    print(f"wrote {out_path}")
