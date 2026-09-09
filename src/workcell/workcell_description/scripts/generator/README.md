# Office floor generator

This directory generates `models/office_floor/model.sdf` — a small campus
of office buildings — from Python instead of hand-written SDF. Run it
after changing anything under here:

```bash
python3 scripts/generator/generate.py
```

It's deterministic for a given `RANDOM_SEED` (in `generate.py`), so
re-running it without touching the code reproduces the exact same file.

## The pipeline, top to bottom

```
generate.py            "campus" assembly: how many floors, where they sit,
                        which one is a multi-story tower and how tall
  -> floors.py          one floor's whole layout: walls + rooms, parametric
                        (FloorLayout) and lightly randomized
       -> room_compositions.py   what goes inside a walled room (desk,
                                  chair, cabinet, ...), given where its door is
       -> models/circulation.py  exterior switchback staircase + a non-
                                  functional elevator, sized to a floor-to-floor rise
            -> models/basic_structures.py   parametric walls (solid / with
                                             a door gap / with windows)
                 -> models/shapes.py         Box, Window: raw inline geometry
            -> models/models.py              the furniture/fixture catalog
                 -> models/base.py           Model, Group: the shared
                                             placement machinery
            -> models/sides.py               Side enum + rotation helpers
                                             ("which wall is the door on")
```

Nothing here talks to SDF text directly except the two leaf layers
(`shapes.py`'s `to_sdf()` and `base.py`'s `Model.to_sdf()`) and the final
`render()` in `generate.py`. Everything above that just builds a tree of
Python objects.

## The core idea: Model, Group, and `.resolve()`

Two types, in `models/base.py`, do all the placement work:

- **`Model`** — one placeable thing: an `id`, a `uri` (which SDF asset it
  is — empty for raw geometry), `dimensions`, and a `pose`
  `(x, y, z, roll, pitch, yaw)`. It knows how to render itself
  (`to_sdf()`), but nothing about where it sits in the wider scene.
- **`Group`** — a named local coordinate frame with its own `pose`,
  holding child `Model`s and/or child `Group`s, each given in *that
  frame's* local coordinates. `Group.resolve()` walks the tree, composing
  poses with real rotation matrices (not just yaw arithmetic), and
  returns a flat list of `Model`s with final world poses.

So a room is authored once, in its own convenient local coordinates
("desk sits 1m left, 0.9m toward the far wall"), wrapped in a `Group`
whose own `pose` says where that room actually sits in the building —
and `.resolve()` does the arithmetic. Nest a room inside a floor inside a
campus and it's still just poses composing correctly all the way up.

`Box` and `Window` (`models/shapes.py`) are `Model` subclasses too: they
reuse `dimensions` as their box size and `pose` as normal, but override
`to_sdf()` to emit inline `<collision>/<visual>` geometry instead of an
`<include>` (there's no separate model file behind a wall segment). This
is what lets `floors.py` mix raw wall geometry and catalog furniture
freely under one `Group` tree — `generate.py`'s `render()` only tells them
apart at the very end, by `isinstance(m, (Box, Window))`, to decide
whether each one belongs inside the structural `<link>` or as a sibling
`<include>`.

## Two kinds of "asset"

- **Catalog models** (`models/models.py`, backed by real SDF files under
  `models/<name>/`) — fixed-size furniture/fixtures: `OfficeDesk`,
  `Chair`, `OfficeCabinet`, `PottedPlant`, `CeilingLight`, `MeetingTable`,
  `BreakCounter`, `ReceptionDesk`, plus the original catalog `Wall`,
  `Floor`, `Table`, `Shelf`, `Pallet`. Placed via `<include>`.
- **Procedural shapes** (`models/shapes.py` + `models/basic_structures.py`
  + `models/circulation.py`) — geometry whose *size* depends on context
  (a wall run's length, a staircase's rise), so it can't be one fixed SDF
  file. Built from `Box`/`Window` primitives at generation time and
  embedded inline.

Add a new piece of furniture by writing its SDF under `models/<name>/`
and a matching `Model` subclass in `models/models.py`. Add a new kind of
structural geometry (not a fixed catalog piece) as a function in
`basic_structures.py` or a new sibling module, returning a `Group` built
from `Box`/`Window`.

## Rooms and doors (`models/sides.py`, `room_compositions.py`)

Every room in `room_compositions.py` (`office_room_group`,
`meeting_room_group`, `break_room_group`) is authored **once**, in a
canonical frame where its door is on the south wall (local `-y`). A
`door_side: Side` argument (`NORTH`/`EAST`/`SOUTH`/`WEST`) rotates the
whole room — every piece of furniture, position and facing both — to put
the door on whichever side it actually needs to be on. `floors.py` just
picks `Side.SOUTH` for north-row rooms (door faces the corridor below
them) and `Side.NORTH` for south-row rooms, and the room composition
functions handle the rest via `models.sides.room_pose()`.

`models/circulation.py`'s staircase and elevator use the exact same
`door_side` convention, but skip the "walled room" part entirely — both
sit entirely outside the building, each reached through its own doorway.

The lobby (`lobby_group`) is the one exception: since it's tied to one
specific building entrance rather than a generic room shape, it takes
final world poses directly instead of a canonical-frame + `door_side`.
`floors.py` still keeps it consistent by computing those poses with
`models.sides.facing_yaw(entrance_side)` rather than hardcoded numbers.

## One floor (`floors.py`)

`FloorLayout` is a dataclass holding every geometric knob for one floor:
office count/size, corridor width, wall thickness/height, colors,
entrance side, and a few probabilities (`plant_probability`,
`cabinet_probability`, `color_jitter`). `build_floor(name, layout, rng)`
builds:

1. The floor slab and all walls (exterior with windows, corridor walls
   with a door per office, partitions between offices) — procedural,
   via `basic_structures.py`.
2. A row of offices on each side of the corridor, with one slot per row
   replaced by a meeting room / break room (their *slot index* is picked
   from `rng`, so it varies floor to floor).
3. A lobby at the entrance end.

It returns `(Group, footprint_width)` — the group isn't placed anywhere
yet (its own pose is the identity); the width is so a caller can space
several floors apart without overlapping (there's a matching
`footprint_depth()` for spacing along y instead of x).

### Randomization

Everything randomized goes through the `random.Random` instance passed
in — never the global `random` module — so a given `(layout, seed)` pair
always produces the same floor. What's randomized: which offices get a
plant/cabinet, which slot on each row hosts the meeting/break room, and a
small per-floor wall/floor color tint (`_jitter_color`). Room *shapes*
(desk offsets, seat layout, ...) are not randomized here — see
`room_compositions.py` if a specific room's arrangement needs to vary.

### Vertical circulation (staircase / elevator)

Neither is a room, and the staircase in particular needs real space —
more than an office-sized bay can give it — so `build_floor(...,
include_vertical_circulation=True)` puts both at the floor's *middle*
office slot (`n_offices_per_side // 2`, not a room, not randomized —
fixed, since a stacked upper level's opening/doorway has to line up with
it exactly) rather than shoehorning them into the office row:

Both are entirely **outside the building**, each reached through its own
doorway rather than an interior opening — so the floor slab is always a
single solid piece, no holes needed:

- **Elevator** (north row): the middle slot's corridor-facing door/wall
  is skipped (an open bay leading to the doorway, nothing else), and the
  exterior wall itself gets an actual doorway
  (`wall_with_windows(..., doors=[...])`) instead of a window,
  leading out to `circulation.py`'s `elevator()` -- a **static,
  non-functional prop** (a shaft + a parked cabin, no doors, no motion)
  parked just outside the north wall. It rises straight up, so its
  doorway sits at the same x on every level -- just one door, always.
- **Staircase** (south row): open the same way on the corridor side, with
  a doorway (or two -- see below) cut into the south exterior wall,
  leading to `circulation.py`'s `staircase()`, a real switchback (dogleg)
  flight built entirely outside the building envelope, the way an
  exterior fire-stair works.

The staircase is **modular in the rise it needs to cover**: each of its
two flights covers half of `floor_to_floor_height`, with its own step
count chosen so that half divides evenly into risers no taller than
`riser_height` — every step in a flight is the same steady height, and a
taller floor just gets more steps rather than taller ones. Its tread
depth is also widened as needed to keep the incline at or below
`max_incline_deg`, so it's never steeper than a normal walkable stair
regardless of `rise_height`.

Real buildings build stairs this way for a reason: one flight climbs away
from the door to an intermediate landing (the "sub-floor" joining the two
flights), then the second flight climbs the rest of the way back —
shifted sideways by `offset` — ending at a doorway that's offset from the
starting one by that same amount, not directly above it. Crucially,
**every flight reuses the exact same shape and offset** — the same
`STAIR_OFFSET`, the same width, floor to floor. The module itself is
never mirrored, resized, or otherwise varied between levels; only its
*position* changes (`door_pose`'s z).

Since every flight is identical, a middle floor generally needs *two*
separate doorways in its south wall, not one: a **departure** doorway (at
the middle slot's own center — where this level's own new flight starts)
and an **arrival** doorway (`STAIR_OFFSET` further along — where the
flight from the level below actually lands). `wall_with_windows` accepts
a list of `(center, width)` pairs precisely for this — a wall can now
have more than one doorway gap on the same run. `build_staircase`
controls whether a level cuts the departure doorway (and builds that new
flight) and `has_incoming_staircase` whether it cuts the arrival one (no
new geometry, just the opening for whatever's already climbing up to
it) — independently, since a middle level needs both, the base level only
the departure, and the top level only the arrival.

`build_elevator` (typically just the base level, with a shaft tall enough
to reach the *top* of the whole stack) is the only other level-dependent
switch — every level always gets the exact same single elevator doorway,
with no incoming/outgoing distinction to make (a straight shaft doesn't
shift sideways the way a switchback does).

## The campus (`generate.py`)

Loops over `N_FLOORS`, building each with `random.Random(RANDOM_SEED + i)`
and placing them beside each other along y by `footprint_depth() + gap`.
`TOWER_FLOOR_INDEX` is instead a `TOWER_LEVELS`-story tower: `build_floor()`
is called once per level, each at `z = level * FLOOR_TO_FLOOR_HEIGHT`,
with only level 0 getting `build_elevator=True` + `has_main_entrance=True`
(a front door and lobby — a level with nothing below it has no
street-level entrance of its own), every level but the last getting
`build_staircase=True`, and every level but the first getting
`has_incoming_staircase=True` — giving that one floor `TOWER_LEVELS`
stories whose elevator opening (identical on every level) and exterior
stair doorways (the same two fixed positions throughout, whichever a
given level actually needs) all line up exactly with the flights actually
arriving there — the same staircase module, reused unmodified at every
level, directly above/below itself the whole way up.

`render()` resolves the whole campus `Group` tree in one shot, splits the
result into inline shapes vs. `<include>`s by type, and writes the final
`<model name="office_floor">` SDF.

## Extending this

- **New floor variation**: add a field to `FloorLayout`, use it in
  `build_floor()`. Existing call sites keep working via the default.
- **New room type**: write a `..._room_group(prefix, room_center_world,
  door_side, *, wall_height, ...)` function in `room_compositions.py`
  (or `circulation.py` if it doesn't need walls), following the existing
  ones — canonical door=SOUTH frame, `models.sides.room_pose()` for the
  Group's pose. Wire it into `build_floor()`'s room loop.
- **New furniture piece**: SDF file under `models/<name>/model.sdf` +
  `model.config`, then a `Model` subclass in `models/models.py`.
- **Different campus shape**: `generate.py` is the only place that knows
  how many floors there are or how they're arranged; `floors.py` and
  everything below it only ever build one floor at a time.
