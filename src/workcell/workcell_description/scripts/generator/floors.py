"""Parametric, randomized single-story floor plans.

build_floor() assembles one full floor (building shell + furniture) as a
single Group, local to its own origin (not yet placed in a larger scene) --
using models/basic_structures.py for the walls/windows/floor slab and
room_compositions.py for what goes inside each room.

Every geometric parameter (office count/size, corridor width, wall
thickness, ...) is collected in FloorLayout, so the same plan can be
resized or reshaped from one call site. A handful of furnishing choices --
whether a given office gets a plant or a cabinet, which office slot on
each row hosts the meeting/break room, and a small per-floor color tint --
are decided from the `random.Random` passed in, so several floors built
from the same FloorLayout still look like distinct buildings.
"""

import random
from dataclasses import dataclass
from typing import Optional, Tuple, cast

from models import basic_structures as structures
from models import circulation
from models.base import Group
from models.shapes import Color
from models.sides import Side, facing_yaw, opposite
from room_compositions import (
    break_room_group,
    lobby_group,
    meeting_room_group,
    office_room_group,
)

# The exterior staircase's two doorways (ground and the level above)
# aren't directly above each other -- its second flight lands shifted
# sideways from the first (see models/circulation.py) -- so the upper
# doorway needs this same offset applied to it. Fixed, not derived from
# the stair's own defaults, so the wall-opening code and the staircase
# itself can never disagree about it.
STAIR_WIDTH = 1.3
STAIR_OFFSET = STAIR_WIDTH + 0.15
STAIR_DOOR_WIDTH = 1.0

# The elevator shaft doesn't shift sideways the way the staircase's two
# flights do -- straight up, not a switchback -- so its doorway sits at
# the same x on every level; no offset constant needed for it.
ELEVATOR_DOOR_WIDTH = 1.2


@dataclass
class FloorLayout:
    n_offices_per_side: int = 6      # regular offices per row (one slot per row
                                      # is replaced by a special room -- see below)
    office_width: float = 4.0         # size along the corridor (x)
    office_depth: float = 4.0         # size across, corridor wall to exterior (y)
    corridor_width: float = 2.4
    lobby_depth: float = 5.0          # open-plan bay in front of the offices
    wall_thickness: float = 0.15
    wall_height: float = 2.8
    door_width: float = 1.0           # office doorway gap
    entrance_width: float = 1.5       # main entrance gap
    window_width: float = 3.0         # big windows -- most of a 4m office bay's width
    window_height: float = 1.8

    wall_color: Color = (0.85, 0.85, 0.8)
    floor_color: Color = (0.55, 0.5, 0.45)
    window_color: Color = (0.6, 0.75, 0.85)

    entrance_side: Side = Side.WEST    # which side of the building the main entrance/lobby faces;
                                        # the lobby's own furniture orients off of this (see build_floor()).
                                        # NOTE: only WEST is currently wired up structurally (which wall
                                        # gets the door gap, where the lobby bay sits) -- changing this
                                        # re-orients the lobby furniture but not the walls themselves.

    plant_probability: float = 0.5
    cabinet_probability: float = 0.9
    color_jitter: float = 0.05        # per-floor random tint amount, see _jitter_color()


def compute_footprint(layout: FloorLayout) -> Tuple[float, float, float]:
    """(x_min, x_max, y_half) of the floor's wall-centerline extent."""
    x_min = -layout.lobby_depth
    x_max = layout.n_offices_per_side * layout.office_width
    y_half = layout.office_depth + layout.corridor_width / 2.0
    return x_min, x_max, y_half


def footprint_width(layout: FloorLayout) -> float:
    """Overall outer width (x) of the floor, wall faces included -- what a
    caller placing several floors side by side along x needs to space
    them by."""
    x_min, x_max, _ = compute_footprint(layout)
    return (x_max - x_min) + 2 * layout.wall_thickness


def footprint_depth(layout: FloorLayout) -> float:
    """Overall outer depth (y) of the floor, wall faces included -- what a
    caller placing several floors side by side along y needs to space
    them by."""
    _, _, y_half = compute_footprint(layout)
    return 2 * y_half + 2 * layout.wall_thickness


def _jitter_color(rng: random.Random, color: Color, amount: float) -> Color:
    if amount <= 0:
        return color
    return cast(Color, tuple(max(0.0, min(1.0, c + rng.uniform(-amount, amount))) for c in color))


def build_floor(
    name: str,
    layout: Optional[FloorLayout] = None,
    rng: Optional[random.Random] = None,
    *,
    include_vertical_circulation: bool = False,
    floor_to_floor_height: float = 3.0,
    build_elevator: bool = False,
    build_staircase: bool = False,
    has_incoming_staircase: bool = False,
    elevator_shaft_height: Optional[float] = None,
    has_main_entrance: bool = True,
) -> Tuple[Group, float]:
    """Build one full floor. Returns (group, footprint_width); the group
    is NOT yet placed anywhere -- its own pose is the identity -- so the
    caller can position several of these (e.g. side by side, offset by
    each other's footprint_width) by wrapping each in an outer Group with
    a translated pose.

    `has_main_entrance` controls the west wall's entrance gap and the
    lobby behind it: a level stacked above another one (see generate.py)
    has no doorway to the outside world at its own height, so pass False
    for it and it gets a plain solid wall there instead (and no lobby --
    a reception area only makes sense next to an actual entrance) -- only
    a building's ground level needs (or gets) either.

    `include_vertical_circulation` gives the floor, at its middle office
    slot on each row, a doorway leading OUTSIDE to a non-functional
    elevator (north row) and an exterior switchback staircase (south row)
    -- both from models/circulation.py. Neither is a walled "room": each
    row's middle slot just skips its corridor-facing door/wall (an open
    bay, nothing to navigate around), and the doorway to the elevator/
    stair is cut directly into the exterior wall instead. That middle
    slot's position is fixed by the layout (`n_offices_per_side // 2`),
    never RNG-derived, so it's identical on every level of a stack.

    Every staircase flight uses the exact same shape, size, and sideways
    offset (STAIR_OFFSET) -- the same module reused floor to floor, not
    mirrored or resized -- so it always starts at the middle slot's own
    doorway position and always lands offset by -STAIR_OFFSET from it.
    That means a middle floor's south wall generally needs *two* separate
    doorways: one where its own flight departs upward (at the middle
    slot's position) and one where the flight from the floor below
    arrives (STAIR_OFFSET further along):

    - `build_staircase` instantiates a new flight departing upward from
      this level -- pass this for every level except the topmost one
      (nothing further up to reach). Cuts the departure doorway.
    - `has_incoming_staircase` cuts the arrival doorway for a flight
      coming up from the level below -- pass this for every level except
      the base one (nothing below it to arrive from).
    - `build_elevator` instantiates the elevator itself -- pass this for
      just one level (typically the base one) and give it
      `elevator_shaft_height` tall enough to reach the *top* of the whole
      stack (it doesn't shift position, so one tall shaft serves every
      level's doorway, which is otherwise identical on every level and
      needs no incoming/outgoing distinction).

    `floor_to_floor_height` is the rise a single new staircase flight
    needs to span to reach the level above.
    """
    layout = layout or FloorLayout()
    rng = rng or random.Random()
    L = layout

    wall_color = _jitter_color(rng, L.wall_color, L.color_jitter)
    floor_color = _jitter_color(rng, L.floor_color, L.color_jitter)

    x_min, x_max, y_half = compute_footprint(L)
    corridor_y_north = L.corridor_width / 2.0
    corridor_y_south = -L.corridor_width / 2.0

    middle_slot = L.n_offices_per_side // 2    # unused unless include_vertical_circulation

    if include_vertical_circulation:
        north_pool = [i for i in range(L.n_offices_per_side) if i != middle_slot]
        south_pool = [i for i in range(L.n_offices_per_side) if i != middle_slot]
    else:
        north_pool = list(range(L.n_offices_per_side))
        south_pool = list(range(L.n_offices_per_side))
    meeting_slot = rng.choice(north_pool)
    break_slot = rng.choice(south_pool)

    root = Group(id=name)

    # ---- floor slab -----------------------------------------------------
    # Both the elevator and the staircase are entirely outside the
    # building now, reached through a doorway rather than an interior
    # opening, so the slab is always a single solid piece -- no holes.
    #
    # Thick enough to span the gap between this level's own walls
    # (wall_height tall) and the underside of the level above (see
    # generate.py, which stacks levels floor_to_floor_height apart) --
    # otherwise there'd be a visible sliver of daylight between the top
    # of the walls and the slab sitting on them.
    slab_thickness = max(0.1, floor_to_floor_height - L.wall_height)
    root.add_group(structures.floor_slab(
        f"{name}_floor_slab",
        x_min - L.wall_thickness, x_max + L.wall_thickness,
        -y_half - L.wall_thickness, y_half + L.wall_thickness,
        slab_thickness, 0.0, floor_color,
    ))

    # ---- exterior walls, with one window per office bay ------------------
    window_centers = [(i + 0.5) * L.office_width for i in range(L.n_offices_per_side)]

    north_doors = []
    south_doors = []
    north_window_centers = window_centers
    south_window_centers = window_centers
    if include_vertical_circulation:
        middle_center = (middle_slot + 0.5) * L.office_width
        # The elevator shaft rises straight up, so its doorway sits at
        # the same x on every level -- always exactly one door.
        north_doors = [(middle_center, ELEVATOR_DOOR_WIDTH)]
        # The staircase's second flight lands offset toward -x from
        # where it started (door_side=NORTH mirrors the sideways shift --
        # see models/circulation.py's Side.NORTH rotation), and every
        # flight uses that exact same offset -- so a level can need a
        # departure doorway (this level's own flight going up, at
        # middle_center) and/or an arrival doorway (the flight coming up
        # from below, landing STAIR_OFFSET further along) independently.
        if build_staircase:
            south_doors.append((middle_center, STAIR_DOOR_WIDTH))
        if has_incoming_staircase:
            south_doors.append((middle_center - STAIR_OFFSET, STAIR_DOOR_WIDTH))
        north_window_centers = [c for c in window_centers if abs(c - middle_center) > 1e-6]
        south_window_centers = north_window_centers

    root.add_group(structures.wall_with_windows(
        f"{name}_ext_wall_north", "x", y_half + L.wall_thickness / 2.0, x_min, x_max,
        L.wall_thickness, L.wall_height, wall_color, north_window_centers,
        window_width=L.window_width, window_height=L.window_height,
        window_color=L.window_color, window_face_sign=-1,
        doors=north_doors,
    ))
    root.add_group(structures.wall_with_windows(
        f"{name}_ext_wall_south", "x", -y_half - L.wall_thickness / 2.0, x_min, x_max,
        L.wall_thickness, L.wall_height, wall_color, south_window_centers,
        window_width=L.window_width, window_height=L.window_height,
        window_color=L.window_color, window_face_sign=1,
        doors=south_doors,
    ))

    # West/east walls are shorter and have no per-office rhythm to key
    # windows off of, so they just get two, symmetric about the corridor
    # centerline -- windows all the way around the building, not just the
    # long north/south runs.
    end_window_centers = [y_half / 2.0, -y_half / 2.0]

    # West exterior wall (front of building): the main entrance gap, but
    # only on a level that actually has one (see has_main_entrance).
    west_doors = [(0.0, L.entrance_width)] if has_main_entrance else None
    root.add_group(structures.wall_with_windows(
        f"{name}_ext_wall_west", "y", x_min - L.wall_thickness / 2.0,
        -y_half - L.wall_thickness, y_half + L.wall_thickness,
        L.wall_thickness, L.wall_height, wall_color, end_window_centers,
        window_width=L.window_width, window_height=L.window_height,
        window_color=L.window_color, window_face_sign=1,
        doors=west_doors,
    ))

    # East exterior wall (back of building), no door.
    root.add_group(structures.wall_with_windows(
        f"{name}_ext_wall_east", "y", x_max + L.wall_thickness / 2.0,
        -y_half - L.wall_thickness, y_half + L.wall_thickness,
        L.wall_thickness, L.wall_height, wall_color, end_window_centers,
        window_width=L.window_width, window_height=L.window_height,
        window_color=L.window_color, window_face_sign=-1,
    ))

    # ---- corridor walls (with a door per office) -------------------------
    # The middle slot on both rows skips its corridor-facing door/wall
    # entirely when circulation is enabled -- fully open, nothing to
    # navigate around, since neither the elevator bay nor the stair
    # doorway is a room that needs one.
    for i in range(L.n_offices_per_side):
        x0 = i * L.office_width
        x1 = (i + 1) * L.office_width
        door_x = (x0 + x1) / 2.0
        if not (include_vertical_circulation and i == middle_slot):
            root.add_group(structures.wall_with_door(f"{name}_corridor_wall_n_{i}", "x", corridor_y_north, x0, x1, door_x, L.door_width, L.wall_thickness, L.wall_height, wall_color))
            root.add_group(structures.wall_with_door(f"{name}_corridor_wall_s_{i}", "x", corridor_y_south, x0, x1, door_x, L.door_width, L.wall_thickness, L.wall_height, wall_color))

    # ---- partition walls between offices ---------------------------------
    for i in range(L.n_offices_per_side + 1):
        x = i * L.office_width
        root.add_group(structures.wall(f"{name}_partition_n_{i}", "y", x, corridor_y_north, y_half, L.wall_thickness, L.wall_height, wall_color))
        root.add_group(structures.wall(f"{name}_partition_s_{i}", "y", x, -y_half, corridor_y_south, L.wall_thickness, L.wall_height, wall_color))

    # ---- rooms ------------------------------------------------------------
    cabinet_wall_x = L.office_width / 2.0 - L.wall_thickness / 2.0     # partition wall's inner face
    near_wall_y = L.wall_thickness / 2.0 - L.office_depth / 2.0        # corridor (door-side) wall's inner face, room-center-relative

    # Each row's rooms open onto the corridor -- north-row rooms have
    # their door on the south side (toward the corridor below them),
    # south-row rooms on the north side (toward the corridor above them).
    # row_sign only places the room in the floor (which side of the
    # corridor); door_side is a separate, independent concern -- which
    # way its furniture is rotated to face -- that room_compositions.py
    # resolves via models.sides.room_rotation_yaw().
    for i in range(L.n_offices_per_side):
        room_x = (i + 0.5) * L.office_width
        for row_sign, door_side, row_name in ((1, Side.SOUTH, "n"), (-1, Side.NORTH, "s")):
            room_center = (room_x, row_sign * (L.corridor_width / 2.0 + L.office_depth / 2.0), 0.0, 0, 0, 0)
            is_meeting = row_name == "n" and i == meeting_slot
            is_break = row_name == "s" and i == break_slot
            is_elevator_bay = include_vertical_circulation and row_name == "n" and i == middle_slot
            is_stair_bay = include_vertical_circulation and row_name == "s" and i == middle_slot

            if is_elevator_bay or is_stair_bay:
                pass    # nothing interior here -- just an open bay leading to the exterior doorway
            elif is_meeting:
                root.add_group(meeting_room_group(f"{name}_meeting_room", room_center, door_side, wall_height=L.wall_height))
            elif is_break:
                root.add_group(break_room_group(f"{name}_break_room", room_center, door_side, wall_height=L.wall_height))
            else:
                prefix = f"{name}_office_{row_name}_{i}"
                root.add_group(office_room_group(
                    prefix, room_center, door_side,
                    wall_height=L.wall_height,
                    cabinet_wall_x=cabinet_wall_x,
                    near_wall_y=near_wall_y,
                    include_plant=(rng.random() < L.plant_probability),
                    include_cabinet=(rng.random() < L.cabinet_probability),
                ))

    # ---- exterior elevator + staircase -----------------------------------
    # Both sit entirely outside the building, each reached through its
    # own doorway rather than an interior opening. The elevator's door
    # faces south (back toward the building, from its position north of
    # the north wall); the staircase's faces north (back toward the
    # building, from south of the south wall), so each rises away from
    # the building it's attached to.
    if include_vertical_circulation and build_elevator:
        shaft_height = elevator_shaft_height if elevator_shaft_height is not None else floor_to_floor_height
        root.add_group(circulation.elevator(
            f"{name}_elevator", (middle_center, y_half + L.wall_thickness, 0.0), Side.SOUTH,
            shaft_height=shaft_height, start_offset=0.1,
        ))
    if include_vertical_circulation and build_staircase:
        door_pose = (middle_center, -y_half - L.wall_thickness, 0.0)
        stair_group, _ = circulation.staircase(
            f"{name}_staircase", door_pose, Side.NORTH,
            rise_height=floor_to_floor_height, width=STAIR_WIDTH, offset=STAIR_OFFSET,
        )
        root.add_group(stair_group)

    # ---- lobby --------------------------------------------------------
    # The lobby is the entrance's reception area -- a level with no front
    # door (has_main_entrance=False) has no reason for one either; that
    # west-end bay is just left as bare enclosed floor space.
    if has_main_entrance:
        lobby_x = x_min / 2.0
        # reception_desk's public side (nameplate side) faces +y at yaw=0;
        # facing_yaw(entrance_side) turns that side to face the entrance.
        # Waiting chairs sit between the entrance and the desk, so they face
        # the opposite way -- toward the desk, away from the entrance.
        reception_yaw = facing_yaw(L.entrance_side)
        chair_yaw = facing_yaw(opposite(L.entrance_side))
        root.add_group(lobby_group(
            f"{name}_lobby",
            reception_pose=(lobby_x + 1.0, 3.2, 0.0, 0, 0, reception_yaw),
            chair_poses=(
                (lobby_x - 1.8, 3.5, 0.0, 0, 0, chair_yaw),
                (lobby_x - 1.8, 2.5, 0.0, 0, 0, chair_yaw),
            ),
            plant_poses=(
                (x_min + 0.6, y_half - 0.6, 0.0, 0, 0, 0),
                (x_min + 0.6, -y_half + 0.6, 0.0, 0, 0, 0),
            ),
            light_poses=(
                (lobby_x, 1.5, L.wall_height - 0.05, 0, 0, 0),
                (lobby_x, -1.5, L.wall_height - 0.05, 0, 0, 0),
            ),
        ))

    return root, footprint_width(L)
