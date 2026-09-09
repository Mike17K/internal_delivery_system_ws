"""Vertical-circulation structures: an exterior switchback staircase, and
a non-functional elevator placeholder.

Unlike office/meeting/break rooms (room_compositions.py), these don't
need any enclosing walls to make sense on their own. The staircase in
particular sits entirely outside the building envelope, the way a real
exterior fire-stair or stairwell tower does -- floors.py just cuts a
doorway for it in the relevant exterior wall, on each level it connects.

Both are procedural (models/shapes.py Box primitives), not fixed-size
catalog models, because their size depends on the actual floor-to-floor
spacing, not a fixed constant -- the same reason walls are procedural
(models/basic_structures.py) rather than catalog assets.

Each is still authored in the canonical door=SOUTH frame and rotated via
`door_side` (models/sides.py): the same room-orientation convention used
throughout room_compositions.py, so either can be oriented to whichever
side of the building it actually needs to attach to.
"""

import math
from typing import Tuple

from .base import Group
from .shapes import Box, Color
from .sides import Side, room_pose


def staircase(
    name: str,
    door_pose: tuple,
    door_side: Side,
    *,
    rise_height: float,
    width: float = 1.3,
    tread_depth: float = 0.28,
    riser_height: float = 0.18,
    max_incline_deg: float = 38.0,
    landing_depth: float = 1.4,
    landing_thickness: float = 0.15,
    offset: float = 1.45,
    color: Color = (0.6, 0.6, 0.62),
) -> Tuple[Group, float]:
    """A real switchback (dogleg) staircase, the way most real buildings
    do it: one flight climbs away from the door to an intermediate
    landing (a "sub-floor"), then a second flight -- shifted sideways by
    `offset` -- climbs the rest of the way back toward the door, ending
    at a doorway offset from the starting one by that same amount rather
    than directly above it.

    Modular in `rise_height`: each flight covers half the total rise, and
    its own step count is chosen so that half divides evenly into steps
    no taller than `riser_height` (so every riser in a flight is the same
    steady height) -- taller floors just get more steps, not taller ones.
    `tread_depth` is widened if needed to keep each flight's incline at
    or below `max_incline_deg`, so the stair is never steeper than a
    normal walkable pitch regardless of how tall `rise_height` is.

    Each step and the landing are thin slabs (`riser_height`/
    `landing_thickness` tall) rather than solid fill down to z=0 -- an
    open stair, not a solid concrete wedge -- which also means a second
    staircase can be stacked directly on top of this one (same door_pose
    x/y, z offset by `rise_height`) for a taller building without its
    mass piling up underneath.

    In a multi-story stack (floors.py/generate.py), every flight reuses
    the exact same `door_pose`-relative shape -- same width, same signed
    `offset` -- floor to floor: the module is never mirrored, rotated, or
    resized between levels. Each flight's own doorway and its landing
    doorway are therefore always offset from each other by that same
    fixed amount, on every floor; the building's walls provide both a
    departure doorway (at this flight's own start) and an arrival doorway
    (`offset` further along) wherever a level actually has both.

    `door_pose` is the doorway this staircase attaches to -- typically
    just outside a building's exterior wall, at the door's own position
    -- and `door_side` which way that door faces (see models/sides.py).
    Ascends away from the door, in the canonical +y direction.

    Returns (group, offset) -- `offset` echoes the sideways shift back,
    in case the caller wants the exact number it used (its default) to
    position the upper doorway to match.
    """
    half_rise = rise_height / 2.0
    n_steps = max(1, round(half_rise / riser_height))
    actual_riser = half_rise / n_steps

    min_tread = actual_riser / math.tan(math.radians(max_incline_deg))
    tread_depth = max(tread_depth, min_tread)
    flight_run = n_steps * tread_depth

    g = Group(id=name, pose=room_pose(door_pose, door_side))

    # Flight 1: climbs away from the door (local +y), centered on x=0.
    # Each step is a thin slab at its own tread height, not solid fill
    # from the ground up.
    y = 0.0
    for i in range(n_steps):
        step_top = (i + 1) * actual_riser
        y_center = y + tread_depth / 2.0
        g.add_model(Box(
            f"{name}_flight1_step{i}",
            (0.0, y_center, step_top - actual_riser / 2.0, 0, 0, 0),
            (width, tread_depth, actual_riser),
            color,
        ))
        y += tread_depth

    # Landing (the "sub-floor" joining the two flights): a thin platform
    # bridging from flight 1's far edge across to flight 2's position.
    # min/max (not a fixed order) so a negative `offset` -- flight 2 on
    # the *other* side of flight 1, used to alternate direction between
    # floors so a multi-story stack stays in one footprint instead of
    # drifting sideways every level -- still gets a valid, correctly
    # spanning landing.
    landing_x0 = min(-width / 2.0, offset - width / 2.0)
    landing_x1 = max(width / 2.0, offset + width / 2.0)
    landing_y0 = flight_run
    landing_y1 = flight_run + landing_depth
    g.add_model(Box(
        f"{name}_landing",
        ((landing_x0 + landing_x1) / 2.0, (landing_y0 + landing_y1) / 2.0, half_rise - landing_thickness / 2.0, 0, 0, 0),
        (landing_x1 - landing_x0, landing_depth, landing_thickness),
        color,
    ))

    # Flight 2: climbs the rest of the way, back toward the door line,
    # offset sideways by `offset` -- its topmost step lands back at
    # local y=0 (the door line), at x=offset, which is where the upper
    # doorway needs to be.
    for i in range(n_steps):
        step_top = half_rise + (i + 1) * actual_riser
        y_far = flight_run - i * tread_depth
        y_near = y_far - tread_depth
        g.add_model(Box(
            f"{name}_flight2_step{i}",
            (offset, (y_far + y_near) / 2.0, step_top - actual_riser / 2.0, 0, 0, 0),
            (width, tread_depth, actual_riser),
            color,
        ))

    return g, offset


def elevator(
    name: str,
    room_center_world: tuple,
    door_side: Side,
    *,
    shaft_height: float,
    start_offset: float = 0.0,
    width: float = 2.0,
    depth: float = 2.0,
    wall_thickness: float = 0.15,
    cabin_height: float = 2.2,
    shaft_color: Color = (0.75, 0.75, 0.78),
    cabin_color: Color = (0.85, 0.8, 0.4),
) -> Group:
    """An elevator shaft (back + two side walls, open on the door side)
    rising the full `shaft_height`, with a stationary cabin parked at
    this floor's level -- a static, non-functional prop: no doors, no
    motion, nothing simulated. The shaft's open (door) face sits at local
    y=`start_offset`, extending `depth` further in +y.
    """
    g = Group(id=name, pose=room_pose(room_center_world, door_side))

    half_w = width / 2.0
    back_y = start_offset + depth - wall_thickness / 2.0
    side_y = start_offset + depth / 2.0
    g.add_model(Box(
        f"{name}_wall_back", (0.0, back_y, shaft_height / 2.0, 0, 0, 0),
        (width, wall_thickness, shaft_height), shaft_color,
    ))
    g.add_model(Box(
        f"{name}_wall_left", (-half_w + wall_thickness / 2.0, side_y, shaft_height / 2.0, 0, 0, 0),
        (wall_thickness, depth, shaft_height), shaft_color,
    ))
    g.add_model(Box(
        f"{name}_wall_right", (half_w - wall_thickness / 2.0, side_y, shaft_height / 2.0, 0, 0, 0),
        (wall_thickness, depth, shaft_height), shaft_color,
    ))

    cabin_margin = 0.1
    g.add_model(Box(
        f"{name}_cabin", (0.0, side_y, cabin_height / 2.0, 0, 0, 0),
        (width - 2 * cabin_margin, depth - wall_thickness - cabin_margin, cabin_height),
        cabin_color,
    ))

    return g
