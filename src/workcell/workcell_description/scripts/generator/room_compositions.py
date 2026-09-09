"""Room-level furniture compositions.

Each function here builds one room's furniture as a Group (models/base.py)
out of the reusable catalog assets in models/models.py. They know nothing
about any specific building's layout constants (office width, corridor
width, ...) -- callers pass in whatever geometry actually matters for a
room (wall height, clearance off a neighboring wall, seat layout, ...), so
the same composition can be reused for a different floor plan, or handed
different numbers for a variation of the "same" room.

Every room (except the lobby -- see below) is authored once in a
canonical frame where its door/entry is on the SOUTH side (local -y,
models/sides.py), then rotated as a whole via `door_side` so the door can
end up on any of the 4 cardinal sides instead: the caller doesn't need to
know or care which way the room "actually" faces, only where its door
really is in the building's walls, and every piece of furniture inside
follows along, position and orientation both. A room's door_side is
therefore fully changeable per call -- pass a different Side and the
whole room's layout re-orients around it.

Furniture-arrangement offsets (how far the desk sits from the door, how
tight the seats hug the table, ...) have sensible module-level defaults
below but are all overridable per call.
"""

from models.base import Group
from models.models import (
    BreakCounter,
    Chair,
    CeilingLight,
    MeetingTable,
    OfficeCabinet,
    OfficeDesk,
    PottedPlant,
    ReceptionDesk,
)
from models.sides import Side, room_pose

# ---------------------------------------------------------------------------
# Default furniture-arrangement offsets (all overridable per call below)
# ---------------------------------------------------------------------------

OFFICE_DESK_X_OFFSET = -1.0
OFFICE_DESK_FAR_OFFSET = 0.9      # desk sits this far from room center, toward the far wall (opposite the door)
OFFICE_CHAIR_NEAR_OFFSET = 0.1    # chair sits this far from room center, toward the far wall (between center and desk)
OFFICE_CABINET_CLEARANCE = 0.55   # cabinet is 0.45m deep once rotated + 0.1 margin off the door-side wall
OFFICE_PLANT_CLEARANCE = 0.5      # foliage radius 0.35 + 0.15 margin off the door-side wall
OFFICE_PLANT_X_OFFSET = -1.5

DEFAULT_MEETING_SEATS = (
    (-0.8, 0.4), (0.0, 0.4), (0.8, 0.4),
    (-0.8, -0.4), (0.0, -0.4), (0.8, -0.4),
)

BREAK_COUNTER_Y_OFFSET = 1.3
BREAK_CHAIR_X_OFFSETS = (-1.0, 1.0)
BREAK_CHAIR_Y_OFFSET = -0.5
BREAK_PLANT_POSE = (1.6, -1.5)


def office_room_group(
    prefix: str,
    room_center_world: tuple,
    door_side: Side,
    *,
    wall_height: float,
    cabinet_wall_x: float,
    near_wall_y: float,
    include_plant: bool = False,
    include_cabinet: bool = True,
    desk_x_offset: float = OFFICE_DESK_X_OFFSET,
    desk_far_offset: float = OFFICE_DESK_FAR_OFFSET,
    chair_near_offset: float = OFFICE_CHAIR_NEAR_OFFSET,
    cabinet_clearance: float = OFFICE_CABINET_CLEARANCE,
    plant_clearance: float = OFFICE_PLANT_CLEARANCE,
    plant_x_offset: float = OFFICE_PLANT_X_OFFSET,
) -> Group:
    """A regular office's furniture (desk, chair, cabinet, ceiling light,
    optional plant), authored in the canonical door=SOUTH frame and
    rotated by `door_side` (models/sides.py) to match wherever the room's
    actual doorway is.

    In that canonical frame: the desk sits against the far wall (north,
    opposite the door) with the chair between it and the door, facing the
    desk; the cabinet sits flush against the room's east wall (its origin
    is its wall-flush back face, opposite the drawer handles), still
    needing its own fixed -90 deg turn to open into the room regardless of
    `door_side`. `cabinet_wall_x`/`near_wall_y` are that canonical frame's
    local x/y of the east wall and the door-side (south) wall's inner
    faces; furniture near either is offset from it by its own footprint
    plus a margin, so it never clips through.
    """
    g = Group(id=prefix, pose=room_pose(room_center_world, door_side))

    # office_desk/office_chair face +y (north, away from the door) at
    # yaw=0 in the canonical frame.
    g.add_model(OfficeDesk(f"{prefix}_desk", pose=(desk_x_offset, desk_far_offset, 0.0, 0, 0, 0.0)))
    g.add_model(Chair(f"{prefix}_chair", pose=(desk_x_offset, chair_near_offset, 0.0, 0, 0, 0.0)))

    if include_cabinet:
        cab_y = near_wall_y + cabinet_clearance
        g.add_model(OfficeCabinet(f"{prefix}_cabinet", pose=(cabinet_wall_x, cab_y, 0.0, 0, 0, -1.5708)))

    g.add_model(CeilingLight(f"{prefix}_light", pose=(0.0, 0.0, wall_height - 0.05, 0, 0, 0)))

    if include_plant:
        plant_y = near_wall_y + plant_clearance
        g.add_model(PottedPlant(f"{prefix}_plant", pose=(plant_x_offset, plant_y, 0.0, 0, 0, 0)))

    return g


def meeting_room_group(
    prefix: str,
    room_center_world: tuple,
    door_side: Side,
    *,
    wall_height: float,
    seats: tuple = DEFAULT_MEETING_SEATS,
) -> Group:
    """A conference table with `seats` chairs (local (dx, dy) offsets from
    the table's own center, in the canonical door=SOUTH frame) around it,
    plus a ceiling light, rotated by `door_side` to match the room's
    actual doorway.

    Seats on the +y side of the table face -y (yaw=pi) to look at it, and
    vice versa for the -y side -- that's relative to the table, not the
    door, so it doesn't change with `door_side`.
    """
    g = Group(id=prefix, pose=room_pose(room_center_world, door_side))
    g.add_model(MeetingTable(f"{prefix}_table", pose=(0.0, 0.0, 0.0, 0, 0, 0)))

    for j, (dx, dy) in enumerate(seats):
        yaw = 3.14159 if dy > 0 else 0.0
        g.add_model(Chair(f"{prefix}_chair_{j}", pose=(dx, dy, 0.0, 0, 0, yaw)))

    g.add_model(CeilingLight(f"{prefix}_light", pose=(0.0, 0.0, wall_height - 0.05, 0, 0, 0)))
    return g


def break_room_group(
    prefix: str,
    room_center_world: tuple,
    door_side: Side,
    *,
    wall_height: float,
    counter_y_offset: float = BREAK_COUNTER_Y_OFFSET,
    chair_x_offsets: tuple = BREAK_CHAIR_X_OFFSETS,
    chair_y_offset: float = BREAK_CHAIR_Y_OFFSET,
    plant_pose: tuple = BREAK_PLANT_POSE,
) -> Group:
    """A kitchenette counter, chairs (one per `chair_x_offsets` entry), a
    plant, and a ceiling light, authored in the canonical door=SOUTH frame
    and rotated by `door_side` to match the room's actual doorway.

    In that frame, the counter sits flush against the far (north) wall,
    front facing back toward the room center (yaw=0), and the chairs sit
    closer to the door, facing the counter (yaw=pi).
    """
    g = Group(id=prefix, pose=room_pose(room_center_world, door_side))

    g.add_model(BreakCounter(f"{prefix}_counter", pose=(0.0, counter_y_offset, 0.0, 0, 0, 0.0)))
    for j, dx in enumerate(chair_x_offsets):
        g.add_model(Chair(f"{prefix}_chair_{j}", pose=(dx, chair_y_offset, 0.0, 0, 0, 3.14159)))
    g.add_model(PottedPlant(f"{prefix}_plant", pose=(plant_pose[0], plant_pose[1], 0.0, 0, 0, 0)))
    g.add_model(CeilingLight(f"{prefix}_light", pose=(0.0, 0.0, wall_height - 0.05, 0, 0, 0)))
    return g


def lobby_group(
    id: str,
    *,
    reception_pose: tuple,
    chair_poses: tuple,
    plant_poses: tuple,
    light_poses: tuple,
) -> Group:
    """A reception desk, waiting chairs, plants, and ceiling lights, all
    given as explicit world poses -- unlike the rooms above, a lobby is
    inherently tied to one specific building's main entrance, so it takes
    final poses directly (the caller can build `reception_pose` with
    models.sides.facing_yaw(entrance_side) to keep it consistent with
    whichever side the entrance is actually on) rather than a
    canonical-frame + door_side. This Group's own pose is the identity;
    it exists purely for id namespacing/organization.
    """
    g = Group(id=id)
    g.add_model(ReceptionDesk(f"{id}_reception_desk", pose=reception_pose))
    for j, pose in enumerate(chair_poses):
        g.add_model(Chair(f"{id}_chair_{j}", pose=pose))
    for j, pose in enumerate(plant_poses):
        g.add_model(PottedPlant(f"{id}_plant_{j}", pose=pose))
    for j, pose in enumerate(light_poses):
        g.add_model(CeilingLight(f"{id}_light_{j}", pose=pose))
    return g
