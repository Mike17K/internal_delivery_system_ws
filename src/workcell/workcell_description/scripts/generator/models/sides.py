"""Cardinal side/orientation helpers shared by room_compositions.py and
floors.py.

Most catalog furniture (office_chair, office_desk, ...) faces +y at
yaw=0. facing_yaw(side) gives the yaw that makes such a model face
`side` instead, and room_rotation_yaw(door_side) gives the yaw needed to
rotate a whole room template -- authored assuming its door/entry sits on
the canonical side (SOUTH, i.e. local -y) -- so the door ends up on any
other side instead. Rotating the room's Group by that one yaw carries
every piece of furniture in it along, rotation and all, so a room's
layout only ever needs to be authored once, in the canonical frame.
"""

from enum import Enum


class Side(Enum):
    NORTH = "north"   # +y
    EAST = "east"     # +x
    SOUTH = "south"   # -y
    WEST = "west"     # -x


_FACING_YAW = {
    Side.NORTH: 0.0,
    Side.WEST: 1.5708,
    Side.SOUTH: 3.14159,
    Side.EAST: -1.5708,
}

_OPPOSITE = {
    Side.NORTH: Side.SOUTH,
    Side.SOUTH: Side.NORTH,
    Side.EAST: Side.WEST,
    Side.WEST: Side.EAST,
}

CANONICAL_DOOR_SIDE = Side.SOUTH


def facing_yaw(side: Side) -> float:
    """Yaw that makes a "faces +y at yaw=0" model (e.g. office_chair,
    office_desk) face toward `side` instead."""
    return _FACING_YAW[side]


def opposite(side: Side) -> Side:
    return _OPPOSITE[side]


def room_rotation_yaw(door_side: Side, canonical_door_side: Side = CANONICAL_DOOR_SIDE) -> float:
    """Yaw to rotate a room template -- authored with its door on
    `canonical_door_side` -- so the door ends up on `door_side` instead."""
    return facing_yaw(door_side) - facing_yaw(canonical_door_side)


def room_pose(room_center_world: tuple, door_side: Side) -> tuple:
    """The Group pose for a room centered at `room_center_world` (x, y[,
    z]) with its door on `door_side`: translation, plus the rotation that
    reorients a canonical (door=SOUTH) layout to match. Shared by any room
    template authored in that canonical frame (room_compositions.py,
    circulation.py)."""
    x, y = room_center_world[0], room_center_world[1]
    z = room_center_world[2] if len(room_center_world) > 2 else 0.0
    return (x, y, z, 0, 0, room_rotation_yaw(door_side))
