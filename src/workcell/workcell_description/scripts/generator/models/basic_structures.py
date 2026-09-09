"""Parametric, scalable wall-building blocks.

These compose the primitive shape models (shapes.py: Box, Window) into
full wall runs -- solid, with a doorway gap, or with one or more window
accents -- of any length/axis/thickness/height. Each function returns a
Group (base.py) posed at the run's own location, with its Box/Window
children at local offsets along the run -- so it composes naturally with
the rest of the Group hierarchy and only needs resolve() at the end to
get world-posed geometry.
"""

from typing import Optional, Sequence, Tuple

from .base import Group
from .shapes import Box, Color, Window

Hole = Tuple[float, float, float, float]    # (x0, x1, y0, y1)


def floor_slab(
    name: str, x_min: float, x_max: float, y_min: float, y_max: float,
    thickness: float, z_top: float, color: Color,
    holes: Optional[Sequence[Hole]] = None,
) -> Group:
    """A flat slab covering (x_min, x_max) x (y_min, y_max), its top face
    at `z_top` -- normally a single box, but with one box removed and
    re-tiled around each axis-aligned rectangle in `holes` (e.g. an
    opening above a staircase/ramp/elevator on the floor below). Holes
    must not overlap each other and must lie within the slab's bounds.
    """
    cz = z_top - thickness / 2.0

    if not holes:
        g = Group(id=name)
        g.add_model(Box(name, ((x_min + x_max) / 2.0, (y_min + y_max) / 2.0, cz, 0, 0, 0),
                         (x_max - x_min, y_max - y_min, thickness), color))
        return g

    # Band the slab by every hole's y0/y1 (plus the outer bounds), then
    # within each band subtract whichever holes are active there from the
    # full x-span, tiling the remaining x-segments.
    ys = sorted({y_min, y_max} | {h[2] for h in holes} | {h[3] for h in holes})
    g = Group(id=name)
    tile = 0
    for y0, y1 in zip(ys[:-1], ys[1:]):
        if y1 - y0 < 1e-9:
            continue
        y_mid = (y0 + y1) / 2.0
        band_holes = [h for h in holes if h[2] <= y_mid <= h[3]]

        segments = [(x_min, x_max)]
        for hx0, hx1, _, _ in band_holes:
            new_segments = []
            for s0, s1 in segments:
                if hx1 <= s0 or hx0 >= s1:
                    new_segments.append((s0, s1))
                    continue
                if hx0 > s0:
                    new_segments.append((s0, hx0))
                if hx1 < s1:
                    new_segments.append((hx1, s1))
            segments = new_segments

        for sx0, sx1 in segments:
            if sx1 - sx0 < 1e-9:
                continue
            g.add_model(Box(f"{name}_tile{tile}", ((sx0 + sx1) / 2.0, y_mid, cz, 0, 0, 0),
                             (sx1 - sx0, y1 - y0, thickness), color))
            tile += 1

    return g


def wall(name: str, axis: str, fixed_coord: float, start: float, end: float,
         thickness: float, height: float, color: Color) -> Group:
    """A single solid wall box running along `axis` ('x' or 'y') from
    `start` to `end`, at `fixed_coord` on the other axis. Scales to any
    length by construction (length = end - start)."""
    length = end - start
    center = (start + end) / 2.0
    if axis == "x":
        group_pose = (center, fixed_coord, 0.0, 0, 0, 0)
        size = (length, thickness, height)
    else:
        group_pose = (fixed_coord, center, 0.0, 0, 0, 0)
        size = (thickness, length, height)

    g = Group(id=name, pose=group_pose)
    g.add_model(Box(name, (0.0, 0.0, height / 2.0, 0, 0, 0), size, color))
    return g


def wall_with_door(name: str, axis: str, fixed_coord: float, start: float, end: float,
                    door_center: float, door_width: float, thickness: float,
                    height: float, color: Color) -> Group:
    """Like `wall`, but leaves a doorway gap of `door_width` centered at
    `door_center`, emitted as up to two flanking segments. Scales to any
    wall length or door position/width."""
    run_center = (start + end) / 2.0
    group_pose = (run_center, fixed_coord, 0.0, 0, 0, 0) if axis == "x" else (fixed_coord, run_center, 0.0, 0, 0, 0)
    g = Group(id=name, pose=group_pose)

    gap_lo = door_center - door_width / 2.0
    gap_hi = door_center + door_width / 2.0
    segments = []
    if gap_lo > start:
        segments.append((start, gap_lo))
    if gap_hi < end:
        segments.append((gap_hi, end))

    for i, (s, e) in enumerate(segments):
        seg_center = (s + e) / 2.0
        seg_length = e - s
        local_offset = seg_center - run_center    # position relative to the run's center, along the run's axis
        if axis == "x":
            local_pose = (local_offset, 0.0, height / 2.0, 0, 0, 0)
            size = (seg_length, thickness, height)
        else:
            local_pose = (0.0, local_offset, height / 2.0, 0, 0, 0)
            size = (thickness, seg_length, height)
        g.add_model(Box(f"{name}_seg{i}", local_pose, size, color))

    return g


def wall_with_windows(
    name: str, axis: str, fixed_coord: float, start: float, end: float,
    thickness: float, height: float, color: Color,
    window_centers: Sequence[float],
    *,
    window_width: float = 1.8,
    window_height: float = 1.2,
    window_thickness: float = 0.05,
    window_color: Color = (0.6, 0.75, 0.85),
    window_z: Optional[float] = None,
    window_face_sign: int = -1,
    doors: Optional[Sequence[Tuple[float, float]]] = None,
) -> Group:
    """A solid wall run (like `wall`) with a tinted glass window accent
    flush against one of its faces at each position in `window_centers`,
    and optionally one or more doorway gaps -- `doors` is a list of
    (center, width) pairs (like `wall_with_door`, but window accents and
    doors can coexist on the same run, and a run can have more than one
    door -- e.g. a middle floor's exterior wall needing both an incoming
    and an outgoing staircase doorway). `window_centers` should simply
    omit whichever positions the doors sit at. Scales to any wall length,
    any number of windows, and any number of non-overlapping doors.

    `window_face_sign` picks which face the windows sit flush with:
    -1 for the face at `fixed_coord - thickness/2` (e.g. a wall whose
    building interior is on the -x/-y side), +1 for the other face.
    `window_z` defaults to the wall's vertical center.
    """
    run_center = (start + end) / 2.0
    length = end - start
    if axis == "x":
        group_pose = (run_center, fixed_coord, 0.0, 0, 0, 0)
    else:
        group_pose = (fixed_coord, run_center, 0.0, 0, 0, 0)

    g = Group(id=name, pose=group_pose)

    if not doors:
        wall_size = (length, thickness, height) if axis == "x" else (thickness, length, height)
        g.add_model(Box(name, (0.0, 0.0, height / 2.0, 0, 0, 0), wall_size, color))
    else:
        gaps = sorted((c - w / 2.0, c + w / 2.0) for c, w in doors)
        segments = [(start, end)]
        for glo, ghi in gaps:
            new_segments = []
            for s0, s1 in segments:
                if ghi <= s0 or glo >= s1:
                    new_segments.append((s0, s1))
                    continue
                if glo > s0:
                    new_segments.append((s0, glo))
                if ghi < s1:
                    new_segments.append((ghi, s1))
            segments = new_segments
        for i, (s, e) in enumerate(segments):
            if e - s < 1e-9:
                continue
            seg_center = (s + e) / 2.0
            seg_length = e - s
            local_offset = seg_center - run_center
            if axis == "x":
                seg_pose = (local_offset, 0.0, height / 2.0, 0, 0, 0)
                seg_size = (seg_length, thickness, height)
            else:
                seg_pose = (0.0, local_offset, height / 2.0, 0, 0, 0)
                seg_size = (thickness, seg_length, height)
            g.add_model(Box(f"{name}_seg{i}", seg_pose, seg_size, color))

    if window_z is None:
        window_z = height / 2.0
    face_offset = window_face_sign * (thickness / 2.0)

    for i, center in enumerate(window_centers):
        local_offset = center - run_center
        if axis == "x":
            win_pose = (local_offset, face_offset, window_z, 0, 0, 0)
            win_size = (window_width, window_thickness, window_height)
        else:
            win_pose = (face_offset, local_offset, window_z, 0, 0, 0)
            win_size = (window_thickness, window_width, window_height)
        g.add_model(Window(f"{name}_window_{i}", win_pose, win_size, window_color))

    return g
