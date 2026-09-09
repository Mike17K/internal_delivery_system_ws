"""Primitive geometry, modeled as Model subclasses (base.py).

These cover the parts of the building that can't come from a single
fixed-size catalog model (models.py) -- walls of arbitrary length, the
floor slab, and window accents. Box and Window reuse Model's `id`,
`dimensions` (as their box size) and `pose`, but override to_sdf() to
render inline <collision>/<visual> geometry embedded directly in the
office_floor model's own structural link, rather than an <include> --
there's no separate model file behind a wall segment's geometry, so `uri`
and `is_static` are unused placeholders here.

basic_structures.py composes these into higher-level wall-building blocks.
"""

from typing import Tuple

from .base import Model, Pose

Size = Tuple[float, float, float]
Color = Tuple[float, float, float]


def _pose_str(pose: Pose) -> str:
    x, y, z, roll, pitch, yaw = pose
    return f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"


class Box(Model):
    """A solid, axis-aligned box: one <collision> plus one opaque
    <visual>."""

    def __init__(self, name: str, pose: Pose, size: Size, color: Color, collision: bool = True):
        super().__init__(id=name, uri="", dimensions=size, is_static=True, pose=pose)
        self.color = color
        self.collision = collision

    def to_sdf(self) -> str:
        pose_str = _pose_str(self.pose)
        sx, sy, sz = self.dimensions
        r, g, b = self.color
        parts = []
        if self.collision:
            parts.append(f"""
      <collision name="{self.id}_collision">
        <pose>{pose_str}</pose>
        <geometry>
          <box>
            <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
          </box>
        </geometry>
      </collision>""")
        parts.append(f"""
      <visual name="{self.id}_visual">
        <pose>{pose_str}</pose>
        <geometry>
          <box>
            <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.2 0.2 0.2 1</specular>
        </material>
      </visual>""")
        return "".join(parts)


class Window(Model):
    """A visual-only tinted glass accent (no collision -- whatever solid
    wall it's embedded in already provides the physics)."""

    def __init__(self, name: str, pose: Pose, size: Size, color: Color, transparency: float = 0.55):
        super().__init__(id=name, uri="", dimensions=size, is_static=True, pose=pose)
        self.color = color
        self.transparency = transparency

    def to_sdf(self) -> str:
        pose_str = _pose_str(self.pose)
        sx, sy, sz = self.dimensions
        r, g, b = self.color
        return f"""
      <visual name="{self.id}_visual">
        <pose>{pose_str}</pose>
        <geometry>
          <box>
            <size>{sx:.4f} {sy:.4f} {sz:.4f}</size>
          </box>
        </geometry>
        <material>
          <ambient>{r} {g} {b} 1</ambient>
          <diffuse>{r} {g} {b} 1</diffuse>
          <specular>0.8 0.8 0.8 1</specular>
        </material>
        <transparency>{self.transparency:.4f}</transparency>
      </visual>"""
