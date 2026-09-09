"""
bottom left floor is the 0,0,0 in general all items

(+y)
^
|
O - > (+x)

Pose convention: (x, y, z, roll, pitch, yaw), angles in radians.
Rotation order: R = Rz(yaw) @ Ry(pitch) @ Rx(roll)  (intrinsic Z-Y-X / "yaw-pitch-roll")
"""

import copy
from dataclasses import dataclass
from typing import List, Tuple, Optional
import numpy as np

Pose = Tuple[float, float, float, float, float, float]


@dataclass
class Model:
    id: str
    uri: str
    dimensions: Tuple[float, float, float]
    is_static: bool
    pose: Pose = (0, 0, 0, 0, 0, 0)  # x, y, z, roll, pitch, yaw

    def __post_init__(self):
        if not isinstance(self.pose, tuple) or len(self.pose) != 6:
            raise ValueError("Pose must be a 6-tuple of numbers")

    def to_sdf(self) -> str:
        """Render this model as an SDF <include> block, at its own pose."""
        x, y, z, roll, pitch, yaw = self.pose
        pose_str = f"{x:.4f} {y:.4f} {z:.4f} {roll:.4f} {pitch:.4f} {yaw:.4f}"
        return f"""
    <include>
      <uri>{self.uri}</uri>
      <name>{self.id}</name>
      <pose>{pose_str}</pose>
    </include>"""


class Group:
    def __init__(
        self,
        id: str,
        models: Optional[List[Model]] = None,
        groups: Optional[List["Group"]] = None,
        pose: Pose = (0, 0, 0, 0, 0, 0),
    ):
        self.id = id
        # use fresh lists per-instance instead of aliasing whatever the caller passed in
        self.models = list(models) if models is not None else []
        self.groups = list(groups) if groups is not None else []
        self.pose = pose

    def add_model(self, model: Model):
        self.models.append(model)

    def add_group(self, group: "Group"):
        self.groups.append(group)

    def resolve(self, parent_world_pose: Pose = (0, 0, 0, 0, 0, 0)) -> List[Model]:
        resolved_models = []
        combined_pose = self.combine_poses(parent_world_pose, self.pose)

        for model in self.models:
            model_world_pose = self.combine_poses(combined_pose, model.pose)
            # A shallow copy preserves the model's actual subclass (e.g.
            # Box/Window in shapes.py) and any extra attributes it carries
            # beyond Model's own fields (color, collision, ...), along with
            # its overridden to_sdf() -- reconstructing a plain Model here
            # would silently drop all of that.
            resolved = copy.copy(model)
            resolved.pose = model_world_pose
            resolved_models.append(resolved)

        for group in self.groups:
            resolved_models.extend(group.resolve(combined_pose))

        return resolved_models

    # ---------- rotation helpers ----------

    @staticmethod
    def _rotation_matrix(roll: float, pitch: float, yaw: float) -> np.ndarray:
        """Build R = Rz(yaw) @ Ry(pitch) @ Rx(roll)."""
        cr, sr = np.cos(roll), np.sin(roll)
        cp, sp = np.cos(pitch), np.sin(pitch)
        cy, sy = np.cos(yaw), np.sin(yaw)

        Rx = np.array([
            [1, 0, 0],
            [0, cr, -sr],
            [0, sr, cr],
        ])
        Ry = np.array([
            [cp, 0, sp],
            [0, 1, 0],
            [-sp, 0, cp],
        ])
        Rz = np.array([
            [cy, -sy, 0],
            [sy, cy, 0],
            [0, 0, 1],
        ])
        return Rz @ Ry @ Rx

    @staticmethod
    def _euler_from_rotation_matrix(R: np.ndarray) -> Tuple[float, float, float]:
        """Inverse of _rotation_matrix, handling gimbal lock at pitch = +-90 deg."""
        sy = -R[2, 0]
        sy = np.clip(sy, -1.0, 1.0)
        pitch = np.arcsin(sy)

        if np.isclose(np.cos(pitch), 0.0, atol=1e-8):
            # gimbal lock: roll and yaw are coupled, pick yaw = 0
            roll = np.arctan2(-R[1, 2], R[1, 1])
            yaw = 0.0
        else:
            roll = np.arctan2(R[2, 1], R[2, 2])
            yaw = np.arctan2(R[1, 0], R[0, 0])

        return float(roll), float(pitch), float(yaw)

    @classmethod
    def combine_poses(cls, pose1: Pose, pose2: Pose) -> Pose:
        """
        Compose pose2 (child, expressed in pose1's local frame) with pose1
        (parent, expressed in the world/outer frame), returning pose2's
        pose in that same outer frame.

        position_world = t1 + R1 @ t2
        rotation_world  = R1 @ R2
        """
        x1, y1, z1, roll1, pitch1, yaw1 = pose1
        x2, y2, z2, roll2, pitch2, yaw2 = pose2

        t1 = np.array([x1, y1, z1], dtype=float)
        t2 = np.array([x2, y2, z2], dtype=float)

        R1 = cls._rotation_matrix(roll1, pitch1, yaw1)
        R2 = cls._rotation_matrix(roll2, pitch2, yaw2)

        t_world = t1 + R1 @ t2
        R_world = R1 @ R2

        roll, pitch, yaw = cls._euler_from_rotation_matrix(R_world)

        return (
            float(t_world[0]), float(t_world[1]), float(t_world[2]),
            roll, pitch, yaw,
        )