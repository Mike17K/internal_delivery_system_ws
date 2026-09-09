from .base import Model, Group, Pose

class Chair(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://office_chair",
            dimensions=(0.5, 0.5, 1.0),
            is_static=True,
            pose=pose,
        )

class ReceptionDesk(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://reception_desk",
            dimensions=(2.0, 1.0, 1.0),
            is_static=True,
            pose=pose,
        )

class OfficeDesk(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://office_desk",
            dimensions=(1.4, 0.7, 0.8),
            is_static=True,
            pose=pose,
        )

class OfficeCabinet(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://office_cabinet",
            dimensions=(0.9, 0.45, 1.8),
            is_static=True,
            pose=pose,
        )

class PottedPlant(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://potted_plant",
            dimensions=(0.7, 0.7, 1.0),
            is_static=True,
            pose=pose,
        )

class CeilingLight(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://ceiling_light",
            dimensions=(0.6, 0.6, 0.1),
            is_static=True,
            pose=pose,
        )

class MeetingTable(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://meeting_table",
            dimensions=(2.4, 1.1, 0.8),
            is_static=True,
            pose=pose,
        )

class BreakCounter(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://break_counter",
            dimensions=(2.7, 0.7, 1.8),
            is_static=True,
            pose=pose,
        )

class Wall(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://wall",
            dimensions=(10.0, 0.2, 2.5),
            is_static=True,
            pose=pose,
        )

class Floor(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://floor",
            dimensions=(20.0, 20.0, 0.1),
            is_static=True,
            pose=pose,
        )

class Table(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://table",
            dimensions=(2.0, 1.0, 1.0),
            is_static=True,
            pose=pose,
        )

class Shelf(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://shelf",
            dimensions=(2.5, 0.9, 3.0),
            is_static=False,
            pose=pose,
        )

class Pallet(Model):
    def __init__(self, id: str, pose: Pose = (0, 0, 0, 0, 0, 0)):
        super().__init__(
            id=id,
            uri="model://pallet",
            dimensions=(1.2, 0.8, 0.2),
            is_static=True,
            pose=pose,
        )