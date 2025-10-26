from enum import Enum, IntEnum


class Position:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class ImagePosition:
    def __init__(self, x: float, y: float):
        self.x = x
        self.y = y

class Vector:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class ObjectType(IntEnum):
    TARGET_BALL = 0
    BALL = 1
    CUP = 2
    DUCK_BLUE = 3
    DUCK_RED = 4
    DUCK_YELLOW = 5
    DUCK_GREEN = 6
    DUCK_WHITE = 7

class HandState(Enum):
    HAND_OPEN = 0
    HAND_CLOSED = 1
    HAND_GRABBED = 2

class Detection:
    def __init__(self, position: Position, image_position: ImagePosition, object_type: ObjectType, object_id: int):
        self.position = position
        self.image_position = image_position
        self.object_type = object_type
        self.object_id = object_id
