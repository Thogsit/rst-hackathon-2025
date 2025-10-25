from enum import Enum


class Position:
    def __init__(self, x: float, y: float, z: float):
        self.x = x
        self.y = y
        self.z = z

class ObjectType(Enum):
    TARGET_BALL = 0
    BALL = 1
    DUCK_YELLOW = 2
    DUCK_BLUE = 3
    DUCK_RED = 4
    DUCK_GREEN = 5
    DUCK_WHITE = 6
    CUP = 7

class HandState(Enum):
    HAND_OPEN = 0
    HAND_CLOSED = 1
    HAND_GRABBED = 2

class Detection:
    def __init__(self, position: Position, object_type: ObjectType, object_id: int):
        self.position = position
        self.object_type = object_type
        self.object_id = object_id
