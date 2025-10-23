from __future__ import annotations

from enum import IntEnum
from dataclasses import dataclass
from typing import (
    Optional,
    Tuple,
    ClassVar,
    Protocol,
    Type,
)

@dataclass(frozen=True)
class ControlField:
    address: int
    size: int
    initial_value: Optional[int]


# Class to represent a Dynamixel motor with shared specs for specific model
class DynamixelMotor(Protocol):
    EEPROM: ClassVar[Type]
    RAM:    ClassVar[Type]

    id: int

    def __init__(self, id: int): ...