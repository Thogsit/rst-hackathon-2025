# This module provides utility functions for working with specific Dynamixel units.

import numpy as np
import struct

from enum import Enum

class Unit(Enum):
    DXL     = 0

    RAD     = 1
    DEG     = 2

    RPM     = 3
    RAD_S   = 4



def to_dxl_units(value: float, from_unit: Unit) -> int:
    if from_unit == Unit.DXL:
        return int(value)
    
    elif from_unit == Unit.DEG:
        return int(value * (4096 / 360))
    
    elif from_unit == Unit.RAD:
        return int(value * (4096 / (2 * np.pi)))
    
    elif from_unit == Unit.RPM:
        return int(value / 0.229)

    elif from_unit == Unit.RAD_S:
        return int((value * (60 / (2*np.pi))) / 0.229)

    else:
        raise ValueError(f"Unsupported input unit: {from_unit}")

def from_dxl_units(value: float, to_unit: Unit) -> float:
    if to_unit == Unit.DXL:
        return value
    
    elif to_unit == Unit.DEG:
        return value / (4096 / 360)
    
    elif to_unit == Unit.RAD:
        return value / (4096 / (2 * np.pi))
    
    elif to_unit == Unit.RPM:
        return value * 0.229
    
    elif to_unit == Unit.RAD_S:
        return (value * 0.229) * (2*np.pi / 60)

    else:
        raise ValueError(f"Unsupported output unit: {to_unit}")

def convert_angle(value: float, from_unit: Unit, to_unit: Unit) -> float:
    dxl = to_dxl_units(value, from_unit)
    return from_dxl_units(dxl, to_unit)

# conver signed int to its unsigned 32-bit representation
def to_u32(x):
    return x & 0xFFFFFFFF

def pack_i32_le(value: int) -> bytes:
    return struct.pack('<i', value)

def unpack_i32_le(raw: int | bytes | bytearray) -> int:
    if isinstance(raw, int):
        raw_bytes = struct.pack('<I', raw & 0xFFFFFFFF) # ensure 32-bit integer is incoming - eventually not needed, because SDK-outputs should be trusted values
    elif isinstance(raw, (bytes, bytearray)):
        if (len(raw) != 4):
            raise ValueError("Byte sequence must be exactly 4 bytes!") 
        raw_bytes = bytes(raw)

    return struct.unpack('<i', raw_bytes)[0]   


def check_limits(number, limits) -> bool:
    return limits[0] <= number <= limits[1]