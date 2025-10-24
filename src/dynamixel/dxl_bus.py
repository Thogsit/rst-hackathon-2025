import os

from dynamixel_sdk import (
    PortHandler,
    PacketHandler,
    GroupSyncWrite,
    GroupSyncRead,
    GroupBulkWrite,
    GroupBulkRead,

    COMM_SUCCESS,
)

from .motor_specs.base import ControlField, DynamixelMotor

from typing import (
    List,
    Dict,
)

class SyncGroup:
    def __init__(self, port_handler: PortHandler, packet_handler: PacketHandler, 
                 control_field: ControlField, motor_ids: List[int]):
        
        self.address = control_field.address
        self.data_length = control_field.size
        self.motor_ids = motor_ids

        self._group_sync_write = GroupSyncWrite(port_handler, packet_handler, self.address, self.data_length)
        self._group_sync_read = GroupSyncRead(port_handler, packet_handler, self.address, self.data_length)

        for id in self.motor_ids:
            self._group_sync_read.addParam(id)
            self._group_sync_write.addParam(id, bytes(self.data_length))

    def write(self, data: Dict[int, int]):  # use Dict[mototr_id, value_write] for precised changes
        for id, value in data.items():
            self._group_sync_write.changeParam(id, value.to_bytes(self.data_length, 'little', signed=True))

        res = self._group_sync_write.txPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError(f"Failed to write data to motors!")
        
    def read(self) -> Dict[int, int]:
        res = self._group_sync_read.txRxPacket()
        if res != COMM_SUCCESS:
            raise RuntimeError(f"Failed to read data from motors!")
        
        output = {}
        for id in self.motor_ids:
            if self._group_sync_read.isAvailable(id, self.address, self.data_length):
                output[id] = self._group_sync_read.getData(id, self.address, self.data_length)

        return output
    

class DxlBus:
    def __init__(self, port_name: str = "", baudrate: int = 57_600, protocol_version: float = 2.0, motors: List[DynamixelMotor] = []):
        self.port_name = port_name if port_name else self._get_active_port()
        self.baudrate = baudrate
        self.port_handler = PortHandler(self.port_name)
        self.packet_handler = PacketHandler(protocol_version)

        self.motors: List[DynamixelMotor] = motors

        self._sync_groups: Dict[str, SyncGroup] = {}
        self._is_open = False

    def __del__(self):
        self.disconnect()

    def __enter__(self):
        self.connect()
        
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        self.disconnect()

        if exc_type is not None:
            raise exc_value

    def connect(self):        
        if not self.port_handler.openPort():
            raise Exception(f'Failed to open port {self.port_name}')

        if not self.port_handler.setBaudRate(self.baudrate):
            raise Exception(f'failed to set baudrate to {self.baudrate}')
        
        self._is_open = True
        
    def disconnect(self):
        if self._is_open:
            self.port_handler.closePort()
            self._is_open = False

    def write_reg(self, motor_id, control_field: ControlField, value: int):
        if control_field.size == 1:
            res, err = self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        elif control_field.size == 2:
            res, err = self.packet_handler.write2ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        elif control_field.size == 4:
            res, err = self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, control_field.address, value)
        
        if res != COMM_SUCCESS or err:
            raise RuntimeError(f"Write fail id={motor_id} res={res} err={err}")

    def read_reg(self, motor_id: int, control_field: ControlField) -> int:
        if control_field.size == 1:
            val, res, err = self.packet_handler.read1ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        elif control_field.size == 2:
            val, res, err = self.packet_handler.read2ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        elif control_field.size == 4:
            val, res, err = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, control_field.address)
        
        if res != COMM_SUCCESS or err:
            raise RuntimeError(f"Read fail id={motor_id} res={res} err={err}")
        
        return val

    def make_sync_group(self, sync_group_name: str, control_field: ControlField, ids: List[int]) -> SyncGroup:
        key = sync_group_name
        if key not in self._sync_groups:
            self._sync_groups[key] = SyncGroup(self.port_handler, self.packet_handler, control_field, ids)
        return self._sync_groups[key]
    
    def get_sync_group(self, sync_group_name: str) -> SyncGroup:
        if sync_group_name not in self._sync_groups:
            raise KeyError(f"Sync group '{sync_group_name}' does not exist.")
        return self._sync_groups[sync_group_name]

    def find_active_motors(self, id_range = range(1, 253)) -> List[int]:   # 253 - see SDK doc
        found_ids: List[int] = []
        for motor_id in id_range:
            model, res, err = self.packet_handler.ping(self.port_handler, motor_id)
            if res == COMM_SUCCESS and not err:
                found_ids.append(motor_id)
        return found_ids

    def _get_active_port(self) -> str:
        for port_name in os.listdir('/dev'):
            if 'ttyUSB' in port_name or 'ttyACM' in port_name:
                full_port_name = '/dev/' + port_name
                return full_port_name
        
        raise RuntimeError("No active port found. Please connect a Dynamixel device.")