import logging
import time

from kochV1_package import KochV1_Robot, KochV1_DxlBus
from utils import Unit

def set_velocities(
        dxl_bus: KochV1_DxlBus, 
        profile_velocity: int, profile_acceleration: int,
    ) -> None:
    dxl_bus.write_reg(1, dxl_bus.motors[0].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(1, dxl_bus.motors[0].RAM.PROFILE_ACCELERATION, profile_acceleration)
    dxl_bus.write_reg(2, dxl_bus.motors[1].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(2, dxl_bus.motors[1].RAM.PROFILE_ACCELERATION, profile_acceleration)
    dxl_bus.write_reg(3, dxl_bus.motors[2].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(3, dxl_bus.motors[2].RAM.PROFILE_ACCELERATION, profile_acceleration)
    dxl_bus.write_reg(4, dxl_bus.motors[3].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(4, dxl_bus.motors[3].RAM.PROFILE_ACCELERATION, profile_acceleration)
    dxl_bus.write_reg(5, dxl_bus.motors[4].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(5, dxl_bus.motors[4].RAM.PROFILE_ACCELERATION, profile_acceleration)
    dxl_bus.write_reg(6, dxl_bus.motors[5].RAM.PROFILE_VELOCITY, profile_velocity)
    dxl_bus.write_reg(6, dxl_bus.motors[5].RAM.PROFILE_ACCELERATION, profile_acceleration)

def main():
    logging.basicConfig(level=logging.DEBUG)
    # Physical home position of the motors in the assembly
    motor_physical_home_positions = [2048, 1024, 2048, 2048, 2048, 2048]

    # Recommended: use the bus as a context manager
    with KochV1_DxlBus(motor_physical_home_positions) as dxl_bus:
        robot = KochV1_Robot(dxl_bus)
        
        set_velocities(dxl_bus, profile_velocity=150, profile_acceleration=10)
        
        # Move to a joint configuration (degrees shown for readability)
        robot.set_joints([0, 90, -90, -90, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        robot.set_joints([-45, 90, -90, -90, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        robot.set_joints([0, 0, -90, 0, 0], unit=Unit.DEG)
        time.sleep(3)  # allow time to reach goal position

        # Read back the current configuration
        print(robot.read_joints(unit=Unit.DEG))
        time.sleep(1)

if __name__ == "__main__":
    main()
