from datetime import datetime
import time
from typing import List, Tuple

from kochV1_package import KochV1_Robot
from utils import Unit


class Controls:
    WAIT_TIME = 0.1 # How long to wait between new check whether we reached target joint values
    WAIT_TIMEOUT = 3 # After 3 sec of not reaching the margin, continue anyway
    DEFAULT_MARGIN = 15.0

    ENTERED_VEL_MODE = False

    def __init__(self, robot: KochV1_Robot):
        self.robot = robot

    def set_direction_in_degrees(self, direction: float, degree_margin: float = 2.0):
        joints = self.robot.read_joints(Unit.DEG)
        joints[0] = direction
        self.robot.set_joints(joints, unit=Unit.DEG)
        self._wait_until_joints_reached(joints, margin=5.0, margin_overrides=[(0, degree_margin)])

    def set_hand_turn_in_degrees(self, turn: float, degree_margin: float = 2.0):
        joints = self.robot.read_joints(Unit.DEG)
        joints[-1] = turn
        self.robot.set_joints(joints, unit=Unit.DEG)
        self._wait_until_joints_reached(joints, margin=5.0, margin_overrides=[(len(joints) - 1, degree_margin)])

    def open_hand(self):
        self.robot.set_gripper_percentage(0.3)
        time.sleep(0.3)

    def close_hand(self):
        self.robot.set_gripper_percentage(0.55)
        time.sleep(0.3)

    def completely_close_hand(self):
        self.robot.set_gripper_percentage(1.0)
        time.sleep(0.3)

    def change_arm_joints(self, joint_angles: List[float], unit: Unit = Unit.DEG, margin: float = DEFAULT_MARGIN, margin_overrides: List[Tuple[int, float]] = None):
        if len(joint_angles) != 3:
            raise Exception("Wrong number of joints!")
        cur_joints = self.robot.read_joints(unit)
        joints = [cur_joints[0]] # Direction
        for val in joint_angles:
            joints.append(val)
        joints.append(cur_joints[-1]) # Hand turn
        self.go_to_joints(joints, unit, margin, margin_overrides)

    def go_to_joints(self, joint_angles: List[float], unit: Unit = Unit.DEG, margin: float = DEFAULT_MARGIN, margin_overrides: List[Tuple[int, float]] = None):
        if len(joint_angles) != 5:
            raise Exception("Wrong number of joints!")
        self.robot.set_joints(joint_angles, unit)
        self._wait_until_joints_reached(joint_angles, unit, margin, margin_overrides)

    def safe_return(self, bottom_safety_pos: bool):
        if self.ENTERED_VEL_MODE:
            return
        cur_joints = self.robot.read_joints(Unit.DEG)

        if bottom_safety_pos:
            if cur_joints[1] >= 20 and cur_joints[2] >= -60:
                self.change_arm_joints([80, -30, 0])
                self.change_arm_joints([110, -90, -90])
            self.change_arm_joints([0, -90, 0])
        else:
            final_pos = [-75, 88, -110, -55, 0]
            joints = self.robot.read_joints(Unit.DEG)
            final_pos_already_reached = True
            for i, val in enumerate(joints):
                if abs(final_pos[i] - val) > 20.0:
                    final_pos_already_reached = False
            if not final_pos_already_reached:
                self.change_arm_joints([80, -30, 0])
                self.set_hand_turn_in_degrees(0)
                self.set_direction_in_degrees(-85)
                mid_pos = [val for val in final_pos[1:-1]]
                mid_pos[0] = 100
                self.change_arm_joints(mid_pos, margin=7.0)
                time.sleep(1)
                self.change_arm_joints(final_pos[1:-1], margin=7.0)

    def _wait_until_joints_reached(self, joint_angles: List[float], unit: Unit = Unit.DEG, margin: float = DEFAULT_MARGIN, margin_overrides: List[Tuple[int, float]] = None):
        margin_overrides = margin_overrides or []
        now = datetime.now()
        while (datetime.now() - now).seconds < self.WAIT_TIME:
            joints = self.robot.read_joints(unit)
            if not joints:
                time.sleep(self.WAIT_TIME)

            print(joints)

            is_in_margin = True
            for i, value in enumerate(joint_angles):
                offset = 0
                for margin_override in margin_overrides:
                    if margin_override[0] == i:
                        offset = margin_override[1]
                        print("Apply margin of " + str(offset))
                if abs(value - joints[i]) > (margin + offset):
                    is_in_margin = False
                    time.sleep(self.WAIT_TIME)
                    break
            if is_in_margin:
                print("Margin reached!")
                break
