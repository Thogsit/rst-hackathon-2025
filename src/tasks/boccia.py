import time

from kochV1_package import KochV1_Robot
from tasks.abstract_task import AbstractTask
from utils import Unit


class BocciaTask(AbstractTask):
    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, True)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([90, -90, -30])
        self.controls.set_direction_in_degrees(0)
        self.controls.change_arm_joints([90, -65, 10])

        while True:
            # Phase 1: Receive ball, i.e. move back, open hand, close hand
            self.controls.change_arm_joints([90, 20, 10])
            self.controls.change_arm_joints([125, 30, -10])
            self.controls.set_hand_turn_in_degrees(90)
            self.controls.open_hand()
            time.sleep(4)
            self.controls.close_hand()
            self.controls.set_hand_turn_in_degrees(-90)

            # Phase 2: Move to throw position
            self.controls.change_arm_joints([90, 20, 10])
            self.controls.change_arm_joints([55, -40, -20])
            time.sleep(1)
            #self.controls.change_arm_joints([0, -90, -40])
            self.controls.open_hand()

            # Phase 3: Throw ball
            #self.controls.change_arm_joints([0, 0, 45])
            self.controls.ENTERED_VEL_MODE = True
            self.robot.set_epcm_control_mode()
            self.controls.change_arm_joints([55, -20, 20])
            time.sleep(1)
            self.controls.close_hand()
            self.controls.change_arm_joints([30, 0, 0])
            time.sleep(1)
            #self.robot.set_joints([0, 20, -10, 40, -90], unit=Unit.DEG)
            #self.robot.set_velocity_control_mode()
            #self.robot.set_goal_velocities([0.01, -11.0, -16.0, -5.0, 0.01])
            #time.sleep(4)

            break

        #time.sleep(20)

