import time

from kochV1_package import KochV1_Robot
from tasks.abstract_task import AbstractTask
from utils import Unit


class DuckGrabberTask(AbstractTask):
    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, False)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([100, -50, -30])
        self.controls.set_direction_in_degrees(0)

        # Task loop
        while True:
            # Phase 1: Move arm over ducks
            self.controls.set_hand_turn_in_degrees(-90)
            self.controls.change_arm_joints([90, -60, -30], margin_overrides=[(2, 20)])

            # Phase 2: Get arm down to catch duck
            # TODO: Implement this!

            # Phase 3: Get arm back behind to receive duck
            self.controls.change_arm_joints([90, -65, 30])
            self.controls.change_arm_joints([90, 20, 30])
            self.controls.change_arm_joints([125, 30, 30])
            time.sleep(5) # Wait until duck is taken




        # Straighten arm to prepare it for forwards movement
        #self.controls.change_arm_joints([120, -90, 0])

        # Move arm forward
        #self.controls.change_arm_joints([40, -10, 0], margin_overrides=[(2, 20)])

        # Move hand towards duck
        #self.controls.change_arm_joints([40, -10, -10], margin_overrides=[(2, 20)])


        time.sleep(20)

