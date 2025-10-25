import time

from kochV1_package import KochV1_Robot
from tasks.abstract_task import AbstractTask
from utils import Unit


class TestTask(AbstractTask):
    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, True)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([90, -90, -30])
        self.controls.set_direction_in_degrees(0)
        self.controls.change_arm_joints([90, -65, 10])

        print(self.robot.read_joints(Unit.DEG))
        self.controls.set_direction_in_degrees(5, degree_margin=0.5)
        print(self.robot.read_joints(Unit.DEG))

        time.sleep(20)

