import time

from kochV1_package import KochV1_Robot
from perception.perception_controller import PerceptionController
from tasks.abstract_task import AbstractTask


class DuckGrabberTask(AbstractTask):
    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, False)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([100, -50, -30])
        self.controls.set_direction_in_degrees(0)
        self.controls.completely_close_hand()

        # Task loop
        while True:
            # Phase 1: Move arm over ducks
            self.controls.set_hand_turn_in_degrees(-90)
            self.controls.change_arm_joints([90, -60, -30], margin_overrides=[(2, 20)])

            # Phase 2: Get arm down to catch duck
            target_duck = None
            while True:
                duck_data = PerceptionController.read_duck_data()
                if len(duck_data) == 0:
                    print("Not seeing any ducks atm")
                    time.sleep(0.5)
                    continue
                target_duck = duck_data[0]
                for duck in duck_data:
                    if duck.object_id > target_duck.object_id:
                        target_duck = duck
            target_radius = target_duck.radius
            # TODO: Implement this!

            # Phase 3: Get arm back behind to receive duck
            self.controls.change_arm_joints([90, -65, 30])
            self.controls.change_arm_joints([90, 20, 30])
            self.controls.change_arm_joints([125, 30, 30])
            time.sleep(5) # Wait until duck is taken
