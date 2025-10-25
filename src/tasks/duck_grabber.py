import time

from kochV1_package import KochV1_Robot
from perception.perception_controller import PerceptionController
from tasks.abstract_task import AbstractTask


class DuckGrabberTask(AbstractTask):

    END_POS_MAP = {
        13.0: [70, -30, -90],
        11.0: [65, -27, -90],
        9.0: [62, -18, -90],
        7.0: [57, -15, -80],
        5.0: [53, -15, -66],
        3.0: [50, -17, -55],
        1.5: [48, -19, -40],
        0.0: [40, -15, -30],
    }

    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, False)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([100, -50, -30])
        self.controls.set_direction(0)
        self.controls.completely_close_hand()

        # Task loop
        while True:
            # Phase 1: Move arm over ducks
            self.controls.set_hand_turn(-90)
            radius = 1.0
            radius_keys = list(self.END_POS_MAP.keys())
            radius_keys.sort()
            radius_key = radius_keys[0]
            for k in radius_keys:
               if k < radius:
                   radius_key = k
            end_pos = self.END_POS_MAP[radius_key]

            self.controls.change_arm_joints([92, end_pos[1], end_pos[2]])
            self.controls.change_arm_joints(end_pos)

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

            break
