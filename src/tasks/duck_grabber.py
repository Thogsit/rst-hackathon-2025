import datetime
import time

from kochV1_package import KochV1_Robot
from perception.perception_controller import PerceptionController
from tasks.abstract_task import AbstractTask


class DuckGrabberTask(AbstractTask):
    GRAB_POS_MAP = {
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
        begin_time = datetime.datetime.now()
        while datetime.datetime.now() - begin_time < datetime.timedelta(seconds=35):
            # Phase 1: Move arm over ducks
            self.controls.set_hand_turn(-90)

            # Detect ducks and choose highest value one
            duck_radius_map = PerceptionController.calc_duck_radius()
            if len(duck_radius_map) == 0:
                print("No ducks found")
                time.sleep(0.5)
                continue
            duck_types = list(duck_radius_map.keys())
            duck_types.sort()
            radius = duck_radius_map[duck_types[0]]

            radius_keys = list(self.GRAB_POS_MAP.keys())
            radius_keys.sort()
            radius_key = radius_keys[0]
            for k in radius_keys:
                if k < radius:
                    radius_key = k
            end_pos = self.GRAB_POS_MAP[radius_key]

            # Prepare down movement
            self.controls.change_arm_joints([92, end_pos[1], end_pos[2]])

            # Move down
            self.controls.change_arm_joints(end_pos)
            time.sleep(5)

            # Phase 3: Get arm back behind to receive duck
            self.controls.change_arm_joints([90, -65, 30])
            self.controls.change_arm_joints([90, 20, 30])
            self.controls.change_arm_joints([125, 30, 30])
            time.sleep(4) # Wait until duck is taken
