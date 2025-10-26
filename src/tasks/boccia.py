import math
import time
from enum import Enum
from typing import List, Tuple

from kochV1_package import KochV1_Robot
from models import Detection, ObjectType, Position
from perception.perception_controller import PerceptionController
from tasks.abstract_task import AbstractTask
from utils import Unit


class PlatformPosition(Enum):
    LEFT = 0
    NONE = 1
    RIGHT = 2

class BocciaTask(AbstractTask):
    PLATFORM_POS = PlatformPosition.NONE
    TEST_DETECTIONS = [
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
        #[
        #    Detection(
        #        Position(0.25, -0.0, 0.0),
        #        ObjectType.TARGET_BALL,
        #        1,
        #    ),
        #],
    ]

    #DIST_TO_JOINT = {
    #    0.25: ([45, -40, -5], [50, -10.0, -5]),
    #    0.35: ([45, -45, -10], [55, -10.0, 30]),
    #    0.45: ([45, -45, -10], [55, -10.0, 30]),
    #}

    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, True)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([90, -90, -50])
        self.controls.set_direction(0)
        self.controls.change_arm_joints([90, -65, 10])

        detections = PerceptionController.read_detections()
        for detection in detections:
            if detection.object_id != ObjectType.TARGET_BALL:
                continue

            # Phase 1: Receive ball, i.e. move back, open hand, close hand
            self.controls.change_arm_joints([90, 20, 10])
            self.controls.change_arm_joints([125, 30, -10])
            self.controls.set_hand_turn(90)
            self.controls.open_hand()
            input("Press ENTER to begin throw!")
            self.controls.close_hand()
            self.controls.set_hand_turn(-90)

            # Phase 2: Move to throw position
            self.controls.change_arm_joints([90, 20, 10])
            #self.controls.change_arm_joints([55, -40, -20])
            target_deg, dist_to_target = self.get_direction_in_deg_and_dist_by_detections(detection)
            print("Target degrees: " + str(target_deg))
            print("Distance to target: " + str(dist_to_target))
            #dist_keys = list(self.DIST_TO_JOINT.keys())
            #dist_keys.sort()
            #dist_key = dist_keys[0]
            #for k in dist_keys:
            #    if k < dist_to_target:
            #        dist_key = k
            #start_pos, end_pos = self.DIST_TO_JOINT[dist_key]
            start_pos = [37, -60, -10]
            self.controls.change_arm_joints(start_pos)
            self.controls.set_direction(target_deg, degree_margin=0.5)
            time.sleep(1.5)
            #self.controls.open_hand()

            if dist_to_target < 0.35:
                print("Short throw")
                end_pos = [55, -10, 10]
            else:
                print("Long throw")
                end_pos = [55, -10, 10]

            # Phase 3: Throw ball
            self.robot.set_velocity_and_accel(0, 0)
            #self.controls.change_arm_joints(end_pos)
            self.robot.set_joints([target_deg, end_pos[0], end_pos[1], end_pos[2], -90], unit=Unit.DEG)
            self.controls.open_hand()
            time.sleep(1)
            self.controls.close_hand()
            self.robot.set_velocity_and_accel() # Reset to defaults

    @staticmethod
    def get_direction_in_deg_and_dist_by_detections(detection: Detection) -> Tuple[float, float]:
        # Filter out non-targets
        if detection.image_position.x < 430:
            target_degrees = 15
            print("Very Left mode!")
        elif detection.image_position.x < 960:
            target_degrees = 0
            print("Left mode!")
        elif detection.image_position.x < 960:
            target_degrees = -15
            print("Right mode!")
        else:
            target_degrees = -30
            print("Very Right mode!")

        print("Raw target degrees: " + str(target_degrees))
        left_max_deg = float(17 if BocciaTask.PLATFORM_POS == PlatformPosition.LEFT else 17)
        right_max_deg = float(-17 if BocciaTask.PLATFORM_POS == PlatformPosition.RIGHT else -17)
        target_degrees = max(target_degrees, right_max_deg)
        target_degrees = min(target_degrees, left_max_deg)
        dist_to_target = 80
        return target_degrees, dist_to_target
