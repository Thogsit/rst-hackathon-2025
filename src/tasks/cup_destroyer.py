import time
from typing import List

from kochV1_package import KochV1_Robot
from models import Detection, ObjectType, Position
from tasks.abstract_task import AbstractTask


class CupDestroyerTask(AbstractTask):
    TEST_DETECTIONS = [
        [
            Detection(
                Position(0, 0.0, 0),
                ObjectType.CUP,
                1,
            ),
            Detection(
                Position(0, -0.11, 0),
                ObjectType.CUP,
                2,
            ),
        ]
    ]
    TEST_DETECTIONS2 = [
        [
            Detection(
                Position(0, 0.09, 0),
                ObjectType.CUP,
                1,
            ),
            Detection(
                Position(0, 0, 0),
                ObjectType.CUP,
                2,
            ),
            Detection(
                Position(0, -0.1, 0),
                ObjectType.CUP,
                3,
            ),
            Detection(
                Position(0, 0.05, 0.1),
                ObjectType.CUP,
                4,
            ),
            Detection(
                Position(0, -0.05, 0.1),
                ObjectType.CUP,
                5,
            ),
            Detection(
                Position(0, 0, 0.2),
                ObjectType.CUP,
                6,
            ),
        ],
        [
            Detection(
                Position(0, 0, 0),
                ObjectType.CUP,
                2,
            ),
            Detection(
                Position(0, -0.1, 0),
                ObjectType.CUP,
                3,
            ),
            Detection(
                Position(0, -0.05, 0.1),
                ObjectType.CUP,
                5,
            ),
        ],
        [
            Detection(
                Position(0, -0.1, 0),
                ObjectType.CUP,
                3,
            ),
        ],
    ]

    def __init__(self, robot: KochV1_Robot):
        super().__init__(robot, True)

    def _task_run(self):
        # Move arm to starting position, i.e. straight direction and a bit up
        self.controls.change_arm_joints([90, -90, -30])
        self.controls.set_direction_in_degrees(0)
        self.controls.change_arm_joints([90, -65, -20])

        for detections in self.TEST_DETECTIONS:
            # Phase 1: Receive ball, i.e. move back, open hand, close hand
            self.controls.change_arm_joints([90, 20, -20])
            self.controls.change_arm_joints([130, 10, -30])
            target_degree = self.calculate_degrees_from_detections(detections)
            print("Target Degree: " + str(target_degree))
            self.controls.set_direction_in_degrees(target_degree, degree_margin=0.5)
            self.controls.set_hand_turn_in_degrees(90)
            self.controls.open_hand()
            time.sleep(4)

            # Phase 2: Throw from top
            self.robot.set_velocity_and_accel(0, 0)
            self.controls.change_arm_joints([80, -30, 0])
            self.robot.set_velocity_and_accel() # Reset to defaults
            time.sleep(0.2)

    @staticmethod
    def calculate_degrees_from_detections(detections: List[Detection]) -> float:
        # Filter all irrelevant detections
        detections = [d for d in detections if d.object_type == ObjectType.CUP]
        if len(detections) == 0:
            return 0.0 # TODO Fix

        target_position = detections[0].position
        if len(detections) > 1:
            # Find lowest cup
            lowest_cup = detections[0]
            for detection in detections:
                if detection.position.z < lowest_cup.position.z:
                    lowest_cup = detection

            # Now find all cups that are roughly at the same height
            low_level_cups: List[Detection] = []
            for detection in detections:
                if abs(detection.position.z - lowest_cup.position.z) < 0.04:
                    low_level_cups.append(detection)
            print("Found " + str(len(low_level_cups)) + " low level cups!")

            if len(low_level_cups) == 1:
                target_position = low_level_cups[0].position
            else:
                closest_cup_pair = (low_level_cups[0], low_level_cups[1])
                for i, detection1 in enumerate(low_level_cups):
                    for j, detection2 in enumerate(low_level_cups):
                        if i == j:
                            continue

                        cur_dist = CupDestroyerTask.y_dist_between_detections(detection1, detection2)
                        if cur_dist < CupDestroyerTask.y_dist_between_detections(closest_cup_pair[0], closest_cup_pair[1]):
                            closest_cup_pair = (detection1, detection2)

                dist_between_cups = CupDestroyerTask.y_dist_between_detections(low_level_cups[0], low_level_cups[1])
                if abs(dist_between_cups) < 0.11:
                    print("Found 2 adjacent cups: " + str(closest_cup_pair[0].object_id) + ", " + str(closest_cup_pair[1].object_id))
                    left_detection = closest_cup_pair[0] if closest_cup_pair[0].position.y > closest_cup_pair[1].position.y else closest_cup_pair[1]
                    target_position = left_detection.position
                    target_position.y -= abs(closest_cup_pair[0].position.y - closest_cup_pair[1].position.y) / 2
                else:
                    print("Too far in between, taking " + str(closest_cup_pair[0].object_id))
                    target_position = closest_cup_pair[0].position # Just take one randomly if they're too far away

        # Calculate degrees from target position
        return target_position.y * 100

    @staticmethod
    def y_dist_between_detections(d1: Detection, d2: Detection) -> float:
        return abs(d1.position.y - d2.position.y)
