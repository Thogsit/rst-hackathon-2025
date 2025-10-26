from threading import Lock
from typing import List, Dict, Tuple

import numpy as np
from models import Detection, ObjectType, Position, ImagePosition
from perception.duck_data import DuckData

import cv2
from ultralytics import YOLO
import pygame
from scipy.spatial.transform import Rotation as R

class PerceptionController:
    # DO NOT TOUCH THIS VAR DIRECTLY!
    CUR_DETECTIONS: List[Detection] = []
    KILL_ALL: bool = False

    DETECTIONS_LOCK = Lock()
    DUCK_DATA_LOCK = Lock()
    KILL_LOCK = Lock()

    INTERNAL_LOCK = Lock()
    yolo_screen = None
    dect_screen = None

    # Object ID to positions mapping
    DUCK_DATA: Dict[int, Tuple[ObjectType, List[ImagePosition]]] = {}

    DUCK_RADIUS_MAP = {
        0: ImagePosition(279, 321),
        2: ImagePosition(277, 335),
        4: ImagePosition(267, 354),
        6: ImagePosition(275, 376),
        8: ImagePosition(274, 399),
        10: ImagePosition(271, 428),
        12: ImagePosition(267, 456),
        14: ImagePosition(279, 479),
    }

    def __init__(self):
        self.model = YOLO("./model.pt")
        self.cap = cv2.VideoCapture("/dev/video0")

        if not self.cap.isOpened():
            print("Cannot open camera")
        else:
            print("Camera opened with: ", self.cap.isOpened())

        self.INTERNAL_LOCK = Lock()
        self.results = []

        self.WIDTH, self.HEIGHT = 1440, 810

        self.camera_matrix = np.array([[1604.920153, 0.0, 950.360359],
                                       [0.0, 1643.085788, 437.179307],
                                       [0.0, 0.0, 1.0]], dtype=np.float32)

        self.world_points = np.array([
            [0.8, -0.1325, 0.0],
            [0.663, -0.1325, 0.0],
            [0.663, 0.1325, 0.0],
            [0.8, 0.1325, 0.0]
        ], dtype=np.float32)

        self.t_vec = np.array([0.02, 0.132, 0.23], dtype=np.float32)
        self.r_vec = np.array([0.0, -90.0, 90.0], dtype=np.float32) # DEGREE
        yaw = -30
        pitch = 19

        self.rotation_matrix = R.from_euler('xyz', self.r_vec, degrees=True).as_matrix()
        self.pitch_matrix = R.from_euler('xyz', [0, yaw, 0], degrees=True).as_matrix()
        self.yaw_matrix = R.from_euler('xyz', [pitch, 0, 0], degrees=True).as_matrix()


        self.last_detections = []

    @staticmethod
    def calc_duck_radius() -> Dict[ObjectType, int]:
        PerceptionController.DUCK_DATA_LOCK.acquire()
        duck_data = PerceptionController.DUCK_DATA.copy()
        PerceptionController.DUCK_DATA = {}
        PerceptionController.DUCK_DATA_LOCK.release()

        # Object Type -> radius mapping
        duck_to_radius: Dict[ObjectType, int] = {}

        for duck in duck_data.keys():
            closest_points: Dict[int, int] = {} # Radius to count mapping
            positions = duck_data[duck][1]
            for img_pos in positions:
                for radius in PerceptionController.DUCK_RADIUS_MAP.keys():
                    rad_pos = PerceptionController.DUCK_RADIUS_MAP[radius]
                    if abs(img_pos.y - rad_pos.y) < 30:
                        if radius not in closest_points:
                            closest_points[radius] = 1
                        else:
                            closest_points[radius] += 1

            if len(closest_points) > 0:
                max_count_key = list(closest_points.keys())[0]
                for radius in closest_points:
                    if closest_points[radius] > closest_points[max_count_key]:
                        max_count_key = radius
                duck_to_radius[duck_data[duck][0]] = max_count_key

        return duck_to_radius

    @staticmethod
    def write_kill_all(kill_all: bool):
        PerceptionController.KILL_LOCK.acquire()
        PerceptionController.KILL_ALL = kill_all
        PerceptionController.KILL_LOCK.release()

    @staticmethod
    def get_kill_all() -> bool:
        PerceptionController.KILL_LOCK.acquire()
        kill_all = PerceptionController.KILL_ALL
        PerceptionController.KILL_LOCK.release()
        return kill_all

    @staticmethod
    def write_detections(new_detections: List[Detection]):
        PerceptionController.DETECTIONS_LOCK.acquire()
        PerceptionController.CUR_DETECTIONS = new_detections
        PerceptionController.DETECTIONS_LOCK.release()

    @staticmethod
    def read_detections() -> List[Detection]:
        PerceptionController.DETECTIONS_LOCK.acquire()
        cur_detections = [val for val in PerceptionController.CUR_DETECTIONS]
        PerceptionController.DETECTIONS_LOCK.release()
        return cur_detections

    def run_yolo(self):
        while self.cap.isOpened() and not PerceptionController.get_kill_all():
            success, frame = self.cap.read()

            if success:
                results = self.model.track(frame, persist=True, verbose=False, imgsz=1920)

                self.INTERNAL_LOCK.acquire(timeout=10)
                self.results = results
                self.INTERNAL_LOCK.release()


    def run_processing(self):
        if PerceptionController.yolo_screen is None:
            pygame.init()
            PerceptionController.yolo_screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT))
            pygame.display.set_caption("DuckHack4 YOLO")

        while self.cap.isOpened() and not PerceptionController.get_kill_all():
            self.INTERNAL_LOCK.acquire(timeout=10)
            if len(self.results) == 0:
                self.INTERNAL_LOCK.release()
                continue
            annotated_frame = self.results[0].plot()
            result = self.results[0]
            self.INTERNAL_LOCK.release()
            annotated_frame = cv2.resize(annotated_frame, (self.WIDTH, self.HEIGHT))
            annotated_frame = cv2.cvtColor(annotated_frame, cv2.COLOR_BGR2RGB)
            surf = pygame.surfarray.make_surface(np.transpose(annotated_frame, (1, 0, 2)))
            PerceptionController.yolo_screen.blit(surf, (0, 0))

            for i, point in enumerate(self.world_points):
                point = point - self.t_vec
                point = np.dot(self.rotation_matrix, point)
                point = np.dot(self.pitch_matrix, point)
                point = np.dot(self.yaw_matrix, point)
                point = np.dot(self.camera_matrix, point)
                point =  point / point[2]
                point = point * (3/4)

                if i == 0:
                    pygame.draw.circle(PerceptionController.yolo_screen, (0, 255, 0), (int(point[0]), int(point[1])), 5)
                elif i == 1:
                    pygame.draw.circle(PerceptionController.yolo_screen, (255, 0, 0), (int(point[0]), int(point[1])), 5)
                elif i == 2:
                    pygame.draw.circle(PerceptionController.yolo_screen, (0, 0, 255), (int(point[0]), int(point[1])), 5)
                else:
                    pygame.draw.circle(PerceptionController.yolo_screen, (255, 0, 255), (int(point[0]), int(point[1])), 5)


            detections: List[Detection] = []

            for bbox in result.boxes.cpu():
                class_id = int(bbox.cls[0])
                class_type = None
                if class_id == 0:
                    class_type = ObjectType.BALL
                elif class_id == 1:
                    class_type = ObjectType.CUP
                elif class_id == 2:
                    class_type = ObjectType.DUCK_BLUE
                elif class_id == 3:
                    class_type = ObjectType.DUCK_GREEN
                elif class_id == 4:
                    class_type = ObjectType.DUCK_RED
                elif class_id == 5:
                    class_type = ObjectType.DUCK_WHITE
                elif class_id == 6:
                    class_type = ObjectType.DUCK_YELLOW
                elif class_id == 7:
                    class_type = ObjectType.TARGET_BALL
                else:
                    print(f"Unknown class id: {class_id}")
                    continue

                u = bbox.xywh[0][0]
                v = bbox.xywh[0][1]
                h = bbox.xywh[0][3]
                v = v + h/2

                world_point = np.array([u, v, 1], dtype=np.float32)


                if bbox.id is not None:
                    object_id = int(bbox.id)
                else:
                    object_id = -1

                position = Position(x=world_point[0], y=world_point[1], z=world_point[2])
                print(f"World Point = {world_point}")
                image_position = ImagePosition(x=u, y=v)
                detections.append(Detection(position, image_position, class_type, object_id))

            self.DUCK_DATA_LOCK.acquire()
            for d in detections:
                # Filter for ducks
                if d.object_type > ObjectType.CUP and d.image_position.x -20 < self.DUCK_RADIUS_MAP[0].x < d.image_position.x + 20:
                    if d.object_id not in self.DUCK_DATA:
                        self.DUCK_DATA[d.object_id] = (d.object_type,[])
                    self.DUCK_DATA[d.object_id][1].append(d.image_position)
            self.DUCK_DATA_LOCK.release()

            self.write_detections(detections)
            pygame.display.flip()
            self.last_detections = detections



    def __del__(self):
        self.cap.release()
        pygame.quit()