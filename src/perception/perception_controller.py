from threading import Lock
from typing import List

import numpy as np
from models import Detection, ObjectType, Position
from perception.duck_data import DuckData

import cv2
from ultralytics import YOLO
import pygame
from scipy.spatial.transform import Rotation as R

class PerceptionController:
    # DO NOT TOUCH THIS VAR DIRECTLY!
    CUR_DETECTIONS: List[Detection] = []
    DUCK_DATA: List[DuckData] = []

    DETECTIONS_LOCK = Lock()
    DUCK_DATA_LOCK = Lock()

    INTERNAL_LOCK = Lock()
    yolo_screen = None
    dect_screen = None

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
            [0.8, -0.15, 0.0],
            [0.663, -0.15, 0.0],
            [0.663, 0.15, 0.0],
            [0.8, 0.15, 0.0]
        ], dtype=np.float32)

        self.t_vec = np.array([0.02, 0.132, 0.23], dtype=np.float32)
        self.r_vec = np.array([0.0, -90.0, 90.0], dtype=np.float32) # DEGREE
        yaw = -15
        pitch = 19

        self.rotation_matrix = R.from_euler('xyz', self.r_vec, degrees=True).as_matrix()
        self.pitch_matrix = R.from_euler('xyz', [0, yaw, 0], degrees=True).as_matrix()
        self.yaw_matrix = R.from_euler('xyz', [pitch, 0, 0], degrees=True).as_matrix()


        self.last_detections = []

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

    @staticmethod
    def read_duck_data() -> List[DuckData]:
        PerceptionController.DUCK_DATA_LOCK.acquire()
        cur_duck_data = [val for val in PerceptionController.DUCK_DATA]
        PerceptionController.DUCK_DATA_LOCK.release()
        return cur_duck_data

    @staticmethod
    def write_duck_data(new_duck_data: List[DuckData]):
        PerceptionController.DUCK_DATA_LOCK.acquire()
        PerceptionController.DUCK_DATA = new_duck_data
        PerceptionController.DUCK_DATA_LOCK.release()

    def run_yolo(self):
        while self.cap.isOpened():
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

        while self.cap.isOpened():
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


            detections = []

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

                u = bbox.xywh[0][0]
                v = bbox.xywh[0][1]
                h = bbox.xywh[0][3]
                v = v + h/2

                if bbox.id is not None:
                    object_id = int(bbox.id)
                else:
                    object_id = -1

                position = Position(x=v, y=v, z=1)
                detections.append(Detection(position, class_type, object_id))

            pygame.display.flip()
            self.last_detections = detections



    def __del__(self):
        self.cap.release()
        pygame.quit()