from threading import Lock
from typing import List

from models import Detection
from perception.duck_data import DuckData


class PerceptionController:
    # DO NOT TOUCH THIS VAR DIRECTLY!
    CUR_DETECTIONS: List[Detection] = []
    DUCK_DATA: List[DuckData] = []

    DETECTIONS_LOCK = Lock()
    DUCK_DATA_LOCK = Lock()

    def __init__(self):
        pass

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
        pass # TODO: @Lukas implement this you lazy boy!

    def run_processing(self):
        pass # TODO: @Lukas Do this also pls :(
