from threading import Lock
from typing import List

from models import Detection


class PerceptionController:
    # DO NOT TOUCH THIS VAR DIRECTLY!
    CUR_DETECTIONS: List[Detection] = []

    DETECTIONS_LOCK = Lock()

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

    def run_yolo(self):
        pass # TODO: @Lukas implement this you lazy boy!

    def run_processing(self):
        pass # TODO: @Lukas Do this also pls :(