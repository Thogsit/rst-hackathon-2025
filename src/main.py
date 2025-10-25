import logging
from threading import Thread
from typing import List

from kochV1_package import KochV1_Robot, KochV1_DxlBus
from tasks.duck_grabber import DuckGrabberTask
from perception.perception_controller import PerceptionController


THREADS: List[Thread] = []

def main():
    logging.basicConfig(level=logging.INFO)
    motor_physical_home_positions = [2048, 1024, 2048, 2048, 2048, 2048]

    # Start perception threads
    perception_controller = PerceptionController()
    perception_yolo_thread = Thread(target=perception_controller.run_yolo, daemon=True)
    THREADS.append(perception_yolo_thread)
    perception_yolo_thread.start()
    perception_processing_thread = Thread(target=perception_controller.run_processing, daemon=True)
    THREADS.append(perception_processing_thread)
    perception_processing_thread.start()

    # Recommended: use the bus as a context manager
    with KochV1_DxlBus(motor_physical_home_positions) as dxl_bus:
        robot = KochV1_Robot(dxl_bus)

        print("-=-= Throw'n Grab v1.3.3.7 =-=-")
        print("[1] Test mode")
        print("[2] Duck Grabber")
        print("[3] Cup Destroyer")
        print("[4] Boccia")

        task = None
        while True:
            raw_task = input("Select task> ")
            try:
                task = int(raw_task)
            except ValueError:
                print("Invalid task")
            if 0 < task < 5:
                break

        task_controller = None
        if task == 2:
            task_controller = DuckGrabberTask(robot)

        if not task_controller:
            print("[!] Task currently not supported")
            exit(1)
        task_controller.run()

if __name__ == "__main__":
    main()
