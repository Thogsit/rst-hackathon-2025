from abc import abstractmethod, ABC

from kochV1_package import KochV1_Robot
from tasks.controls import Controls
from utils import Unit


class AbstractTask(ABC):
    controls: Controls = None
    def __init__(self, robot: KochV1_Robot, bottom_safety_pos: bool):
        self.robot = robot
        self.controls = Controls(robot)
        self.bottom_safety_pos = bottom_safety_pos

    @abstractmethod
    def _task_run(self):
        pass

    def run(self):
        try:
            self._task_run()
        except KeyboardInterrupt:
            print("Going back to safety position...")
        finally:
            self.controls.safe_return(self.bottom_safety_pos)
