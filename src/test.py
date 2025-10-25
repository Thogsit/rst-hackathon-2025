import perception.perception_controller as perception
import threading

controller = perception.PerceptionController()

thread_1 = threading.Thread(target=controller.run_yolo)
thread_2 = threading.Thread(target=controller.run_processing)

thread_1.start()
thread_2.start()

thread_1.join()
thread_2.join()

exit(0)