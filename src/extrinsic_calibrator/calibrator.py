import cv2
import pygame
import numpy as np
from scipy.spatial.transform import Rotation
import logging

from triton.language import dtype

logging.basicConfig(
    level=logging.DEBUG,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)

logger.info("Script was started!")

pygame.init()
WIDTH, HEIGHT = 1440, 810
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Webcam Calibration")
font = pygame.font.SysFont(None, 30)
clock = pygame.time.Clock()

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
if not cap.isOpened():
    logger.error("Cannot open camera")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
logger.info(f"Camera opened with: {cap.isOpened()}")

def calibrate_camera(grey, img):
    logger.info("Calibrating camera ...")
    square_size = 25.0
    pattern_size = (7, 10)

    camera_matrix = np.array([[1604.920153, 0, 950.360359],
                              [0, 1643.085788, 437.179307],
                              [0, 0, 1]
                              ], dtype=np.float32)
    dist_coeffs = np.array([0.183037, -0.508367, -0.039670, -0.006225], dtype=np.float32)

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2) * square_size

    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    ret, corners = cv2.findChessboardCorners(grey, pattern_size, None)

    if ret:
        corners = cv2.cornerSubPix(grey, corners, (11, 11), (-1, -1), criteria)
        ret, rvecs, tvecs = cv2.solvePnP(objp, corners, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_ITERATIVE)

        if ret:
            rmat = cv2.Rodrigues(rvecs)[0]
            rot = Rotation.from_matrix(rmat)
            euler = rot.as_euler('xyz', degrees=True)

            logger.info("Camera was successfully calibrated")
            logger.info(f"Rotation vector: {rvecs}")
            logger.info(f"Translation vector: {tvecs}")

            axis = np.float32([[3*square_size, 0, 0], [0, 3*square_size, 0], [0, 0, -3*square_size]]).reshape(-1, 3)
            imgpts, jac = cv2.projectPoints(axis, rvecs, tvecs, camera_matrix, dist_coeffs)
            imgpts = imgpts.astype("int32")
            corner = tuple(corners[0].ravel().astype("int32"))
            img = cv2.line(img, corner, tuple(imgpts[0].ravel()), (255, 0, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[1].ravel()), (0, 255, 0), 3)
            img = cv2.line(img, corner, tuple(imgpts[2].ravel()), (0, 0, 255), 3)

    return img



running = True
while running:
    ret, frame = cap.read()
    if not ret:
        logger.error("Cannot receive frame")
        break
    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    grey = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    calibration_frame = calibrate_camera(grey, frame)

    surf = pygame.surfarray.make_surface(np.transpose(frame, (1, 0, 2)))
    screen.blit(surf, (0, 0))

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    clock.tick(30)

cap.release()
pygame.quit()
logger.info("Program ended")