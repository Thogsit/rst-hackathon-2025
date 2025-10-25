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

world_points = np.array([
    [0.8, -0.15, 0.0],
    [0.663, -0.15, 0.0],
    [0.526, -0.15, 0.0],
    [0.526, 0.15, 0.0],
    [0.663, 0.15, 0.0],
    [0.8, 0.15, 0.137],
], dtype=np.float32)

colors = [(0, 0, 255), (0, 255, 0), (255, 0, 0), (255, 0, 255), (255, 255, 0), (0, 255, 255)]

image_points = []
current_point = 0
calibration_result = None

cap = cv2.VideoCapture("/dev/video0", cv2.CAP_V4L2)
if not cap.isOpened():
    logger.error("Cannot open camera")
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)
logger.info(f"Camera opened with: {cap.isOpened()}")

def calibrate_camera():
    global calibration_result
    logger.info("Calibrating camera ...")

    if len(image_points) != len(world_points):
        logger.warning("Number of points does not match the number of image points")
        return {'error': f"Number of points does not match the number of image points"}

    try:
        camera_matrix = np.array([[1604.920153, 0, 950.360359],
                                  [0, 1643.085788, 437.179307],
                                  [0, 0, 1]
                                ], dtype=np.float32)
        dist_coeffs = np.array([0.183037, -0.508367, -0.039670, -0.006225], dtype=np.float32)
        img_pts = np.array(image_points, dtype=np.float32)

        tvec = np.array([0, 0.15, 0.22], dtype=np.float32)
        ret, rvec, tvec = cv2.solvePnP(world_points, img_pts, camera_matrix, dist_coeffs, tvec=tvec, useExtrinsicGuess=True)
        if not ret:
            logger.error("Cannot solve the camera matrix")
            return {'error': "Cannot solve the camera matrix"}
        rmat = cv2.Rodrigues(rvec)[0]
        rot = Rotation.from_matrix(rmat)
        euler = rot.as_euler('xyz', degrees=True)

        print(f"TVEC RAW: {tvec}")
        print(f"RVEC RAW: {rvec}")
        if tvec[2] < 0:
            tvec = -tvec
            rmat = np.linalg.inv(rmat)
            rot = Rotation.from_matrix(rmat)
            euler = rot.as_euler('xyz', degrees=True)

        result = {'position': {'x': float(tvec[0]), 'y': float(tvec[1]), 'z': float(tvec[2])},
                  'rotation': {'yaw': float(euler[0]), 'pitch': float(euler[1]), 'roll': float(euler[2])},}
        logger.info(f"Camera calibration solved with: {result}")
        calibration_result = result
        return result
    except Exception as e:
        logger.error(f"Camera calibration failed with: {e}")
        return {'error': f"Camera calibration failed with: {e}"}

def draw_button(x, y, h, w, text, color, hover_color, mouse_pos):
    rect = pygame.Rect(x, y, w, h)
    if rect.collidepoint(mouse_pos):
        pygame.draw.rect(screen, hover_color, rect)
    else:
        pygame.draw.rect(screen, color, rect)
    label = font.render(text, True, (0, 0, 0))
    screen.blit(label, (rect.x + 10, rect.y + 5))
    return rect

def reset_points():
    global image_points, current_point, calibration_result
    image_points = []
    current_point = 0
    calibration_result = None
    logger.info(f"Camera reset points ...")

running = True
while running:
    ret, frame = cap.read()
    if not ret:
        logger.error("Cannot receive frame")
        break
    frame = cv2.resize(frame, (WIDTH, HEIGHT))
    frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    surf = pygame.surfarray.make_surface(np.transpose(frame, (1, 0, 2)))
    screen.blit(surf, (0, 0))
    mouse_pos = pygame.mouse.get_pos()

    for i, pt in enumerate(image_points):
        scaled = ((int(pt[0]/4)*3), int((pt[1]/4)*3), 5, 5)
        pygame.draw.rect(screen, colors[i % len(colors)], scaled, 8)

    txt = font.render(f"Point {current_point + 1} / {len(world_points)}", True, (255, 255, 255))
    screen.blit(txt, (20, 20))

    reset_rect = draw_button(20, HEIGHT - 50, 35, 150, f"RESET", (180, 180, 180), (220, 220, 220), mouse_pos)
    calib_rect = draw_button(200, HEIGHT - 50, 35, 150, f"CALIBRATE", (180, 180, 180), (220, 220, 220), mouse_pos)

    if calibration_result:
        if 'error' in calibration_result:
            msg = font.render(f"ERROR: {calibration_result['error']}", True, (255, 80, 80))
            screen.blit(msg, (20, 70))
        else:
            pos = calibration_result['position']
            rot = calibration_result['rotation']

            screen.blit(font.render(f"x={pos['x']:.2f}, y={pos['y']:.2f}, z={pos['z']:.2f}", True, (255, 255, 255)), (20, 70))

            screen.blit(font.render(f"yaw={rot['yaw']:.2f}, pitch={rot['pitch']:.2f}, roll={rot['roll']:.2f}", True, (255, 255, 255)), (20, 100))

    pygame.display.flip()

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if reset_rect.collidepoint(mouse_pos):
                reset_points()
            elif calib_rect.collidepoint(mouse_pos):
                calibrate_camera()
            else:
                x, y = event.pos
                if current_point < len(world_points):
                    image_points.append((x * (4/3), y * (4/3)))
                    current_point += 1
                    logger.info(f"Point was set at x={x*(4/3):.0f}, y={y*(4/3):.0f}")
                else:
                    logger.warning(f"All points were already set")

    clock.tick(30)

cap.release()
pygame.quit()
logger.info("Program ended")