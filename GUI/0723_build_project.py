# Import
import pygame
import cv2
import numpy as np
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector

# Initialize
pygame.init()

# Create Window/Display
width, height = 1225, 700
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("Human Robot Safety Collaboration")

# Initialize Clock for FPS
fps = 30
clock = pygame.time.Clock()

# Webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height
#coloring
font = pygame.font.Font('assets/Inter-SemiBold.otf', 24)
font_reg = pygame.font.Font('assets/Inter-Regular.otf', 24)
red, green, gray = (255, 0, 0), (0, 255, 0), (229, 229, 229)
window.fill((255, 255, 255))

#Device connection
fpsReader = cvzone.FPS()
detector = FaceMeshDetector(maxFaces=1)

# ===layout interface===
pygame.draw.rect(window, gray, (677, 50, 533, 213), border_radius=5)
pygame.draw.rect(window, gray, (18, 513, 640, 172), border_radius=5)
pygame.draw.rect(window, gray, (677, 276, 533, 409), border_radius=5)

# ===Title Text===
text = font.render("Safety Human Robot Collaboration", True, (50, 50, 50))
window.blit(text, (748, 13))

# ====Robot domain===
text_Robot = font.render("Robot Domain", True, (50, 50, 50))
window.blit(text_Robot, (693, 59))
text_RobotPos = font_reg.render("Robot Position", True, (50, 50, 50))
window.blit(text_RobotPos, (822, 89))
text_xRobot = font_reg.render("x:", True, (50, 50, 50))
window.blit(text_xRobot, (825, 119))


# Main loop
start = True

while start:
    # Get Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            start = False
            pygame.quit()

    # Apply Logic

    # OpenCV
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=True)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    imgRGB = np.rot90(imgRGB)
    frame = pygame.surfarray.make_surface(imgRGB).convert()
    frame = pygame.transform.flip(frame, True, False)


    window.blit(frame, (18, 18))


    # Update Display
    pygame.display.update()
    # Set FPS
    clock.tick(fps)