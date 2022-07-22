# Import
import pygame
import cv2
import numpy as np
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector

# Initialize
pygame.init()

# Create Window/Display
width, height = 1000, 600
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("My Awesome Game")

# Initialize Clock for FPS
fps = 30
clock = pygame.time.Clock()

# Webcam
cap = cv2.VideoCapture(0)
cap.set(3, 640)  # width
cap.set(4, 480)  # height
#Device connection
fpsReader = cvzone.FPS()
detector = FaceMeshDetector(maxFaces=1)

# Main loop
start = True
while start:
    # Get Events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            start = False
            pygame.quit()

    # Apply Logic
    window.fill((255, 255, 255))
    red, green, blue = (255, 0, 0), (0, 255, 0), (0, 0, 255)

    # OpenCV
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=True)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
    imgRGB = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    imgRGB = np.rot90(imgRGB)
    frame = pygame.surfarray.make_surface(imgRGB).convert()
    frame = pygame.transform.flip(frame, True, False)



    window.blit(frame, (0, 0))


    # Update Display
    pygame.display.update()
    # Set FPS
    clock.tick(fps)