import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
import cv2

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)

while True:
# Detect human skeleton
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=True)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)

    if faces:
#skeleton detection
        face = faces[0]
    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()