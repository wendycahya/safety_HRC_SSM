import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.FaceDetectionModule import FaceDetector
from cvzone.PoseModule import PoseDetector
import cv2

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)
detectFace = FaceDetector()
detectPose = PoseDetector()
#Record data csv opening
while True:
# Detect human skeleton
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=False)
    imgFace, bboxs = detectFace.findFaces(img)
    imgPose = detectPose.findPose(img)
    #imList, boxPose = detectPose.findPosition(imgPose, bboxWithHands=False)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)

    if faces:
#skeleton detection
        face = faces[0]
        #print(faces[0])
        pointLeft = face[145]
        pointRight = face[374]
        #v2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
        #cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
        #cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
        w, _ = detector.findDistance(pointLeft, pointRight)
        #center = bboxs[0]["center"]
        #print(center)
        #cv2.circle(img, center, 8, (255, 0, 255), cv2.FILLED)
        #print(w)
        W = 6.3  # default real width eyes distance
        #Finding the focal length

        #d = 60  #distance human and camera
        #f = (w*d) / W
        #print(f)

        #finding distance
        f = 714 #finding the average for focal length
        d = (W*f) / w
        #print(d)
        dist = round(d, 3)
        cvzone.putTextRect(img, f'Depth: {dist} cm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)

#Human velocity

#Robot velocity read and robot position comparison

#Human and robot distance calculation

#SSM Calculation

#Vmax allowable

#area calculation




    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()