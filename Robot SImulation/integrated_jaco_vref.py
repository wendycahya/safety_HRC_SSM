import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import threading
from cvzone.PoseModule import PoseDetector
import cv2

# Position Data:
pickPos = [400, -150, 0, 180, 0, 0]
# naik tiap 100mm
liftPos11 = [400, -150, 100, 180, 0, 0]
liftPos12 = [400, -150, 200, 180, 0, 0]
liftPos13 = [400, -150, 300, 180, 0, 0]
liftPos14 = [400, -150, 400, 180, 0, 0]

liftPos21 = [500, -100, 500, 180, 60, 0]
liftPos22 = [500, -50, 500, 180, 60, 0]
liftPos23 = [500, 0,   500, 180, 60, 0]
liftPos24 = [500, 100, 500, 180, 60, 0]
liftPos25 = [500, 200, 500, 180, 60, 0]

goalPos = [500, 200, 0, 180, 0, 0]
liftPos2 = [500, 200, 500, 180, 60, 0]
liftPos3 = [500, 200, 100, 180, 0, 0]


# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)
def thread_robotMovement():
    jacoRobot.setSpeed(1200, 90)
    while True:
        jacoRobot.gripperRelease()
        jacoRobot.setPosition2(liftPos11, True)
        jacoRobot.setPosition2(pickPos, True)
        time.sleep(1)
        jacoRobot.gripperCatch()

        # Robot Naik
        jacoRobot.setPosition2(liftPos11, True)
        jacoRobot.setPosition2(liftPos12, True)
        jacoRobot.setPosition2(liftPos13, True)
        jacoRobot.setPosition2(liftPos14, True)

        # Robot Geser Kiri
        jacoRobot.setPosition2(liftPos21, True)
        jacoRobot.setPosition2(liftPos22, True)
        jacoRobot.setPosition2(liftPos23, True)
        jacoRobot.setPosition2(liftPos24, True)
        jacoRobot.setPosition2(liftPos25, True)

        # Robot Turun
        jacoRobot.setPosition2(liftPos2, True)
        jacoRobot.setPosition2(goalPos, True)
        jacoRobot.gripperRelease()
        time.sleep(1)
        jacoRobot.setPosition2(liftPos3, True)
# ====================================================




# INITIALIZATION:
# ====================================================
mSim=CoppeliaSim()
ret=mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)

# start thread:
t = threading.Thread(target=thread_robotMovement)
t.start()





# MAIN PRORGAM:
# ======================================================
# masukkan program utama disini (looping program)
cap = cv2.VideoCapture(0)
detector = PoseDetector()

while True:
    success, img = cap.read()
    img = detector.findPose(img)
    lmList, bboxInfo = detector.findPosition(img, bboxWithHands=False)
    if bboxInfo:
        center = bboxInfo["center"]
        cv2.circle(img, center, 5, (255, 0, 255), cv2.FILLED)

    cv2.imshow("Image", img)
    cv2.waitKey(1)
