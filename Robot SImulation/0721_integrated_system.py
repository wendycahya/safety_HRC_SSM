#Robot library
import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import threading
from datetime import datetime

#camera library
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.FaceDetectionModule import FaceDetector
import cv2

#computation
import mediapipe as mp
import numpy as np
import math as mt
import csv

#=================function SSM =============================
def SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr):
    Tb = Vr / ac
    Ss  = pow(Vr, 2) / (2*ac)
    Ctot = C + Zd + Zr
    Sp = Vh * ( Tr + Tb ) + (Vr * Tr) + Ss + Ctot
    return Sp

def center_point(a,b):
    a = np.array(a)
    b = np.array(b)
    d = mt.sqrt(mt.pow((a[0] - b[0]),2) + mt.pow((a[1] - b[1]),2) + mt.pow((a[2] - b[2]),2))
    mid = [(a[0]+b[0])/2 , (a[1]+b[1])/2, (a[2]+b[2])/2]
    return d, mid

def center_pointXY(a,b):
    a = np.array(a)
    b = np.array(b)
    dXY = mt.sqrt(mt.pow((a[0] - b[0]),2) + mt.pow((a[1] - b[1]),2))
    midXY = [(a[0]+b[0])/2 , (a[1]+b[1])/2]
    return dXY, midXY

def velXYZ(Xn, Xn_last, ts):
    velX = (Xn[0] - Xn_last[0]) / ts
    velY = (Xn[1] - Xn_last[1]) / ts
    velZ = (Xn[2] - Xn_last[2]) / ts

    return velX, velY, velZ

def Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr):
    Ts = Vr / ac
    T = Tr + Ts
    Ctot = C + Zd + Zr
    Vrmax = ((Sp - (Vh*T) - Ctot) / T) - (ac*pow(Ts, 2)/(2*T))
    return Vrmax

# ===== initialization & variables declaration =====
#SSM variables
Vr = 2000
Vh = 1600
Tr = 0.41
ac = 2000
C = 1000
Zd = 90
Zr = 25

#velocity
Xn1D = 0
Xn_last1D = 0
Xn = [0, 0, 0]
Xn_last = [0, 0, 0]
velX, velY, velZ = [0, 0, 0]
ts = 0.05

XnRob = [0, 0, 0]
XnRob_last = [0, 0, 0]
velXR, velYR, velZR = [0, 0, 0]
#Sp = SSM_calculation(Vr, Vh, Tr, ac, Ch, Cr)
interval = 0
Vrmax = 0
Vr_max_command = 0

vrchest = 230
vrface = 60
vrstop = 1
vrmax = 2000

distView = 0
sampleDistance = 1
#distance measurement
#x shoulder in cm
#y real measurement
x = [-2, -4, -9, -15, -17, -23, -38, -50, -76, -80]
y = [150, 140, 130, 120, 110, 70, 60, 50, 40, 30]

coff = np.polyfit(x, y, 2)
A, B, C = coff

xHeight = [0.2707, 0.2799, 0.2703, 0.2716, 0.29167, 0.634, 0.634, 0.6299, 0.6025, 0.6072, 0.1201, 0.1188, 0.0990, 0.1237, 0.1666]
yHeight = [140, 140, 140, 140, 140, 90, 90, 90, 90, 90, 150, 150, 150, 150, 150]

coffHeight = np.polyfit(xHeight, yHeight, 2)
Ah, Bh, Ch = coffHeight

#initialization
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

#information
write_file = "output_eye_humanmidpointdistance.csv"
mode_collab = 0

#====================ROBOT POSITION==========================
start = datetime.now()

# Position Data:
pickPos = [400, -150, 0, 180, 0, 0]
# naik tiap 100mm
liftPos11 = [400, -150, 100, 180, 0, 0]
liftPos12 = [400, -150, 200, 180, 0, 0]
liftPos13 = [400, -150, 300, 180, 0, 0]
liftPos14 = [400, -150, 400, 180, 0, 0]

# movement to goal place
liftPos21 = [500, -100, 500, 180, 60, 0]
liftPos22 = [500, -50, 500, 180, 60, 0]
liftPos23 = [500, 0,   500, 180, 60, 0]
liftPos24 = [500, 100, 500, 180, 60, 0]
liftPos25 = [500, 200, 500, 180, 60, 0]

goalPos = [500, 200, 0, 180, 0, 0]
liftPos2 = [500, 200, 500, 180, 60, 0]
liftPos3 = [500, 200, 100, 180, 0, 0]


#object pic location
objPos1 = [500, 260, 0, 180, 0, 0]
objPos2 = [500, 260, 50, 180, 0, 0]
objPos3 = [500, 260, 100, 180, 0, 0]
objPos3 = [500, 260, 300, 180, 0, 0]

#movement to goal place
movePos21 = [400, 200, 500, 180, 60, 0]
movePos22 = [400, 100, 500, 180, 60, 0]
movePos23 = [400, 0,   500, 180, 60, 0]
movePos24 = [400, -50, 500, 180, 60, 0]
movePos25 = [400, -100, 500, 180, 60, 0]


#Cube A location
APos11 = [400, -120, 400, 180, 0, 0]
APos12 = [400, -120, 300, 180, 0, 0]
APos13 = [400, -120, 200, 180, 0, 0]
APos14 = [400, -120, 100, 180, 0, 0]
APlace = [400, -120, 0, 180, 0, 0]

#Cube B location
BPos11 = [550, -120, 400, 180, 0, 0]
BPos12 = [550, -120, 300, 180, 0, 0]
BPos13 = [550, -120, 200, 180, 0, 0]
BPos14 = [550, -120, 100, 180, 0, 0]
BPlace = [550, -120, 0, 180, 0, 0]


#Cube C location
CPos11 = [400, 40, 400, 180, 0, 0]
CPos12 = [400, 40, 300, 180, 0, 0]
CPos13 = [400, 40, 200, 180, 0, 0]
CPos14 = [400, 40, 100, 180, 0, 0]
CPlace = [400, 40, 0, 180, 0, 0]

#Cube D location
DPos11 = [550, 40, 400, 180, 0, 0]
DPos12 = [550, 40, 300, 180, 0, 0]
DPos13 = [550, 40, 200, 180, 0, 0]
DPos14 = [550, 40, 100, 180, 0, 0]
DPlace = [550, 40, 0, 180, 0, 0]

def moveToGoal():
    jacoRobot.setPosition2(movePos21, True)
    jacoRobot.setPosition2(movePos22, True)
    jacoRobot.setPosition2(movePos23, True)
    jacoRobot.setPosition2(movePos24, True)
    jacoRobot.setPosition2(movePos25, True)

def moveToBack():
    jacoRobot.setPosition2(movePos25, True)
    jacoRobot.setPosition2(movePos24, True)
    jacoRobot.setPosition2(movePos23, True)
    jacoRobot.setPosition2(movePos22, True)
    jacoRobot.setPosition2(movePos21, True)

def pick_goDown():
    jacoRobot.setPosition2(objPos3, True)
    jacoRobot.setPosition2(objPos1, True)

def pick_goUp():
    jacoRobot.setPosition2(objPos2, True)
    jacoRobot.setPosition2(objPos3, True)

def place_A():
    jacoRobot.setPosition2(APos11, True)
    jacoRobot.setPosition2(APos12, True)
    jacoRobot.setPosition2(APos13, True)
    jacoRobot.setPosition2(APos14, True)
    jacoRobot.setPosition2(APlace, True)

def place_B():
    jacoRobot.setPosition2(BPos11, True)
    jacoRobot.setPosition2(BPos12, True)
    jacoRobot.setPosition2(BPos13, True)
    jacoRobot.setPosition2(BPos14, True)
    jacoRobot.setPosition2(BPlace, True)

def place_C():
    jacoRobot.setPosition2(CPos11, True)
    jacoRobot.setPosition2(CPos12, True)
    jacoRobot.setPosition2(CPos13, True)
    jacoRobot.setPosition2(CPos14, True)
    jacoRobot.setPosition2(CPlace, True)

def place_D():
    jacoRobot.setPosition2(DPos11, True)
    jacoRobot.setPosition2(DPos12, True)
    jacoRobot.setPosition2(DPos13, True)
    jacoRobot.setPosition2(DPos14, True)
    jacoRobot.setPosition2(DPlace, True)


progress = [0, 0, 0, 0]
finish = [1, 1, 1, 1]

# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)
def thread_robotMovement():
    jacoRobot.setSpeed(1200, 90)
    while True:
        print("reading the condition", progress)

        jacoRobot.gripperRelease()
        pick_goDown()
        jacoRobot.gripperCatch()
        pick_goUp()
        moveToGoal()

        # condition A belum terisi
        if (progress == [0, 0, 0, 0]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 1, 0, 0]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 0, 1, 0]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 0, 0, 1]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 1, 1, 0]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 1, 0, 1]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 0, 1, 1]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        elif (progress == [0, 1, 1, 1]):
            progress[0] = 1
            print("robot mengisi progress[0]", progress)
            place_A()
        # condition B belum terisi
        elif (progress == [1, 0, 0, 0]):
            progress[1] = 1
            print("robot mengisi progress[1]", progress)
            place_B()
        elif (progress == [1, 0, 1, 0]):
            progress[1] = 1
            print("robot mengisi progress[1]", progress)
            place_B()
        elif (progress == [1, 0, 0, 1]):
            progress[1] = 1
            print("robot mengisi progress[1]", progress)
            place_B()
        elif (progress == [1, 0, 1, 1]):
            progress[1] = 1
            print("robot mengisi progress[1]", progress)
            place_B()
        # condition C belum terisi
        elif (progress == [1, 1, 0, 0]):
            progress[2] = 1
            print("robot mengisi progress[2]", progress)
            place_C()
        elif (progress == [1, 1, 0, 1]):
            progress[2] = 1
            print("robot mengisi progress[2]", progress)
            place_C()
        # condition D belum terisi
        elif (progress == [1, 1, 1, 0]):
            progress[3] = 1
            print("robot mengisi progress[3]", progress)
            place_D()
        if (progress == finish):
            print("Robot Stop\n")
            jacoRobot.gripperRelease()
            pick_goUp()
            print(datetime.now() - start)
            break

        jacoRobot.gripperRelease()
        time.sleep(1)

        moveToBack()

# ====================================================




# ROBOT INITIALIZATION:
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
# ===== camera installation =====
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)
detectFace = FaceDetector()

# ======================================================
# masukkan program utama disini (looping program)
with open(write_file, "wt", encoding="utf-8") as output:
    while True:
    # Detect human skeleton
        success, img = cap.read()
        imgMesh, faces = detector.findFaceMesh(img, draw=False)
        imgFace, bboxs = detectFace.findFaces(img)
        # imList, boxPose = detectPose.findPosition(imgPose, bboxWithHands=False)
        #curRobotPos = jacoRobot.readPosition()
        #jacoRobot.setSpeed(10, 90)
        #print("Posisi terbaca", curRobotPos)
        interval = interval + 1
        #output.write(str(interval) + ',' + str(curRobotPos[0])+ ',' + str(curRobotPos[1]) + ',' + str(curRobotPos[2]) + '\n')

        cv2.imshow("SSM Application", img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()