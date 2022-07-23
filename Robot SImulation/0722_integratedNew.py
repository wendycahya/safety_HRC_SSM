#Robot library
import time as t
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
    midXY = [(a[0]+b[0])/2, (a[1]+b[1])/2]
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
C = 1200
Zd = 90
Zr = 25

#velocity
Xn1D = 0
Xn_last1D = 0
Xn = [0, 0, 0]
Xn_last = [0, 0, 0]
velX, velY, velZ = [0, 0, 0]
ts = 0.05

#velocity human
velHum = 1600

XnRob = [0, 0, 0]
XnRob_last = [0, 0, 0]
velXR, velYR, velZR = [0, 0, 0]
#Sp = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
interval = 0
Vrmax = 0
Vr_max_command = 0

#Robot Velocity

vrchest = 230
vrface = 60
vrstop = 0
vrmax = 2000
vrot = 90
velRob = 0
robotZ = 0

distView = 0
sampleDistance = 1
pause_active = 0
Spmax = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
Sp = Spmax
Scurrent = Spmax + 2000
Spmin = 0
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
write_file = "test 1.csv"
mode_collab = 0

# === coppelia connect
mSim = CoppeliaSim()
ret = mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")

movePos21 = [400, 200, 500, 180, 60, 0]
movePos22 = [400, 150, 500, 180, 60, 0]
movePos23 = [400, 100, 500, 180, 60, 0]
movePos24 = [400, 50, 500, 180, 60, 0]
movePos25 = [400, 0,   500, 180, 60, 0]
movePos26 = [400, -50, 500, 180, 60, 0]
movePos27 = [400, -100, 500, 180, 60, 0]
MovetoObj = [movePos21, movePos22, movePos23, movePos24, movePos25, movePos26, movePos27]
MovetoBack = [movePos27, movePos26, movePos25, movePos24, movePos23, movePos22, movePos21]

#object pic location
objPos1 = [500, 260, 0, 180, 0, 0]
objPos2 = [500, 260, 50, 180, 0, 0]
objPos3 = [500, 260, 100, 180, 0, 0]
objPos4 = [500, 260, 150, 180, 0, 0]
objPos5 = [500, 260, 200, 180, 0, 0]
ObjPick =[objPos5, objPos4, objPos3, objPos2, objPos1]
ObjRelease =[objPos1, objPos2, objPos3, objPos4, objPos5]

#Cube A location
APos11 = [400, -120, 400, 180, 0, 0]
APos12 = [400, -120, 350, 180, 0, 0]
APos13 = [400, -120, 300, 180, 0, 0]
APos14 = [400, -120, 250, 180, 0, 0]
APos15 = [400, -120, 200, 180, 0, 0]
APos16 = [400, -120, 150, 180, 0, 0]
APos17 = [400, -120, 100, 180, 0, 0]
APos18 = [400, -120, 50, 180, 0, 0]
APlace = [400, -120, 0, 180, 0, 0]
AObjPlace =[APos11, APos12, APos13, APos14, APos15, APos16, APos17, APos18, APlace]


#Cube B location
BPos11 = [550, -120, 400, 180, 0, 0]
BPos12 = [550, -120, 350, 180, 0, 0]
BPos13 = [550, -120, 300, 180, 0, 0]
BPos14 = [550, -120, 250, 180, 0, 0]
BPos15 = [550, -120, 200, 180, 0, 0]
BPos16 = [550, -120, 150, 180, 0, 0]
BPos17 = [550, -120, 100, 180, 0, 0]
BPos18 = [550, -120, 50, 180, 0, 0]
BPlace = [550, -120, 0, 180, 0, 0]
BObjPlace =[BPos11, BPos12, BPos13, BPos14, BPos15, BPos16, BPos17, BPos18, BPlace]

#Cube C location
CPos11 = [400, 40, 400, 180, 0, 0]
CPos12 = [400, 40, 350, 180, 0, 0]
CPos13 = [400, 40, 300, 180, 0, 0]
CPos14 = [400, 40, 250, 180, 0, 0]
CPos15 = [400, 40, 200, 180, 0, 0]
CPos16 = [400, 40, 150, 180, 0, 0]
CPos17 = [400, 40, 100, 180, 0, 0]
CPos18 = [400, 40, 50, 180, 0, 0]
CPlace = [400, 40, 0, 180, 0, 0]
CObjPlace =[CPos11, CPos12, CPos13, CPos14, CPos15, CPos16, CPos17, CPos18, CPlace]

#Cube D location
DPos11 = [550, 40, 400, 180, 0, 0]
DPos12 = [550, 40, 350, 180, 0, 0]
DPos13 = [550, 40, 300, 180, 0, 0]
DPos14 = [550, 40, 250, 180, 0, 0]
DPos15 = [550, 40, 200, 180, 0, 0]
DPos16 = [550, 40, 150, 180, 0, 0]
DPos17 = [550, 40, 100, 180, 0, 0]
DPos18 = [550, 40, 50, 180, 0, 0]
DPlace = [550, 40, 0, 180, 0, 0]
DObjPlace =[DPos11, DPos12, DPos13, DPos14, DPos15, DPos16, DPos17, DPos18, DPlace]

progress = [0, 0, 0, 0]
finish = [1, 1, 1, 1]
start_time = datetime.now()
class Job(threading.Thread):
    def __init__(self, *args, **kwargs):
        super(Job, self).__init__(*args, **kwargs)
        self.__flag = threading.Event()     # 用于暂停线程的标识
        self.__flag.set()       # 设置为True
        self.__running = threading.Event()      # 用于停止线程的标识
        self.__running.set()      # 将running设置为True

    def run(self):
        jacoRobot.setSpeed(1200, 90)
        while self.__running.isSet():
            print("reading condition", progress)
            jacoRobot.gripperRelease()
            for i in ObjPick:
                self.__flag.wait()
                jacoRobot.setPosition2(i, True)

            jacoRobot.gripperCatch()

            for i in ObjRelease:
                self.__flag.wait()
                jacoRobot.setPosition2(i, True)

            for i in MovetoObj:
                self.__flag.wait()
                jacoRobot.setPosition2(i, True)

            # condition A belum terisi
            if (progress[0] == 0):
                for i in AObjPlace:
                    self.__flag.wait()
                    jacoRobot.setPosition2(i, True)
                progress[0] = 1
            elif (progress[1] == 0):
                for i in BObjPlace:
                    self.__flag.wait()
                    jacoRobot.setPosition2(i, True)
                progress[1] = 1
            elif (progress[2] == 0):
                for i in CObjPlace:
                    self.__flag.wait()
                    jacoRobot.setPosition2(i, True)
                progress[2] = 1
            elif (progress[3] == 0):
                for i in DObjPlace:
                    self.__flag.wait()
                    jacoRobot.setPosition2(i, True)
                progress[3] = 1
            if (progress == finish):
                print("Robot Stop\n")
                jacoRobot.gripperRelease()
                for i in ObjPick:
                    self.__flag.wait()
                    jacoRobot.setPosition2(i, True)
                print(datetime.now() - start_time)
                break

    def pause(self):
        self.__flag.clear()     # 设置为False, 让线程阻塞

    def resume(self):
        self.__flag.set()    # 设置为True, 让线程停止阻塞

    def stop(self):
        self.__flag.set()       # 将线程从暂停状态恢复, 如何已经暂停的话
        self.__running.clear()        # 设置为False


if __name__ == '__main__':
    server = Job()
    server.start()
    # MAIN PRORGAM:
    # ===== camera installation =====
    fpsReader = cvzone.FPS()
    cap = cv2.VideoCapture(0)
    detector = FaceMeshDetector(maxFaces=1)
    detectFace = FaceDetector()
    print("Nilai S Current adalah ", Scurrent)
    # ======================================================
    # masukkan program utama disini (looping program)
    with open(write_file, "wt", encoding="utf-8") as output:
        while True:
            # Detect human skeleton
            success, img = cap.read()
            imgMesh, faces = detector.findFaceMesh(img, draw=False)
            imgFace, bboxs = detectFace.findFaces(img)
            # imList, boxPose = detectPose.findPosition(imgPose, bboxWithHands=False)
            # curRobotPos = jacoRobot.readPosition()
            # jacoRobot.setSpeed(10, 90)
            # print("Posisi terbaca", curRobotPos)

            # === Robot analysis Velocity ===
            curRobotPos = jacoRobot.readPosition()
            print("robot position ", curRobotPos[0], curRobotPos[1], curRobotPos[2])

            XnRob = [curRobotPos[0], curRobotPos[1], curRobotPos[2]]
            velR = velXYZ(XnRob, XnRob_last, ts)
            print("Robot velocity X Y Z: ", velR[0], velR[1], velR[2])

            VelRnew = (velR[0] + velR[1] + velR[2]) / 3
            VelRnew = abs(VelRnew)
            print("Robor average velocity", VelRnew)
            # ===== Visualization information ======
            # ===== background information =====
            cv2.rectangle(img, (0, 0), (650, 110), (255, 255, 255), -1)

            # ===== information visualization =====
            # ====== left information
            cv2.putText(img, 'Vh = ' + str(round(velHum, 4)) + ' mm/s',
                        (10, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )
            cv2.putText(img, 'Vr = ' + str(round(velRob, 4)) + ' mm/s',
                        (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )
            cv2.putText(img, 'Vrmax = ' + str(round(Vr_max_command, 4)) + ' mm/s',
                        (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )
            cv2.putText(img, 'Robot (z) = ' + str(round(robotZ, 4)) + ' mm',
                        (10, 80),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )

            # ==== right information
            cv2.putText(img, 'Sp = ' + str(Sp) + ' mm',
                        (410, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )
            cv2.putText(img, 'Sp min = ' + str(Spmin) + ' mm',
                        (410, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )
            cv2.putText(img, 'HR distance = ' + str(round(Scurrent, 4)) + ' mm',
                        (410, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        )

            if faces:
                with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                    # skeleton detection
                    face = faces[0]
                    # print(faces[0])
                    pointLeft = face[145]
                    pointRight = face[374]
                    # v2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
                    # cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
                    # cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
                    w, _ = detector.findDistance(pointLeft, pointRight)
                    # center = bboxs[0]["center"]
                    # print(center)
                    # cv2.circle(img, center, 8, (255, 0, 255), cv2.FILLED)
                    # print(w)
                    W = 6.3  # default real width eyes distance
                    # Finding the focal length
                    # d = 60  #distance human and camera
                    # f = (w*d) / W
                    # print(f)
                    # finding distance
                    f = 714  # finding the average for focal length
                    d = (W * f) / w
                    # print(d)
                    d = d * 10  # distance in mm
                    eye_dist = round(d, 3)

                    # skeleton mediapipe migrasion
                    # Recolor image to RGB
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    img.flags.writeable = False
                    # Make detection
                    results = pose.process(img)
                    # Recolor back to BGR
                    img.flags.writeable = True
                    img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    # Extract landmarks
                    try:
                        landmarks = results.pose_landmarks.landmark
                        # Get coordinates
                        xyzFoot = [landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].x,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].y,
                                   landmarks[mp_pose.PoseLandmark.RIGHT_FOOT_INDEX.value].z]

                        xyzKnee = [landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].x,
                                   landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].y,
                                   landmarks[mp_pose.PoseLandmark.LEFT_KNEE.value].z]

                        xyzNose = [landmarks[mp_pose.PoseLandmark.NOSE.value].x,
                                   landmarks[mp_pose.PoseLandmark.NOSE.value].y,
                                   landmarks[mp_pose.PoseLandmark.NOSE.value].z]
                        # landmarks shoulder left and right
                        rightShoulder = [landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].x,
                                         landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].y,
                                         landmarks[mp_pose.PoseLandmark.RIGHT_SHOULDER.value].z]
                        leftShoulder = [landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].x,
                                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].y,
                                        landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER.value].z]

                        rightHip = [landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].x,
                                    landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].y,
                                    landmarks[mp_pose.PoseLandmark.RIGHT_HIP.value].z]
                        leftHip = [landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].x,
                                   landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].y,
                                   landmarks[mp_pose.PoseLandmark.LEFT_HIP.value].z]

                        # Calculate angle
                        Shodis, Shomid = center_point(leftShoulder, rightShoulder)
                        Hipdis, Hipmid = center_point(leftHip, rightHip)

                        # distance mid
                        middis, midCoor = center_point(Shomid, Hipmid)

                        # read the Xn
                        RAWdist = round(Shomid[2] * 100, 3)
                        distanceCM = A * RAWdist ** 2 + B * RAWdist + C
                        # shoulder distance
                        Xn1D = round(Shodis, 4)
                        vel = (Xn1D - Xn_last1D) / ts

                        vel = abs(vel)
                        # Xn = [round(Shomid[0],4), round(Shomid[1],4), round(distanceCM,4)]
                        # velX, velY, veLZ = velXYZ(Xn, Xn_last, ts)
                        # print("Nilai Vel x: ", velX , "Nilai Vel y: ", velY ," Nilai Vel z:", velZ)

                        # Scol active pada saat terdapat vr pada rentang kecepatan
                        # Human Height detection
                        noseLoc = Ah * xyzNose[1] ** 2 + Bh * xyzNose[1] + Ch
                        shoulderLoc = Ah * Shomid[1] ** 2 + Bh * Shomid[1] + Ch
                        hipsLoc = Ah * Hipmid[1] ** 2 + Bh * Hipmid[1] + Ch

                        zRob = curRobotPos[2]
                        minHead = 1500
                        maxHead = 1800
                        zHead = [minHead, maxHead]
                        minChest = 1000
                        maxChest = 1400
                        zChest = [minChest, maxChest]

                        # print("lokasi hidung ", noseLoc)
                        # print("lokasi mid shoulder ", shoulderLoc)
                        # print("lokasi mid hips ", hipsLoc)

                        # ===== SSM calculation ======
                        Sp = SSM_calculation(VelRnew, vel, Tr, ac, C, Zd, Zr)
                        Sp = round(Sp, 4)
                        disHR = distanceCM / 100

                        Spmin = vel + C + Zd + Zr
                        Spspace = Sp - Spmin
                        Scol = Spmin + Spspace
                        # separation protective condition
                        # if Spmin > disHR:
                        #    cv2.putText(image, 'Mode = STOPPPPPPPPPPP',
                        #            (420,60),
                        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        #            )
                        #    cv2.circle(image, (380,40), radius = 10, color =(0,0,255), thickness = 20)

                        Vr_max_command = Vr_max(Sp, Vh, VelRnew, Tr, ac, C, Zd, Zr)

                        # ===== information visualization =====
                        # left monitoring input
                        velHum = vel * 1000
                        velRob = VelRnew
                        ShodisXY, ShoXYmid = center_pointXY(leftShoulder, rightShoulder)
                        robotZ = 1000 + curRobotPos[2]

                        #  right monitoring output

                        Spmin = round(Spmin, 4)

                        # Skeleton visualization
                        cv2.putText(img, str(Shodis),
                                    tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                    )

                        cv2.circle(img, tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        # Visualize
                        cv2.putText(img, str(Hipdis),
                                    tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2, cv2.LINE_AA
                                    )

                        cv2.circle(img, tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        cv2.line(img, tuple(np.multiply([Shomid[0], Shomid[1]], [640, 480]).astype(int)),
                                 tuple(np.multiply([Hipmid[0], Hipmid[1]], [640, 480]).astype(int)), (255, 255, 0),
                                 thickness=2)

                        cv2.circle(img, tuple(np.multiply([midCoor[0], midCoor[1]], [640, 480]).astype(int)), radius=5,
                                   color=(255, 255, 0), thickness=10)

                        # visualization for eyes distance
                        # cvzone.putTextRect(img, f'Depth: {eye_dist} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
                        cv2.putText(img, 'Current speed = ' + str(round(Vr, 4)) + ' mm/s',
                                    (410, 100),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        print("Nilai S Current adalah ", Scurrent)
                        Scurrent = eye_dist
                        #if pause_active == 1:
                        #   server.resume()
                        # logical SSM send robot
                        if Scurrent < Spmin:
                            server.pause()
                            print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0
                            cv2.putText(img, 'Mode = Robot Stop',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(0, 0, 255), thickness=20)
                            jacoRobot.message("Robot stop")
                            t.sleep(0.5)
                        elif Spmin <= Scurrent and Scol > Scurrent:
                            server.resume()
                            print("Robot working on collaboration mode")
                            mode_collab = 3
                            cv2.putText(img, 'Mode = Collaboration',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(255, 0, 0), thickness=20)

                            if zHead[0] < zRob and zHead[1] >= zRob:
                                print("Velocity limitation on head area: ", vrface)
                                Vr = vrface
                                jacoRobot.setSpeed(Vr, vrot)
                                print("Succes send speed VrFace")
                            elif zChest[0] < zRob and zChest[1] >= zRob:
                                print("Velocity limitation on chest: ", vrchest)
                                Vr = vrchest
                                jacoRobot.setSpeed(Vr, vrot)
                                print("Succes send speed VrChest")
                            else:
                                Vr = Vr_max_command
                                if Vr_max_command <= vrface:
                                    Vr = 1000
                                    jacoRobot.setSpeed(Vr, vrot)
                                    print("Succes send speed Vr Command")
                            jacoRobot.message("Collaboration speed")
                            t.sleep(0.5)
                        elif Scol <= Scurrent and Sp + 500 >= Scurrent:
                            server.resume()
                            print("Robot speed reduction")
                            mode_collab = 2
                            # calculate the Vmax allowable
                            print("Vmax allowable in this workspace: ", Vr_max_command)
                            # Vr = Vr_max_command
                            Vr = 1000
                            jacoRobot.setSpeed(Vr, vrot)
                            print("Succes send speed Vr Mid")
                            cv2.putText(img, 'Mode = Reduction Area',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(0, 255, 255), thickness=20)
                            jacoRobot.message("Robot speed reduction")
                            t.sleep(0.5)
                        else:
                            server.resume()
                            print("Robot bekerja maximal")
                            mode_collab = 1
                            Vr = vrmax
                            jacoRobot.setSpeed(Vr, vrot)
                            print("Succes send speed Vr Full Speed")
                            cv2.putText(img, 'Mode = Free Full Speed',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(0, 255, 0), thickness=20)
                            jacoRobot.message("Robot free speed")
                            t.sleep(0.5)

                        t.sleep(ts)
                        # Xn_last = Xn
                        Xn_last1D = Xn1D
                        XnRob_last = XnRob
                    except:
                        if Scurrent < Spmin:
                            server.pause()
                            print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0
                            cv2.putText(img, 'Mode = Robot Stop',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(0, 0, 255), thickness=20)
                            jacoRobot.message("Robot stop")
                            print("Be careful! Sensor not detected")
                            jacoRobot.message("Your position is too close. Be careful sensor not detected")
                            t.sleep(0.5)
                        elif Scurrent > Sp:
                            server.resume()
                            print("Robot bekerja maximal")
                            mode_collab = 1
                            Vr = vrmax
                            jacoRobot.setSpeed(Vr, vrot)
                            print("Succes send speed Vr Full Speed")
                            cv2.putText(img, 'Mode = Free Full speed',
                                        (420, 80),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                            cv2.circle(img, (380, 80), radius=10, color=(0, 255, 0), thickness=20)
                            jacoRobot.message("Robot free speed")
                            t.sleep(0.5)

                    t.sleep(0.5)
                    # distance calculation robot speed

                # Render detections
                mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                          mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                          )

            # ===== research documentation =====
            interval = interval + 1
            output.write(str(interval) + ',' + str(Scurrent) + ',' + str(Sp) + ',' + str(mode_collab) + '\n')
            print("Scurrent update ", Scurrent)
            cv2.imshow("SSM Application", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        cap.release()
        cv2.destroyAllWindows()