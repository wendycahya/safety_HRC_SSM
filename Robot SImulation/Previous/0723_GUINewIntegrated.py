#Robot library
import math
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

#GUI
import pygame

#====================INITIALIZE GUI=========================
# Initialize
pygame.init()

# Create Window/Display
width, height = 1280, 700
window = pygame.display.set_mode((width, height))
pygame.display.set_caption("Human Robot Safety Collaboration")

# Initialize Clock for FPS
fps = 30
clock = pygame.time.Clock()

#font and coloring
font = pygame.font.Font('assets/Inter-SemiBold.otf', 24)
font_reg = pygame.font.Font('assets/Inter-Regular.otf', 24)
green, yellow, blue, red, purple, gray = (20, 128, 10), (236, 190, 35), (0, 103, 230), (197, 54, 55), (123, 97, 255), (229, 229, 229)
window.fill((255, 255, 255))

# ===layout interface===
pygame.draw.rect(window, gray, (667, 50, 590, 213), border_radius=5)
pygame.draw.rect(window, gray, (18, 513, 640, 172), border_radius=5)
pygame.draw.rect(window, gray, (667, 276, 590, 409), border_radius=5)
#mode collaboration
pygame.draw.rect(window, purple, (442, 523, 203, 152), border_radius=5)

#warning head and chest
pygame.draw.rect(window, purple, (1104, 414, 148, 127), border_radius=5)
pygame.draw.rect(window, gray, (1112, 444, 134, 95), border_radius=5)
text = font.render("WARNING!!!", True, (242, 242, 247))
window.blit(text, (1112, 414))

#img assets
imgKinova = pygame.image.load("assets/kinova.png").convert()
imgKinova = pygame.transform.scale(imgKinova, (166, 167))
window.blit(imgKinova, (693, 89))
imgHuman = pygame.image.load("assets/human.png").convert()
imgHuman = pygame.transform.scale(imgHuman, (361, 360))
window.blit(imgHuman, (693, 325))

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
text_yRobot = font_reg.render("y:", True, (50, 50, 50))
window.blit(text_yRobot, (825, 156))
text_zRobot = font_reg.render("z:", True, (50, 50, 50))
window.blit(text_zRobot, (825, 194))
text_rmRobot = font_reg.render("Robot Movement", True, (50, 50, 50))
window.blit(text_rmRobot, (1006, 89))
text_spRobot = font_reg.render("Speed:", True, (50, 50, 50))
window.blit(text_spRobot, (1006, 125))
text_vrRobot = font_reg.render("Vr:", True, (50, 50, 50))
window.blit(text_vrRobot, (1006, 156))

# ====Human domain===
text_Human = font.render("Human Domain", True, (50, 50, 50))
window.blit(text_Human, (693, 292))
text_mvHuman = font_reg.render("Human Movement", True, (50, 50, 50))
window.blit(text_mvHuman, (987, 292))
text_fHuman = font_reg.render("FACE", True, (50, 50, 50))
window.blit(text_fHuman, (860, 329))
text_fminHuman = font_reg.render("min:", True, (50, 50, 50))
window.blit(text_fminHuman, (860, 358))
text_fmaxHuman = font_reg.render("max:", True, (50, 50, 50))
window.blit(text_fmaxHuman, (860, 391))
text_cHuman = font_reg.render("CHEST", True, (50, 50, 50))
window.blit(text_cHuman, (860, 435))
text_cminHuman = font_reg.render("min:", True, (50, 50, 50))
window.blit(text_cminHuman, (860, 472))
text_cmaxHuman = font_reg.render("max:", True, (50, 50, 50))
window.blit(text_cmaxHuman, (860, 505))
text_vhHuman = font_reg.render("Vh:", True, (50, 50, 50))
window.blit(text_vhHuman, (987, 329))


# ====SSM Information===
text_SSMInf = font.render("SSM Information", True, (50, 50, 50))
window.blit(text_SSMInf, (28, 519))
text_currDistance = font_reg.render("Current Distance:", True, (50, 50, 50))
window.blit(text_currDistance, (28, 555))
text_spSSM = font_reg.render("Sp:", True, (50, 50, 50))
window.blit(text_spSSM, (28, 584))
text_spminSSM = font_reg.render("Sp min:", True, (50, 50, 50))
window.blit(text_spminSSM, (28, 616))
text_vrMax = font_reg.render("Vr Max:", True, (50, 50, 50))
window.blit(text_vrMax, (28, 646))


# ===mode collaboration===
text_mode = font_reg.render("Mode", True, (242, 242, 247))
window.blit(text_mode, (511, 526))
#robot task
pygame.draw.rect(window, purple, (913, 547, 339, 127), border_radius=5)
pygame.draw.rect(window, gray, (1077, 557, 166, 110), border_radius=5)

text_Rtask = font.render("Robot Task", True, (242, 242, 247))
window.blit(text_Rtask, (929, 557))


# =================function SSM =============================
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
    velX = (Xn_last[0] - Xn[0]) / ts
    velY = (Xn_last[1] - Xn[1]) / ts
    velZ = (Xn_last[2] - Xn[2]) / ts

    return velX, velY, velZ

def Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr):
    Ts = Vr / ac
    T = Tr + Ts
    Ctot = C + Zd + Zr
    Vrmax = ((Sp - (Vh*T) - Ctot) / T) - (ac*pow(Ts, 2)/(2*T))
    return Vrmax

def Spmin(Vh, Tr, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    SpminVal = Vh * Tr + Ctot
    return SpminVal

# ===== initialization & variables declaration =====
#SSM variables
Vrinitial = 200
Vr = Vrinitial
Vh = 1600
Tr = 0.41
ac = 200
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
vrchest = 100
vrface = 60
vrstop = 0
vrmax = 200
vrot = 90
velRob = 0
robotZ = 0
vel = 0
RobotVrmax = 200


distView = 0
sampleDistance = 1
pause_active = 0

Spmax = SSM_calculation(Vrinitial, Vh, Tr, ac, C, Zd, Zr)
Sp = Spmax
Scurrent = Spmax + 2000
SpminInitial = Spmin(Vh, Tr, ac, C, Zd, Zr)
SpminVal = SpminInitial
#calibration position variable
zHead = [0, 0]
zChest = [0, 0]

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

progress = [1, 1, 1, 0]
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
        jacoRobot.setSpeed(200, 90)
        pygame.draw.rect(window, purple, (929, 602, 140, 29), border_radius=5)
        text_process = font_reg.render("PROCESS", True, (242, 242, 247))
        window.blit(text_process, (940, 600))
        pygame.draw.rect(window, gray, (1119, 562, 99, 99), border_radius=5)
        imgPro = pygame.image.load("assets/process.png").convert()
        imgPro = pygame.transform.scale(imgPro, (99, 99))
        window.blit(imgPro, (1119, 562))
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
                finish_task = datetime.now() - start_time
                finish_task = str(finish_task)
                print(datetime.now() - start_time)
                pygame.draw.rect(window, gray, (1119, 562, 99, 99), border_radius=5)
                imgSuc = pygame.image.load("assets/success.png").convert()
                imgSuc = pygame.transform.scale(imgSuc, (99, 99))
                window.blit(imgSuc, (1119, 562))
                pygame.draw.rect(window, purple, (929, 602, 140, 29), border_radius=5)
                text_process = font_reg.render("FINISH", True, (242, 242, 247))
                window.blit(text_process, (940, 600))
                pygame.draw.rect(window, purple, (916, 638, 157, 29), border_radius=5)
                text_process = font_reg.render(finish_task[0:9], True, (242, 242, 247))
                window.blit(text_process, (922, 638))
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
    cap.set(3, 640)  # width
    cap.set(4, 480)  # height

    detector = FaceMeshDetector(maxFaces=1)
    detectFace = FaceDetector()
    print("Nilai S Current adalah ", Scurrent)
    # ======================================================
    # masukkan program utama disini (looping program)
    with open(write_file, "wt", encoding="utf-8") as output:
        while True:
            # Get Events
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    start = False
                    pygame.quit()

    # ================= Apply Logic =================
            # Detect human skeleton
            success, img = cap.read()
            imgMesh, faces = detector.findFaceMesh(img, draw=False)
            imgFace, bboxs = detectFace.findFaces(img)

            # === Robot analysis Velocity ===
            curRobotPos = jacoRobot.readPosition()
            print("robot position ", curRobotPos[0], curRobotPos[1], curRobotPos[2])
            xRob = round(curRobotPos[0], 2)
            yRob = round(curRobotPos[1], 2)
            zRob = round(curRobotPos[2], 2)
            XnRob = [xRob, yRob, zRob]
            print("Robot Position X Y Z: ", xRob, yRob, zRob)
            print("Robot Last Position X Y Z: ", XnRob_last[0], XnRob_last[1], XnRob_last[2])
            velR = velXYZ(XnRob, XnRob_last, ts)
            print("Robot velocity X Y Z: ", velR[0], velR[1], velR[2])
            # velR[0] = round(velR[0], 2)
            # velR[1] = round(velR[1], 2)
            # velR[2] = round(velR[2], 2)
            VelRnew = math.sqrt(velR[0]**2 + velR[1]**2 + velR[2]**2)
            VelRnew = abs(VelRnew)
            if VelRnew > RobotVrmax:
                VelRnew = RobotVrmax
            print("Robot average velocity", VelRnew)
            SpStatis = SSM_calculation(Vrinitial, Vh, Tr, ac, C, Zd, Zr)
            print("SSM Statis", SpStatis)
            # ===== SSM calculation ======
            Sp = SSM_calculation(VelRnew, Vh, Tr, ac, C, Zd, Zr)
            print("SSM Dynamic", Sp)

            if faces:
                with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
                    # skeleton detection
                    face = faces[0]
                    # print(faces[0])
                    pointLeft = face[145]
                    pointRight = face[374]
                    w, _ = detector.findDistance(pointLeft, pointRight)
                    W = 6.3  # default real width eyes distance
                    f = 714  # finding the average for focal length
                    d = (W * f) / w
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

                        zRob = curRobotPos[2] + 1000
                        minHead = noseLoc - 150
                        maxHead = noseLoc + 150
                        zHead = [minHead, maxHead]
                        minChest = hipsLoc
                        maxChest = shoulderLoc
                        zChest = [minChest, maxChest]


                        #Sp = 400
                        disHR = distanceCM / 100
                        #Spmin = 100
                        SpminVal = Spmin(Vh, Tr, ac, C, Zd, Zr)
                        if SpminVal > Sp:
                            SpminVal = SpminInitial
                        # separation protective condition
                        # if Spmin > disHR:
                        #    cv2.putText(image, 'Mode = STOPPPPPPPPPPP',
                        #            (420,60),
                        #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                        #            )
                        #    cv2.circle(image, (380,40), radius = 10, color =(0,0,255), thickness = 20)

                        Vr_max_command = Vr_max(Sp, Vh, VelRnew, Tr, ac, C, Zd, Zr)
                        Vr_max_command = abs(Vr_max_command)
                        # ===== information visualization =====
                        # left monitoring input
                        velHum = vel * 1000
                        velRob = VelRnew
                        ShodisXY, ShoXYmid = center_pointXY(leftShoulder, rightShoulder)


                        #  right monitoring output

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


                        print("Nilai S Current adalah ", Scurrent)
                        Scurrent = eye_dist
                        Scurrent = round(Scurrent, 2)

                        # logical SSM send robot
                        if Scurrent < SpminVal:
                            server.pause()
                            print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_stop = font_reg.render("Stop", True, (242, 242, 247))
                            window.blit(text_stop, (467, 555))
                            pygame.draw.rect(window, red, (460, 588, 166, 81), border_radius=5)

                            jacoRobot.message("Robot stop")
                            t.sleep(0.5)
                        elif SpminVal <= Scurrent and Sp > Scurrent:
                            server.resume()
                            print("Robot working on collaboration mode")
                            mode_collab = 3
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Collaboration", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, blue, (460, 588, 166, 81), border_radius=5)

                            if zHead[0] < zRob and zHead[1] >= zRob:
                                print("Velocity limitation on head area: ", vrface)
                                pygame.draw.rect(window, gray, (1134, 442, 90, 90), border_radius=5)
                                imgHead = pygame.image.load("assets/head.png").convert()
                                imgHead = pygame.transform.scale(imgHead, (90, 90))
                                window.blit(imgHead, (1134, 442))
                                Vr = vrface
                                jacoRobot.setSpeed(Vr, vrot)
                                print("Succes send speed VrFace")
                                t.sleep(0.5)

                            elif zChest[0] < zRob and zChest[1] >= zRob:
                                print("Velocity limitation on chest: ", vrchest)
                                pygame.draw.rect(window, gray, (1134, 442, 90, 90), border_radius=5)
                                imgHead = pygame.image.load("assets/chest.png").convert()
                                imgHead = pygame.transform.scale(imgHead, (90, 90))
                                window.blit(imgHead, (1134, 442))
                                Vr = vrchest
                                jacoRobot.setSpeed(Vr, vrot)
                                print("Succes send speed VrChest")
                                t.sleep(0.5)
                            else:
                                Vr = Vr_max_command
                                if Vr_max_command <= vrface:
                                    Vr = 100
                                    jacoRobot.setSpeed(Vr, vrot)
                                    pygame.draw.rect(window, gray, (1134, 442, 90, 90), border_radius=5)
                                    imgHead = pygame.image.load("assets/safe.png").convert()
                                    imgHead = pygame.transform.scale(imgHead, (90, 90))
                                    window.blit(imgHead, (1134, 442))
                                    print("Succes send speed Vr Command")
                                    t.sleep(0.5)
                            jacoRobot.message("Collaboration speed")
                            t.sleep(0.5)
                        elif Sp <= Scurrent and Sp + 150 >= Scurrent:
                            server.resume()
                            print("Robot speed reduction")
                            mode_collab = 2
                            # calculate the Vmax allowable
                            print("Vmax allowable in this workspace: ", Vr_max_command)
                            # Vr = Vr_max_command
                            Vr = 100
                            jacoRobot.setSpeed(Vr, vrot)
                            print("Succes send speed Vr Mid")
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_reduce = font_reg.render("Reduce Speed", True, (242, 242, 247))
                            window.blit(text_reduce, (467, 555))
                            pygame.draw.rect(window, yellow, (460, 588, 166, 81), border_radius=5)
                            jacoRobot.message("Robot speed reduction")
                            t.sleep(0.5)
                        else:
                            server.resume()
                            print("Robot bekerja maximal")
                            mode_collab = 1
                            Vr = vrmax
                            jacoRobot.setSpeed(Vr, vrot)
                            print("Succes send speed Vr Full Speed")
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_freespeed = font_reg.render("Full Speed", True, (242, 242, 247))
                            window.blit(text_freespeed, (467, 555))
                            pygame.draw.rect(window, green, (460, 588, 166, 81), border_radius=5)
                            jacoRobot.message("Robot free speed")
                            t.sleep(0.5)

                        t.sleep(ts)
                        # Xn_last = Xn
                        Xn_last1D = Xn1D
                        XnRob_last = XnRob
                    except:
                        if Scurrent < SpminVal:
                            server.pause()
                            print("Robot harus berhenti", vrstop)
                            mode_collab = 4
                            Vr = 0
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Stop", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, red, (460, 588, 166, 81), border_radius=5)
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
                            pygame.draw.rect(window, purple, (467, 555, 165, 29), border_radius=5)
                            text_coll = font_reg.render("Full Speed", True, (242, 242, 247))
                            window.blit(text_coll, (467, 555))
                            pygame.draw.rect(window, green, (460, 588, 166, 81), border_radius=5)
                            jacoRobot.message("Robot free speed")
                            t.sleep(0.5)

                    t.sleep(0.5)
                    # distance calculation robot speed

                # Render detections
                mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                          mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                          mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                          )
            pygame.draw.rect(window, gray, (1134, 442, 90, 90), border_radius=5)
            imgHead = pygame.image.load("assets/safe.png").convert()
            imgHead = pygame.transform.scale(imgHead, (90, 90))
            window.blit(imgHead, (1134, 442))
            # ===== research documentation =====
            interval = interval + 1
            output.write(str(interval) + ',' + str(Scurrent) + ',' + str(Sp) + ',' + str(mode_collab) + '\n')
            print("Scurrent update ", Scurrent)

            # ======= Information Visualization =========

            # SSM output value
            pygame.draw.rect(window, gray, (233, 555, 196, 29), border_radius=5)
            text_currval = font_reg.render(str(Scurrent) + " mm", True, (50, 50, 50))
            window.blit(text_currval, (233, 555))

            Sp = round(Sp, 2)
            pygame.draw.rect(window, gray, (74, 584, 196, 29), border_radius=5)
            text_Spval = font_reg.render(str(Sp) + " mm", True, (50, 50, 50))
            window.blit(text_Spval, (74, 584))

            SpminVal = round(SpminVal, 2)
            pygame.draw.rect(window, gray, (115, 616, 196, 29), border_radius=5)
            text_Spminval = font_reg.render(str(SpminVal) + " mm", True, (50, 50, 50))
            window.blit(text_Spminval, (115, 616))

            Vr_max_command = round(Vr_max_command, 2)
            pygame.draw.rect(window, gray, (118, 647, 196, 29), border_radius=5)
            text_vmaxcmdval = font_reg.render(str(Vr_max_command) + " mm/s", True, (50, 50, 50))
            window.blit(text_vmaxcmdval, (118, 647))

            # ============== Robot Domain ===============
            # +1000 depends on the table height
            curRobotPos[0] = round(curRobotPos[0] + 1000, 1)
            curRobotPos[1] = round(curRobotPos[1] + 1000, 1)
            curRobotPos[2] = round(curRobotPos[2] + 1000, 1)
            pygame.draw.rect(window, gray, (845, 119, 150, 29), border_radius=5)
            text_xRval = font_reg.render(str(curRobotPos[0]) + " mm", True, (50, 50, 50))
            window.blit(text_xRval, (855, 119))

            pygame.draw.rect(window, gray, (845, 156, 150, 29), border_radius=5)
            text_yRval = font_reg.render(str(curRobotPos[1]) + " mm", True, (50, 50, 50))
            window.blit(text_yRval, (855, 156))

            pygame.draw.rect(window, gray, (845, 194, 150, 29), border_radius=5)
            text_zRval = font_reg.render(str(curRobotPos[2]) + " mm", True, (50, 50, 50))
            window.blit(text_zRval, (855, 194))

            Vr = round(Vr, 2)
            pygame.draw.rect(window, gray, (1088, 125, 165, 29), border_radius=5)
            text_speedRval = font_reg.render(str(Vr) + " mm/s", True, (50, 50, 50))
            window.blit(text_speedRval, (1088, 125))

            VelRnew = round(VelRnew, 2)
            pygame.draw.rect(window, gray, (1045, 160, 170, 29), border_radius=5)
            text_veloRval = font_reg.render(str(VelRnew) + " mm/s", True, (50, 50, 50))
            window.blit(text_veloRval, (1045, 156))

            # === Human domain
            zHead[0] = abs(round(zHead[0] * 10, 2))
            if zHead[0] > 2000:
                zHead[0] = 1800
            pygame.draw.rect(window, gray, (929, 358, 165, 29), border_radius=5)
            text_minHead = font_reg.render(str(zHead[0]) + " mm", True, (50, 50, 50))
            window.blit(text_minHead, (929, 358))
            if zHead[1] > 2000:
                zHead[1] = 1800
            zHead[1] = abs(round(zHead[1] * 10, 2))
            pygame.draw.rect(window, gray, (929, 391, 165, 29), border_radius=5)
            text_maxHead = font_reg.render(str(zHead[1]) + " mm", True, (50, 50, 50))
            window.blit(text_minHead, (929, 391))

            if zChest[0] > 2000:
                zChest[0] = 1800
            zChest[0] = abs(round(zChest[0] * 10, 2))
            pygame.draw.rect(window, gray, (935, 475, 165, 29), border_radius=5)
            text_minHead = font_reg.render(str(zChest[0]) + " mm", True, (50, 50, 50))
            window.blit(text_minHead, (935, 475))

            if zChest[1] > 2000:
                zChest[1] = 1800
            zChest[1] = abs(round(zChest[1] * 10, 2))
            pygame.draw.rect(window, gray, (935, 508, 165, 29), border_radius=5)
            text_minHead = font_reg.render(str(zChest[1]) + " mm", True, (50, 50, 50))
            window.blit(text_minHead, (935, 508))

            velHum = abs(round(velHum, 2))
            pygame.draw.rect(window, gray, (1031, 329, 165, 29), border_radius=5)
            text_velHum = font_reg.render(str(velHum) + " mm/s", True, (50, 50, 50))
            window.blit(text_velHum, (1031, 329))

            imgRGB = np.rot90(img)
            frame = pygame.surfarray.make_surface(imgRGB).convert()
            frame = pygame.transform.flip(frame, True, False)

            window.blit(frame, (18, 18))

            # Update Display
            pygame.display.update()
            # Set FPS
            clock.tick(fps)