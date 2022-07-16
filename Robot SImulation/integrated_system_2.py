#the parameter unit presents in milimeters (mm)
# ===== library integration =====
# camera library
import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from cvzone.FaceDetectionModule import FaceDetector
import cv2

#simulation library
import time as t
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot

#additional
import mediapipe as mp
import numpy as np
import math as mt
import csv

# ===== function program =====
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
write_file = "output_SSM_separation_distance.csv"
mode_collab = 0
# ===== camera installation =====
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)
detectFace = FaceDetector()

# ===== robot connection =====
mSim = CoppeliaSim()
mSim.connect(19997)
ur10_robot = CoppeliaArmRobot("UR10")

# ===== looping system =====
# open record csv
with open(write_file, "wt", encoding="utf-8") as output:
    while True:
    # Detect human skeleton
        success, img = cap.read()
        imgMesh, faces = detector.findFaceMesh(img, draw=False)
        imgFace, bboxs = detectFace.findFaces(img)
        #imList, boxPose = detectPose.findPosition(imgPose, bboxWithHands=False)

        if faces:
            with mp_pose.Pose(min_detection_confidence=0.5, min_tracking_confidence=0.5) as pose:
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
                d = d * 10 # distance in mm
                eye_dist = round(d, 3)

           #skeleton mediapipe migrasion
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

                    curRobotPos = ur10_robot.readPosition()
                    print("robot position ", curRobotPos[0], curRobotPos[1], curRobotPos[2])

                    XnRob = [curRobotPos[0], curRobotPos[1], curRobotPos[2]]
                    velR = velXYZ(XnRob, XnRob_last, ts)
                    print("Robot velocity X Y Z: ", velR[0], velR[1], velR[2])

                    # Scol active pada saat terdapat vr pada rentang kecepatan

                    minChest = Ah * Hipmid[1] ** 2 + Bh * Hipmid[1] + Ch
                    maxChest = Ah * Shomid[1] ** 2 + Bh * Shomid[1] + Ch

                    nosePos = Ah * xyzNose[1] ** 2 + Bh * xyzNose[1] + Ch

                    zRob = curRobotPos[2]
                    minHead = (xyzNose - 200) * 1000
                    maxHead = (xyzNose + 200) * 1000
                    zHead = [minHead, maxHead]
                    minChest = minChest * 1000
                    maxChest = maxChest * 1000
                    zChest = [minChest, maxChest]

                    print("Batas bawah human: ", minChest, maxChest)

                    VelRnew = (velR[0]*velR[1]*velR[2]) / 3
                    VelRnew = abs(VelRnew)


                # Human Height detection
                    noseLoc = Ah * xyzNose[1] ** 2 + Bh * xyzNose[1] + Ch
                    shoulderLoc = Ah * Shomid[1] ** 2 + Bh * Shomid[1] + Ch
                    hipsLoc = Ah * Hipmid[1] ** 2 + Bh * Hipmid[1] + Ch
                    kneeLoc = Ah * xyzKnee[1] ** 2 + Bh * xyzKnee[1] + Ch

                    #print("lokasi hidung ", noseLoc)
                    #print("lokasi mid shoulder ", shoulderLoc)
                    #print("lokasi mid hips ", hipsLoc)
                    #print("lokasi knee ", kneeLoc)

                # ===== SSM calculation ======
                    Sp = SSM_calculation(VelRnew, Vh, Tr, ac, C, Zd, Zr)
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


    # ===== Visualization information ======
                # ===== background information =====
                    cv2.rectangle(img, (8, 0), (250, 110), (255, 255, 255), -1)
                    cv2.rectangle(img, (350, 0), (650, 110), (255, 255, 255), -1)

                # ===== information visualization =====
                # left monitoring input
                    velHum = vel * 1000
                    cv2.putText(img, 'Vh = ' + str(round(velHum, 4)) + ' mm/s',
                                (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    velRob = VelRnew
                    cv2.putText(img, 'Vr = ' + str(round(velRob, 4)) + ' mm/s',
                                (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    cv2.putText(img, 'Vrmax = ' + str(round(Vr_max_command, 4)) + ' mm/s',
                                (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    ShodisXY, ShoXYmid = center_pointXY(leftShoulder, rightShoulder)
                    robotZ = curRobotPos[2]
                    cv2.putText(img, 'Robot (z) = ' + str(round(robotZ, 4)) + ' mm',
                                (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )


                #  right monitoring output
                    cv2.putText(img, 'Sp = ' + str(Sp) + ' mm',
                                (410, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    Spmin = round(Spmin, 4) + 300
                    cv2.putText(img, 'Sp min = ' + str(Spmin) + ' mm',
                                (410, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    distView = eye_dist
                    cv2.putText(img, 'HR distance = ' + str(round(distView, 4)) + ' mm',
                                (410, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

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
                    #cvzone.putTextRect(img, f'Depth: {eye_dist} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
                    cv2.putText(img, 'Current speed = ' + str(round(Vr, 4)) + ' mm/s',
                                (410, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    Scurrent = eye_dist

                    # logical SSM send robot
                    if Scurrent < Spmin:
                        print("Robot harus berhenti", vrstop)
                        mode_collab = 4
                        Vr = vrstop
                        ur10_robot.setSpeed(Vr, 1)
                        cv2.putText(img,'Mode = Robot Stop',
                                    (420, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 80), radius=10, color=(0, 0, 255), thickness=20)

                    elif Spmin <= Scurrent and Scol > Scurrent:
                        print("Robot working on collaboration mode")
                        print("Batas bawah human: ", minChest, maxChest)
                        mode_collab = 3
                        cv2.putText(img, 'Mode = Collaboration',
                                    (420, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 80), radius=10, color=(255, 0, 0), thickness=20)

                        if zHead[0] < zRob and zHead[1] >= zRob:
                            print("Velocity limitation on head area: ", vrface)
                            Vr = vrface
                            ur10_robot.setSpeed(Vr, 1)
                            cv2.putText(img, 'Warning: Face area',
                                        (420, 100),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )
                        elif zChest[0] < zRob and zChest[1] >= zRob:
                            print("Velocity limitation on chest: ", vrchest)
                            Vr = vrchest
                            ur10_robot.setSpeed(Vr, 1)
                            cv2.putText(img, 'Warning: Chest area',
                                        (10, 100),
                                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                        )

                    elif Scol <= Scurrent and Sp + 500 >= Scurrent:
                        print("Robot speed reduction")
                        mode_collab = 2
                        # calculate the Vmax allowable
                        print("Vmax allowable in this workspace: ", Vr_max_command)
                        Vr = Vr_max_command
                        ur10_robot.setSpeed(Vr, 1)
                        cv2.putText(img, 'Mode = Reduction Area',
                                    (10, 100),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 80), radius=10, color=(0, 255, 255), thickness=20)

                    else:
                        print("Robot bekerja maximal")
                        mode_collab = 1
                        Vr = vrmax
                        ur10_robot.setSpeed(Vr, 1)
                        cv2.putText(img, 'Mode = Free Collision',
                                    (420, 80),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 80), radius=10, color=(0, 255, 0), thickness=20)

                    t.sleep(ts)
                    # Xn_last = Xn
                    Xn_last1D = Xn1D
                    XnRob_last = XnRob
                except:
                    pass

                t.sleep(0.5)
                #distance calculation robot speed


            # Render detections
            mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                      mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                      )

        # ===== research documentation =====
            interval = interval + 1
            #note robot mode
            # 1: free speed, 2: reduce speed, 3: collaboration mode, 4: stop
            output.write(str(interval) + ',' + str(distView) + ',' + str(distanceCM*10) +'\n')

            cv2.imshow("SSM Application", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()