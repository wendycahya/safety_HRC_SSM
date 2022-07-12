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

#function program
def SSM_calculation(Vr, Vh, Tr, ac, Ch, Cr):
    Tb = Vr / ac
    B  = pow(Vr,2) / (2*ac)
    Cz = Ch + Cr
    Sp = Vh * ( Tr + Tb ) + (Vr * Tr) + B + Cz
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

def Vr_max2(a,b,c):
    a = float(a) ; b = float(b) ; c = float(c)
    D = b**2 - 4*a*c
    x1 = (-b + mt.sqrt(D)) / (2*a)
    x2 = (-b - mt.sqrt(D)) / (2*a)
    hasil = (x1,x2)
    return hasil


#SSM variables
Vr = 0
Vh = 0
ac = 5
Tr = 0.41
Ch = 0.09
Cr = 0.025

#velocity
Xn1D = 0
Xn_last1D = 0
Xn = [0, 0, 0]
Xn_last = [0, 0, 0]
velX, velY, velZ = [0, 0, 0]
ts = 0.05
#Sp = SSM_calculation(Vr, Vh, Tr, ac, Ch, Cr)
interval = 0
Vrmax = 0

sampleDistance = 1
#distance measurement
#x shoulder in cm
#y real measurement
x = [-2, -4, -9, -15, -17, -23, -38, -50, -76, -80]
y = [150, 140, 130, 120, 110, 70, 60, 50, 40, 30]

coff = np.polyfit(x, y, 2)



#initialization
mp_drawing = mp.solutions.drawing_utils
mp_pose = mp.solutions.pose

write_file = "output.csv"

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)
detectFace = FaceDetector()

mSim = CoppeliaSim()
mSim.connect(19997)
ur10_robot = CoppeliaArmRobot("UR10")
#Record data csv opening
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
                dist = round(d, 3)
                cvzone.putTextRect(img, f'Depth: {dist} cm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)



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
                    A, B, C = coff
                    RAWdist = round(Shomid[2] * 100, 3)
                    distanceCM = A * RAWdist ** 2 + B * RAWdist + C
                    # shoulder distance
                    Xn1D = round(Shodis, 4)
                    vel = (Xn1D - Xn_last1D) / ts
                    vel = abs(vel)
                    # Xn = [round(Shomid[0],4), round(Shomid[1],4), round(distanceCM,4)]
                    # velX, velY, veLZ = velXYZ(Xn, Xn_last, ts)
                    # print("Nilai Vel x: ", velX , "Nilai Vel y: ", velY ," Nilai Vel z:", velZ)

                    Sp = SSM_calculation(Vr, vel, Tr, ac, Ch, Cr)
                    Sp = round(Sp, 4)
                    disHR = distanceCM / 100

                    # background information
                    cv2.rectangle(img, (8, 0), (250, 120), (255, 255, 255), -1)
                    cv2.rectangle(img, (360, 0), (600, 80), (255, 255, 255), -1)
                    # information visualization
                    # left align
                    cv2.putText(img, 'Vh = ' + str(round(vel, 5)) + ' m/s',
                                (10, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    cv2.putText(img, 'Face area = ' + str(round(Shodis, 5)) + ' m2',
                                (10, 40),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    ShodisXY, ShoXYmid = center_pointXY(leftShoulder, rightShoulder)
                    cv2.putText(img, 'Distance (z) = ' + str(round(distanceCM, 4)) + ' cm',
                                (10, 100),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    # chest area
                    chest_area = middis * Shodis
                    cv2.putText(img, 'Chest area = ' + str(round(chest_area, 5)) + ' m2',
                                (10, 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )

                    cv2.putText(img, 'Vrmax = ' + str(round(Vr, 5)) + ' m/s',
                                (10, 80),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    # right align
                    cv2.putText(img, 'Sp = ' + str(Sp) + ' m',
                                (425, 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                )
                    cv2.putText(img, 'HR dist = ' + str(round(disHR, 4)) + ' m',
                                (425, 40),
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


                    Spmin = vel + Ch + Cr
                    # separation protective condition
                    # if Spmin > disHR:
                    #    cv2.putText(image, 'Mode = STOPPPPPPPPPPP',
                    #            (420,60),
                    #            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                    #            )
                    #    cv2.circle(image, (380,40), radius = 10, color =(0,0,255), thickness = 20)

                    if disHR < Spmin:
                        #robot stop
                        print("robot stop")
                    elif Sp >= disHR and Spmin >= disHR:
                        #collaboration area

                        cv2.putText(img, 'Mode = Collaboration',
                                    (420, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 40), radius=10, color=(0, 0, 255), thickness=20)

                        # condition if z robot greater then z chest human area
                        # elseif z robot greater then z face human area
                        # else

                    elif Sp + 0.0001 < disHR and Sp + 0.1 > disHR:
                        cv2.putText(img, 'Mode = Reduction Area',
                                    (420, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 40), radius=10, color=(0, 255, 255), thickness=20)
                    else:
                        cv2.putText(img, 'Mode = Free Collision',
                                    (420, 60),
                                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 1, cv2.LINE_AA
                                    )
                        cv2.circle(img, (380, 40), radius=10, color=(0, 255, 0), thickness=20)

                    t.sleep(ts)
                    # Xn_last = Xn
                    Xn_last1D = Xn1D
                except:
                    pass


        #Human velocity

        #Robot velocity read and robot position comparison

        #Human and robot distance calculation

        #SSM Calculation

        #Vmax allowable

        #area calculation
                print("lokasi hidung ", xyzNose[1])
                print("lokasi shoulder", Shomid[1])
                print("lokasi hips ", Hipmid[1])

                t.sleep(0.5)
                #distance calculation robot speed
                Sp = 50
                if dist < Sp:
                    #speed 0
                    ur10_robot.setSpeed(10, 1)
                else:
                    ur10_robot.setSpeed(1000, 1)
                    #speed max

            interval = interval + 1
            # Render detections
            mp_drawing.draw_landmarks(img, results.pose_landmarks, mp_pose.POSE_CONNECTIONS,
                                      mp_drawing.DrawingSpec(color=(245, 117, 66), thickness=2, circle_radius=2),
                                      mp_drawing.DrawingSpec(color=(245, 66, 230), thickness=2, circle_radius=2)
                                      )

            output.write(str(interval) + ',' + str(Shodis) + ',' + str(vel) + '\n')


            cv2.imshow("SSM Application", img)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()