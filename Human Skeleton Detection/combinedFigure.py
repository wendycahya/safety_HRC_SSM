import numpy as np
import matplotlib.pyplot as plt
import argparse
import cv2

import cvzone
from cvzone.FaceMeshModule import FaceMeshDetector
from datetime import datetime
import csv

Sp, Spfull, SpminVal, SpSafeVal, SpPFLVal = 0, 0, 0, 0, 0
def SSM_calculation(Vr, Vh, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr*Ts + ((ac*pow(Ts, 2))/2)
    Sp   = Vh * (Tr + Ts) + (Vr * Tr) + Ss + Ctot
    return Sp

def Vr_SSM(D, Vh, Tr, Ts, ac, C, Zd, Zr, Vr_PFL):
    T = Tr + Ts
    Ctot = C + Zd + Zr
    VrSSM = (((D - (Vh*T) - Ctot)) / T) - ((ac*pow(Ts, 2))/(2*T))
    if VrSSM < Vr_PFL:
        Reduce_Value = Vr_PFL
    else:
        Reduce_Value = VrSSM
    return Reduce_Value

def Vr_SSM2(D, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    VrSSM2 = (D / Ts) - ((ac*Ts)/2) - (Ctot/Ts)
    if VrSSM2 > 0:
        Stop_Value = VrSSM2
    else:
        Stop_Value = 0
    return Stop_Value

def SpMax(Vr_Max, Vh_Max, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_Max*Ts + ((ac*pow(Ts, 2))/2)
    SpMax   = Vh_Max * (Tr + Ts) + (Vr_Max * Tr) + Ss + Ctot
    return SpMax

def Spmin(C, Zd, Zr):
    SpminVal = C + Zd + Zr
    return SpminVal

def SpPFL(Vr_PFL, Vh, Tr, Ts, ac, C, Zd, Zr):
    global SpPFLVal
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts,2))/2)
    SpPFLVal   = Vh * ( Tr + Ts ) + (Vr_PFL * Tr) + Ss + Ctot
    return SpPFLVal

def SpSafe(Vr_PFL, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr_PFL*Ts + ((ac*pow(Ts, 2))/2)
    SpSafeVal = Ss + Ctot
    return SpSafeVal


#SSM variables
D = 0
VrPaper = 1000
Vr = 1500
Vr_PFL = 400
Tr = 0.1
Ts = 0.08
ac = 3000
C_SSM = 200
Zd = 106.7
Zr = 1

#velocity human
velHum_Ori = 1600
VrSSM = 0
VrSSM2 = 0

#Robot Velocity
RobotVrmax = 1500

Spfull = SpMax(RobotVrmax, velHum_Ori, Tr, Ts, ac, C_SSM, Zd, Zr)
SpminVal = Spmin(C_SSM, Zd, Zr)
SpPFLVal = SpPFL(Vr_PFL, velHum_Ori, Tr, Ts, ac, C_SSM, Zd, Zr)
SpSafeVal = SpSafe(Vr_PFL, Ts, ac, C_SSM, Zd, Zr)

start_time = datetime.now()
start = datetime.now()
milliseconds = 0
d = 0

#Device connection
fpsReader = cvzone.FPS()
cap = cv2.VideoCapture(0)
detector = FaceMeshDetector(maxFaces=1)

bins = 100
# Initialize plot.
fig, ax = plt.subplots()
par = ax.twinx()

ax.set_title('Collaborative Workspace Analysis')
ax.set_xlabel('time')
ax.set_ylabel('distance(mm)')
par.set_ylabel('speed (mm/s)')
# Initialize plot line object(s). Turn on interactive plotting and show plot.
lw = 3
alpha = 0.5
lineR, = ax.plot(np.arange(bins), np.zeros((bins,)), c='r', lw=lw, alpha=alpha, label='Red')
lineG, = par.plot(np.arange(bins), np.zeros((bins,)), c='g', lw=lw, alpha=alpha, label='Green')

ax.set_xlim(0, bins-1)
ax.set_ylim(0, 3000)
par.set_ylim(0, 2000)
ax.legend()
plt.ion()
plt.show()

# Grab, process, and display video frames. Update plot line object(s).
while True:
    success, img = cap.read()
    imgMesh, faces = detector.findFaceMesh(img, draw=False)
    fps, imgCap = fpsReader.update(img, pos=(20, 20), color=(0, 255, 0), scale=2, thickness=2)
    if faces:
        # skeleton detection
        face = faces[0]
        print(faces[0])
        pointLeft = face[145]
        pointRight = face[374]
        cv2.line(imgMesh, pointLeft, pointRight, (0, 200, 0), 3)
        cv2.circle(imgMesh, pointLeft, 5, (255, 0, 255), cv2.FILLED)
        cv2.circle(imgMesh, pointRight, 5, (255, 0, 255), cv2.FILLED)
        w, _ = detector.findDistance(pointLeft, pointRight)
        W = 6.3  # default real width eyes distance
        # Finding the focal length

        # d = 60  #distance human and camera
        # f = (w*d) / W
        # print(f)

        # finding distance
        f = 714  # finding the average for focal length
        d = ((W * f) / w) * 10
        offset = 500
        print(d)
        d = round(d - offset, 3)
        cvzone.putTextRect(img, f'Depth: {d} mm', (face[10][0] - 100, face[10][1] - 50), scale=1.5)
        if d < 0:
            d = 0
        else:
            d = abs(d)

        # logical SSM send robot
        if d <= SpminVal:
            # server.pause()
            Vr = 0
            speed = 0
            # print("Robot harus berhenti", Vr)
            mode_collab = 0
            # t.sleep(0.5)

        elif d > SpminVal and d <= SpSafeVal:
            # server.resume()
            # print("Robot speed reduction")
            Vr = Vr_SSM2(d, Tr, Ts, ac, C_SSM, Zd, Zr)
            Vr = round(Vr, 2)
            # speed = int(remap(Vr, 0, 1500, 0, 800))
            # calculate the Vmax allowable
            # print("Vmax allowable in this workspace: ", Vr_max_command)
            # Vr = Vr_max_command
            mode_collab = 1
            # print("change value speed safe: ", Vr)
            # t.sleep(0.5)

        elif d > SpSafeVal and d <= SpPFLVal:
            # server.resume()
            # print("Robot speed reduction")
            mode_collab = 2
            Vr = 400
            Vr = round(Vr, 2)
            # speed = int(remap(Vr, 0, 1500, 0, 800))
            # print("change value speed PFL: ", Vr)
            # t.sleep(0.5)

        elif d > SpPFLVal and d <= Spfull:
            # server.resume()
            Vr = Vr_SSM(d, velHum_Ori, Tr, Ts, ac, C_SSM, Zd, Zr, Vr_PFL)
            Vr = round(Vr, 2)
            # speed = int(remap(Vr, 0, 1500, 0, 800))
            # print("change value speed Reduce: ", Vr)
            mode_collab = 3
            # t.sleep(0.5)
        else:
            mode_collab = 4
            # print("Robot bekerja maximal")
            # mode_collab = 1
            Vr = RobotVrmax
            # speed = int(remap(Vr, 0, 1500, 0, 800))
            # print("change value speed maximum: ", Vr)
            # t.sleep(0.5)

    # Normalize histograms based on number of pixels per frame.
    cv2.imshow('RGB', img)
    lineR.set_ydata(d)
    lineG.set_ydata(Vr)

    fig.canvas.draw()

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()