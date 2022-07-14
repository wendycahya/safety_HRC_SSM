import numpy as np
import math as mt
import time as t
import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot

#the unit present in mm
#function program
def SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr):
    Tb = Vr / ac
    Ss  = pow(Vr,2) / (2*ac)
    Ctot = C + Zd + Zr
    Sp = Vh * ( Tr + Tb ) + (Vr * Tr) + Ss + Ctot
    return Sp

def Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr):
    Ts = Vr / ac
    T = Tr + Ts
    Ctot = C + Zd + Zr
    Vrmax = ((Sp - (Vh*T) - Ctot) / T) - (ac*pow(Ts,2)/(2*T))
    return Vrmax

Vr = 2000
Vh = 1600
Tr = 0.41
ac = 2000
C  = 100
Zd = 90
Zr = 25

vrchest = 230
vrface = 60
vrstop = 0
vrmax = 2000

Sp = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
print("Separation Minimum Distance Calculation: ", Sp)

Scurrent = 4000
print("Current Distance: ", Scurrent)

Spmin = Vh*Tr + C + Zd + Zr
print("Minimum Protection: ", Spmin)

Spspace = Sp - Spmin
print("Distance Protection: ", Spspace)

mSim = CoppeliaSim()
mSim.connect(19997)
ur10_robot = CoppeliaArmRobot("UR10")
#Scol active pada saat terdapat vr pada rentang kecepatan
zRob = 170
minHead = 150
maxHead = 180
zHead = [minHead, maxHead]
minChest = 100
maxChest = 140
zChest = [minChest, maxChest]

Spspace = Sp - Spmin
Scol = Spmin + Spspace
print("Distance for collaboration: ", Scol)

#logical SSM send robot
if Scurrent < Spmin:
    print("Robot harus berhenti", vrstop)
    Vr = vrstop
    ur10_robot.setSpeed(Vr, 1)

elif Spmin <= Scurrent and Scol  > Scurrent:
    print("Robot working on collaboration mode")
    if zHead[0] < zRob and zHead[1] >= zRob:
        print("Velocity limitation on head area: ", vrface)
        Vr = vrface
        ur10_robot.setSpeed(Vr, 1)
    elif zChest[0] < zRob and zChest[1] >= zRob:
        print("Velocity limitation on chest: ", vrchest)
        Vr = vrchest
        ur10_robot.setSpeed(Vr, 1)
    else:
        Vr_max_command = Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr)
        print("Vmax allowable in this workspace: ", Vr_max_command)
        Vr = Vr_max_command
        ur10_robot.setSpeed(Vr, 1)

elif Scol <= Scurrent and Sp + 500 >= Scurrent:
    print("Robot speed reduction")
    # calculate the Vmax allowable
    Vr_max_command = Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr)
    print("Vmax allowable in this workspace: ", Vr_max_command)
    Vr = Vr_max_command
    ur10_robot.setSpeed(Vr, 1)
else:
    print("Robot bekerja maximal")
    Vr = vrmax
    ur10_robot.setSpeed(Vr, 1)
