import numpy as np
import math as mt
import time as t

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

Vr = 2
Vh = 1.6
Tr = 0.41
ac = 2
C  = 0.1
Zd = 0.09
Zr = 0.025

Sp = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
print("Separation Minimum Distance Calculation: ", Sp)

Scurrent = 4.1

Spmin = Vh*Tr + C + Zd + Zr
print("Minimum Protection: ", Spmin)

Spspace = Sp - Spmin
print("Distance Protection: ", Spspace)



#Scol active pada saat terdapat vr pada rentang kecepatan
zRob = 0.12
minHead = 0.15
maxHead = 0.18
zHead = [minHead, maxHead]
minChest = 0.10
maxChest = 0.14
zChest = [minChest, maxChest]

Spspace = Sp - Spmin
Scol = Spmin + Spspace
print("Distance for collaboration: ", Scol)

#logical SSM send robot
if Scurrent < Spmin:
    print("Robot harus berhenti")
elif Spmin <= Scurrent and Scol  > Scurrent:
    print("Robot working on collaboration mode")
    if zHead[0] < zRob and zHead[1] >= zRob:
        print("Velocity limitation on head area")
    elif zChest[0] < zRob and zChest[1] >= zRob:
        print("Velocity limitation on chest")
    else:
        Vr_max_command = Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr)
        print("Vmax allowable in this workspace: ", Vr_max_command)
elif Scol <= Scurrent and Sp + 0.5 >= Scurrent:
    print("Robot speed reduction")
    # calculate the Vmax allowable
    Vr_max_command = Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr)
    print("Vmax allowable in this workspace: ", Vr_max_command)
else:
    print("Free Speed")
