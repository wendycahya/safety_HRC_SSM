import numpy as np
import math as mt
import time as t

#function program
def SSM_calculation(Vr, Vh, Tr, Ts, ac, C, Zd, Zr):
    Ctot = C + Zd + Zr
    Ss   = Vr*Ts + ((ac*pow(Ts, 2))/2)
    Sp   = Vh * (Tr + Ts) + (Vr * Tr) + Ss + Ctot
    return Sp

def Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr):
    Ts = Vr / ac
    T = Tr + Ts
    Ctot = C + Zd + Zr
    Vrmax = ((Sp - (Vh*T) - Ctot) / T) - (ac*pow(Ts,2)/(2*T))
    return Vrmax

#SSM variables
Vr = 750
Vh = 1600
Tr = 0.1
Ts = 0.08
ac = 3000
C_SSM = 200
Zd = 106.7
Zr = 1

Sp = SSM_calculation(Vr, Vh, Tr, Ts, ac, C_SSM, Zd, Zr)
print("Separation Minimum Distance Calculation: ", Sp+750)

Scurrent = 6000

#logical SSM send robot
if Scurrent < Sp:
    print("Robot harus berhenti")
elif Sp <= Scurrent and Sp + 0.5 >= Scurrent:
    print("Robot speed reduction")
    # calculate the Vmax allowable
    Vr_max_command = Vr_max(Sp, Vh, Vr, Tr, ac, C, Zd, Zr)
    print("Vmax allowable in this workspace: ", Vr_max_command)

else:
    print("Free Speed")
