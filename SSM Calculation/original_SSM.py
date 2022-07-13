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

Scurrent = 5

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
