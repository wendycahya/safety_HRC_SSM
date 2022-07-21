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

#SSM variables
Vr = 2000
Vh = 1600
Tr = 0.41
ac = 2000
C = 1200
Zd = 90
Zr = 25

Sp = SSM_calculation(Vr, Vh, Tr, ac, C, Zd, Zr)
print("Separation Minimum Distance Calculation: ", Sp)

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
