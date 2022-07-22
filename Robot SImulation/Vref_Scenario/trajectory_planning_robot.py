import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import threading

# Position Data:
pickPos = [400, -150, 0, 180, 0, 0]
# naik tiap 100mm
liftPos11 = [400, -150, 100, 180, 0, 0]
liftPos12 = [400, -150, 200, 180, 0, 0]
liftPos13 = [400, -150, 300, 180, 0, 0]
liftPos14 = [400, -150, 400, 180, 0, 0]

# movement to goal place
liftPos21 = [500, -100, 500, 180, 60, 0]
liftPos22 = [500, -50, 500, 180, 60, 0]
liftPos23 = [500, 0,   500, 180, 60, 0]
liftPos24 = [500, 100, 500, 180, 60, 0]
liftPos25 = [500, 200, 500, 180, 60, 0]

goalPos = [500, 200, 0, 180, 0, 0]
liftPos2 = [500, 200, 500, 180, 60, 0]
liftPos3 = [500, 200, 100, 180, 0, 0]


#object pic location
objPos1 = [500, 260, 0, 180, 0, 0]
objPos2 = [500, 260, 50, 180, 0, 0]
objPos3 = [500, 260, 100, 180, 0, 0]
objPos3 = [500, 260, 300, 180, 0, 0]

#movement to goal place
movePos21 = [400, 200, 500, 180, 60, 0]
movePos22 = [400, 100, 500, 180, 60, 0]
movePos23 = [400, 0,   500, 180, 60, 0]
movePos24 = [400, -50, 500, 180, 60, 0]
movePos25 = [400, -100, 500, 180, 60, 0]


#Cube A location
APos11 = [400, -120, 400, 180, 0, 0]
APos12 = [400, -120, 300, 180, 0, 0]
APos13 = [400, -120, 200, 180, 0, 0]
APos14 = [400, -120, 100, 180, 0, 0]
APlace = [400, -120, 0, 180, 0, 0]

#Cube B location
BPos11 = [550, -120, 400, 180, 0, 0]
BPos12 = [550, -120, 300, 180, 0, 0]
BPos13 = [550, -120, 200, 180, 0, 0]
BPos14 = [550, -120, 100, 180, 0, 0]
BPlace = [550, -120, 0, 180, 0, 0]


#Cube C location
CPos11 = [400, 40, 400, 180, 0, 0]
CPos12 = [400, 40, 300, 180, 0, 0]
CPos13 = [400, 40, 200, 180, 0, 0]
CPos14 = [400, 40, 100, 180, 0, 0]
CPlace = [400, 40, 0, 180, 0, 0]

#Cube D location
DPos11 = [550, 40, 400, 180, 0, 0]
DPos12 = [550, 40, 300, 180, 0, 0]
DPos13 = [550, 40, 200, 180, 0, 0]
DPos14 = [550, 40, 100, 180, 0, 0]
DPlace = [550, 40, 0, 180, 0, 0]

def moveToGoal():
    jacoRobot.setPosition2(movePos21, True)
    jacoRobot.setPosition2(movePos22, True)
    jacoRobot.setPosition2(movePos23, True)
    jacoRobot.setPosition2(movePos24, True)
    jacoRobot.setPosition2(movePos25, True)

def moveToBack():
    jacoRobot.setPosition2(movePos25, True)
    jacoRobot.setPosition2(movePos24, True)
    jacoRobot.setPosition2(movePos23, True)
    jacoRobot.setPosition2(movePos22, True)
    jacoRobot.setPosition2(movePos21, True)

def pick_goDown():
    jacoRobot.setPosition2(objPos3, True)
    jacoRobot.setPosition2(objPos1, True)

def pick_goUp():
    jacoRobot.setPosition2(objPos2, True)
    jacoRobot.setPosition2(objPos3, True)

def place_A():
    jacoRobot.setPosition2(APos11, True)
    jacoRobot.setPosition2(APos12, True)
    jacoRobot.setPosition2(APos13, True)
    jacoRobot.setPosition2(APos14, True)
    jacoRobot.setPosition2(APlace, True)

def place_B():
    jacoRobot.setPosition2(BPos11, True)
    jacoRobot.setPosition2(BPos12, True)
    jacoRobot.setPosition2(BPos13, True)
    jacoRobot.setPosition2(BPos14, True)
    jacoRobot.setPosition2(BPlace, True)

def place_C():
    jacoRobot.setPosition2(CPos11, True)
    jacoRobot.setPosition2(CPos12, True)
    jacoRobot.setPosition2(CPos13, True)
    jacoRobot.setPosition2(CPos14, True)
    jacoRobot.setPosition2(CPlace, True)

def place_D():
    jacoRobot.setPosition2(DPos11, True)
    jacoRobot.setPosition2(DPos12, True)
    jacoRobot.setPosition2(DPos13, True)
    jacoRobot.setPosition2(DPos14, True)
    jacoRobot.setPosition2(DPlace, True)

# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)
def thread_robotMovement():
    jacoRobot.setSpeed(1200, 90)
    while True:
        jacoRobot.gripperRelease()
        pick_goDown()
        jacoRobot.gripperCatch()
        pick_goUp()
        moveToGoal()
        place_A()
        jacoRobot.gripperRelease()
        time.sleep(1)

        moveToBack()

        pick_goDown()
        jacoRobot.gripperCatch()
        pick_goUp()
        moveToGoal()
        place_B()
        jacoRobot.gripperRelease()
        time.sleep(1)

        moveToBack()

        pick_goDown()
        jacoRobot.gripperCatch()
        pick_goUp()
        moveToGoal()
        place_C()
        jacoRobot.gripperRelease()
        time.sleep(1)

        moveToBack()

        pick_goDown()
        jacoRobot.gripperCatch()
        pick_goUp()
        moveToGoal()
        place_D()
        jacoRobot.gripperRelease()
        time.sleep(1)

# ====================================================




# INITIALIZATION:
# ====================================================
mSim=CoppeliaSim()
ret=mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)

# start thread:
t = threading.Thread(target=thread_robotMovement)
t.start()





# MAIN PRORGAM:
# ======================================================
# masukkan program utama disini (looping program)
while True:
    pass
