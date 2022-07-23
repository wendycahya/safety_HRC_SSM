import threading
import time
from coppeliasim import CoppeliaSim, CoppeliaArmRobot
from datetime import datetime

mSim = CoppeliaSim()
ret = mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)

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

progress = [1, 0, 1, 0]
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
        jacoRobot.setSpeed(1200, 90)
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
                print(datetime.now() - start_time)
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

    while 1:
        X = input("enter:")
        if X == "1":
            server.start()
        elif X == "2":
            server.pause()
        elif X == "3":
            server.resume()
        elif X == "4":
            server.stop()