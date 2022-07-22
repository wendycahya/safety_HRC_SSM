import threading
import time
from coppeliasim import CoppeliaSim, CoppeliaArmRobot



mSim = CoppeliaSim()
ret = mSim.connect(19997)
if ret == -1:
    exit()

# Initialize the robot model
jacoRobot = CoppeliaArmRobot("Jaco")
time.sleep(1)


post_move =[liftPos12, liftPos13, liftPos14]

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
            print("out of for loop")
            for i in post_move:
                self.__flag.wait()
                jacoRobot.setPosition2(i, True)




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
        if X =="1":
            server.start()
        elif X == "2":
            server.pause()
        elif X == "3":
            server.resume()
        elif X == "4":
            server.stop()