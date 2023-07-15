import time
from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
from include.interruptThread import Job
import threading





# THREAD FUNCTION:
# Multithread function khusus buat menggerakkan robot
# Program ini akan terus melalukan looping pick & place
# dan tidak terpengaruh dengan main program (pose detection)
def thread_robotMovement():
    print("Belajar coding multithread")
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
