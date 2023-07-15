from include.coppeliasim import CoppeliaSim, CoppeliaArmRobot
import time

# === coppelia connect
mSim = CoppeliaSim()
ret = mSim.connect(19997)
if ret == -1:
    exit()

jacoRobot = CoppeliaArmRobot("Jaco")
jacoRobot.message("Alhamdulillah bisa jalan")

time.sleep(0.5)