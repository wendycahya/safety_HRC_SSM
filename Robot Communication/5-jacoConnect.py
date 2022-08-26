#from abr_control.arms import jaco2
from abr_control.controllers import OSC
#from abr_control.interfaces import CoppeliaSim
import abr_jaco2
import numpy as np

robot_config = abr_jaco2.Config()
interface = abr_jaco2.Interface(robot_config, display_error_level=2)

ctrlr = OSC(robot_config, kp=20,
            # control (x, y, z) out of [x, y, z, alpha, beta, gamma]
            ctrlr_dof=[True, True, True, False, False, False])

interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.JACO_HORIZONTAL_POSE)


target_xyz = [.2, .2, .5]  # in metres
target_orientation = [0, 0, 0]  # Euler angles, relevant when controlled
interface.init_force_mode()

for ii in range(1000):
    feedback = interface.get_feedback()  # returns a dictionary with q, dq
    u = ctrlr.generate(
        q=feedback['q'],
        dq=feedback['dq'],
        target=np.hstack([target_xyz, target_orientation]))
    interface.send_forces(u, dtype='float32')  # send forces and step CoppeliaSim sim forward

interface.disconnect()