import abr_jaco2
from abr_control.controllers import OSC
import numpy as np

robot_config = abr_jaco2.Config()
interface = abr_jaco2.Interface(robot_config)
ctrlr = OSC(robot_config)
# instantiate things to avoid creating 200ms delay in main loop
zeros = np.zeros(robot_config.N_LINKS)
ctrlr.generate(q=zeros, dq=zeros, target=np.zeros(3))
# run once outside main loop as well, returns the cartesian
# coordinates of the end effector
robot_config.Tx('EE', q=zeros)

interface.connect()
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)

target_xyz = [0.57, 0.03, 0.87]  # (x, y, z) target (metres)
interface.init_force_mode()

while True:
    # returns a dictionary with q, dq
    feedback = interface.get_feedback()
    # ee position
    xyz = robot_config.Tx('EE', q = feedback, target_pos = target_xyz)
    u = ctrlr.generate(feedback['q'], feedback['dq'], target_xyz)
    interface.send_forces(u, dtype='float32')

    error = np.sqrt(np.sum((xyz - TARGET_XYZ[ii])**2))

    if error < 0.02:
        break

# switch back to position mode to move home and disconnect
interface.init_position_mode()
interface.send_target_angles(robot_config.INIT_TORQUE_POSITION)
interface.disconnect()