import numpy as np

from abr_control.arms import jaco2 as arm
from abr_control.controllers import OSC, Damping
from abr_control.controllers.path_planners import PathPlanner
from abr_control.controllers.path_planners.position_profiles import Linear
from abr_control.controllers.path_planners.velocity_profiles import Gaussian
from abr_control.interfaces import CoppeliaSim
from abr_control.utils import transformations

# Sim step size
dt = 0.005

# Initialize our robot config
robot_config = arm.Config()

# Damp the movements of the arm
damping = Damping(robot_config, kv=10)

# Create opreational space controller controlling all 6 DOF
ctrlr = OSC(
    robot_config,
    kp=100,  # position gain
    ko=250,  # orientation gain
    null_controllers=[damping],
    vmax=None,  # [m/s, rad/s]
    # control all DOF [x, y, z, alpha, beta, gamma]
    ctrlr_dof=[True, True, True, True, True, True],
)

# Create our interface
interface = CoppeliaSim(robot_config, dt=dt)
interface.connect()

# Create a path planner with a linear shape and gaussian velocity curve
path_planner = PathPlanner(
    pos_profile=Linear(),
    vel_profile=Gaussian(dt=dt, acceleration=2)
)

# Get our starting state
feedback = interface.get_feedback()
hand_xyz = robot_config.Tx("EE", feedback["q"])
starting_orientation = robot_config.quaternion("EE", feedback["q"])

# Generate a target
target_orientation = np.random.random(3)
target_orientation /= np.linalg.norm(target_orientation)
# convert our orientation to a quaternion
target_orientation = [0] + list(target_orientation)
target_position = [-0.4, -0.3, 0.6]

starting_orientation = transformations.euler_from_quaternion(
    starting_orientation, axes='rxyz')

target_orientation = transformations.euler_from_quaternion(
    target_orientation, axes='rxyz')

# Generate our 12D path
path_planner.generate_path(
    start_position=hand_xyz,
    target_position=target_position,
    start_orientation=starting_orientation,
    target_orientation=target_orientation,
    start_velocity=0,
    target_velocity=0,
    max_velocity=2
)

count = 0

# Step through the planned path, with the OSC trying to
# bring the end-effector to the filtered target state
while count < path_planner.n_timesteps:
    # get arm feedback
    feedback = interface.get_feedback()
    hand_xyz = robot_config.Tx("EE", feedback["q"])

    next_target = path_planner.next()
    pos = next_target[:3]
    vel = next_target[3:6]
    orient = next_target[6:9]

    u = ctrlr.generate(
        q=feedback["q"],
        dq=feedback["dq"],
        target=np.hstack([pos, orient]),
        target_velocity=np.hstack([vel, np.zeros(3)])
    )

    # apply the control signal, step the sim forward
    interface.send_forces(u)

    count += 1

interface.disconnect()