from robodk.robolink import *       # import the robolink library (bridge with RoboDK)
RDK = Robolink()                    # establish a link with the simulator
robot = RDK.Item('Yaskawa GP7')      # retrieve the robot by name
robot.setJoints([0, 0, 0, 0, 0, 0])      # set all robot axes to zero

#robot.setJoints([10, 0, 0, 0, 0, 0])
#robot.setJoints([20, 0, 0, 0, 0, 0])
#target = RDK.Item('Target 1')         # retrieve the Target item
#robot.MoveJ(target)                 # move the robot to the target

# calculate a new approach position 100 mm along the Z axis of the tool with respect to the target
# from robodk.robomath import *       # import the robotics toolbox
# approach = target.Pose()*transl(0, 0, -100)
# robot.MoveL(approach)