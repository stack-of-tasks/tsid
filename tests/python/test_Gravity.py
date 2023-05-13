import os

import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test setGravity")
print("")


filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)

robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)

init_gravity = robot.model().gravity.copy()

robot.setGravity(se3.Motion.Zero())

no_gravity = robot.model().gravity.copy()

print(init_gravity)
print(no_gravity)

assert init_gravity != no_gravity

print("All test is done")
