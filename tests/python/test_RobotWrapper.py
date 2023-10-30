import os

import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test RobotWrapper")
print("")


filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)

robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
model = robot.model()
lb = model.lowerPositionLimit
lb[0:3] = -10.0 * np.ones(3)
lb[3:7] = -1.0 * np.ones(4)

ub = model.upperPositionLimit
ub[0:3] = 10.0 * np.ones(3)
ub[3:7] = 1.0 * np.ones(4)

q = se3.randomConfiguration(robot.model(), lb, ub)
print(q.transpose())

data = robot.data()
v = np.ones(robot.nv)
robot.computeAllTerms(data, q, v)
print(robot.com(data))

print("All test is done")
