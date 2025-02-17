import copy
import os

import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test Contact")
print("")

tol = 1e-5
filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
model = robot.model()
data = robot.data()

mu = 0.3
fMin = 10.0
fMax = 1000.0
frameName = "RAnkleRoll"
contactNormal = np.zeros(3)
contactNormal[2] = 1.0

contact = tsid.ContactPoint(
    "contactPoint", robot, frameName, contactNormal, mu, fMin, fMax
)

assert contact.n_motion == 3
assert contact.n_force == 3

Kp = np.ones(3)
Kd = 2 * Kp
contact.setKp(Kp)
contact.setKd(Kd)

assert np.linalg.norm(contact.Kp - Kp, 2) < tol
assert np.linalg.norm(contact.Kd - Kd, 2) < tol

q = model.neutralConfiguration
v = np.zeros(robot.nv)
robot.computeAllTerms(data, q, v)

H_ref = robot.position(data, robot.model().getJointId(frameName))
contact.setReference(H_ref)

t = 0.0
contact.computeMotionTask(t, q, v, data)
forceIneq = contact.computeForceTask(t, q, v, data)
f = np.zeros(3)
f[2] = 100.0

assert (forceIneq.matrix * f <= forceIneq.upperBound).all()
assert (forceIneq.matrix * f >= forceIneq.lowerBound).all()

forceGenMat = contact.getForceGeneratorMatrix
assert forceGenMat.shape[0] == 3 and forceGenMat.shape[1] == 3
contact.computeForceRegularizationTask(t, q, v, data)

print("All test is done")
