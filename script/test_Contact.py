import pinocchio as se3
import tsid
import numpy as np
import copy

print ""
print "Test Contact"
print ""

tol = 1e-5
import os
filename = str(os.path.dirname(os.path.abspath(__file__)))
path = filename + '/../models/romeo'
urdf = path + '/urdf/romeo.urdf'
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
model = robot.model()
data = robot.data()

lx = 0.07
ly = 0.12
lz = 0.105
mu = 0.3
fMin = 10.0
fMax = 1000.0
frameName = "RAnkleRoll"

contactNormal = np.matrix(np.zeros(3)).transpose()
contactNormal[2] = 1.0
contact_Point = np.matrix(np.ones((3,4)) * lz)
contact_Point[0, :] = [-lx, -lx, lx, lx]
contact_Point[1, :] = [-ly, ly, -ly, ly]

contact =tsid.Contact6d("contact6d", robot, frameName, contact_Point, contactNormal, mu, fMin, fMax, 1e-3)

assert contact.n_motion == 6
assert contact.n_force == 12

Kp = np.matrix(np.ones(6)).transpose()
Kd = 2*Kp
contact.setKp(Kp)
contact.setKd(Kd)

assert np.linalg.norm(contact.Kp - Kp, 2) < tol
assert np.linalg.norm(contact.Kd - Kd, 2) < tol

q = model.neutralConfiguration
v = np.matrix(np.zeros(robot.nv)).transpose()
robot.computeAllTerms(data, q, v)

H_ref = robot.position(data, robot.model().getJointId(frameName))
contact.setReference(H_ref)

t = 0.0
contact.computeMotionTask(t, q, v, data)
forceIneq = contact.computeForceTask(t, q, v, data)
f3 = np.matrix(np.zeros(3)).transpose()
f3[2] = 100.0
f = np.matrix(np.zeros(12)).transpose()
for i in range(0, 4):
    f[i*3 : 3*(i+1)] = f3

assert (forceIneq.matrix * f <= forceIneq.upperBound).all()
assert (forceIneq.matrix * f >= forceIneq.lowerBound).all()

forceGenMat = contact.getForceGeneratorMatrix
assert forceGenMat.shape[0] == 6 and forceGenMat.shape[1] == 12
contact.computeForceRegularizationTask(t, q, v, data)

print "All test is done"
