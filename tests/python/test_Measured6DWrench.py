from pathlib import Path

import numpy as np
import pinocchio as se3

import tsid

print("")
print("Test Measured6dWrench")
print("")

tol = 1e-5

filename = str(Path(__file__).resolve().parent)
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
model = robot.model()
data = robot.data()
frameName = "RAnkleRoll"
contact = tsid.Measured6dWrench("Measured6dwrench", robot, frameName)
wrench = np.asarray([1.5, 1.5, 1.5, 2.5, 2.5, 2.5])  # Some random wrench
contact.setMeasuredContactForce(wrench)
measured_wrench = contact.measuredContactForce
print(measured_wrench)

invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.addMeasuredForce(contact)
