#!/usr/bin/env python3
from robotSimulation import RaiSim
import time
import tsid
import pinocchio as pin
import numpy as np
from whole_body_state_conversions import whole_body_state_publisher
import rospy
import numpy as np

def pinToRaiSim(qin, vin):
    qout = np.array(qin)
    qout[10:13] = qin[13:16]
    qout[13:16] = qin[10:13]
    qout[3] = qin[6]
    qout[6] = qin[3]
    vout = np.array(vin)
    vout[9:12] = vin[12:15]
    vout[12:15] = vin[9:12]
    return qout, vout

rospy.init_node('raisimTest', anonymous=True)

path = '/home/kian/catkin_ws/src/example-robot-data/robots'
urdf = path + '/anymal_raisim/urdf/anymal.urdf'
srdf = path + '/anymal_raisim/srdf/anymal.srdf'

robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
model = robot.model()

pin.loadReferenceConfigurations(model, srdf, False)
q = model.referenceConfigurations['standing']
v = np.zeros(18)

dt = 0.0001
sim = RaiSim(dt)
q, v = pinToRaiSim(q, v)
sim.addRobot(urdf, q, v)

t = 0

q_p = np.array(q)

q_i = np.array(q_p)
v_i = np.zeros(18)

for i in range(0, 100000):

    q_i[0:7] *= 0
    sim.setTarget(q_i + np.random.randn(19)*0.5, v_i)
    sim.integrate()
    if rospy.is_shutdown():
        print('shutdown')
        break

    time.sleep(dt)
