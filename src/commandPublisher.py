#!/usr/bin/env python3

import time
import tsid
import rospy
import numpy as np
import pinocchio as pin
from whole_body_state_conversions import whole_body_state_publisher

rospy.init_node('command_publisher', anonymous=True)

path = '/home/kian/catkin_ws/src/example-robot-data/robots'
urdf = path + '/anymal_c_simple_description/urdf/anymal.urdf'
# srdf = path + '/anymal_c_simple_description/srdf/anymal.srdf'

robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
model = robot.model()

wbcp = whole_body_state_publisher.WholeBodyStatePublisher('/whole_body_state', model)

q = np.array([ 0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., -0.1, 0.7, -1., -0.1, -0.7, 1.])
v = np.zeros(18)
tau = np.zeros(12)

offset = np.array([ 0., 0., 0.4792])
amp = np.array([0.01, 0.01, 0])  # amplitude function of 0.05 along the y axis
two_pi_f = 2 * np.pi * np.array([4, 2, 0])  # 2π function along the y axis with 0.5 amplitude
two_pi_f_amp = two_pi_f * amp  # 2π function times amplitude function
two_pi_f_squared_amp = two_pi_f * two_pi_f_amp  # 2π function times squared amplitude function

t = 0
dt = 0.1

while True:
    # q[0:3] = offset + amp*np.sin(two_pi_f*t)
    wbcp.publish(t, q, v, tau)
    t = t + dt
    time.sleep(dt)
    if rospy.is_shutdown():
        print('shutdown')
        break
