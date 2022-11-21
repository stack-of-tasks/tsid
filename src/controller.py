#!/usr/bin/env python3

import time

import rospy
import quadruped
import numpy as np
import sys

rospy.init_node('controller', anonymous=True)

dt = 0.001

anymal = quadruped.Anymal(dt, 0.01)


#
# N_SIMULATION = 50000
#

t = 0
i = 0

q = np.array([ 0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., 0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, -0.7, 1.])
v = np.zeros(18)
tau = np.zeros(12)

i = 0

while True:
    # *100 to slow down for easier viewing
    time.sleep(dt*10)
    anymal.updateRaiSim()
    if rospy.is_shutdown():
        print('shutdown')
        break
