#!/usr/bin/env python3

import time

import rospy
import quadruped
import numpy as np
import matplotlib.pyplot as plt

rospy.init_node('controller', anonymous=True)

dt = 0.001

anymal = quadruped.Anymal(dt, 0.1)


#
# N_SIMULATION = 50000
#

t = 0
i = 0

q = np.array([ 0., 0., 0.4792, 0., 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
v = np.zeros(18)
tau = np.zeros(12)

i = 0

tArr = []
errArr = [[] for i in range(12)]



def on_close(event):
    quit()

fig = plt.figure()
fig.canvas.mpl_connect('close_event', on_close)
# fig.axis([0, 10, 0, 1])

while True:
    # *100 to slow down for easier viewing
    if rospy.is_shutdown():
        print('shutdown')
        # for i in range(len(errArr)):
        #     plt.plot(tArr, errArr[i])
        # plt.show()
        break
    else:
        time.sleep(dt)
        anymal.updateRaiSim()
        # shouldAdd, t, err = anymal.getPlots()
        # if shouldAdd:
        #     tArr.append(t)
        #     for i in range(len(err)):
        #         errArr[i].append(err[i])


