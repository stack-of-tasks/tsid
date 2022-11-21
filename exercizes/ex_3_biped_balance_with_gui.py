# -*- coding: utf-8 -*-
"""
Created on Wed Apr 17 22:31:22 2019

@author: student
"""

import threading
import time
import tkinter as tk
from tkinter import HORIZONTAL, Button, Entry, Frame, Label, Scale, Tk, mainloop

import numpy as np
import pinocchio as pin

import talos_conf as conf
#import romeo_conf as conf
import vizutils
from tsid_biped import TsidBiped

AXES = ['X', 'Y', 'Z']


class Scale3d:
    def __init__(self, master, name, from_, to, tickinterval, length, orient, command):
        self.s = 3 * [None]
        for i in range(3):
            self.s[i] = Scale(master, label='%s %s' % (name, AXES[i]), from_=from_[i], to=to[i],
                              tickinterval=tickinterval[i], orient=orient[i], length=length[i], command=command)
            self.s[i].pack()
        separator = Frame(height=2, bd=1, relief=tk.SUNKEN)
        separator.pack(fill=tk.X, padx=5, pady=5)

    def get(self):
        return self.s[0].get(), self.s[1].get(), self.s[2].get()


class Entry3d:
    def __init__(self, master, name):
        self.s = 3 * [None]
        for i in range(3):
            Label(master, text='%s %s' % (name, AXES[i])).pack()  # side=tk.TOP)
            self.s[i] = Entry(master, width=5)
            self.s[i].pack()  # side=tk.BOTTOM)
            separator = Frame(height=1, bd=1, relief=tk.SUNKEN)
            separator.pack(fill=tk.X, padx=2, pady=2)  # , side=tk.BOTTOM)

    def get(self):
        try:
            return [float(self.s[i].get()) for i in range(3)]
        except:
            print("could not convert string to float", [self.s[i].get() for i in range(3)])
            return 3 * [0.0]

scale_com, scale_RF, scale_LF = None, None, None
button_contact_RF, button_contact_LF = None, None
push_robot_active, push_robot_com_vel, com_vel_entry = False, 3 * [0.0], None


def update_com_ref_scale(value):
    x, y, z = scale_com.get()
    tsid.trajCom.setReference(com_0 + np.array([1e-2 * x, 1e-2 * y, 1e-2 * z]).T)


def update_RF_ref_scale(value):
    x, y, z = scale_RF.get()
    H_rf_ref = H_rf_0.copy()
    H_rf_ref.translation += + np.array([1e-2 * x, 1e-2 * y, 1e-2 * z]).T
    tsid.trajRF.setReference(H_rf_ref)


def update_LF_ref_scale(value):
    x, y, z = scale_LF.get()
    H_lf_ref = H_lf_0.copy()
    H_lf_ref.translation += + np.array([1e-2 * x, 1e-2 * y, 1e-2 * z]).T
    tsid.trajLF.setReference(H_lf_ref)


def switch_contact_RF():
    if tsid.contact_RF_active:
        tsid.remove_contact_RF()
        button_contact_RF.config(text='Make contact right foot')
    else:
        tsid.add_contact_RF()
        button_contact_RF.config(text='Break contact right foot')


def switch_contact_LF():
    if tsid.contact_LF_active:
        tsid.remove_contact_LF()
        button_contact_LF.config(text='Make contact left foot')
    else:
        tsid.add_contact_LF()
        button_contact_LF.config(text='Break contact left foot')


def toggle_wireframe_mode():
    tsid.gui.setWireFrameMode('world', 'WIREFRAME')


def push_robot():
    global push_robot_com_vel, push_robot_active
    push_robot_com_vel = com_vel_entry.get()
    push_robot_active = True


def create_gui():
    """thread worker function"""
    global scale_com, scale_RF, scale_LF, button_contact_RF, button_contact_LF, com_vel_entry
    master = Tk(className='TSID GUI')
    scale_com = Scale3d(master, 'CoM', [-10, -15, -40], [10, 15, 40], [5, 5, 10], [200, 250, 300],
                        3*[HORIZONTAL], update_com_ref_scale)
    scale_RF = Scale3d(master, 'Right foot', 3 * [-30], 3 * [30], 3 * [10], 3 * [300],
                       3*[HORIZONTAL], update_RF_ref_scale)
    scale_LF = Scale3d(master, 'Left foot', 3 * [-30], 3 * [30], 3 * [10], 3 * [300],
                       3*[HORIZONTAL], update_LF_ref_scale)
    button_contact_RF = Button(master, text='Break contact right foot', command=switch_contact_RF)
    button_contact_RF.pack(side=tk.LEFT)
    button_contact_LF = Button(master, text='Break contact left foot', command=switch_contact_LF)
    button_contact_LF.pack(side=tk.LEFT)
    Button(master, text='Toggle wireframe', command=toggle_wireframe_mode).pack(side=tk.LEFT)

    # Frame(height=2, bd=1, relief=tk.SUNKEN).pack(fill=tk.X, padx=5, pady=5)
    Button(master, text='Push robot CoM', command=push_robot).pack()
    com_vel_entry = Entry3d(master, 'CoM vel')
    mainloop()


def run_simu():
    global push_robot_active
    i, t = 0, 0.0
    q, v = tsid.q, tsid.v
    time_avg = 0.0
    while True:
        time_start = time.time()

        tsid.comTask.setReference(tsid.trajCom.computeNext())
        tsid.postureTask.setReference(tsid.trajPosture.computeNext())
        tsid.rightFootTask.setReference(tsid.trajRF.computeNext())
        tsid.leftFootTask.setReference(tsid.trajLF.computeNext())

        HQPData = tsid.formulation.computeProblemData(t, q, v)

        sol = tsid.solver.solve(HQPData)
        if sol.status != 0:
            print("QP problem could not be solved! Error code:", sol.status)
            break

        # tau = tsid.formulation.getActuatorForces(sol)
        dv = tsid.formulation.getAccelerations(sol)
        q, v = tsid.integrate_dv(q, v, dv, conf.dt)
        i, t = i + 1, t + conf.dt

        if push_robot_active:
            push_robot_active = False
            data = tsid.formulation.data()
            if tsid.contact_LF_active:
                J_LF = tsid.contactLF.computeMotionTask(0.0, q, v, data).matrix
            else:
                J_LF = np.zeros((0, tsid.model.nv))
            if tsid.contact_RF_active:
                J_RF = tsid.contactRF.computeMotionTask(0.0, q, v, data).matrix
            else:
                J_RF = np.zeros((0, tsid.model.nv))
            J = np.vstack((J_LF, J_RF))
            J_com = tsid.comTask.compute(t, q, v, data).matrix
            A = np.vstack((J_com, J))
            b = np.concatenate((np.array(push_robot_com_vel), np.zeros(J.shape[0])))
            v = np.linalg.lstsq(A, b, rcond=-1)[0]

        if i % conf.DISPLAY_N == 0:
            tsid.display(q)
            x_com = tsid.robot.com(tsid.formulation.data())
            x_com_ref = tsid.trajCom.getSample(t).pos()
            H_lf = tsid.robot.framePosition(tsid.formulation.data(), tsid.LF)
            H_rf = tsid.robot.framePosition(tsid.formulation.data(), tsid.RF)
            x_lf_ref = tsid.trajLF.getSample(t).pos()[:3]
            x_rf_ref = tsid.trajRF.getSample(t).pos()[:3]
            vizutils.applyViewerConfiguration(tsid.viz, 'world/com', x_com.tolist() + [0, 0, 0, 1.])
            vizutils.applyViewerConfiguration(tsid.viz, 'world/com_ref', x_com_ref.tolist() + [0, 0, 0, 1.])
            vizutils.applyViewerConfiguration(tsid.viz, 'world/rf', pin.SE3ToXYZQUATtuple(H_rf))
            vizutils.applyViewerConfiguration(tsid.viz, 'world/lf', pin.SE3ToXYZQUATtuple(H_lf))
            vizutils.applyViewerConfiguration(tsid.viz, 'world/rf_ref', x_rf_ref.tolist() + [0, 0, 0, 1.])
            vizutils.applyViewerConfiguration(tsid.viz, 'world/lf_ref', x_lf_ref.tolist() + [0, 0, 0, 1.])

        if i % 1000 == 0:
            print("Average loop time: %.1f (expected is %.1f)" % (1e3 * time_avg, 1e3 * conf.dt))

        time_spent = time.time() - time_start
        time_avg = (i * time_avg + time_spent) / (i + 1)

        if time_avg < 0.9 * conf.dt:
            time.sleep(10 * (conf.dt - time_avg))


print("#" * conf.LINE_WIDTH)
print(" Test Task Space Inverse Dynamics ".center(conf.LINE_WIDTH, '#'))
print("#" * conf.LINE_WIDTH)

tsid = TsidBiped(conf, conf.viewer)
tsid.q0[2] = 1.02127

com_0 = tsid.robot.com(tsid.formulation.data())
H_rf_0 = tsid.robot.framePosition(tsid.formulation.data(), tsid.model.getFrameId(conf.rf_frame_name))
H_lf_0 = tsid.robot.framePosition(tsid.formulation.data(), tsid.model.getFrameId(conf.lf_frame_name))

# vizutils.addViewerSphere(tsid.viz, 'world/com', conf.SPHERE_RADIUS, conf.COM_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/com_ref', conf.REF_SPHERE_RADIUS, conf.COM_REF_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/rf', conf.SPHERE_RADIUS, conf.RF_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/rf_ref', conf.REF_SPHERE_RADIUS, conf.RF_REF_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/lf', conf.SPHERE_RADIUS, conf.LF_SPHERE_COLOR)
# vizutils.addViewerSphere(tsid.viz, 'world/lf_ref', conf.REF_SPHERE_RADIUS, conf.LF_REF_SPHERE_COLOR)

th_gui = threading.Thread(target=create_gui)
th_gui.start()

th_simu = threading.Thread(target=run_simu)
th_simu.start()
