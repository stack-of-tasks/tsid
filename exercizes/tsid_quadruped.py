import os
import subprocess
import time

import numpy as np
import pinocchio as pin
import tsid


class TsidQuadruped:
    ''' Standard TSID formulation for a biped robot standing on its rectangular feet.
        - Center of mass task
        - Postural task
        - 6d rigid contact constraint for both feet
        - Regularization task for contact forces
    '''

    def __init__(self, conf, viewer=pin.visualize.MeshcatVisualizer):
        self.conf = conf
        self.robot = tsid.RobotWrapper(conf.urdf, [conf.path], pin.JointModelFreeFlyer(), False)
        robot = self.robot
        self.model = robot.model()
        pin.loadReferenceConfigurations(self.model, conf.srdf, False)
        self.q0 = q = self.model.referenceConfigurations["standing_with_arm_up"]
        v = np.zeros(robot.nv)

        # assert self.model.existFrame(conf.rf_frame_name)
        # assert self.model.existFrame(conf.lf_frame_name)
        # assert self.model.existFrame(conf.rh_frame_name)
        # assert self.model.existFrame(conf.lh_frame_name)

        formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, True)
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()
        contact_Point = np.ones((3, 4)) * (-conf.lz)
        contact_Point[0, :] = [-conf.lxn, -conf.lxn, conf.lxp, conf.lxp]
        contact_Point[1, :] = [-conf.lyn, conf.lyp, -conf.lyn, conf.lyp]

        contactRF = tsid.Contact6d("contact_rfoot", robot, conf.rf_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactRF.setKp(conf.kp_contact * np.ones(6))
        contactRF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.RF = robot.model().getFrameId(conf.rf_frame_name)
        H_rf_ref = robot.framePosition(data, self.RF)

        # modify initial robot configuration so that foot is on the ground (z=0)
        q[2] -= H_rf_ref.translation[2] - conf.lz
        formulation.computeProblemData(0.0, q, v)
        data = formulation.data()
        H_rf_ref = robot.framePosition(data, self.RF)
        contactRF.setReference(H_rf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactRF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactRF, conf.w_forceRef)

        contactRH = tsid.Contact6d("contact_r_hind_foot", robot, conf.rh_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactRH.setKp(conf.kp_contact * np.ones(6))
        contactRH.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.RH = robot.model().getFrameId(conf.rh_frame_name)
        H_rh_ref = robot.framePosition(data, self.RH)
        contactRH.setReference(H_rh_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactRH, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactRH, conf.w_forceRef)

        contactLF = tsid.Contact6d("contact_lfoot", robot, conf.lf_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactLF.setKp(conf.kp_contact * np.ones(6))
        contactLF.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.LF = robot.model().getFrameId(conf.lf_frame_name)
        H_lf_ref = robot.framePosition(data, self.LF)
        contactLF.setReference(H_lf_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactLF, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactLF, conf.w_forceRef)

        contactLH = tsid.Contact6d("contact_l_hind_foot", robot, conf.lh_frame_name, contact_Point,
                                   conf.contactNormal, conf.mu, conf.fMin, conf.fMax)
        contactLH.setKp(conf.kp_contact * np.ones(6))
        contactLH.setKd(2.0 * np.sqrt(conf.kp_contact) * np.ones(6))
        self.LH = robot.model().getFrameId(conf.lh_frame_name)
        H_lh_ref = robot.framePosition(data, self.LH)
        contactLH.setReference(H_lh_ref)
        if conf.w_contact >= 0.0:
            formulation.addRigidContact(contactLH, conf.w_forceRef, conf.w_contact, 1)
        else:
            formulation.addRigidContact(contactLH, conf.w_forceRef)

        comTask = tsid.TaskComEquality("task-com", robot)
        comTask.setKp(conf.kp_com * np.ones(3))
        comTask.setKd(2.0 * np.sqrt(conf.kp_com) * np.ones(3))
        formulation.addMotionTask(comTask, conf.w_com, 1, 0.0)

        copTask = tsid.TaskCopEquality("task-cop", robot)
        formulation.addForceTask(copTask, conf.w_cop, 1, 0.0)

        amTask = tsid.TaskAMEquality("task-am", robot)
        amTask.setKp(conf.kp_am * np.array([1., 1., 0.]))
        amTask.setKd(2.0 * np.sqrt(conf.kp_am * np.array([1., 1., 0.])))
        formulation.addMotionTask(amTask, conf.w_am, 1, 0.)
        sampleAM = tsid.TrajectorySample(3)
        amTask.setReference(sampleAM)

        postureTask = tsid.TaskJointPosture("task-posture", robot)
        postureTask.setKp(conf.kp_posture * conf.gain_vector)
        postureTask.setKd(2.0 * np.sqrt(conf.kp_posture * conf.gain_vector))
        postureTask.setMask(conf.masks_posture)
        formulation.addMotionTask(postureTask, conf.w_posture, 1, 0.0)

        self.leftFootTask = tsid.TaskSE3Equality("task-left-foot", self.robot, self.conf.lf_frame_name)
        self.leftFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.leftFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajLF = tsid.TrajectorySE3Constant("traj-left-foot", H_lf_ref)
        formulation.addMotionTask(self.leftFootTask, self.conf.w_foot, 1, 0.0)

        self.rightFootTask = tsid.TaskSE3Equality("task-right-foot", self.robot, self.conf.rf_frame_name)
        self.rightFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.rightFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajRF = tsid.TrajectorySE3Constant("traj-right-foot", H_rf_ref)
        formulation.addMotionTask(self.rightFootTask, self.conf.w_foot, 1, 0.0)

        self.leftHindFootTask = tsid.TaskSE3Equality("task-left-hind-foot", self.robot, self.conf.lh_frame_name)
        self.leftHindFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.leftHindFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajLH = tsid.TrajectorySE3Constant("traj-left-hind-foot", H_lh_ref)
        formulation.addMotionTask(self.leftHindFootTask, self.conf.w_foot, 1, 0.0)

        self.rightHindFootTask = tsid.TaskSE3Equality("task-right-hind-foot", self.robot, self.conf.rh_frame_name)
        self.rightHindFootTask.setKp(self.conf.kp_foot * np.ones(6))
        self.rightHindFootTask.setKd(2.0 * np.sqrt(self.conf.kp_foot) * np.ones(6))
        self.trajRH = tsid.TrajectorySE3Constant("traj-right-hind-foot", H_rh_ref)
        formulation.addMotionTask(self.rightHindFootTask, self.conf.w_foot, 1, 0.0)

        self.tau_max = conf.tau_max_scaling * robot.model().effortLimit[-robot.na:]
        self.tau_min = -self.tau_max
        actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
        actuationBoundsTask.setBounds(self.tau_min, self.tau_max)
        if conf.w_torque_bounds > 0.0:
            formulation.addActuationTask(actuationBoundsTask, conf.w_torque_bounds, 0, 0.0)

        jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", robot, conf.dt)
        self.v_max = conf.v_max_scaling * robot.model().velocityLimit[-robot.na:]
        self.v_min = -self.v_max
        jointBoundsTask.setVelocityBounds(self.v_min, self.v_max)
        if conf.w_joint_bounds > 0.0:
            formulation.addMotionTask(jointBoundsTask, conf.w_joint_bounds, 0, 0.0)

        com_ref = robot.com(data)
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
        self.sample_com = self.trajCom.computeNext()

        q_ref = q[7:]
        self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
        postureTask.setReference(self.trajPosture.computeNext())

        self.sampleLF = self.trajLF.computeNext()
        self.sample_LF_pos = self.sampleLF.value
        self.sample_LF_vel = self.sampleLF.derivative
        self.sample_LF_acc = self.sampleLF.second_derivative

        self.sampleRF = self.trajRF.computeNext()
        self.sample_RF_pos = self.sampleRF.value
        self.sample_RF_vel = self.sampleRF.derivative
        self.sample_RF_acc = self.sampleRF.second_derivative

        self.solver = tsid.SolverHQuadProgFast("qp solver")
        self.solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

        self.comTask = comTask
        self.copTask = copTask
        self.amTask = amTask
        self.postureTask = postureTask
        self.contactRF = contactRF
        self.contactLF = contactLF
        self.actuationBoundsTask = actuationBoundsTask
        self.jointBoundsTask = jointBoundsTask
        self.formulation = formulation
        self.q = q
        self.v = v

        self.contact_LF_active = True
        self.contact_RF_active = True

        if viewer == pin.visualize.MeshcatVisualizer:
            self.robot_display = pin.RobotWrapper.BuildFromURDF(conf.urdf, [conf.path], pin.JointModelFreeFlyer())
            self.viz = viewer(self.robot_display.model, self.robot_display.collision_model,
                              self.robot_display.visual_model)
            self.viz.initViewer(loadModel=True)
            self.viz.display(q)

    def display(self, q):
        if hasattr(self, 'viz'):
            self.viz.display(q)

    def integrate_dv(self, q, v, dv, dt):
        v_mean = v + 0.5 * dt * dv
        v += dt * dv
        q = pin.integrate(self.model, q, dt * v_mean)
        return q, v

    def get_placement_LF(self):
        return self.robot.framePosition(self.formulation.data(), self.LF)

    def get_placement_RF(self):
        return self.robot.framePosition(self.formulation.data(), self.RF)

    def set_com_ref(self, pos, vel, acc):
        self.sample_com.pos(pos)
        self.sample_com.vel(vel)
        self.sample_com.acc(acc)
        self.comTask.setReference(self.sample_com)

    def set_RF_3d_ref(self, pos, vel, acc):
        self.sample_RF_pos[:3] = pos
        self.sample_RF_vel[:3] = vel
        self.sample_RF_acc[:3] = acc
        self.sampleRF.pos(self.sample_RF_pos)
        self.sampleRF.vel(self.sample_RF_vel)
        self.sampleRF.acc(self.sample_RF_acc)
        self.rightFootTask.setReference(self.sampleRF)

    def set_LF_3d_ref(self, pos, vel, acc):
        self.sample_LF_pos[:3] = pos
        self.sample_LF_vel[:3] = vel
        self.sample_LF_acc[:3] = acc
        self.sampleLF.pos(self.sample_LF_pos)
        self.sampleLF.vel(self.sample_LF_vel)
        self.sampleLF.acc(self.sample_LF_acc)
        self.leftFootTask.setReference(self.sampleLF)

    def get_LF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.LF)
        v = self.robot.frameVelocity(data, self.LF)
        a = self.leftFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]

    def get_RF_3d_pos_vel_acc(self, dv):
        data = self.formulation.data()
        H = self.robot.framePosition(data, self.RF)
        v = self.robot.frameVelocity(data, self.RF)
        a = self.rightFootTask.getAcceleration(dv)
        return H.translation, v.linear, a[:3]

    def remove_contact_RF(self, transition_time=0.0):
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.trajRF.setReference(H_rf_ref)
        self.rightFootTask.setReference(self.trajRF.computeNext())

        self.formulation.removeRigidContact(self.contactRF.name, transition_time)
        self.contact_RF_active = False

    def remove_contact_LF(self, transition_time=0.0):
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.trajLF.setReference(H_lf_ref)
        self.leftFootTask.setReference(self.trajLF.computeNext())

        self.formulation.removeRigidContact(self.contactLF.name, transition_time)
        self.contact_LF_active = False

    def add_contact_RF(self, transition_time=0.0):
        H_rf_ref = self.robot.framePosition(self.formulation.data(), self.RF)
        self.contactRF.setReference(H_rf_ref)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef, self.conf.w_contact, 1)
        else:
            self.formulation.addRigidContact(self.contactRF, self.conf.w_forceRef)

        self.contact_RF_active = True

    def add_contact_LF(self, transition_time=0.0):
        H_lf_ref = self.robot.framePosition(self.formulation.data(), self.LF)
        self.contactLF.setReference(H_lf_ref)
        if self.conf.w_contact >= 0.0:
            self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef, self.conf.w_contact, 1)
        else:
            self.formulation.addRigidContact(self.contactLF, self.conf.w_forceRef)

        self.contact_LF_active = True
