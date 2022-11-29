import time
import tsid
import rospy
import numpy as np
import pinocchio as pin
from whole_body_state_conversions.whole_body_interface import WholeBodyStateInterface
from whole_body_state_msgs.msg import WholeBodyState
from whole_body_state_conversions import whole_body_controller_publisher
from whole_body_state_conversions import whole_body_state_publisher
from robotSimulation import RaiSim

class Anymal():
    def __init__(self, dt_RaiSim, dt_TSID):

        # File paths
        path = '/home/kian/catkin_ws/src/example-robot-data/robots'
        urdf = path + '/anymal_c_simple_description/urdf/anymal.urdf'
        srdf = path + '/anymal_raisim/srdf/anymal.srdf'

        # dt for Raisim
        self.dt = dt_RaiSim

        self.dt_TSID = dt_TSID

        # robot and model for use in TSID
        self.robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
        self.model = self.robot.model()

        self.pub_act = whole_body_state_publisher.WholeBodyStatePublisher('/whole_body_state_actual', self.model)
        self.pub_des = whole_body_state_publisher.WholeBodyStatePublisher('/whole_body_state_desired', self.model)

        self.q = np.array([ 0., 0., 0.4792, 0, 0., 0., 1., -0.1, 0.7, -1., -0.1, -0.7, 1., 0.1, 0.7, -1., 0.1, -0.7, 1.])
        self.v = np.zeros(self.robot.nv)

        self.q[2] += 0.1

        # Initialise an instance of RaiSim
        self.sim = RaiSim(self.dt)
        self.sim.addRobot(urdf, self.q, self.v)

        # Used for integration
        self.t = 0

        # Create inverse dynamics formulation
        self.invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", self.robot, False)
        HQP = self.invdyn.computeProblemData(self.t, self.q, self.v)
        self.data = self.invdyn.data()

        # Setup solver
        self.solver = tsid.SolverHQuadProgFast("qp solver")
        # Resize the solver to fit the number of variables, equality and inequality constraints
        self.solver.resize(self.invdyn.nVar, self.invdyn.nEq, self.invdyn.nIn)  # THIS LINE COULD BE USEFUL!!!
        self.comTaskInit()
        self.contactTaskInit()
        self.postureTaskInit()
        # TO DO: Make task base class with an init and update function

        # Used in subscriber callback later
        self.interface = WholeBodyStateInterface(self.model)

        # Setup subscriber
        # self.wbcpActual = whole_body_state_publisher.WholeBodyStatePublisher('/whole_body_state_actual', self.model)
        # self.wbcpController = whole_body_controller_publisher.WholeBodyControllerPublisher('/whole_body_state_controller', self.model)
        rospy.Subscriber("/whole_body_state", WholeBodyState, self.callback)

        # Setup variables for output of TSID controller
        self.q_command, self.v_command = self.q, self.v

        self.q_des = np.array(self.q)
        self.v_des = np.array(self.v)
        self.tau = np.zeros(12)

        v_max = self.model.velocityLimit[6:]
        v_min = -v_max
        tau_max = self.model.effortLimit[6:]
        tau_min = -tau_max
        print(v_max)
        print(tau_max)
        # p_max = self.model.upperPositionLimit
        # p_min = self.model.lowerPositionLimit
        self.jointBoundsTask = tsid.TaskJointBounds("task-joint-bounds", self.robot, self.dt_TSID)
        # self.jointBoundsTask.setPositionBounds(p_min, p_max)
        self.jointBoundsTask.setVelocityBounds(v_min, v_max)

        self.actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", self.robot)
        self.actuationBoundsTask.setBounds(tau_min, tau_max)

        w_joint_bounds = 1
        w_torque_bounds = 1

        self.invdyn.addMotionTask(self.jointBoundsTask, w_joint_bounds, 0, 0.0)
        self.invdyn.addActuationTask(self.actuationBoundsTask, w_torque_bounds, 0, 0.0)

    def pinToRaiSim(self, qin, vin):
        qout = np.array(qin)
        qout[10:13] = qin[13:16]
        qout[13:16] = qin[10:13]
        qout[3] = qin[6]
        qout[6] = qin[3]
        vout = np.array(vin)
        vout[9:12] = vin[12:15]
        vout[12:15] = vin[9:12]
        return qout, vout

    def callback(self, msg):
        t, self.q_des, self.v_des, self.tau_des, p, pd, f, s = self.interface.writeFromMessage(msg)

        comQ = self.q_des[0:3]
        comV = self.v_des[0:3]
        self.sampleCom.value(comQ)
        self.sampleCom.derivative(comV)
        self.comTask.setReference(self.sampleCom)

        self.v_des[3:6] = np.matmul(self.sim.getAngularVelTransform(), self.v_des[3:6])
        self.samplePosture = self.trajPosture.computeNext()
        self.samplePosture.value(self.q_des)
        self.samplePosture.derivative(self.v_des)
        self.postureTask.setReference(self.samplePosture)
        self.updateTSID()

    def getModel(self):
        return self.model

    def getOffest(self):
        return self.robot.com(self.data)

    def comTaskInit(self):
        w_com = 1 # CoM task weight
        kp_com = 50  # proportional gain of center of mass task
        self.comTask = tsid.TaskComEquality("task-com", self.robot)
        self.comTask.setKp(kp_com * np.ones(3))  # Proportional gain defined before = 20
        self.comTask.setKd(2 * np.sqrt(kp_com) * np.ones(3))  # Derivative gain = 2 * sqrt(20)
        # Add the task to the HQP with weight = w_com, priority level = 0 (as constraint) and a transition duration = 0.0
        self.invdyn.addMotionTask(self.comTask, w_com, 1, 0)

        com_ref = self.data.com[0]  # Initial value of the CoM
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
        self.sampleCom = self.trajCom.computeNext()  # Compute the first step of the trajectory from the initial value


    def postureTaskInit(self):
        # SHOULD THE FIRST 7 ELEMENTS BE 1 or 0?
        kp_posture = np.array(  # proportional gain of joint posture task
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        )*50
        w_posture = 1

        self.postureTask = tsid.TaskJointPosture("task-posture", self.robot)
        self.postureTask.setKp(kp_posture)  # Proportional gain defined before (different for each joints)
        self.postureTask.setKd(3 * kp_posture)  # Derivative gain = 2 * kp
        # Add the task with weight = w_posture, priority level = 1 (in cost function) and a transition duration = 0.0
        self.invdyn.addMotionTask(self.postureTask, w_posture, 1, 0)

        self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", self.q)
        self.samplePosture = self.trajPosture.computeNext()
        self.samplePosture.value(self.q)
        self.samplePosture.derivative(self.v)
        self.postureTask.setReference(self.samplePosture)

    def contactTaskInit(self):
        mu = 0.3  # friction coefficient
        fMin = 0.0  # minimum normal force
        fMax = 500.0  # maximum normal force

        rh_frame_name = "RH_FOOT"
        lf_frame_name = "LF_FOOT"
        rf_frame_name = "RF_FOOT"
        lh_frame_name = "LH_FOOT"

        # rh_frame_name = "RH_KFE"
        # lf_frame_name = "LF_KFE"
        # rf_frame_name = "RF_KFE"
        # lh_frame_name = "LH_KFE"

        kp_contact = 2 # proportional gain of contact constraint
        w_forceRef = 1e-3 # weight of force regularization task 1e-3

        contactNormal = np.array([0., 0., 1.])  # direction of the normal to the contact surface


        self.contactLF = tsid.ContactPoint("contact_lfoot", self.robot, lf_frame_name, contactNormal, mu, fMin, fMax)
        self.contactLF.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactLF.setKd(2 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_lf_ref = self.robot.position(self.data, self.model.getJointId(lf_frame_name))
        self.contactLF.setReference(H_lf_ref)
        self.contactLF.useLocalFrame(False)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactLF, w_forceRef, 1, 1)

        self.contactRH = tsid.ContactPoint("contact_rhfoot", self.robot, rh_frame_name, contactNormal, mu, fMin, fMax)
        self.contactRH.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactRH.setKd(2 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_rh_ref = self.robot.position(self.data, self.model.getJointId(rh_frame_name))
        self.contactRH.setReference(H_rh_ref)
        self.contactRH.useLocalFrame(False)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactRH, w_forceRef, 1, 1)

        self.contactLH = tsid.ContactPoint("contact_lhfoot", self.robot, lh_frame_name, contactNormal, mu, fMin, fMax)
        self.contactLH.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactLH.setKd(2 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # # Reference position of the left ankle -> initial position
        H_lh_ref = self.robot.position(self.data, self.model.getJointId(lh_frame_name))
        self.contactLH.setReference(H_lh_ref)
        self.contactLH.useLocalFrame(False)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactLH, w_forceRef, 1, 1)

        self.contactRF = tsid.ContactPoint("contact_rfoot", self.robot, rf_frame_name, contactNormal, mu, fMin, fMax)
        self.contactRF.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactRF.setKd(2 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_rf_ref = self.robot.position(self.data, self.model.getJointId(rf_frame_name))
        self.contactRF.setReference(H_rf_ref)
        self.contactRF.useLocalFrame(False)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactRF, w_forceRef, 1, 1)

        self.H_lh_ref = H_lh_ref
        self.H_rh_ref = H_rh_ref
        self.H_lf_ref = H_lf_ref
        self.H_rf_ref = H_rf_ref

        self.activeContacts = []
        self.allContacts = [None] * 13
        self.allContacts[3] = self.contactLF
        self.allContacts[6] = self.contactLH
        self.allContacts[9] = self.contactRF
        self.allContacts[12] = self.contactRH

        self.frameNames = [None] * 13
        self.frameNames[3] = lf_frame_name
        self.frameNames[6] = lh_frame_name
        self.frameNames[9] = rf_frame_name
        self.frameNames[12] = rh_frame_name

    def updateRaiSim(self):
        self.q, self.v = self.sim.getQ(), self.sim.getV()
        kp = 80
        kd = 1.5
        self.qd, self.vd = self.q_command, self.v_command
        tau = self.tau*0 + kp*(self.qd - self.q)[7:] + kd*(self.vd - self.v)[6:]
        tau = np.concatenate((np.zeros(6), tau))
        self.sim.updateTau(tau)
        self.sim.integrate()

    def getPlots(self):
        if self.updatedPlot:
            self.updatedPlot = 0
            return True, self.t, np.abs((self.qd-self.q)[7:])
        else:
            return False, None, None

    def updateTSID(self):
    #     self.samplePosture
        # self.sampleCom
        # print(self.qd)

        # self.q, self.v = self.pinToRaiSim(self.sim.getQ(), self.sim.getV())
        self.q, self.v = self.sim.getQ(), self.sim.getV()
        saved = self.q[3]
        self.q[3] = self.q[6]
        self.q[6] = saved


        R = self.sim.getAngularVelTransform()
        self.v[3:6] = np.matmul(R, self.v[3:6])
        forces = self.sim.getForces()

        contacts = []
        for i in range(0, len(forces)):
            contacts.append(forces[i][0])
        # print(contacts)
        w_forceRef = 1
        for i in [3, 6, 9, 12]:
            if i in contacts:
                # print(i)
                if not(i in self.activeContacts):
                    self.activeContacts.append(i)
                    self.invdyn.addRigidContact(self.allContacts[i], w_forceRef, 2, 1)
                else:
                    for k in range(0, len(forces)):
                        if forces[k][0] == i:
                            f_index = k
                            f = forces[f_index][1]
                            self.allContacts[i].setForceReference(f)
            elif i in self.activeContacts:
                self.invdyn.removeRigidContact(self.allContacts[i].name, 0)
                self.activeContacts.remove(i)


        self.HQPData = self.invdyn.computeProblemData(self.t, self.q, self.v)
        # self.HQPData.print_all()

        sol = self.solver.solve(self.HQPData)
        if sol.status != 0:
            print("QP problem could not be solved! Error code:", sol.status)
            return 0
        self.tau = self.invdyn.getActuatorForces(sol)

        dv = self.invdyn.getAccelerations(sol)

        v_mean = self.v + 0.5 * self.dt_TSID * dv
        self.v_command += self.dt_TSID* dv
        self.q_command = pin.integrate(self.model, self.q, self.dt_TSID * v_mean)

        # print(self.q_des - self.q)

        self.t += self.dt_TSID

        self.pub_act.publish(self.t, self.q, self.v, self.tau)
