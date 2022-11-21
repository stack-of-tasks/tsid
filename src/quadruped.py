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
        urdf = path + '/anymal_raisim/urdf/anymal.urdf'
        srdf = path + '/anymal_raisim/srdf/anymal.srdf'

        # dt for Raisim
        self.dt = dt_RaiSim

        self.dt_TSID = dt_TSID

        # robot and model for use in TSID
        self.robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
        self.model = self.robot.model()

        # Load initial configuration
        pin.loadReferenceConfigurations(self.model, srdf, False)
        self.q = self.model.referenceConfigurations['standing']
        self.v = np.zeros(self.robot.nv)
        # print(self.q)
        self.q, self.v = self.pinToRaiSim(self.q, self.v)
        # print(self.q)
        # self.q[2] += 1.2

        # Initialise an instance of RaiSim
        self.sim = RaiSim(self.dt)
        # Add robot (with appropriate conversion of coordinates)
        # Conversion of coordinates can be automated in future using alphabetical sorting (as pinocchio uses this)
        # qs, vs = self.pinToRaiSim(self.q, self.v)
        self.sim.addRobot(urdf, self.q, self.v)

        self.sim.setTarget(self.q, self.v)

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
        self.q_command = np.array(self.q)
        self.v_command = np.array(self.v)
        time.sleep(5)

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
        w_com = 0.5 # CoM task weight
        kp_com = 20.0  # proportional gain of center of mass task
        self.comTask = tsid.TaskComEquality("task-com", self.robot)
        self.comTask.setKp(kp_com * np.ones(3))  # Proportional gain defined before = 20
        self.comTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(3))  # Derivative gain = 2 * sqrt(20)
        # Add the task to the HQP with weight = w_com, priority level = 0 (as constraint) and a transition duration = 0.0
        self.invdyn.addMotionTask(self.comTask, w_com, 1, 0)

        com_ref = self.data.com[0]  # Initial value of the CoM
        self.trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
        self.sampleCom = self.trajCom.computeNext()  # Compute the first step of the trajectory from the initial value


    def postureTaskInit(self):
        # SHOULD THE FIRST 7 ELEMENTS BE 1 or 0?
        kp_posture = np.array(  # proportional gain of joint posture task
            [1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1]
        )
        w_posture = 1000

        self.postureTask = tsid.TaskJointPosture("task-posture", self.robot)
        self.postureTask.setKp(kp_posture)  # Proportional gain defined before (different for each joints)
        self.postureTask.setKd(2 * kp_posture)  # Derivative gain = 2 * kp
        # Add the task with weight = w_posture, priority level = 1 (in cost function) and a transition duration = 0.0
        self.invdyn.addMotionTask(self.postureTask, w_posture, 1, 0.0)

        self.trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", self.q)
        self.samplePosture = self.trajPosture.computeNext()
        self.samplePosture.value(self.q)
        self.postureTask.setReference(self.samplePosture)

    def contactTaskInit(self):
        mu = 0.8  # friction coefficient
        fMin = 1.0  # minimum normal force
        fMax = 1000.0  # maximum normal force

        rh_frame_name = "RH_ADAPTER_TO_FOOT"
        lf_frame_name = "LF_ADAPTER_TO_FOOT"
        rf_frame_name = "RF_ADAPTER_TO_FOOT"
        lh_frame_name = "LH_ADAPTER_TO_FOOT"

        # rh_frame_name = "RH_KFE"
        # lf_frame_name = "LF_KFE"
        # rf_frame_name = "RF_KFE"
        # lh_frame_name = "LH_KFE"

        kp_contact = 30 # proportional gain of contact constraint
        w_forceRef = 0.5 # weight of force regularization task 1e-3

        contactNormal = np.array([0., 0., 1.])  # direction of the normal to the contact surface


        self.contactLF = tsid.ContactPoint("contact_lfoot", self.robot, lf_frame_name, contactNormal, mu, fMin, fMax)
        self.contactLF.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactLF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_lf_ref = self.robot.position(self.data, self.model.getJointId(lf_frame_name))
        self.contactLF.setReference(H_lf_ref)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactLF, w_forceRef, 1, 1)

        self.contactRH = tsid.ContactPoint("contact_rhfoot", self.robot, rh_frame_name, contactNormal, mu, fMin, fMax)
        self.contactRH.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactRH.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_rh_ref = self.robot.position(self.data, self.model.getJointId(rh_frame_name))
        self.contactRH.setReference(H_rh_ref)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactRH, w_forceRef, 1, 1)

        self.contactLH = tsid.ContactPoint("contact_lhfoot", self.robot, lh_frame_name, contactNormal, mu, fMin, fMax)
        self.contactLH.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactLH.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # # Reference position of the left ankle -> initial position
        H_lh_ref = self.robot.position(self.data, self.model.getJointId(lh_frame_name))
        self.contactLH.setReference(H_lh_ref)
        # Add the contact to the HQP with weight = 0.1 for the force regularization task,
        # and priority level = 0 (as real constraint) for the motion constraint
        #self.invdyn.addRigidContact(self.contactLH, w_forceRef, 1, 1)

        self.contactRF = tsid.ContactPoint("contact_rfoot", self.robot, rf_frame_name, contactNormal, mu, fMin, fMax)
        self.contactRF.setKp(kp_contact * np.ones(6))  # Proportional gain defined before = 30
        self.contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))  # Derivative gain = 2 * sqrt(30)
        # Reference position of the left ankle -> initial position
        H_rf_ref = self.robot.position(self.data, self.model.getJointId(rf_frame_name))
        self.contactRF.setReference(H_rf_ref)
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
        self.allContacts[6] = self.contactRF
        self.allContacts[9] = self.contactLH
        self.allContacts[12] = self.contactRH

        self.frameNames = [None] * 13
        self.frameNames[3] = lf_frame_name
        self.frameNames[6] = rf_frame_name
        self.frameNames[9] = lh_frame_name
        self.frameNames[12] = rh_frame_name

    def updateRaiSim(self):
        # self.q[9] += 0.0001
        # self.q[12] += 0.0001
        # self.q[15] -= 0.0001
        # self.q[18] -= 0.0001
        # self.sim.setTarget(self.q_command, self.v_command)
        self.sim.integrate()

    def updateTSID(self):
    #     self.samplePosture
        # self.sampleCom

        self.q, self.v = self.pinToRaiSim(self.sim.getQ(), self.sim.getV())

        forces = self.sim.getForces()

        ########## TAKING THIS BIT OUT REMOVES LEGS SWINGING OUT PROBLEM ##############################

        contacts = []
        for i in range(0, len(forces)):
            contacts.append(forces[i][0])
        # print(contacts)
        w_forceRef = 0.5
        for i in [3, 6, 9, 12]:
            if i in contacts:
                # print(i)
                if not(i in self.activeContacts):
                    self.activeContacts.append(i)
                    self.invdyn.addRigidContact(self.allContacts[i], w_forceRef, 1, 1)
                else:
                    for k in range(0, len(forces)):
                        if forces[k][0] == i:
                            f_index = k
                            f = forces[f_index][1]
                            self.allContacts[i].setForceReference(f)

        ########## TAKING THIS BIT OUT REMOVES LEGS SWINGING OUT PROBLEM ##############################

        self.HQPData = self.invdyn.computeProblemData(self.t, self.q, self.v)
        # self.HQPData.print_all()

        sol = self.solver.solve(self.HQPData)
        if sol.status != 0:
            print("QP problem could not be solved! Error code:", sol.status)
            return 0
        self.tau = self.invdyn.getActuatorForces(sol)
        tau_out = np.array(self.tau)

        dv = self.invdyn.getAccelerations(sol)

        v_mean = self.v + 0.5 * self.dt * dv
        self.v_command += self.dt * dv
        # self.dt or self.dt_TSID? ###############################################################
        self.q_command = pin.integrate(self.model, self.q, self.dt * v_mean)

        self.t += self.dt
        kp = 0
        kd = 0
        tau_out = tau_out + kp*(self.q_command - self.q)[7:] + kd*(self.v_command - self.v)[6:]
        print(self.q_command - self.q)
        print(self.v_command - self.v)

        # To convert from TSID joint order to RaiSim
        tau_out[3:6] = self.tau[6:9]
        tau_out[6:9] = self.tau[3:6]
        print(tau_out)
        tau_out = np.concatenate((np.zeros(6), tau_out), axis=None)
        self.sim.updateTau(tau_out)