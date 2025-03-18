"""Simple demo of usage of ContactTwoFramePositions in TSID
Make the Talos gripper model works with closed kinematic chains
(c) MIPT
"""

import time
from pathlib import Path

import numpy as np
import pinocchio as pin
import tsid
from numpy.linalg import norm as norm

np.set_printoptions(precision=3, linewidth=200, suppress=True)

LINE_WIDTH = 60
print("".center(LINE_WIDTH, "#"))
print(
    " Demo of TSID with Closed Kinematic Chains via ContactTwoFramePositions".center(
        LINE_WIDTH, "#"
    )
)
print("".center(LINE_WIDTH, "#"), "\n")

w_ee = 1.0  # weight of end effector task
w_posture = 1e-3  # weight of joint posture task

kp_ee = 10.0  # proportional gain of end effector task
kp_posture = 10.0  # proportional gain of joint posture task

dt = 0.001  # controller time step
PRINT_N = 500  # print every PRINT_N time steps
DISPLAY_N = 25  # update robot configuration in viwewer every DISPLAY_N time steps
N_SIMULATION = 6000  # number of time steps simulated

# Loading Talos gripper model modified with extra links to mark the position of contact creation
# Talos gripepr model (c) 2016, PAL Robotics, S.L.
# Please use https://github.com/egordv/tsid_demo_closed_kinematic_chain repo for the model files

filename = str(Path(__file__).resolve().parent)
path = filename + "../../tsid_demo_closed_kinematic_chain/models/talos_gripper"
urdf = path + "../../tsid_demo_closed_kinematic_chain/urdf/talos_gripper_half.urdf"
vector = pin.StdVec_StdString()
vector.extend(item for item in path)

robot = tsid.RobotWrapper(urdf, vector, False)  # Load with fixed base

# for viewer
robot_display = pin.RobotWrapper.BuildFromURDF(urdf, [path])
robot_display.initViewer(loadModel=True)

model = robot.model()
q = np.zeros(robot.nq)
v = np.zeros(robot.nv)

viz = pin.visualize.MeshcatVisualizer(
    robot_display.model,
    robot_display.collision_model,
    robot_display.visual_model,
)
viz.initViewer(loadModel=True, open=True)

t = 0.0  # time
invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()


fingertip_name = "gripper_left_fingertip_3_link"
H_fingertip_ref = robot.framePosition(invdyn.data(), model.getFrameId(fingertip_name))

fingertipPositionTask = tsid.TaskSE3Equality(
    "task-fingertip-position", robot, fingertip_name
)
fingertipPositionTask.useLocalFrame(False)
fingertipPositionTask.setKp(kp_ee * np.ones(6))
fingertipPositionTask.setKd(2.0 * np.sqrt(kp_ee) * np.ones(6))
trajFingertipPosition = tsid.TrajectorySE3Constant(
    "traj-fingertip-position", H_fingertip_ref
)
sampleFingertipPosition = trajFingertipPosition.computeNext()
fingertipPositionTask.setReference(sampleFingertipPosition)
invdyn.addMotionTask(fingertipPositionTask, w_ee, 1, 0.0)

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * np.ones(robot.nv))
postureTask.setKd(2.0 * np.sqrt(kp_posture) * np.ones(robot.nv))
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)


# Creating a closed kinematic chain in TSID formulation by creating a contact between two frames, for which there are special links in URDF
ContactTwoFramePositionsFingertipBottomAxis = tsid.ContactTwoFramePositions(
    "contact-two-frame-positions-fingertip-bottom-axis",
    robot,
    "gripper_left_motor_single_link_ckc_axis",
    "gripper_left_fingertip_3_link_ckc_axis",
    -1000,
    1000,
)
twoFramesContact_Kp = 300
ContactTwoFramePositionsFingertipBottomAxis.setKp(twoFramesContact_Kp * np.ones(3))
ContactTwoFramePositionsFingertipBottomAxis.setKd(
    2.0 * np.sqrt(twoFramesContact_Kp) * np.ones(3)
)

twoFramesContact_w_forceRef = 1e-5
invdyn.addRigidContact(
    ContactTwoFramePositionsFingertipBottomAxis, twoFramesContact_w_forceRef, 1.0, 1
)

# Setting actuation to zero for passive joints in kinematic chain via TaskActuationBounds
tau_max = model.effortLimit[-robot.na :]
# setting gripper_left_inner_single_joint to passive contstrainig it's actuation bounds to zero
tau_max[0] = 0.0
# setting gripper_left_fingertip_3_joint to passive contstrainig it's actuation bounds to zero
tau_max[1] = 0.0
tau_min = -tau_max
actuationBoundsTask = tsid.TaskActuationBounds("task-actuation-bounds", robot)
actuationBoundsTask.setBounds(tau_min, tau_max)
invdyn.addActuationTask(actuationBoundsTask, 1.0, 0, 0.0)

q_ref = q
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)

print(
    "Create QP solver with ",
    invdyn.nVar,
    " variables, ",
    invdyn.nEq,
    " equality and ",
    invdyn.nIn,
    " inequality constraints",
)
solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

offset = sampleFingertipPosition.pos()
offset[:3] += np.array([0, -0.04, 0])
amp = np.array([0.0, 0.04, 0.0])
two_pi_f = 2 * np.pi * np.array([0.0, 0.5, 0.0])
two_pi_f_amp = np.multiply(two_pi_f, amp)
two_pi_f_squared_amp = np.multiply(two_pi_f, two_pi_f_amp)

pEE = offset.copy()
vEE = np.zeros(6)
aEE = np.zeros(6)

i = 0
while True:
    time_start = time.time()

    # Setting gripper finger task target to sine motion
    pEE[:3] = offset[:3] + amp * np.sin(two_pi_f * t)
    vEE[:3] = two_pi_f_amp * np.cos(two_pi_f * t)
    aEE[:3] = -two_pi_f_squared_amp * np.sin(two_pi_f * t)
    sampleFingertipPosition.value(pEE)
    sampleFingertipPosition.derivative(vEE)
    sampleFingertipPosition.second_derivative(aEE)

    fingertipPositionTask.setReference(sampleFingertipPosition)

    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)

    # Computing HQP
    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0:
        HQPData.print_all()

    sol = solver.solve(HQPData)
    if sol.status != 0:
        print(f"[{i}] QP problem could not be solved! Error code:", sol.status)
        break

    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    if i % PRINT_N == 0:
        print(f"Time {t:.3f}")

    v_mean = v + 0.5 * dt * dv
    v += dt * dv
    q = pin.integrate(robot.model(), q, dt * v_mean)
    t += dt

    if i % DISPLAY_N == 0:
        viz.display(q)

    time_spent = time.time() - time_start
    if time_spent < dt:
        time.sleep(dt - time_spent)

    i = i + 1
