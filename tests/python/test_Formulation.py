from pathlib import Path

import numpy as np
import pinocchio as se3
import tsid
from numpy.linalg import norm

print("")
print("Test InvDyn")
print("")

filename = str(Path(__file__).resolve().parent)
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)
robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)

srdf = path + "/srdf/romeo_collision.srdf"

model = robot.model()
se3.loadReferenceConfigurations(model, srdf, False)
q = model.referenceConfigurations["half_sitting"]

q[2] += 0.84
v = np.zeros(robot.nv)

t = 0.0
lxp = 0.14
lxn = 0.077
lyp = 0.069
lyn = 0.069
lz = 0.105
mu = 0.3
fMin = 5.0
fMax = 1000.0
rf_frame_name = "RAnkleRoll"
lf_frame_name = "LAnkleRoll"
contactNormal = np.array([0.0, 0.0, 1.0])
w_com = 1.0
w_posture = 1e-2
w_forceRef = 1e-5
kp_contact = 100.0
kp_com = 30.0
kp_posture = 30.0

assert robot.model().existFrame(rf_frame_name)
assert robot.model().existFrame(lf_frame_name)

invdyn = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
invdyn.computeProblemData(t, q, v)
data = invdyn.data()
contact_Point = np.ones((3, 4)) * lz
contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]

contactRF = tsid.Contact6d(
    "contact_rfoot",
    robot,
    rf_frame_name,
    contact_Point,
    contactNormal,
    mu,
    fMin,
    fMax,
    w_forceRef,
)
contactRF.setKp(kp_contact * np.ones(6))
contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))
H_rf_ref = robot.position(data, robot.model().getJointId(rf_frame_name))
contactRF.setReference(H_rf_ref)
invdyn.addRigidContact(contactRF)

contactLF = tsid.Contact6d(
    "contact_lfoot",
    robot,
    lf_frame_name,
    contact_Point,
    contactNormal,
    mu,
    fMin,
    fMax,
    w_forceRef,
)
contactLF.setKp(kp_contact * np.ones(6))
contactLF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))
H_lf_ref = robot.position(data, robot.model().getJointId(lf_frame_name))
contactLF.setReference(H_lf_ref)
invdyn.addRigidContact(contactLF)

comTask = tsid.TaskComEquality("task-com", robot)
comTask.setKp(kp_com * np.ones(3))
comTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(3))
invdyn.addMotionTask(comTask, w_com, 1, 0.0)

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * np.ones(robot.nv - 6))
postureTask.setKd(2.0 * np.sqrt(kp_posture) * np.ones(robot.nv - 6))
invdyn.addMotionTask(postureTask, w_posture, 1, 0.0)

# ########## Test 1 ##################3
dt = 0.01
PRINT_N = 100
REMOVE_CONTACT_N = 100
CONTACT_TRANSITION_TIME = 1.0
kp_RF = 100.0
w_RF = 1e3
max_it = 1000
rightFootTask = tsid.TaskSE3Equality("task-right-foot", robot, rf_frame_name)
rightFootTask.setKp(kp_RF * np.ones(6))
rightFootTask.setKd(2.0 * np.sqrt(kp_com) * np.ones(6))
H_rf_ref = robot.position(data, robot.model().getJointId(rf_frame_name))
invdyn.addMotionTask(rightFootTask, w_RF, 1, 0.0)

s = tsid.TrajectorySample(12, 6)
H_rf_ref_vec = np.zeros(12)
H_rf_ref_vec[0:3] = H_rf_ref.translation
for i in range(0, 3):
    H_rf_ref_vec[3 * i + 3 : 3 * i + 6] = H_rf_ref.rotation[:, i]
s.value(H_rf_ref_vec)
rightFootTask.setReference(s)

com_ref = robot.com(data)
com_ref[1] += 0.1
trajCom = tsid.TrajectoryEuclidianConstant("traj_com", com_ref)
sampleCom = tsid.TrajectorySample(3)

q_ref = q[7:]
trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q_ref)
samplePosture = tsid.TrajectorySample(robot.nv - 6)

solver = tsid.SolverHQuadProg("qp solver")
solver.resize(invdyn.nVar, invdyn.nEq, invdyn.nIn)

tau_old = np.zeros(robot.nv - 6)

for i in range(0, max_it):
    if i == REMOVE_CONTACT_N:
        print("Start breaking contact right foot")
        invdyn.removeRigidContact(contactRF.name, CONTACT_TRANSITION_TIME)

    sampleCom = trajCom.computeNext()
    comTask.setReference(sampleCom)
    samplePosture = trajPosture.computeNext()
    postureTask.setReference(samplePosture)

    HQPData = invdyn.computeProblemData(t, q, v)
    if i == 0:
        HQPData.print_all()

    sol = solver.solve(HQPData)
    tau = invdyn.getActuatorForces(sol)
    dv = invdyn.getAccelerations(sol)

    if i > 0:
        # assert norm(tau-tau_old) < 2e1
        tau_old = tau
        if i % PRINT_N == 0:
            print("Time ", i)
            if invdyn.checkContact(contactRF.name, sol):
                f = invdyn.getContactForce(contactRF.name, sol)
                print("   ", contactRF.name, "force: ", contactRF.getNormalForce(f))

            if invdyn.checkContact(contactLF.name, sol):
                f = invdyn.getContactForce(contactLF.name, sol)
                print("   ", contactLF.name, "force: ", contactLF.getNormalForce(f))

            print("   ", comTask.name, " err:", norm(comTask.position_error, 2))
            print("   ", "v: ", norm(v, 2), "dv: ", norm(dv))

    v += dt * dv
    q = se3.integrate(robot.model(), q, dt * v)
    t += dt

    assert norm(dv) < 1e6
    assert norm(v) < 1e6

print("Final COM Position", robot.com(invdyn.data()).transpose())
print("Desired COM Position", com_ref.transpose())
