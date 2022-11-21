import numpy as np
from numpy import nan
from numpy.linalg import norm as norm
import matplotlib.pyplot as plt
import plot_utils as plut
import time
import pinocchio as pin
import tsid



print("".center(30, '#'))
print(" TSID - Quadruped ".center(30, '#'))
print("".center(30, '#'), '\n')

path = '/home/kian/catkin_ws/src/example-robot-data/robots/ur_description'
urdf = '/home/kian/catkin_ws/src/example-robot-data/robots/ur_description/urdf/ur5_robot.urdf'
srdf = '/home/kian/catkin_ws/src/example-robot-data/robots/ur_description/srdf/ur5.srdf'

robot = tsid.RobotWrapper(urdf, [path], pin.JointModelFreeFlyer(), False)
model = robot.model()
pin.loadReferenceConfigurations(model, srdf, False)
# q0 = model.referenceConfigurations
q0 = pin.neutral(model)
v0 = np.zeros(robot.nv)

formulation = tsid.InverseDynamicsFormulationAccForce("tsid", robot, False)
formulation.computeProblemData(0.0, q0, v0)

w_posture = 0.1
kp_posture = 1.0               # proportional gain of joint posture task

postureTask = tsid.TaskJointPosture("task-posture", robot)
postureTask.setKp(kp_posture * np.ones(robot.nv))
postureTask.setKd(2.0 * np.sqrt(kp_posture) * np.ones(robot.nv))
formulation.addMotionTask(postureTask, w_posture, 1, 0.0)

trajPosture = tsid.TrajectoryEuclidianConstant("traj_joint", q0)
postureTask.setReference(trajPosture.computeNext())

v_max_scaling = 0.8
v_max = v_max_scaling * model.velocityLimit
v_min = -v_max

solver = tsid.SolverHQuadProgFast("qp solver")
solver.resize(formulation.nVar, formulation.nEq, formulation.nIn)

# NUMBER OF SIMULATION STEPS
N = 5000

tau    = np.empty((robot.na, N))*nan
q      = np.empty((robot.nq, N+1))*nan
v      = np.empty((robot.nv, N+1))*nan
dv     = np.empty((robot.nv, N+1))*nan
q_ref  = np.empty((robot.nq, N))*nan
v_ref  = np.empty((robot.nv, N))*nan
dv_ref = np.empty((robot.nv, N))*nan
dv_des = np.empty((robot.nv, N))*nan
samplePosture = trajPosture.computeNext()

viewer=pin.visualize.MeshcatVisualizer


robot_display = pin.RobotWrapper.BuildFromURDF(urdf, [path], pin.JointModelFreeFlyer())
viz = viewer(robot_display.model, robot_display.collision_model, robot_display.visual_model)
viz.initViewer(loadModel=True)
viz.display(q0)

# t = 0.0
# q, v = tsid.q, tsid.v

print(q0)
t = 0
dt = 0.002
q[:,0], v[:,0] = q0, v0

amp = [0.0001, 0.0001, 0.0001, 0.0001, 0.0001, 0.0001]

# lz = 0.01
# lxn = 0.1
# lxp = 0.1
# lyn = 0.1
# lyp = 0.1
# contactNormal = np.array([0., 0., 1.])
# # tip_frame_name =
# fMin = 5.0                          # minimum normal force
# fMax = 1000.0                       # maximum normal force
# mu = 0.3
#
# contact_Point = np.ones((3, 4)) * (-lz)
# contact_Point[0, :] = [-lxn, -lxn, lxp, lxp]
# contact_Point[1, :] = [-lyn, lyp, -lyn, lyp]
#
# contactRF = tsid.Contact6d("contact_rfoot", robot, tip_frame_name, contact_Point,
#                            contactNormal, mu, fMin, fMax)
# contactRF.setKp(kp_contact * np.ones(6))
# contactRF.setKd(2.0 * np.sqrt(kp_contact) * np.ones(6))
# self.RF = robot.model().getFrameId(rf_frame_name)
# H_rf_ref = robot.framePosition(data, self.RF)

# modify initial robot configuration so that foot is on the ground (z=0)
# q[2] -= H_rf_ref.translation[2] - lz
# formulation.computeProblemData(0.0, q, v)
# data = formulation.data()
# H_rf_ref = robot.framePosition(data, self.RF)
# contactRF.setReference(H_rf_ref)
# if w_contact >= 0.0:
#     formulation.addRigidContact(contactRF, w_forceRef, w_contact, 1)
# else:
#     formulation.addRigidContact(contactRF, w_forceRef)

for i in range(0, N):
    q_ref[0:7, i] = q0[0:7]
    v_ref[0:6, i] = np.zeros(6)
    dv_ref[0:6, i] = np.zeros(6)


for i in range(0, N):
    time_start = time.time()

    # set reference trajectory
    q_ref[0, i] = i*0.0001
    if (i < N/2):
        q_ref[7:13,i]  = amp * np.ones(6)*i
    elif (i < N*0.6):
        q_ref[7:13, i] = amp * np.ones(6)*(N/2)
    elif (i < N*0.8):
        q_ref[7:13, i] = amp * np.ones(6) * (i-(N/10))
    else:
        q_ref[7:13, i] = amp * np.ones(6) * (N*0.7)
        q_ref[0, i] = N*0.8*0.0001
    v_ref[6:12,i]  = np.zeros(6)
    dv_ref[6:12,i] = np.zeros(6)
    samplePosture.pos(q_ref[:,i])
    samplePosture.vel(v_ref[:,i])
    samplePosture.acc(dv_ref[:,i])
    postureTask.setReference(samplePosture)

    HQPData = formulation.computeProblemData(t, q_ref[:,i], v_ref[:,i])
    sol = solver.solve(HQPData)
    if(sol.status!=0):
        print("Time %.3f QP problem could not be solved! Error code:"%t, sol.status)
        break
    tau[:,i] = formulation.getActuatorForces(sol)
    dv[:,i] = formulation.getAccelerations(sol)
    dv_des[:,i] = postureTask.getDesiredAcceleration

    if i%50 == 0:
        print("Time %.3f"%(t))
        print("\ttracking err %s: %.3f"%(postureTask.name.ljust(20,'.'), norm(postureTask.position_error, 2)))

    # numerical integration
    v_mean = v[:,i] + 0.5*dt*dv[:,i]
    v[:,i+1] = v[:,i] + dt*dv[:,i]
    q[:,i+1] = pin.integrate(model, q[:,i], dt*v_mean)
    t += dt

    if i % 20 == 0:
        viz.display(q_ref[:,i])

    time_spent = time.time() - time_start
    if(time_spent < dt): time.sleep(dt-time_spent)