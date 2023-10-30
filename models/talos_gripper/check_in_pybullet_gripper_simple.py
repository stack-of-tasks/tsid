import time

import pybullet as p
import pybullet_data as pd

p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath()) # to load plane/ball/etc

# p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(
    cameraDistance=0.15,
    cameraYaw=90,
    cameraPitch=-25,
    cameraTargetPosition=[0, 0, 0.05],
)
humanoid = p.loadURDF(
    "urdf/talos_gripper_half.urdf", [0, 0, 0], [1, 0, 0, 0], useFixedBase=True
)

gravId = p.addUserDebugParameter("gravity", -10, 10, -10)
jointIds = []
paramIds = []

p.setPhysicsEngineParameter(numSolverIterations=100)
p.changeDynamics(humanoid, -1, linearDamping=0, angularDamping=0)


print(p.getNumJoints(humanoid))

# jointAngles=[0,0,1.0204,-1.97,-0.084,2.06,-1.9,0,0,1.0204,-1.97,-0.084,2.06,-1.9,0]
jointAngles = [
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
    0,
]
activeJoint = 0
for j in range(p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid, j, linearDamping=0, angularDamping=0)
    info = p.getJointInfo(humanoid, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
        jointIds.append(j)
        paramIds.append(
            p.addUserDebugParameter(
                jointName.decode("utf-8"), -4, 4, jointAngles[activeJoint]
            )
        )
        p.resetJointState(humanoid, j, jointAngles[activeJoint])
        activeJoint += 1

p.setRealTimeSimulation(1)
while 1:
    # p.getCameraImage(320,200)
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)
        p.setJointMotorControl2(
            humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=140.0
        )
    time.sleep(0.01)
