import time

import pybullet as p
import pybullet_data as pd

p.connect(p.GUI)
# p.setAdditionalSearchPath(pd.getDataPath()) # to load plane/ball/etc

# p.loadURDF("plane.urdf")
p.resetDebugVisualizerCamera(
    cameraDistance=0.2,
    cameraYaw=115,
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

passiveJointIds = []

jointAnglesRefByName = {}

activeJoint = 0
_joint_name_to_index = {}
for j in range(p.getNumJoints(humanoid)):
    p.changeDynamics(humanoid, j, linearDamping=0.1, angularDamping=0.1)
    info = p.getJointInfo(humanoid, j)
    # print(info)
    jointName = info[1]
    jointType = info[2]
    if jointType == p.JOINT_PRISMATIC or jointType == p.JOINT_REVOLUTE:
        name = jointName.decode("utf-8")
        jointIds.append(j)
        _joint_name_to_index[name] = j
        refAngle = 0
        if name in jointAnglesRefByName.keys():
            refAngle = jointAnglesRefByName[name]
        paramIds.append(
            p.addUserDebugParameter(jointName.decode("utf-8"), -4, 4, refAngle)
        )
        p.resetJointState(humanoid, j, refAngle)
        activeJoint += 1

        if name.find("_PASSIVE") != -1:
            passiveJointIds.append(j)
            print("Passive joint found: ", j, " - ", name)
        # if name.find("_GEAR") != -1:
        # 	gearJointIds.append(j)
        # 	print("Gear joint found: ", j, " - ", name)

_link_name_to_index = {
    p.getBodyInfo(humanoid)[0].decode("UTF-8"): -1,
}
for _id in range(p.getNumJoints(humanoid)):
    _name = p.getJointInfo(humanoid, _id)[12].decode("UTF-8")
    _link_name_to_index[_name] = _id

for j in passiveJointIds:
    p.setJointMotorControl2(humanoid, j, p.VELOCITY_CONTROL, force=0.0)
# for j in gearJointIds:
# 	p.setJointMotorControl2(humanoid, j, p.VELOCITY_CONTROL, force=0.)


c1 = p.createConstraint(
    humanoid,
    _link_name_to_index["gripper_left_motor_single_link_ckc_axis"],
    humanoid,
    _link_name_to_index["gripper_left_fingertip_3_link_ckc_axis"],
    p.JOINT_POINT2POINT,
    jointAxis=[0, 0, 0],
    parentFramePosition=[0, 0, 0],
    childFramePosition=[0, 0, 0],
)
p.changeConstraint(c1, maxForce=10000, erp=1.0)


p.setRealTimeSimulation(1)
while 1:
    # p.getCameraImage(320,200)
    p.setGravity(0, 0, p.readUserDebugParameter(gravId))
    for i in range(len(paramIds)):
        c = paramIds[i]
        targetPos = p.readUserDebugParameter(c)

        if jointIds[i] in passiveJointIds:
            # Passive joints (like back thigh lever upper joint) - set zero torque
            p.setJointMotorControl2(
                humanoid, jointIds[i], p.TORQUE_CONTROL, targetPos, force=0.0
            )
        else:
            p.setJointMotorControl2(
                humanoid, jointIds[i], p.POSITION_CONTROL, targetPos, force=140.0
            )
    time.sleep(0.01)
