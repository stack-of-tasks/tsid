from pathlib import Path

import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test RobotWrapper")
print("")


filename = str(Path(__file__).resolve().parent)
path = filename + "/../../models/romeo"
urdf = path + "/urdf/romeo.urdf"
vector = se3.StdVec_StdString()
vector.extend(item for item in path)

robot = tsid.RobotWrapper(urdf, vector, se3.JointModelFreeFlyer(), False)
model = robot.model()
lb = model.lowerPositionLimit
lb[0:3] = -10.0 * np.ones(3)
lb[3:7] = -1.0 * np.ones(4)

ub = model.upperPositionLimit
ub[0:3] = 10.0 * np.ones(3)
ub[3:7] = 1.0 * np.ones(4)

q = se3.randomConfiguration(robot.model(), lb, ub)
print(q.transpose())


data = robot.data()
v = np.ones(robot.nv)
robot.computeAllTerms(data, q, v)
print(robot.com(data))
base_mass_matrix = robot.mass(data)

# Adding motor inertia
rng = np.random.default_rng()
rotor_inertia = rng.uniform(0, 1, size=(robot.nq - 7))
gear_ratio = rng.uniform(0, 1, size=(robot.nq - 7))
expected_inertia = rotor_inertia * gear_ratio**2
print(f"Expected motors inertia: {expected_inertia}")

robot.set_rotor_inertias(rotor_inertia)
robot.set_gear_ratios(gear_ratio)

robot.computeAllTerms(data, q, v)

mass_matrix_with_motor_inertia = robot.mass(data)

# Assert that mass matrices are different by motor's inertia
np.testing.assert_allclose(
    np.diag(mass_matrix_with_motor_inertia - base_mass_matrix)[6:], expected_inertia
)


print("All test is done")
