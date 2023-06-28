import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test Trajectory Euclidian")
print("")


tol = 1e-5
n = 5
q_ref = np.ones(n)
zero = np.zeros(n)

traj_euclidian = tsid.TrajectoryEuclidianConstant("traj_eucl", q_ref)
assert traj_euclidian.has_trajectory_ended()
assert np.linalg.norm(traj_euclidian.computeNext().value() - q_ref, 2) < tol
assert np.linalg.norm(traj_euclidian.getSample(0.0).value() - q_ref, 2) < tol

traj_sample = tsid.TrajectorySample(n)
traj_euclidian.getLastSample(traj_sample)
assert np.linalg.norm(traj_sample.value() - q_ref, 2) < tol
assert np.linalg.norm(traj_sample.derivative() - zero, 2) < tol
assert np.linalg.norm(traj_sample.second_derivative() - zero, 2) < tol

print("")
print("Test Trajectory SE3")
print("")

M_ref = se3.SE3.Identity()
M_vec = np.zeros(12)
M_vec[0:3] = M_ref.translation
zero = np.zeros(6)

for i in range(1, 4):
    M_vec[3 * i : 3 * i + 3] = M_ref.rotation[:, i - 1]

traj_se3 = tsid.TrajectorySE3Constant("traj_se3")
traj_se3.setReference(M_ref)

assert traj_se3.has_trajectory_ended()
assert np.linalg.norm(traj_se3.computeNext().value() - M_vec, 2) < tol
assert np.linalg.norm(traj_se3.getSample(0.0).value() - M_vec, 2) < tol

traj_sample = tsid.TrajectorySample(12, 6)
traj_se3.getLastSample(traj_sample)
assert np.linalg.norm(traj_sample.value() - M_vec, 2) < tol
assert np.linalg.norm(traj_sample.derivative() - zero, 2) < tol
assert np.linalg.norm(traj_sample.second_derivative() - zero, 2) < tol

print("All test is done")
