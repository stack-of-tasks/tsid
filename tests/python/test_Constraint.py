import numpy as np
import pinocchio as se3
import tsid

print("")
print("Test Constraint Bound")
print("")


tol = 1e-5
n = 5
lb = -1.0 * np.ones(n)
ub = np.ones(n)
ConstBound = tsid.ConstraintBound("bounds", lb, ub)

assert ConstBound.isBound
assert not ConstBound.isEquality
assert not ConstBound.isInequality

assert ConstBound.rows == n
assert ConstBound.cols == n

assert lb.all() == ConstBound.lowerBound.all()
assert ub.all() == ConstBound.upperBound.all()

lb *= 2.0
assert np.linalg.norm(lb - ConstBound.lowerBound, 2) != 0
ConstBound.setLowerBound(lb)
assert lb.all() == ConstBound.lowerBound.all()

ub *= 2.0
assert np.linalg.norm(ub - ConstBound.upperBound, 2) != 0
ConstBound.setUpperBound(ub)
assert ub.all() == ConstBound.upperBound.all()
assert np.linalg.norm(ub - ConstBound.upperBound, 2) < tol

print("")
print("Test Constraint Equality")
print("")
n = 5
m = 2
A = np.ones((m, n))
b = np.ones(m)
equality = tsid.ConstraintEquality("equality", A, b)

assert not equality.isBound
assert equality.isEquality
assert not equality.isInequality

assert equality.rows == m
assert equality.cols == n

assert np.linalg.norm(A - equality.matrix, 2) < tol
assert np.linalg.norm(b - equality.vector, 2) < tol

b *= 2.0
assert np.linalg.norm(b - equality.vector, 2) != 0
equality.setVector(b)
assert np.linalg.norm(b - equality.vector, 2) < tol

A *= 2.0
assert np.linalg.norm(A - equality.matrix, 2) != 0
equality.setMatrix(A)
assert np.linalg.norm(A - equality.matrix, 2) < tol

print("")
print("Test Constraint Inequality")
print("")

n = 5
m = 2
A = np.ones((m, n))
lb = -np.ones(m)
ub = np.ones(m)
inequality = tsid.ConstraintInequality("inequality", A, lb, ub)

assert not inequality.isBound
assert not inequality.isEquality
assert inequality.isInequality

assert inequality.rows == m
assert inequality.cols == n

lb *= 2.0
assert np.linalg.norm(lb - inequality.lowerBound, 2) != 0
inequality.setLowerBound(lb)
assert np.linalg.norm(lb - inequality.lowerBound, 2) < tol

A *= 2.0
assert np.linalg.norm(A - inequality.matrix, 2) != 0
inequality.setMatrix(A)
assert np.linalg.norm(A - inequality.matrix, 2) < tol

print("All test is done")
