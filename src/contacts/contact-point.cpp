//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
//

#include "tsid/math/utils.hpp"
#include "tsid/contacts/contact-point.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

ContactPoint::ContactPoint(const std::string& name, RobotWrapper& robot,
                           const std::string& frameName,
                           ConstRefVector contactNormal,
                           const double frictionCoefficient,
                           const double minNormalForce,
                           const double maxNormalForce)
    : ContactBase(name, robot),
      m_motionTask(name, robot, frameName),
      m_forceInequality(name, 5, 3),
      m_forceRegTask(name, 3, 3),
      m_contactNormal(contactNormal),
      m_mu(frictionCoefficient),
      m_fMin(minNormalForce),
      m_fMax(maxNormalForce) {
  m_weightForceRegTask << 1, 1, 1e-3;
  m_forceGenMat.resize(3, 3);
  m_fRef = Vector3::Zero();
  m_contactPoints.resize(3, 1);
  m_contactPoints.setZero();
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();

  math::Vector motion_mask(6);
  motion_mask << 1., 1., 1., 0., 0., 0.;
  m_motionTask.setMask(motion_mask);
}

void ContactPoint::useLocalFrame(bool local_frame) {
  m_motionTask.useLocalFrame(local_frame);
}

void ContactPoint::updateForceInequalityConstraints() {
  Vector3 t1, t2;
  const int n_in = 4 * 1 + 1;
  const int n_var = 3 * 1;
  Matrix B = Matrix::Zero(n_in, n_var);
  Vector lb = -1e10 * Vector::Ones(n_in);
  Vector ub = Vector::Zero(n_in);
  t1 = m_contactNormal.cross(Vector3::UnitX());
  if (t1.norm() < 1e-5) t1 = m_contactNormal.cross(Vector3::UnitY());
  t2 = m_contactNormal.cross(t1);
  t1.normalize();
  t2.normalize();

  B.block<1, 3>(0, 0) = (-t1 - m_mu * m_contactNormal).transpose();
  B.block<1, 3>(1, 0) = (t1 - m_mu * m_contactNormal).transpose();
  B.block<1, 3>(2, 0) = (-t2 - m_mu * m_contactNormal).transpose();
  B.block<1, 3>(3, 0) = (t2 - m_mu * m_contactNormal).transpose();

  B.block<1, 3>(n_in - 1, 0) = m_contactNormal.transpose();
  ub(n_in - 1) = m_fMax;
  lb(n_in - 1) = m_fMin;

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double ContactPoint::getNormalForce(ConstRefVector f) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      f.size() == n_force(),
      "Size of f is wrong - needs to be " + std::to_string(n_force()));
  return m_contactNormal.dot(f);
}

const Matrix3x& ContactPoint::getContactPoints() const {
  return m_contactPoints;
}

void ContactPoint::setRegularizationTaskWeightVector(ConstRefVector& w) {
  m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void ContactPoint::updateForceRegularizationTask() {
  Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A);
  m_forceRegTask.setVector(A * m_fRef);
}

void ContactPoint::updateForceGeneratorMatrix() { m_forceGenMat.setIdentity(); }

unsigned int ContactPoint::n_motion() const { return m_motionTask.dim(); }
unsigned int ContactPoint::n_force() const { return 3; }

const Vector& ContactPoint::Kp() {
  m_Kp3 = m_motionTask.Kp().head<3>();
  return m_Kp3;
}

const Vector& ContactPoint::Kd() {
  m_Kd3 = m_motionTask.Kd().head<3>();
  return m_Kd3;
}

void ContactPoint::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 3,
                                 "Size of Kp vector needs to equal 3");
  Vector6 Kp6;
  Kp6.head<3>() = Kp;
  m_motionTask.Kp(Kp6);
}

void ContactPoint::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 3,
                                 "Size of Kd vector needs to equal 3");
  Vector6 Kd6;
  Kd6.head<3>() = Kd;
  m_motionTask.Kd(Kd6);
}

bool ContactPoint::setContactNormal(ConstRefVector contactNormal) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      contactNormal.size() == 3,
      "Size of contact normal vector needs to equal 3");
  if (contactNormal.size() != 3) return false;
  m_contactNormal = contactNormal;
  updateForceInequalityConstraints();
  return true;
}

bool ContactPoint::setFrictionCoefficient(const double frictionCoefficient) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(frictionCoefficient > 0.0,
                                 "Friction coefficient needs to be positive");
  if (frictionCoefficient <= 0.0) return false;
  m_mu = frictionCoefficient;
  updateForceInequalityConstraints();
  return true;
}

bool ContactPoint::setMinNormalForce(const double minNormalForce) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      minNormalForce > 0.0 && minNormalForce <= m_fMax,
      "The minimal normal force needs to be greater than 0 and less than or "
      "equal to the maximum force.");
  if (minNormalForce <= 0.0 || minNormalForce > m_fMax) return false;
  m_fMin = minNormalForce;
  Vector& lb = m_forceInequality.lowerBound();
  lb(lb.size() - 1) = m_fMin;
  return true;
}

bool ContactPoint::setMaxNormalForce(const double maxNormalForce) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(maxNormalForce >= m_fMin,
                                 "The maximal normal force needs to be greater "
                                 "than or equal to the minimal force");
  if (maxNormalForce < m_fMin) return false;
  m_fMax = maxNormalForce;
  Vector& ub = m_forceInequality.upperBound();
  ub(ub.size() - 1) = m_fMax;
  return true;
}

void ContactPoint::setForceReference(ConstRefVector& f_ref) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      f_ref.size() == 3, "The size of the force reference needs to equal 3");
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

void ContactPoint::setReference(const SE3& ref) {
  m_motionTask.setReference(ref);
}

const ConstraintBase& ContactPoint::computeMotionTask(const double t,
                                                      ConstRefVector q,
                                                      ConstRefVector v,
                                                      Data& data) {
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality& ContactPoint::computeForceTask(const double,
                                                           ConstRefVector,
                                                           ConstRefVector,
                                                           const Data&) {
  return m_forceInequality;
}

const Matrix& ContactPoint::getForceGeneratorMatrix() { return m_forceGenMat; }

const ConstraintEquality& ContactPoint::computeForceRegularizationTask(
    const double, ConstRefVector, ConstRefVector, const Data&) {
  return m_forceRegTask;
}

double ContactPoint::getMinNormalForce() const { return m_fMin; }
double ContactPoint::getMaxNormalForce() const { return m_fMax; }

const TaskSE3Equality& ContactPoint::getMotionTask() const {
  return m_motionTask;
}

const ConstraintBase& ContactPoint::getMotionConstraint() const {
  return m_motionTask.getConstraint();
}

const ConstraintInequality& ContactPoint::getForceConstraint() const {
  return m_forceInequality;
}

const ConstraintEquality& ContactPoint::getForceRegularizationTask() const {
  return m_forceRegTask;
}
