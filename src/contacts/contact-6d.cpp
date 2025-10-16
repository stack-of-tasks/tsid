//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
//

#include "tsid/math/utils.hpp"
#include "tsid/contacts/contact-6d.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

Contact6d::Contact6d(const std::string& name, RobotWrapper& robot,
                     const std::string& frameName, ConstRefMatrix contactPoints,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce, const double maxNormalForce)
    : ContactBase(name, robot),
      m_motionTask(name, robot, frameName),
      m_forceInequality(name, 17, 12),
      m_forceRegTask(name, 6, 12),
      m_contactPoints(contactPoints),
      m_contactNormal(contactNormal),
      m_mu(frictionCoefficient),
      m_fMin(minNormalForce),
      m_fMax(maxNormalForce) {
  this->init();
}

Contact6d::Contact6d(const std::string& name, RobotWrapper& robot,
                     const std::string& frameName, ConstRefMatrix contactPoints,
                     ConstRefVector contactNormal,
                     const double frictionCoefficient,
                     const double minNormalForce, const double maxNormalForce,
                     const double)
    : ContactBase(name, robot),
      m_motionTask(name, robot, frameName),
      m_forceInequality(name, 17, 12),
      m_forceRegTask(name, 6, 12),
      m_contactPoints(contactPoints),
      m_contactNormal(contactNormal),
      m_mu(frictionCoefficient),
      m_fMin(minNormalForce),
      m_fMax(maxNormalForce) {
  std::cout << "[Contact6d] The constructor with forceRegWeight is deprecated "
               "now. forceRegWeight should now be specified when calling "
               "addRigidContact()\n";
  this->init();
}

void Contact6d::init() {
  m_weightForceRegTask << 1, 1, 1e-3, 2, 2, 2;
  m_forceGenMat.resize(6, 12);
  m_fRef = Vector6::Zero();
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();
}

void Contact6d::updateForceInequalityConstraints() {
  Vector3 t1, t2;
  const int n_in = 4 * 4 + 1;
  const int n_var = 3 * 4;
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

  for (int i = 1; i < 4; i++) {
    B.block<4, 3>(4 * i, 3 * i) = B.topLeftCorner<4, 3>();
  }

  B.block<1, 3>(n_in - 1, 0) = m_contactNormal.transpose();
  B.block<1, 3>(n_in - 1, 3) = m_contactNormal.transpose();
  B.block<1, 3>(n_in - 1, 6) = m_contactNormal.transpose();
  B.block<1, 3>(n_in - 1, 9) = m_contactNormal.transpose();
  ub(n_in - 1) = m_fMax;
  lb(n_in - 1) = m_fMin;

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double Contact6d::getNormalForce(ConstRefVector f) const {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      f.size() == n_force(),
      "f needs to contain " + std::to_string(n_force()) + " rows");
  double n = 0.0;
  for (int i = 0; i < 4; i++) n += m_contactNormal.dot(f.segment<3>(i * 3));
  return n;
}

void Contact6d::setRegularizationTaskWeightVector(ConstRefVector& w) {
  m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void Contact6d::updateForceRegularizationTask() {
  typedef Eigen::Matrix<double, 6, 6> Matrix6;
  Matrix6 A = Matrix6::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A * m_forceGenMat);
  m_forceRegTask.setVector(A * m_fRef);
}

void Contact6d::updateForceGeneratorMatrix() {
  assert(m_contactPoints.rows() == 3);
  assert(m_contactPoints.cols() == 4);
  for (int i = 0; i < 4; i++) {
    m_forceGenMat.block<3, 3>(0, i * 3).setIdentity();
    m_forceGenMat.block<3, 3>(3, i * 3) =
        pinocchio::skew(m_contactPoints.col(i));
  }
}

unsigned int Contact6d::n_motion() const { return 6; }
unsigned int Contact6d::n_force() const { return 12; }

const Vector& Contact6d::Kp() const { return m_motionTask.Kp(); }
const Vector& Contact6d::Kd() const { return m_motionTask.Kd(); }
void Contact6d::Kp(ConstRefVector Kp) { m_motionTask.Kp(Kp); }
void Contact6d::Kd(ConstRefVector Kd) { m_motionTask.Kd(Kd); }

bool Contact6d::setContactPoints(ConstRefMatrix contactPoints) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(contactPoints.rows() == 3,
                                 "The number of rows needs to be 3");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(contactPoints.cols() == 4,
                                 "The number of cols needs to be 4");
  if (contactPoints.rows() != 3 || contactPoints.cols() != 4) return false;
  m_contactPoints = contactPoints;
  updateForceGeneratorMatrix();
  return true;
}

const Matrix3x& Contact6d::getContactPoints() const { return m_contactPoints; }

bool Contact6d::setContactNormal(ConstRefVector contactNormal) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      contactNormal.size() == 3,
      "The size of the contactNormal vector needs to equal 3");
  if (contactNormal.size() != 3) return false;
  m_contactNormal = contactNormal;
  updateForceInequalityConstraints();
  return true;
}

bool Contact6d::setFrictionCoefficient(const double frictionCoefficient) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      frictionCoefficient > 0.0,
      "The friction coefficient needs to be positive");
  if (frictionCoefficient <= 0.0) return false;
  m_mu = frictionCoefficient;
  updateForceInequalityConstraints();
  return true;
}

bool Contact6d::setMinNormalForce(const double minNormalForce) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      minNormalForce > 0.0 && minNormalForce <= m_fMax,
      "The minimal normal force needs to be greater than 0 and less than or "
      "equal to the maximal force");
  if (minNormalForce <= 0.0 || minNormalForce > m_fMax) return false;
  m_fMin = minNormalForce;
  Vector& lb = m_forceInequality.lowerBound();
  lb(lb.size() - 1) = m_fMin;
  return true;
}

bool Contact6d::setMaxNormalForce(const double maxNormalForce) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(maxNormalForce >= m_fMin,
                                 "The maximal force needs to be greater than "
                                 "or equal to the minimal force");
  if (maxNormalForce < m_fMin) return false;
  m_fMax = maxNormalForce;
  Vector& ub = m_forceInequality.upperBound();
  ub(ub.size() - 1) = m_fMax;
  return true;
}

void Contact6d::setForceReference(ConstRefVector& f_ref) {
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

void Contact6d::setReference(const SE3& ref) { m_motionTask.setReference(ref); }

const ConstraintBase& Contact6d::computeMotionTask(const double t,
                                                   ConstRefVector q,
                                                   ConstRefVector v,
                                                   Data& data) {
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality& Contact6d::computeForceTask(const double,
                                                        ConstRefVector,
                                                        ConstRefVector,
                                                        const Data&) {
  return m_forceInequality;
}

const Matrix& Contact6d::getForceGeneratorMatrix() { return m_forceGenMat; }

const ConstraintEquality& Contact6d::computeForceRegularizationTask(
    const double, ConstRefVector, ConstRefVector, const Data&) {
  return m_forceRegTask;
}

double Contact6d::getMinNormalForce() const { return m_fMin; }
double Contact6d::getMaxNormalForce() const { return m_fMax; }

const TaskSE3Equality& Contact6d::getMotionTask() const { return m_motionTask; }

const ConstraintBase& Contact6d::getMotionConstraint() const {
  return m_motionTask.getConstraint();
}

const ConstraintInequality& Contact6d::getForceConstraint() const {
  return m_forceInequality;
}

const ConstraintEquality& Contact6d::getForceRegularizationTask() const {
  return m_forceRegTask;
}
