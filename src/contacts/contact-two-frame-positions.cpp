//
// Copyright (c) 2023 MIPT
//

#include "tsid/math/utils.hpp"
#include "tsid/contacts/contact-two-frame-positions.hpp"

#include <pinocchio/spatial/skew.hpp>

using namespace tsid;
using namespace contacts;
using namespace math;
using namespace trajectories;
using namespace tasks;

ContactTwoFramePositions::ContactTwoFramePositions(
    const std::string& name, RobotWrapper& robot, const std::string& frameName1,
    const std::string& frameName2, const double minNormalForce,
    const double maxNormalForce)
    : ContactBase(name, robot),
      m_motionTask(
          name, robot, frameName1,
          frameName2),  // Actual motion task with type TaskTwoFramesEquality
      m_forceInequality(name, 3, 3),
      m_forceRegTask(name, 3, 3),
      m_fMin(minNormalForce),
      m_fMax(maxNormalForce) {
  m_weightForceRegTask << 1, 1, 1;
  m_forceGenMat.resize(3, 3);
  m_fRef = Vector3::Zero();
  m_contactPoints.resize(3, 1);
  m_contactPoints.setZero();
  updateForceGeneratorMatrix();
  updateForceInequalityConstraints();
  updateForceRegularizationTask();

  // This contact has forceGenMat as 3x3 identity matrix, so it can be used only
  // for emulating a ball joint between two frames The forces calculated will
  // have only linear part (rotation will be unconstrained) So we need to set
  // the appropriate mask for motion task (which can take into account rotation
  // but we don't need it)
  math::Vector motion_mask(6);
  motion_mask << 1., 1., 1., 0., 0., 0.;
  m_motionTask.setMask(motion_mask);
}

void ContactTwoFramePositions::updateForceInequalityConstraints() {
  Matrix B = Matrix::Identity(3, 3);  // Force "gluing" two frames together can
                                      // be arbitrary in sign/direction
  Vector lb = m_fMin * Vector::Ones(3);
  Vector ub = m_fMax * Vector::Ones(3);

  m_forceInequality.setMatrix(B);
  m_forceInequality.setLowerBound(lb);
  m_forceInequality.setUpperBound(ub);
}

double ContactTwoFramePositions::getNormalForce(ConstRefVector f) const {
  return 0.0;
}

const Matrix3x& ContactTwoFramePositions::getContactPoints() const {
  return m_contactPoints;
}

void ContactTwoFramePositions::setRegularizationTaskWeightVector(
    ConstRefVector& w) {
  m_weightForceRegTask = w;
  updateForceRegularizationTask();
}

void ContactTwoFramePositions::updateForceRegularizationTask() {
  typedef Eigen::Matrix<double, 3, 3> Matrix3;
  Matrix3 A = Matrix3::Zero();
  A.diagonal() = m_weightForceRegTask;
  m_forceRegTask.setMatrix(A);
  m_forceRegTask.setVector(A * m_fRef);
}

void ContactTwoFramePositions::updateForceGeneratorMatrix() {
  m_forceGenMat.setIdentity();
}

unsigned int ContactTwoFramePositions::n_motion() const {
  return m_motionTask.dim();
}
unsigned int ContactTwoFramePositions::n_force() const { return 3; }

const Vector& ContactTwoFramePositions::Kp() {
  m_Kp3 = m_motionTask.Kp().head<3>();
  return m_Kp3;
}

const Vector& ContactTwoFramePositions::Kd() {
  m_Kd3 = m_motionTask.Kd().head<3>();
  return m_Kd3;
}

void ContactTwoFramePositions::Kp(ConstRefVector Kp) {
  assert(Kp.size() == 3);
  Vector6 Kp6;
  Kp6.head<3>() = Kp;
  m_motionTask.Kp(Kp6);
}

void ContactTwoFramePositions::Kd(ConstRefVector Kd) {
  assert(Kd.size() == 3);
  Vector6 Kd6;
  Kd6.head<3>() = Kd;
  m_motionTask.Kd(Kd6);
}

bool ContactTwoFramePositions::setContactNormal(ConstRefVector contactNormal) {
  return true;
}

bool ContactTwoFramePositions::setFrictionCoefficient(
    const double frictionCoefficient) {
  return true;
}

bool ContactTwoFramePositions::setMinNormalForce(const double minNormalForce) {
  m_fMin = minNormalForce;
  updateForceInequalityConstraints();
  return true;
}

bool ContactTwoFramePositions::setMaxNormalForce(const double maxNormalForce) {
  m_fMax = maxNormalForce;
  updateForceInequalityConstraints();
  return true;
}

void ContactTwoFramePositions::setForceReference(ConstRefVector& f_ref) {
  m_fRef = f_ref;
  updateForceRegularizationTask();
}

const ConstraintBase& ContactTwoFramePositions::computeMotionTask(
    const double t, ConstRefVector q, ConstRefVector v, Data& data) {
  return m_motionTask.compute(t, q, v, data);
}

const ConstraintInequality& ContactTwoFramePositions::computeForceTask(
    const double, ConstRefVector, ConstRefVector, const Data&) {
  return m_forceInequality;
}

const Matrix& ContactTwoFramePositions::getForceGeneratorMatrix() {
  return m_forceGenMat;
}

const ConstraintEquality&
ContactTwoFramePositions::computeForceRegularizationTask(const double,
                                                         ConstRefVector,
                                                         ConstRefVector,
                                                         const Data&) {
  return m_forceRegTask;
}

double ContactTwoFramePositions::getMinNormalForce() const { return m_fMin; }
double ContactTwoFramePositions::getMaxNormalForce() const { return m_fMax; }

const TaskTwoFramesEquality& ContactTwoFramePositions::getMotionTask() const {
  return m_motionTask;
}

const ConstraintBase& ContactTwoFramePositions::getMotionConstraint() const {
  return m_motionTask.getConstraint();
}

const ConstraintInequality& ContactTwoFramePositions::getForceConstraint()
    const {
  return m_forceInequality;
}

const ConstraintEquality& ContactTwoFramePositions::getForceRegularizationTask()
    const {
  return m_forceRegTask;
}
