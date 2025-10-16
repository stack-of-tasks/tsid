//
// Copyright (c) 2017 CNRS
//

#include <tsid/tasks/task-joint-posture.hpp>
#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskJointPosture::TaskJointPosture(const std::string& name, RobotWrapper& robot)
    : TaskMotion(name, robot),
      m_ref(robot.nq_actuated(), robot.na()),
      m_constraint(name, robot.na(), robot.nv()) {
  m_ref_q_augmented = pinocchio::neutral(robot.model());
  m_Kp.setZero(robot.na());
  m_Kd.setZero(robot.na());
  Vector m = Vector::Ones(robot.na());
  setMask(m);
}

const Vector& TaskJointPosture::mask() const { return m_mask; }

void TaskJointPosture::mask(const Vector& m) {
  // std::cerr<<"The method TaskJointPosture::mask is deprecated. Use
  // TaskJointPosture::setMask instead.\n";
  return setMask(m);
}

void TaskJointPosture::setMask(ConstRefVector m) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      m.size() == m_robot.na(),
      "The size of the mask needs to equal " + std::to_string(m_robot.na()));
  m_mask = m;
  const Vector::Index dim = static_cast<Vector::Index>(m.sum());
  Matrix S = Matrix::Zero(dim, m_robot.nv());
  m_activeAxes.resize(dim);
  unsigned int j = 0;
  for (unsigned int i = 0; i < m.size(); i++)
    if (m(i) != 0.0) {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          m(i) == 1.0, "Valid mask values are either 0.0 or 1.0 received: " +
                           std::to_string(m(i)));
      S(j, m_robot.nv() - m_robot.na() + i) = 1.0;
      m_activeAxes(j) = i;
      j++;
    }
  m_constraint.resize((unsigned int)dim, m_robot.nv());
  m_constraint.setMatrix(S);
}

int TaskJointPosture::dim() const { return (int)m_mask.sum(); }

const Vector& TaskJointPosture::Kp() { return m_Kp; }

const Vector& TaskJointPosture::Kd() { return m_Kd; }

void TaskJointPosture::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == m_robot.na(),
                                 "The size of the Kp vector needs to equal " +
                                     std::to_string(m_robot.na()));
  m_Kp = Kp;
}

void TaskJointPosture::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == m_robot.na(),
                                 "The size of the Kd vector needs to equal " +
                                     std::to_string(m_robot.na()));
  m_Kd = Kd;
}

void TaskJointPosture::setReference(const TrajectorySample& ref) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ref.getValue().size() == m_robot.nq_actuated(),
      "The size of the reference value vector needs to equal " +
          std::to_string(m_robot.nq_actuated()));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ref.getDerivative().size() == m_robot.na(),
      "The size of the reference value derivative vector needs to equal " +
          std::to_string(m_robot.na()));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ref.getSecondDerivative().size() == m_robot.na(),
      "The size of the reference value second derivative vector needs to "
      "equal " +
          std::to_string(m_robot.na()));
  m_ref = ref;
}

const TrajectorySample& TaskJointPosture::getReference() const { return m_ref; }

const Vector& TaskJointPosture::getDesiredAcceleration() const {
  return m_a_des;
}

Vector TaskJointPosture::getAcceleration(ConstRefVector dv) const {
  return m_constraint.matrix() * dv;
}

const Vector& TaskJointPosture::position_error() const { return m_p_error; }

const Vector& TaskJointPosture::velocity_error() const { return m_v_error; }

const Vector& TaskJointPosture::position() const { return m_p; }

const Vector& TaskJointPosture::velocity() const { return m_v; }

const Vector& TaskJointPosture::position_ref() const {
  return m_ref.getValue();
}

const Vector& TaskJointPosture::velocity_ref() const {
  return m_ref.getDerivative();
}

const ConstraintBase& TaskJointPosture::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskJointPosture::compute(const double, ConstRefVector q,
                                                ConstRefVector v, Data&) {
  m_ref_q_augmented.tail(m_robot.nq_actuated()) = m_ref.getValue();

  // Compute errors
  m_p_error = pinocchio::difference(m_robot.model(), m_ref_q_augmented, q)
                  .tail(m_robot.na());

  m_v = v.tail(m_robot.na());
  m_v_error = m_v - m_ref.getDerivative();
  m_a_des = -m_Kp.cwiseProduct(m_p_error) - m_Kd.cwiseProduct(m_v_error) +
            m_ref.getSecondDerivative();

  for (unsigned int i = 0; i < m_activeAxes.size(); i++)
    m_constraint.vector()(i) = m_a_des(m_activeAxes(i));
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
