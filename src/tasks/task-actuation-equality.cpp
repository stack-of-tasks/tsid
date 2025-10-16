//
// Copyright (c) 2021 CNRS INRIA LORIA
//

#include <tsid/tasks/task-actuation-equality.hpp>
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace tasks {
using namespace math;
using namespace pinocchio;

TaskActuationEquality::TaskActuationEquality(const std::string& name,
                                             RobotWrapper& robot)
    : TaskActuation(name, robot), m_constraint(name, robot.na(), robot.na()) {
  m_ref = Vector::Zero(robot.na());
  m_weights = Vector::Ones(robot.na());
  Vector m = Vector::Ones(robot.na());
  mask(m);
}

const Vector& TaskActuationEquality::mask() const { return m_mask; }

void TaskActuationEquality::mask(const Vector& m) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(m.size() == m_robot.na(),
                                 "The size of the mask vector needs to equal " +
                                     std::to_string(m_robot.na()));
  m_mask = m;

  const Vector::Index dim = static_cast<Vector::Index>(m.sum());
  Matrix S = Matrix::Zero(dim, m_robot.na());
  m_activeAxes.resize(dim);
  unsigned int j = 0;
  for (unsigned int i = 0; i < m.size(); i++)
    if (m(i) != 0.0) {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          m(i) == 1.0,
          "Entries in the mask vector need to be either 0.0 or 1.0");
      S(j, i) = m_weights(i);
      m_activeAxes(j) = i;
      j++;
    }
  m_constraint.resize((unsigned int)dim, m_robot.na());
  m_constraint.setMatrix(S);

  for (unsigned int i = 0; i < m_activeAxes.size(); i++)
    m_constraint.vector()(i) =
        m_ref(m_activeAxes(i)) * m_weights(m_activeAxes(i));
}

int TaskActuationEquality::dim() const { return (int)m_mask.sum(); }

// Reference should be the same size as robot.na(), even if a mask is used
// (masked dof values will just be ignored)
void TaskActuationEquality::setReference(ConstRefVector ref) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      ref.size() == m_robot.na(),
      "The size of the reference vector needs to equal " +
          std::to_string(m_robot.na()));
  m_ref = ref;

  for (unsigned int i = 0; i < m_activeAxes.size(); i++)
    m_constraint.vector()(i) =
        m_ref(m_activeAxes(i)) * m_weights(m_activeAxes(i));
}

const Vector& TaskActuationEquality::getReference() const { return m_ref; }

// Weighting vector should be the same size as robot.na(), even if a mask is
// used (masked dof values will just be ignored)
void TaskActuationEquality::setWeightVector(ConstRefVector weights) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      weights.size() == m_robot.na(),
      "The size of the weight vector needs to equal " +
          std::to_string(m_robot.na()));
  m_weights = weights;

  for (unsigned int i = 0; i < m_activeAxes.size(); i++) {
    m_constraint.matrix()(i, m_activeAxes(i)) = m_weights(m_activeAxes(i));
    m_constraint.vector()(i) =
        m_ref(m_activeAxes(i)) * m_weights(m_activeAxes(i));
  }
}

const Vector& TaskActuationEquality::getWeightVector() const {
  return m_weights;
}

const ConstraintBase& TaskActuationEquality::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskActuationEquality::compute(const double,
                                                     ConstRefVector,
                                                     ConstRefVector, Data&) {
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
