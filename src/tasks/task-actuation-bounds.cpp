//
// Copyright (c) 2017 CNRS
//

#include <tsid/tasks/task-actuation-bounds.hpp>
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskActuationBounds::TaskActuationBounds(const std::string& name,
                                         RobotWrapper& robot)
    : TaskActuation(name, robot), m_constraint(name, robot.na(), robot.na()) {
  Vector m = Vector::Ones(robot.na());
  mask(m);
}

const Vector& TaskActuationBounds::mask() const { return m_mask; }

void TaskActuationBounds::mask(const Vector& m) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      m.size() == m_robot.na(),
      "The size of the mask needs to equal " + std::to_string(m_robot.na()));
  m_mask = m;
  const Vector::Index dim = static_cast<Vector::Index>(m.sum());
  Matrix S = Matrix::Zero(dim, m_robot.na());
  m_activeAxes.resize(dim);
  unsigned int j = 0;
  for (unsigned int i = 0; i < m.size(); i++)
    if (m(i) != 0.0) {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          m(i) == 1.0, "The mask entries need to equal either 0.0 or 1.0");
      S(j, i) = 1.0;
      m_activeAxes(j) = i;
      j++;
    }
  m_constraint.resize((unsigned int)dim, m_robot.na());
  m_constraint.setMatrix(S);
}

int TaskActuationBounds::dim() const { return (int)m_mask.sum(); }

const Vector& TaskActuationBounds::getLowerBounds() const {
  return m_constraint.lowerBound();
}

const Vector& TaskActuationBounds::getUpperBounds() const {
  return m_constraint.upperBound();
}

void TaskActuationBounds::setBounds(ConstRefVector lower,
                                    ConstRefVector upper) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      lower.size() == dim(),
      "The size of the lower joint bounds vector needs to equal " +
          std::to_string(dim()));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      upper.size() == dim(),
      "The size of the upper joint bounds vector needs to equal " +
          std::to_string(dim()));
  m_constraint.setLowerBound(lower);
  m_constraint.setUpperBound(upper);
}

const ConstraintBase& TaskActuationBounds::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskActuationBounds::compute(const double, ConstRefVector,
                                                   ConstRefVector, Data&) {
  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
