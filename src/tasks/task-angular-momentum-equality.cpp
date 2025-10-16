//
// Copyright (c) 2017 CNRS
//

#include "tsid/tasks/task-angular-momentum-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/centroidal.hpp>

namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskAMEquality::TaskAMEquality(const std::string& name, RobotWrapper& robot)
    : TaskMotion(name, robot), m_constraint(name, 3, robot.nv()) {
  m_Kp.setZero(3);
  m_Kd.setZero(3);
  m_L_error.setZero(3);
  m_dL_error.setZero(3);
  m_L.setZero(3);
  m_dL.setZero(3);
  m_dL_des.setZero(3);
  m_ref.resize(3);
}

int TaskAMEquality::dim() const {
  // return self._mask.sum ()
  return 3;
}

const Vector3& TaskAMEquality::Kp() { return m_Kp; }

const Vector3& TaskAMEquality::Kd() { return m_Kd; }

void TaskAMEquality::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 3,
                                 "The size of the Kp vector needs to equal 3");
  m_Kp = Kp;
}

void TaskAMEquality::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 3,
                                 "The size of the Kd vector needs to equal 3");
  m_Kd = Kd;
}

void TaskAMEquality::setReference(const TrajectorySample& ref) { m_ref = ref; }

const TrajectorySample& TaskAMEquality::getReference() const { return m_ref; }

const Vector3& TaskAMEquality::getDesiredMomentumDerivative() const {
  return m_dL_des;
}

Vector3 TaskAMEquality::getdMomentum(ConstRefVector dv) const {
  return m_constraint.matrix() * dv - m_drift;
}

const Vector3& TaskAMEquality::momentum_error() const { return m_L_error; }

const Vector3& TaskAMEquality::momentum() const { return m_L; }
const Vector& TaskAMEquality::momentum_ref() const { return m_ref.getValue(); }

const Vector& TaskAMEquality::dmomentum_ref() const {
  return m_ref.getDerivative();
}

const ConstraintBase& TaskAMEquality::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskAMEquality::compute(const double, ConstRefVector,
                                              ConstRefVector v, Data& data) {
  // Compute errors
  // Get momentum jacobian
  const Matrix6x& J_am = m_robot.momentumJacobian(data);
  m_L = J_am.bottomRows(3) * v;
  m_L_error = m_L - m_ref.getValue();

  m_dL_des = -m_Kp.cwiseProduct(m_L_error) + m_ref.getDerivative();

#ifndef NDEBUG
//      std::cout<<m_name<<" errors: "<<m_L_error.norm()<<" "
//        <<m_dL_error.norm()<<std::endl;
#endif

  m_drift = m_robot.angularMomentumTimeVariation(data);
  m_constraint.setMatrix(J_am.bottomRows(3));
  m_constraint.setVector(m_dL_des - m_drift);

  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
