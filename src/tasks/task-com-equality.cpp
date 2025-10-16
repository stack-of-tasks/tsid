//
// Copyright (c) 2017-2020 CNRS, Inria
//

#include "tsid/tasks/task-com-equality.hpp"
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskComEquality::TaskComEquality(const std::string& name, RobotWrapper& robot)
    : TaskMotion(name, robot), m_constraint(name, 3, robot.nv()) {
  m_Kp.setZero(3);
  m_Kd.setZero(3);
  m_p_error_vec.setZero(3);
  m_v_error_vec.setZero(3);
  m_p_com.setZero(3);
  m_v_com.setZero(3);
  m_a_des_vec.setZero(3);
  m_ref.resize(3);
  m_mask.resize(3);
  m_mask.fill(1.);
  setMask(m_mask);
}

void TaskComEquality::setMask(math::ConstRefVector mask) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      mask.size() == 3, "The size of the mask vector needs to equal 3");
  TaskMotion::setMask(mask);
  int n = dim();
  m_constraint.resize(n, m_robot.nv());
  m_p_error_masked_vec.resize(n);
  m_v_error_masked_vec.resize(n);
  m_drift_masked.resize(n);
  m_a_des_masked.resize(n);
}

int TaskComEquality::dim() const { return int(m_mask.sum()); }

const Vector3& TaskComEquality::Kp() { return m_Kp; }

const Vector3& TaskComEquality::Kd() { return m_Kd; }

void TaskComEquality::Kp(ConstRefVector Kp) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kp.size() == 3,
                                 "The size of the Kp vector needs to equal 3");
  m_Kp = Kp;
}

void TaskComEquality::Kd(ConstRefVector Kd) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(Kd.size() == 3,
                                 "The size of the Kd vector needs to equal 3");
  m_Kd = Kd;
}

void TaskComEquality::setReference(const TrajectorySample& ref) { m_ref = ref; }

const TrajectorySample& TaskComEquality::getReference() const { return m_ref; }

const Vector& TaskComEquality::getDesiredAcceleration() const {
  return m_a_des_masked;
}

Vector TaskComEquality::getAcceleration(ConstRefVector dv) const {
  return m_constraint.matrix() * dv - m_drift_masked;
}

const Vector& TaskComEquality::position_error() const {
  return m_p_error_masked_vec;
}

const Vector& TaskComEquality::velocity_error() const {
  return m_v_error_masked_vec;
}

const Vector& TaskComEquality::position() const { return m_p_com; }

const Vector& TaskComEquality::velocity() const { return m_v_com; }

const Vector& TaskComEquality::position_ref() const { return m_ref.getValue(); }

const Vector& TaskComEquality::velocity_ref() const {
  return m_ref.getDerivative();
}

const ConstraintBase& TaskComEquality::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskComEquality::compute(const double, ConstRefVector,
                                               ConstRefVector, Data& data) {
  m_robot.com(data, m_p_com, m_v_com, m_drift);

  // Compute errors
  m_p_error = m_p_com - m_ref.getValue();
  m_v_error = m_v_com - m_ref.getDerivative();
  m_a_des = -m_Kp.cwiseProduct(m_p_error) - m_Kd.cwiseProduct(m_v_error) +
            m_ref.getSecondDerivative();

  m_p_error_vec = m_p_error;
  m_v_error_vec = m_v_error;
  m_a_des_vec = m_a_des;
#ifndef NDEBUG
//      std::cout<<m_name<<" errors: "<<m_p_error.norm()<<" "
//        <<m_v_error.norm()<<std::endl;
#endif

  // Get CoM jacobian
  const Matrix3x& Jcom = m_robot.Jcom(data);

  int idx = 0;
  for (int i = 0; i < 3; i++) {
    if (m_mask(i) != 1.) continue;

    m_constraint.matrix().row(idx) = Jcom.row(i);
    m_constraint.vector().row(idx) = (m_a_des - m_drift).row(i);

    m_a_des_masked(idx) = m_a_des(i);
    m_drift_masked(idx) = m_drift(i);
    m_p_error_masked_vec(idx) = m_p_error_vec(i);
    m_v_error_masked_vec(idx) = m_v_error_vec(i);

    idx += 1;
  }

  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
