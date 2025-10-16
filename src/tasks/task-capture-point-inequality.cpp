//
// Copyright (c) 2020 CNRS, NYU, MPI Tübingen, PAL Robotics
//

#include <tsid/tasks/task-capture-point-inequality.hpp>
#include "tsid/math/utils.hpp"
#include "tsid/robots/robot-wrapper.hpp"

/** This class has been implemented following :
 * Ramos, O. E., Mansard, N., & Soueres, P.
 * (2014). Whole-body Motion Integrating the Capture Point in the Operational
 * Space Inverse Dynamics Control. In IEEE-RAS International Conference on
 * Humanoid Robots (Humanoids).
 */
namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskCapturePointInequality::TaskCapturePointInequality(const std::string& name,
                                                       RobotWrapper& robot,
                                                       const double timeStep)
    : TaskMotion(name, robot),
      m_constraint(name, 2, robot.nv()),
      m_nv(robot.nv()),
      m_delta_t(timeStep) {
  m_dim = 2;
  m_p_com.setZero(3);
  m_v_com.setZero(3);

  m_safety_margin.setZero(m_dim);

  m_support_limits_x.setZero(m_dim);
  m_support_limits_y.setZero(m_dim);

  m_rp_max.setZero(m_dim);
  m_rp_min.setZero(m_dim);

  b_lower.setZero(m_dim);
  b_upper.setZero(m_dim);

  m_g = robot.model().gravity.linear().norm();
  m_w = 0;
  m_ka = 0;
}

int TaskCapturePointInequality::dim() const { return m_dim; }

Vector TaskCapturePointInequality::getAcceleration(ConstRefVector dv) const {
  return m_constraint.matrix() * dv - m_drift;
}

const Vector& TaskCapturePointInequality::position() const { return m_p_com; }
const ConstraintBase& TaskCapturePointInequality::getConstraint() const {
  return m_constraint;
}

void TaskCapturePointInequality::setSupportLimitsXAxis(const double x_min,
                                                       const double x_max) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(x_min >= x_max,
                                 "The minimum limit for x needs to be greater "
                                 "or equal to the maximum limit");
  m_support_limits_x(0) = x_min;
  m_support_limits_x(1) = x_max;
}

void TaskCapturePointInequality::setSupportLimitsYAxis(const double y_min,
                                                       const double y_max) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(y_min >= y_max,
                                 "The minimum limit for y needs to be greater "
                                 "or equal to the maximum limit");
  m_support_limits_y(0) = y_min;
  m_support_limits_y(1) = y_max;
}

void TaskCapturePointInequality::setSafetyMargin(const double x_margin,
                                                 const double y_margin) {
  m_safety_margin(0) = x_margin;
  m_safety_margin(1) = y_margin;
}

const ConstraintBase& TaskCapturePointInequality::compute(const double,
                                                          ConstRefVector,
                                                          ConstRefVector,
                                                          Data& data) {
  m_robot.com(data, m_p_com, m_v_com, m_drift);

  const Matrix3x& Jcom = m_robot.Jcom(data);

  m_w = sqrt(m_g / m_p_com(2));
  m_ka = (2 * m_w) / ((m_w * m_delta_t + 2) * m_delta_t);

  m_rp_min(0) =
      m_support_limits_x(0) + m_safety_margin(0);  // x min support polygon
  m_rp_min(1) =
      m_support_limits_y(0) + m_safety_margin(1);  // y min support polygon

  m_rp_max(0) =
      m_support_limits_x(1) - m_safety_margin(0);  // x max support polygon
  m_rp_max(1) =
      m_support_limits_y(1) - m_safety_margin(1);  // y max support polygon

  for (int i = 0; i < m_dim; i++) {
    b_lower(i) =
        m_ka * (m_rp_min(i) - m_p_com(i) - m_v_com(i) * (m_delta_t + 1 / m_w));
    b_upper(i) =
        m_ka * (m_rp_max(i) - m_p_com(i) - m_v_com(i) * (m_delta_t + 1 / m_w));
  }

  m_constraint.lowerBound() = b_lower - m_drift.head(m_dim);
  m_constraint.upperBound() = b_upper - m_drift.head(m_dim);

  m_constraint.setMatrix(Jcom.block(0, 0, m_dim, m_nv));

  return m_constraint;
}

}  // namespace tasks
}  // namespace tsid
