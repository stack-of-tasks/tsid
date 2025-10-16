//
// Copyright (c) 2017 CNRS
//

#include <tsid/tasks/task-joint-posVelAcc-bounds.hpp>
#include "tsid/robots/robot-wrapper.hpp"
// #include <tsid/utils/stop-watch.hpp>

/** This class has been implemented following :
 * Andrea del Prete. Joint Position and Velocity Bounds in Discrete-Time
 * Acceleration/Torque Control of Robot Manipulators. IEEE Robotics and
 * Automation Letters, IEEE 2018, 3 (1),
 * pp.281-288.￿10.1109/LRA.2017.2738321￿. hal-01356989v3 And
 * https://github.com/andreadelprete/pinocchio_inv_dyn/blob/master/python/pinocchio_inv_dyn/acc_bounds_util.py
 */
namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

TaskJointPosVelAccBounds::TaskJointPosVelAccBounds(const std::string& name,
                                                   RobotWrapper& robot,
                                                   double dt, bool verbose)
    : TaskMotion(name, robot),
      m_constraint(name, robot.na(), robot.nv()),
      m_dt(2 * dt),
      m_verbose(verbose),
      m_nv(robot.nv()),
      m_na(robot.na()) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(dt > 0.0, "dt needs to be positive");
  m_eps = 1e-10;
  m_qMin = Vector::Constant(m_na, 1, -1e10);
  m_qMax = Vector::Constant(m_na, 1, 1e10);
  m_dqMax = Vector::Constant(m_na, 1, 1e10);
  m_ddqMax = Vector::Constant(m_na, 1, 1e10);
  m_impose_position_bounds = false;
  m_impose_velocity_bounds = false;
  m_impose_viability_bounds = false;
  m_impose_acceleration_bounds = false;

  // Used in computeAccLimitsFromPosLimits
  m_two_dt_sq = 2.0 / (m_dt * m_dt);
  m_ddqMax_q3 = Vector::Zero(m_na);
  m_ddqMin_q3 = Vector::Zero(m_na);
  m_ddqMax_q2 = Vector::Zero(m_na);
  m_ddqMin_q2 = Vector::Zero(m_na);
  m_minus_dq_over_dt = Vector::Zero(m_na);

  // Used in computeAccLimitsFromViability
  m_dt_square = m_dt * m_dt;
  m_two_a = 2 * m_dt_square;
  m_dt_dq = Vector::Zero(m_na);
  m_dt_two_dq = Vector::Zero(m_na);
  m_two_ddqMax = Vector::Zero(m_na);
  m_dt_ddqMax_dt = Vector::Zero(m_na);
  m_dq_square = Vector::Zero(m_na);
  m_q_plus_dt_dq = Vector::Zero(m_na);
  m_b_1 = Vector::Zero(m_na);
  m_b_2 = Vector::Zero(m_na);
  m_ddq_1 = Vector::Zero(m_na);
  m_ddq_2 = Vector::Zero(m_na);
  m_c_1 = Vector::Zero(m_na);
  m_delta_1 = Vector::Zero(m_na);
  m_c_2 = Vector::Zero(m_na);
  m_delta_2 = Vector::Zero(m_na);

  // Used in computeAccLimits
  m_ub = Vector::Constant(4, 1, 1e10);
  m_lb = Vector::Constant(4, 1, -1e10);

  m_ddqLBPos = Vector::Constant(m_na, 1, -1e10);
  m_ddqUBPos = Vector::Constant(m_na, 1, 1e10);
  m_ddqLBVia = Vector::Constant(m_na, 1, -1e10);
  m_ddqUBVia = Vector::Constant(m_na, 1, 1e10);
  m_ddqLBVel = Vector::Constant(m_na, 1, -1e10);
  m_ddqUBVel = Vector::Constant(m_na, 1, 1e10);
  m_ddqLBAcc = Vector::Constant(m_na, 1, -1e10);
  m_ddqUBAcc = Vector::Constant(m_na, 1, 1e10);
  m_ddqLB = Vector::Constant(m_na, 1, -1e10);
  m_ddqUB = Vector::Constant(m_na, 1, 1e10);
  m_viabViol = Vector::Zero(m_na);

  m_qa = Vector::Zero(m_na);
  m_dqa = Vector::Zero(m_na);

  Vector m = Vector::Ones(robot.na());
  setMask(m);

  for (int i = 0; i < m_na; i++) {
    m_constraint.upperBound()(i) = 1e10;
    m_constraint.lowerBound()(i) = -1e10;
  }
}

const Vector& TaskJointPosVelAccBounds::mask() const { return m_mask; }

void TaskJointPosVelAccBounds::mask(const Vector& m) {
  // std::cerr<<"The method TaskJointPosVelAccBounds::mask is deprecated. Use
  // TaskJointPosVelAccBounds::setMask instead.\n";
  return setMask(m);
}

void TaskJointPosVelAccBounds::setMask(ConstRefVector m) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(m.size() == m_robot.na(),
                                 "The size of the mask vector needs to equal " +
                                     std::to_string(m_robot.na()));
  m_mask = m;
  const Vector::Index dim = static_cast<Vector::Index>(m.sum());
  Matrix S = Matrix::Zero(dim, m_robot.nv());
  m_activeAxes.resize(dim);
  unsigned int j = 0;
  for (unsigned int i = 0; i < m.size(); i++)
    if (m(i) != 0.0) {
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          m(i) == 1.0, "Mask entries need to be either 0.0 or 1.0");
      S(j, m_robot.nv() - m_robot.na() + i) = 1.0;
      m_activeAxes(j) = i;
      j++;
    }
  m_constraint.resize((unsigned int)dim, m_robot.nv());
  m_constraint.setMatrix(S);
}

int TaskJointPosVelAccBounds::dim() const { return m_na; }

const Vector& TaskJointPosVelAccBounds::getAccelerationBounds() const {
  return m_ddqMax;
}

const Vector& TaskJointPosVelAccBounds::getVelocityBounds() const {
  return m_dqMax;
}

const Vector& TaskJointPosVelAccBounds::getPositionLowerBounds() const {
  return m_qMin;
}

const Vector& TaskJointPosVelAccBounds::getPositionUpperBounds() const {
  return m_qMax;
}

void TaskJointPosVelAccBounds::setTimeStep(double dt) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(dt > 0.0, "dt needs to be positive");
  m_dt = dt;
}

void TaskJointPosVelAccBounds::setVerbose(bool verbose) { m_verbose = verbose; }

void TaskJointPosVelAccBounds::setPositionBounds(ConstRefVector lower,
                                                 ConstRefVector upper) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      lower.size() == m_na,
      "The size of the lower position bounds vector needs to equal " +
          std::to_string(m_na));
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      upper.size() == m_na,
      "The size of the upper position bounds vector needs to equal " +
          std::to_string(m_na));
  m_qMin = lower;
  m_qMax = upper;
  m_impose_position_bounds = true;
  m_impose_viability_bounds = true;
}

void TaskJointPosVelAccBounds::setVelocityBounds(ConstRefVector upper) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      upper.size() == m_na,
      "The size of the (absolute) velocity bounds vector needs to equal " +
          std::to_string(m_na));
  m_dqMax = upper;
  m_impose_velocity_bounds = true;
}

void TaskJointPosVelAccBounds::setAccelerationBounds(ConstRefVector upper) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(
      upper.size() == m_na,
      "The size of the (absolute) acceleration bounds vector needs to equal " +
          std::to_string(m_na));
  m_ddqMax = upper;
  m_impose_acceleration_bounds = true;
}

const ConstraintBase& TaskJointPosVelAccBounds::getConstraint() const {
  return m_constraint;
}

const ConstraintBase& TaskJointPosVelAccBounds::compute(const double,
                                                        ConstRefVector q,
                                                        ConstRefVector v,
                                                        Data&) {
  // getProfiler().start("TaskJointPosVelAccBounds");
  // Eigen::internal::set_is_malloc_allowed(false);
  computeAccLimits(q, v, m_verbose);
  m_constraint.upperBound() = m_ddqUB;
  m_constraint.lowerBound() = m_ddqLB;
  // Eigen::internal::set_is_malloc_allowed(true);
  // getProfiler().stop("TaskJointPosVelAccBounds");
  // getProfiler().report_all(9, std::cout);
  return m_constraint;
}

void TaskJointPosVelAccBounds::setImposeBounds(
    bool impose_position_bounds, bool impose_velocity_bounds,
    bool impose_viability_bounds, bool impose_acceleration_bounds) {
  m_impose_position_bounds = impose_position_bounds;
  m_impose_velocity_bounds = impose_velocity_bounds;
  m_impose_viability_bounds = impose_viability_bounds;
  m_impose_acceleration_bounds = impose_acceleration_bounds;
}

void TaskJointPosVelAccBounds::isStateViable(ConstRefVector qa,
                                             ConstRefVector dqa, bool verbose) {
  m_viabViol.setZero(m_na);
  for (int i = 0; i < m_na; i++) {
    if (qa[i] < (m_qMin[i] - m_eps)) {
      if (verbose) {
        std::cout << "State of joint " << i
                  << " is not viable because q[i]< qMin[i] : " << qa[i] << "<"
                  << m_qMin[i] << std::endl;
      }
      m_viabViol[i] = m_qMin[i] - qa[i];
    }
    if (qa[i] > (m_qMax[i] + m_eps)) {
      if (verbose) {
        std::cout << "State of joint " << i
                  << " is not viable because qa[i]>m_qMax[i] : " << qa[i] << ">"
                  << m_qMax[i] << std::endl;
      }
      m_viabViol[i] = qa[i] - m_qMax[i];
    }
    if (std::abs(dqa[i]) > (m_dqMax[i] + m_eps)) {
      if (verbose) {
        std::cout << "State (q,dq) :(" << qa[i] << "," << dqa[i]
                  << ") of joint " << i
                  << " is not viable because |dq|>dqMax : " << std::abs(dqa[i])
                  << ">" << m_dqMax[i] << std::endl;
      }
      m_viabViol[i] = std::abs(dqa[i]) - m_dqMax[i];
    }
    double dqMaxViab =
        std::sqrt(std::max(0.0, 2 * m_ddqMax[i] * (m_qMax[i] - qa[i])));
    if (dqa[i] > (dqMaxViab + m_eps)) {
      if (verbose) {
        std::cout << "State (q,dq,dqMaxViab) :(" << qa[i] << "," << dqa[i]
                  << "," << dqMaxViab << ") of joint " << i
                  << " is not viable because dq>dqMaxViab : " << dqa[i] << ">"
                  << dqMaxViab << std::endl;
      }
      m_viabViol[i] = dqa[i] - dqMaxViab;
    }
    double dqMinViab =
        -std::sqrt(std::max(0.0, 2 * m_ddqMax[i] * (qa[i] - m_qMin[i])));
    if (dqa[i] < (dqMinViab + m_eps)) {
      if (verbose) {
        std::cout << "State (q,dq,dqMinViab) :(" << qa[i] << "," << dqa[i]
                  << "," << dqMinViab << ") of joint " << i
                  << " is not viable because dq<dqMinViab : " << dqa[i] << "<"
                  << dqMinViab << std::endl;
      }
      m_viabViol[i] = dqMinViab - dqa[i];
    }
  }
}

void TaskJointPosVelAccBounds::computeAccLimitsFromPosLimits(ConstRefVector qa,
                                                             ConstRefVector dqa,
                                                             bool verbose) {
  m_ddqMax_q3 = m_two_dt_sq * (m_qMax - qa - m_dt * dqa);
  m_ddqMin_q3 = m_two_dt_sq * (m_qMin - qa - m_dt * dqa);
  m_ddqMax_q2.setZero(m_na);
  m_ddqMin_q2.setZero(m_na);
  m_ddqLBPos.setConstant(m_na, 1, -1e10);
  m_ddqUBPos.setConstant(m_na, 1, 1e10);
  m_minus_dq_over_dt = -dqa / m_dt;
  for (int i = 0; i < m_na; i++) {
    if (dqa[i] <= 0.0) {
      m_ddqUBPos[i] = m_ddqMax_q3[i];
      if (m_ddqMin_q3[i] < m_minus_dq_over_dt[i]) {
        m_ddqLBPos[i] = m_ddqMin_q3[i];
      } else if (qa[i] != m_qMin[i]) {
        m_ddqMin_q2[i] = (dqa[i] * dqa[i]) / (2.0 * (qa[i] - m_qMin[i]));
        m_ddqLBPos[i] = std::max(m_ddqMin_q2[i], m_minus_dq_over_dt[i]);
      } else {
        if (verbose == true) {
          std::cout << "WARNING  qa[i]==m_qMin[i] for joint" << i << std::endl;
          std::cout << "You are going to violate the position bound " << i
                    << std::endl;
        }
        m_ddqLBPos[i] = 0.0;
      }
    } else {
      m_ddqLBPos[i] = m_ddqMin_q3[i];
      if (m_ddqMax_q3[i] > m_minus_dq_over_dt[i]) {
        m_ddqUBPos[i] = m_ddqMax_q3[i];
      } else if (qa[i] != m_qMax[i]) {
        m_ddqMax_q2[i] = -(dqa[i] * dqa[i]) / (2 * (m_qMax[i] - qa[i]));
        m_ddqUBPos[i] = std::min(m_ddqMax_q2[i], m_minus_dq_over_dt[i]);
      } else {
        if (verbose == true) {
          std::cout << "WARNING  qa[i]==m_qMax[i] for joint" << i << std::endl;
          std::cout << "You are going to violate the position bound " << i
                    << std::endl;
        }
        m_ddqUBPos[i] = 0.0;
      }
    }
  }
}
void TaskJointPosVelAccBounds::computeAccLimitsFromViability(ConstRefVector qa,
                                                             ConstRefVector dqa,
                                                             bool verbose) {
  m_ddqLBVia.setConstant(m_na, 1, -1e10);
  m_ddqUBVia.setConstant(m_na, 1, 1e10);
  m_dt_dq = m_dt * dqa;
  m_minus_dq_over_dt = -dqa / m_dt;
  m_dt_two_dq = 2 * m_dt_dq;
  m_two_ddqMax = 2 * m_ddqMax;
  m_dt_ddqMax_dt = m_ddqMax * m_dt_square;
  m_dq_square = dqa.cwiseProduct(dqa);
  m_q_plus_dt_dq = qa + m_dt_dq;
  m_b_1 = m_dt_two_dq + m_dt_ddqMax_dt;
  m_b_2 = m_dt_two_dq - m_dt_ddqMax_dt;
  m_ddq_1.setZero(m_na);
  m_ddq_2.setZero(m_na);
  m_c_1 = m_dq_square - m_two_ddqMax.cwiseProduct(m_qMax - m_q_plus_dt_dq);
  m_delta_1 = m_b_1.cwiseProduct(m_b_1) - 2 * m_two_a * m_c_1;
  m_c_2 = m_dq_square - m_two_ddqMax.cwiseProduct(m_q_plus_dt_dq - m_qMin);
  m_delta_2 = m_b_2.cwiseProduct(m_b_2) - 2 * m_two_a * m_c_2;
  for (int i = 0; i < m_na; i++) {
    if (m_delta_1[i] >= 0.0) {
      m_ddq_1[i] = (-m_b_1[i] + std::sqrt(m_delta_1[i])) / (m_two_a);
    } else {
      m_ddq_1[i] = m_minus_dq_over_dt[i];
      if (verbose == true) {
        std::cout << "Error: state (" << qa[i] << "," << dqa[i] << ") of joint "
                  << i << "not viable because delta is negative: " << m_delta_1
                  << std::endl;
      }
    }
    if (m_delta_2[i] >= 0.0) {
      m_ddq_2[i] = (-m_b_2[i] - std::sqrt(m_delta_2[i])) / (m_two_a);
    } else {
      m_ddq_2[i] = m_minus_dq_over_dt[i];
      if (verbose == true) {
        std::cout << "Error: state (" << qa[i] << "," << dqa[i] << ") of joint "
                  << i << "not viable because delta is negative: " << m_delta_2
                  << std::endl;
      }
    }
  }
  m_ddqUBVia = m_ddq_1.cwiseMax(m_minus_dq_over_dt);
  m_ddqLBVia = m_ddq_2.cwiseMin(m_minus_dq_over_dt);
}

void TaskJointPosVelAccBounds::computeAccLimits(ConstRefVector q,
                                                ConstRefVector dq,
                                                bool verbose) {
  m_qa = q.tail(m_na);
  m_dqa = dq.tail(m_na);
  isStateViable(m_qa, m_dqa, m_verbose);
  if (verbose == true) {
    for (int i = 0; i < m_na; i++) {
      if (m_viabViol[i] > m_eps) {
        std::cout << "WARNING: specified state ( < " << m_qa[i] << " , "
                  << m_dqa[i] << ") is not viable violation : " << m_viabViol[i]
                  << std::endl;
      }
    }
  }

  // Acceleration limits imposed by position bounds
  if (m_impose_position_bounds == true) {
    computeAccLimitsFromPosLimits(m_qa, m_dqa, verbose);
  }
  // Acceleration limits imposed by velocity bounds
  // dq[t+1] = dq + dt*ddq < dqMax
  // ddqMax = (dqMax-dq)/dt
  // ddqMin = (dqMin-dq)/dt = (-dqMax-dq)/dt
  m_ddqLBVel.setConstant(m_na, 1, -1e10);
  m_ddqUBVel.setConstant(m_na, 1, 1e10);
  if (m_impose_velocity_bounds == true) {
    m_ddqLBVel = (-m_dqMax - m_dqa) / m_dt;
    m_ddqUBVel = (m_dqMax - m_dqa) / m_dt;
  }
  // Acceleration limits imposed by viability
  if (m_impose_viability_bounds == true) {
    computeAccLimitsFromViability(m_qa, m_dqa, verbose);
  }
  // Acceleration limits
  m_ddqLBAcc.setConstant(m_na, 1, -1e10);
  m_ddqUBAcc.setConstant(m_na, 1, 1e10);
  if (m_impose_acceleration_bounds == true) {
    m_ddqLBAcc = -m_ddqMax;
    m_ddqUBAcc = m_ddqMax;
  }
  // Take the most conservative limit for each joint
  m_ub.setConstant(4, 1, 1e10);
  m_lb.setConstant(4, 1, -1e10);
  m_ddqLB.setConstant(m_na, 1, -1e10);
  m_ddqUB.setConstant(m_na, 1, 1e10);
  for (int i = 0; i < m_na; i++) {
    m_ub[0] = m_ddqUBPos[i];
    m_ub[1] = m_ddqUBVia[i];
    m_ub[2] = m_ddqUBVel[i];
    m_ub[3] = m_ddqUBAcc[i];

    m_lb[0] = m_ddqLBPos[i];
    m_lb[1] = m_ddqLBVia[i];
    m_lb[2] = m_ddqLBVel[i];
    m_lb[3] = m_ddqLBAcc[i];

    m_ddqLB[i] = m_lb.maxCoeff();
    m_ddqUB[i] = m_ub.minCoeff();

    if (m_ddqUB[i] < m_ddqLB[i]) {
      if (verbose == true) {
        std::cout << "Conflict between pos/vel/acc bound ddqMin " << m_ddqLB[i]
                  << " ddqMax " << m_ddqUB[i] << std::endl;
        std::cout << "ub " << m_ub.transpose() << std::endl;
        std::cout << "lb " << m_lb.transpose() << std::endl;
      }
      if (m_ddqUB[i] == m_ub[0]) {
        m_ddqLB[i] = m_ddqUB[i];
      } else {
        m_ddqUB[i] = m_ddqLB[i];
      }
      if (verbose == true) {
        std::cout << "New bounds are  ddqMin " << m_ddqLB[i] << " ddqMax "
                  << m_ddqUB[i] << std::endl;
      }
    }
  }
}
}  // namespace tasks
}  // namespace tsid
