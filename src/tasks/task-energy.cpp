//
// Copyright (c) 2022 CNRS, Noelie Ramuzat
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-energy.hpp"
#include "tsid/robots/robot-wrapper.hpp"


namespace tsid {
namespace tasks {
using namespace math;
using namespace trajectories;
using namespace pinocchio;

double TaskEnergy::alphaFunction(const double a, const double b, const double x, const double e_val) {
  if ((x >= b) && (e_val > 0)) {
    return 1;
  } else if ((a <= x) && (x < b) && (e_val > 0)) {
    double q = (x - a) / (b - a);
    double value = 6 * pow(q, 5) - 15 * pow(q, 4) + 10 * pow(q, 3);
    return value;
  } else {
    return 0;
  }
}

double TaskEnergy::gammaFunction(const double A, const double P,
                                 const double delta) {
  if (A < P) {
    return P / A;
  } else if (A <= P + delta) {
    double x = (A - P) / delta;
    double value = 6 * pow(x, 5) - 15 * pow(x, 4) + 10 * pow(x, 3);
    double test = (P / A * exp(1 - 1 / (1 - value)) + exp(1 - 1 / value));
    if (test > 1) {
      return 1;
    }
    return test;
  } else {
    return 1;
  }
}

TaskEnergy::TaskEnergy(const std::string & name,
                       RobotWrapper & robot,
                       const double dt):
  TaskBase(name, robot),
  m_dt(dt),
  m_passivityConstraint(name, 1, robot.nv()) {
  m_dim = 1;
  m_E_max_tank = 5.0;
  m_E_min_tank = 0.1;
  m_E_tank = 5.0;     // Set the energy tank first value at its maximum
  m_dE_tank = 0.0;
  m_H = m_E_tank;
  m_dH = 0.0;
  m_H_tot = 0.0;
  m_dH_tot = 0.0;
  m_b_lower = -1e10 * Vector::Ones(m_dim);
  m_b_upper = 1e10 * Vector::Ones(m_dim);
  m_first_iter = true;
  m_alpha = 1.0;
  m_beta = 1.0;
  m_gamma = 1.0;
  m_Plow = -5.0;
  m_test_semi_def_pos = false;  // No test on the positivity of S
  m_Lambda_Kp_pos_def = true;
}

int TaskEnergy::dim() const {
  return m_dim;
}

void TaskEnergy::setTasks(const std::vector<std::shared_ptr<TaskLevelMotion> >&  taskMotions,
                          const std::vector<std::shared_ptr<ContactLevel> >&  taskContacts,
                          const std::vector<std::shared_ptr<TaskLevelForce> >& taskForces, Data & data) {
  auto newTaskMotions = taskMotions;
  // Add the motion tasks of the contact tasks in the vector of newTaskMotions (same handling)
  for (auto cl : taskContacts) {
    TaskSE3Equality& contact_motion = cl->contact.getMotionTask();
    auto tl = std::make_shared<TaskLevelMotion>(contact_motion, 1);
    newTaskMotions.push_back(tl);
  }

  // In case of modification of the number of tasks in the QP
  // update accordingly the vectors to keep them sorted in function of the tasks
  auto ds = m_dS;
  auto A = m_A;
  auto S_prev = m_S_prev;
  auto S = m_S;
  auto maked_Kp_prev = m_masked_Kp_prev;
  m_dS.setZero(newTaskMotions.size());
  m_A.setZero(newTaskMotions.size());
  m_S.setZero(newTaskMotions.size());
  m_S_prev.setZero(newTaskMotions.size());
  m_masked_Kp_prev = std::vector<Matrix>(newTaskMotions.size());

  for (auto i = 0u; i < newTaskMotions.size(); i++) {
    const auto& name = newTaskMotions[i]->task.name();
    const auto it = std::find_if(std::begin(m_taskMotions), std::end(m_taskMotions), [&name](const auto & task) {
      return name == task->task.name();
    });
    // Task was already there, copy the gain to the new vector
    if (it != std::end(m_taskMotions)) {
      const auto idx = std::distance(std::begin(m_taskMotions), it);
      m_dS[i] = ds[idx];
      m_A[i] = A[idx];
      m_S[i] = S[idx];
      m_S_prev[i] = S_prev[idx];
      m_masked_Kp_prev[i] = maked_Kp_prev[idx];
    }
  }

  m_taskMotions = newTaskMotions;
  m_taskContacts = taskContacts;
  m_taskForces = taskForces;

  if (m_first_iter) {
    m_H_tot_prev = data.potential_energy;
    m_first_iter = false;
  }
}

const Vector & TaskEnergy::get_A_vector() const {
  return m_A;
}

const double & TaskEnergy::get_lowerBound() const {
  return m_passivityConstraint.lowerBound()[0];
}

const double & TaskEnergy::get_H() const {
  return m_H;
}
const double & TaskEnergy::get_dH() const {
  return m_dH;
}
const double & TaskEnergy::get_E_tank() const {
  return m_E_tank;
}
void TaskEnergy::set_E_tank(const double & E_tank) {
  m_E_tank = E_tank;
}
const double & TaskEnergy::get_dE_tank() const {
  return m_dE_tank;
}
const double & TaskEnergy::get_H_tot() const {
  return m_H_tot;
}
const double & TaskEnergy::get_dH_tot() const {
  return m_dH_tot;
}
const Vector & TaskEnergy::get_S() const {
  return m_S;
}
const Vector & TaskEnergy::get_dS() const {
  return m_dS;
}
const double & TaskEnergy::get_dt() const {
  return m_dt;
}
const double & TaskEnergy::get_alpha() const {
  return m_alpha;
}
const double & TaskEnergy::get_beta() const {
  return m_beta;
}
const double & TaskEnergy::get_gamma() const {
  return m_gamma;
}

const Matrix & TaskEnergy::get_Lambda() const {
  return m_Lambda;
}

const bool & TaskEnergy::is_S_positive_definite() const {
  return m_Lambda_Kp_pos_def;
}

void TaskEnergy::enable_test_positive_def(const bool & isEnabled) {
  m_test_semi_def_pos = isEnabled;
}

const ConstraintBase & TaskEnergy::getConstraint() const {
  return m_passivityConstraint;
}

const ConstraintBase & TaskEnergy::compute(const double ,
                                           ConstRefVector q,
                                           ConstRefVector v,
                                           Data & data) {
  // Here is computed the energy tank and the regulating coefficients
  // For more details see the RA-L paper "Passive Inverse Dynamics Control using a Global Energy Tank for Torque-Controlled
  // Humanoid Robots in Multi-Contact", by Noelie Ramuzat, Sébastien BORIA, Olivier Stasse.
  if (m_taskMotions.size() <= 0) {
    std::cerr << "No motion tasks for energy calculation !" << std::endl;
    return m_passivityConstraint;
  }
  double E_c = 0.0;
  // Here is the computation of the eq (22) of the paper
  double non_linear_effect_term; // called v^T C v in the paper
  non_linear_effect_term = v.transpose() * m_robot.nonLinearEffects(data);
  double contact_forces_term = 0.0;
  double task_force_term = 0.0;
  for (auto cl : m_taskContacts) {
    Matrix J_k = cl->motionConstraint->matrix().leftCols(m_robot.nv());
    Vector f_k_ref = cl->forceRegTask->vector();
    contact_forces_term += v.transpose() * J_k.transpose() * f_k_ref;
  }
  for (auto f : m_taskForces) {
    Vector f_des = f->task.getConstraint().vector();
    Matrix J_f = Matrix::Zero(6, 38);
    for (auto cl_f : m_taskContacts) {
      if (f->task.getAssociatedContactName() == cl_f->contact.name()) {
        J_f = cl_f->motionConstraint->matrix().leftCols(m_robot.nv());
        break;
      }
    }
    task_force_term += v.transpose() * J_f.transpose() * f_des;
  }

  // Here is computed the B^sigma term of the equation (21) of the paper
  int i = 0;
  for (auto& it : m_taskMotions) {
    std::string frame_name = it->task.getFrameName();
    double damping_term, acc_term;
    Matrix J = it->task.getJacobian();
    Matrix Jpinv, dJ;
    Jpinv.setZero(J.cols(), J.rows());
    dJ.setZero(6, m_robot.nv());
    pseudoInverse(J, Jpinv, 1e-6);

    Vector acc_error;
    m_Lambda = Jpinv.transpose() * m_robot.mass(data) * Jpinv;
    if (frame_name == "com") {
      dJ = data.dAg.topRows(3);
      acc_error = it->task.acceleration_ref() - dJ * v;
    } else if (frame_name == "am") {
      dJ = data.dAg.bottomRows(3);
      acc_error = it->task.acceleration_ref() - dJ * v;
    } else if (frame_name == "posture") {
      acc_error = it->task.acceleration_ref() - v.tail(m_robot.nv() - 6);
    } else {
      pinocchio::computeJointJacobiansTimeVariation(m_robot.model(), data, q, v);
      Index frame_id = m_robot.model().getFrameId(frame_name);
      pinocchio::getFrameJacobianTimeVariation(m_robot.model(), data, frame_id, pinocchio::LOCAL, dJ);
      acc_error = it->task.acceleration_ref() - dJ * v;
    }

    Vector mask = it->task.getMask();
    int mask_size = (int)mask.sum();
    Vector masked_vel(mask_size), masked_vel_ref(mask_size);
    Matrix masked_Kd(mask_size, mask_size), masked_Kp(mask_size, mask_size);
    Vector vel_ref = it->task.velocity_ref();
    Vector task_vel = it->task.velocity();
    Matrix task_Kp = m_Lambda * (it->task.Kp()).asDiagonal();
    Matrix task_Kd = m_Lambda * (it->task.Kd()).asDiagonal();

    /// TEST of the semi-positive definite matrix Lambda*K_p (for the positivity of S)
    if (m_test_semi_def_pos) {
      Matrix U = (task_Kp + task_Kp.transpose()) * 0.5;
      Eigen::LLT<Eigen::MatrixXd> lltOfU(U); // compute the Cholesky decomposition of U
      if (lltOfU.info() == Eigen::NumericalIssue) {
        std::cerr << "Lambda*K_p is not a semi-positive definite matrix, S may be non positive !" << std::endl;
        m_Lambda_Kp_pos_def = false;
      }
    }

    // Take into account the masks of the tasks in TSID
    int idx = 0, idx_col = 0;
    for (int k = 0; k < mask.size(); k++) {
      if (mask(k) != 1.) continue;
      masked_vel[idx] = task_vel[k];
      masked_vel_ref[idx] = vel_ref[k];
      for (int l = 0; l < mask.size(); l++) {
        if (mask(l) != 1.) continue;
        masked_Kp(idx, idx_col) = task_Kp(k, l);
        masked_Kd(idx, idx_col) = task_Kd(k, l);
        idx_col ++;
      }
      idx_col = 0;
      idx ++;
    }
    damping_term = masked_vel.transpose() * masked_Kd * it->task.velocity_error(); // first term of eq (21)
    Vector Lambda_acc;
    Lambda_acc = m_Lambda * acc_error;
    acc_term = (it->task.velocity()).transpose() * Lambda_acc; // fourth term of eq (21)

    Matrix dot_Lambda_Kp; // derivative of Lambda * Kp
    if (m_masked_Kp_prev[i].size() == 0) {
      dot_Lambda_Kp = Matrix::Zero(masked_Kp.rows(), masked_Kp.cols());
    } else {
      dot_Lambda_Kp = (masked_Kp - m_masked_Kp_prev[i]) / m_dt;
    }
    double Lambda_dot_term;
    double vel_ref_term;
    if (frame_name != "am") {
      // Computation of S and dS of eq (16)
      m_S[i] = 0.5 * it->task.position_error().transpose() * masked_Kp * it->task.position_error();
      m_dS[i] = it->task.velocity_error().transpose() * masked_Kp * it->task.position_error();
      Lambda_dot_term = 0.5 * it->task.position_error().transpose() * dot_Lambda_Kp * it->task.position_error(); // third term of eq (21)
      vel_ref_term = masked_vel_ref.transpose() * masked_Kp * it->task.position_error(); // second term of eq (21)
      m_dS[i] += Lambda_dot_term;
    } else {
      m_dS[i] = 0.0;
      m_S[i] = 0.0;
      Lambda_dot_term = 0.0;
      vel_ref_term = 0.0;
    }

    // Update the previous terms
    m_S_prev[i] = m_S[i];
    m_masked_Kp_prev[i].resize(masked_Kp.rows(), masked_Kp.cols());
    m_masked_Kp_prev[i] = masked_Kp;

    double A = damping_term - acc_term - Lambda_dot_term + vel_ref_term; // B^sigma for one task
    m_A[i] = A;
    E_c += 0.5 * (it->task.velocity()).transpose() * m_Lambda * (it->task.velocity()); // Kinetic energy

    i++;
  }
  // Sum of the term along the tasks
  double A = m_A.sum();
  double B = non_linear_effect_term - contact_forces_term; // B^phi of eq (22)
  double A_tot = A + task_force_term; // B^sigma total of eq (22)

  // Smooth funtion on the gamma coefficient: eq(24) of the paper
  m_gamma = gammaFunction(A_tot, m_Plow, -m_Plow + .5);

  // Computation of beta in eq(25)
  if ((m_E_tank <= m_E_min_tank) && ((m_gamma * (A_tot) - B) < 0)) {
    m_beta = 0.0;
  } else {
    m_beta = 1.0;
  }

  // Smooth funtion on the alpha coefficient: eq(26) of the paper
  m_alpha = alphaFunction(0.0, m_E_max_tank, m_E_tank, m_beta * m_gamma * A_tot - B);

  m_dE_tank = (1 - m_alpha) * (m_beta * m_gamma * A_tot);
  m_dE_tank -= (1 - m_alpha) * B;

  // Compute the derivative of E_tank and check the bounds
  m_E_tank += m_dE_tank * m_dt;
  if (m_E_tank < m_E_min_tank) {
    m_E_tank = m_E_min_tank;
  } else if (m_E_tank > m_E_max_tank) {
    m_E_tank = m_E_max_tank;
  }

  // Multiply S by beta and gamma (as it will be done in the QP formulation in inverse-dynamics-formulation-acc-force.cpp)
  m_S = m_beta * m_gamma * m_S;
  m_dS = m_beta * m_gamma * m_dS;
  double S = m_S.sum();
  double dS = m_dS.sum();

  // Compute H and d_H finally
  m_H = S + m_E_tank;
  m_dH = dS + m_dE_tank;

  // Compute H_tot, H with the kinetic energy of the tasks and the gravity potential, eq(31) of the paper
  double V_g_i = data.potential_energy;
  m_H_tot = m_H + E_c + V_g_i;
  m_dH_tot = (m_H_tot - m_H_tot_prev) / m_dt;
  m_H_tot_prev = m_H_tot;

  // ENERGY DERIVATIVE PASSIVITY CONSTRAINT
  Matrix matrix = Matrix::Zero(1, m_robot.nv());
  matrix = - v.transpose();
  m_passivityConstraint.setMatrix(matrix); // multiplied by "tau" in the QP formulation (Ma - J^TF)
  m_b_lower = m_dH * Vector::Ones(m_dim);
  m_passivityConstraint.upperBound() = m_b_upper;
  m_passivityConstraint.lowerBound() = m_b_lower;

  return m_passivityConstraint;
}

}
}
