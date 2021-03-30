//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
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

#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-energy.hpp"
#include "tsid/robots/robot-wrapper.hpp"


namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    Vector TaskEnergy::qQuatToRPY(const Vector & q){
      Vector q_rpy(m_robot.nv());
      Eigen::Quaterniond quat(q.segment<4>(3));
      Vector rpy = (quat.toRotationMatrix().eulerAngles(2, 1, 0)).reverse();
      q_rpy.head<3>() = q.head<3>();
      q_rpy.segment<3>(3) = rpy;
      q_rpy.tail(m_robot.nv()-6) = q.tail(m_robot.nv()-6);
      return q_rpy;
    }

    TaskEnergy::TaskEnergy(const std::string & name,
                           RobotWrapper & robot,
                           const Vector & q,
                           const Vector & v,
                           const double dt,
                           const double timePreview):
      TaskBase(name, robot),
      m_q_prev(v.size()),
      m_v_prev(v.size()),
      m_dt(dt),
      m_time_preview(timePreview),
      m_lyapunovConstraint(name, 1, 2*robot.nv()),
      m_maxEnergyConstraint(name, 1, robot.nv()),
      m_energyTask(name, 1, robot.nv()),
      m_ref(robot.na())
    {
      m_q_init = qQuatToRPY(q); 
      m_q_prev = qQuatToRPY(q);
      m_v_prev = v;
      m_dim = 1;
      m_K.setZero(robot.nv());
      m_BK.setZero(robot.nv());
      m_q_error.setZero(robot.nv());
      m_q_prev_error.setZero(robot.nv());
      m_A.setZero(robot.nv());
      m_A = m_A.transpose();
      m_E_c = 0.0;
      m_E_p = 0.0;
      m_H_max = 20.0;
      m_H_d = 2.0;
      m_b_lower = -1e10 * Vector::Ones(m_dim);
      m_b_upper = 1e10 * Vector::Ones(m_dim);
    }

    int TaskEnergy::dim() const
    {
      return m_dim;
    }

    const Vector & TaskEnergy::K() const { return m_K; }

    void TaskEnergy::K(ConstRefVector K)
    {
      assert(K.size()==m_robot.nv());
      m_K = K;
    }

    const double & TaskEnergy::H_d() const { return m_H_d; }

    void TaskEnergy::setH_d(const double H)
    {
      m_H_d = H;
    }
    void TaskEnergy::setReference(const TrajectorySample & ref)
    {
      assert(ref.pos.size()==m_robot.nv());
      assert(ref.vel.size()==m_robot.nv());
      assert(ref.acc.size()==m_robot.nv());
      m_ref = ref;
    }

    const TrajectorySample & TaskEnergy::getReference() const
    {
      return m_ref;
    }

    const Vector & TaskEnergy::position_ref() const
    {
      return m_ref.pos;
    }

    const Vector & TaskEnergy::get_BK_vector() const
    {
      return m_BK;
    }

    const double & TaskEnergy::get_upperBound() const
    {
      return m_b_upper[0];
    }
    const double & TaskEnergy::get_E_c() const
    {
      return m_E_c;
    }
    const double & TaskEnergy::get_E_p() const
    {
      return m_E_p;
    }
    const Vector & TaskEnergy::get_A() const
    {
      return m_A;
    }

    const ConstraintBase & TaskEnergy::getConstraint() const
    {
      return m_lyapunovConstraint;
    }

    const ConstraintBase & TaskEnergy::getLyapunovConstraint() const
    {
      return m_lyapunovConstraint;
    }

    const ConstraintInequality & TaskEnergy::getMaxEnergyConstraint() const
    {
      return m_maxEnergyConstraint;
    }

    const ConstraintEquality & TaskEnergy::getEnergyTask() const 
    {
      return m_energyTask;
    }

    // const ConstraintEquality & TaskEnergy::computeEnergyTask(const double ,
    //                                                          ConstRefVector q,
    //                                                          ConstRefVector v,
    //                                                          Data & data)
    // {
    //   return m_energyTask;
    // }

    // const ConstraintInequality & TaskEnergy::computeMaxEnergy(const double ,
    //                                            ConstRefVector q,
    //                                            ConstRefVector v,
    //                                            Data & data)
    // {
    //   return m_maxEnergyConstraint;
    // }

    const ConstraintBase & TaskEnergy::compute(const double ,
                                               ConstRefVector q,
                                               ConstRefVector v,
                                               Data & data)
    {
      // Compute errors
      Vector q_rpy = qQuatToRPY(q);
      m_q_error = q_rpy - m_ref.pos;
      m_q_prev_error = m_ref.pos - m_q_prev;
      Vector q_error_init = m_q_init - m_ref.pos;

      // E_c and E_p for debug
      const Matrix & M = m_robot.mass(data);
      m_E_c = 0.5 * v.transpose() * M * v;
      m_E_p = 0.5 * m_q_error.transpose() * m_K.cwiseProduct(m_q_error);
      m_E_p -= 0.5 * q_error_init.transpose() * m_K.cwiseProduct(q_error_init);

      Vector a_des = m_q_prev_error/m_dt - m_v_prev;
      double K_error = - v.transpose() * m_K.cwiseProduct(m_q_error);

      // ENERGY MAX CONSTRAINT
      Matrix maxEnergyMatrix = m_maxEnergyConstraint.matrix();
      Vector A_maxEnergy = (v*m_time_preview + a_des * (m_time_preview * m_time_preview)/2).transpose();
      maxEnergyMatrix.leftCols(m_robot.nv()) = A_maxEnergy; // * M + B_maxEnergy.cwiseProduct(m_K)
      m_maxEnergyConstraint.setMatrix(maxEnergyMatrix);
      // Vector B_maxEnergy = (m_q_error*m_time_preview + m_q_prev_error * (m_time_preview * m_time_preview)/2);
      // double BKv = B_maxEnergy.transpose() * m_K.cwiseProduct(m_v_prev/m_dt);
      double BKv = - K_error;
      Vector up_maxEnergy = (m_H_max - m_E_p - m_E_p - BKv)*Vector::Ones(m_dim);
      m_maxEnergyConstraint.upperBound() = up_maxEnergy;
      m_maxEnergyConstraint.lowerBound() = (- m_E_p - m_E_p - BKv)*Vector::Ones(m_dim); //B_maxEnergy.cwiseProduct(m_K);

      // ENERGY TASK
      m_energyTask.setMatrix(maxEnergyMatrix); // * M + B_maxEnergy.cwiseProduct(m_K)
      Vector up_energyTask = (m_H_d - m_E_p - m_E_p - BKv)*Vector::Ones(m_dim);
      m_energyTask.setVector(up_energyTask);

      
      // ENERGY DERIVATIVE (LYAPUNOV) CONSTRAINT
      double time_ratio = (m_time_preview * m_time_preview)/(2*m_dt); // delta_t^2 / 2*delta_t_iter
      // std::cout << "time_ratio: "  << time_ratio << std::endl;
      Vector a_des_t = a_des * time_ratio;
      // Vector a_des_t = m_ref.acc * time_ratio;
      // std::cout << "a_des_t: "  << a_des_t << std::endl;
      Vector v_des_t = m_q_prev_error * time_ratio;

      Vector diff = a_des_t - v;
      // std::cout << "diff: "  << diff << std::endl;
      m_A = ((1/m_dt) * (v * m_time_preview) + diff).transpose();
      // std::cout << "A : "  << A << std::endl;
      Vector B = m_q_error * m_dt + v_des_t;
      m_BK = B.cwiseProduct(m_K);
      double BK_error = B.transpose() * m_K.cwiseProduct(m_v_prev/m_dt);
      Vector v_tail = v.tail(m_robot.nv()-6);
      Vector tau_ext = m_ref.acc;
      double mult = v.transpose() * tau_ext;
      // double mult = v_tail.transpose() * tau_ext.tail(m_robot.nv()-6);
      // - mult
      // double K_error = - BK_error;
      m_q_prev = q_rpy;
      m_v_prev = v;
      //Vector preview_v = (m_p + m_v * m_time_preview).transpose();
      //Vector preview_a = (m_v * m_time_preview + 0.5 * m_ref.acc * m_time_preview * m_time_preview).transpose();
      
      //double E_p = m_p_error.transpose() * K_p_error;
      //double bound = -2*E_p - (m_v_error * m_time_preview).transpose() * K_p_error;
      //Vector v_bound = bound * Vector::Ones(m_dim);

      Matrix matrix = Matrix::Zero(1, 2*m_robot.nv());
      matrix.leftCols(m_robot.nv()) = m_A; // * M + m_BK
      matrix.rightCols(m_robot.nv()) = v.transpose();
      m_lyapunovConstraint.setMatrix(matrix);
      m_b_upper = K_error* Vector::Ones(m_dim);
      m_lyapunovConstraint.upperBound() = m_b_upper;
      m_lyapunovConstraint.lowerBound() = m_b_lower;//m_BK;

      return m_lyapunovConstraint;
    }
    
  }
}
