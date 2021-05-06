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
      m_v(v.size()),
      m_dt(dt),
      m_time_preview(timePreview),
      m_lyapunovConstraint(name, 1, 2*robot.nv()),
      m_maxEnergyConstraint(name, 1, robot.nv()),
      m_energyTask(name, 1, robot.nv()),
      m_ref(robot.na())
    {
      m_q_init = qQuatToRPY(q); 
      // m_q_prev = qQuatToRPY(q);
      m_v = v;
      m_dim = 1;
      m_K.setZero(robot.nv());
      m_BK.setZero(robot.nv());
      m_q_error.setZero(robot.nv());
      // m_q_prev_error.setZero(robot.nv());
      m_A.setZero(robot.nv());
      m_A = m_A.transpose();
      m_E_c = 0.0;
      m_E_p = 0.0;
      m_E_tank = 5.0;
      m_E_max_tank = 5.0;
      m_E_max = 10.0;
      m_E_d = 2.0;
      m_E_m_ctrl = 0.0;
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

    const double & TaskEnergy::E_d() const { return m_E_d; }

    void TaskEnergy::setE_d(const double H)
    {
      m_E_d = H;
    }

    void TaskEnergy::setLyapunovMatrix(const Matrix M) {
      m_LyapMat = M;
    }

    void TaskEnergy::setE_m_ctrl(const double E_m){
      m_E_m_ctrl = E_m;
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
      return m_lyapunovConstraint.upperBound()[0];
    }
    
    const double & TaskEnergy::get_upperBoundMaxEnergyCst() const
    {
      return m_maxEnergyConstraint.upperBound()[0];
    }
    const double & TaskEnergy::get_lowerBoundMaxEnergyCst() const
    {
      return m_maxEnergyConstraint.lowerBound()[0];
    }
    const double & TaskEnergy::get_vectorEnergyTask() const
    {
      return m_energyTask.vector()[0];
    }
    const Matrix & TaskEnergy::get_matrixEnergyTask() const
    {
      return m_maxEnergyConstraint.matrix();
    }
    
    const double & TaskEnergy::get_E_c() const
    {
      return m_E_c;
    }
    const double & TaskEnergy::get_E_p() const
    {
      return m_E_p;
    }
    const double & TaskEnergy::get_E_tank() const
    {
      return m_E_tank;
    }
    const double & TaskEnergy::get_dt() const
    {
      return m_dt;
    }
    const Vector & TaskEnergy::get_A() const
    {
      return m_A;
    }
    const Vector & TaskEnergy::get_v() const
    {
      return m_v;
    }
    const Matrix & TaskEnergy::get_LyapunovMatrix() const
    {
      return m_LyapMat;
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
      Vector v_error = v - m_ref.vel;
      //m_q_prev_error = m_ref.pos - m_q_prev;
      Vector q_error_init = m_q_init - m_ref.pos;

      // E_c and E_p
      const Matrix & M = m_robot.mass(data);
      m_E_c = 0.5 * v.transpose() * M * v;
      m_E_p = 0.5 * m_q_error.transpose() * m_K.cwiseProduct(m_q_error);
      m_E_p -= 0.5 * q_error_init.transpose() * m_K.cwiseProduct(q_error_init);

      // Energy tank
      double diff_E_tank_mech = m_E_tank - m_E_m_ctrl;
      double E_tank_prev = m_E_tank;
      if (diff_E_tank_mech <= 0.0){
        m_E_tank = 0.0;
      } else if (diff_E_tank_mech >= m_E_max_tank){
        m_E_tank = m_E_max_tank;
      } else {
        m_E_tank = diff_E_tank_mech;
      }
      // s dot -> "derivative" of Energy tank
      double d_s;
      if (m_E_tank == 0.0){
        d_s = 0.0;
      } else {
        double diff_E_tank = m_E_tank - E_tank_prev;
        // double diff_E_des = m_E_c + m_E_p - m_E_d;
        // double diff_E_mech = m_E_c + m_E_p - m_E_m_ctrl;
        // if (diff_E_mech < 0.0) {
        //   d_s = - (m_E_tank + diff_E_mech) * exp(-(m_E_tank + diff_E_mech)*m_dt);
        // } else {
        //   d_s = - (m_E_tank) * exp(-(m_E_tank)*m_dt);
        // }
        d_s = - abs(diff_E_tank)/m_dt;
        //d_s = - abs(diff_E_des) * m_E_tank * exp(-abs(diff_E_des)*m_dt);
      }

      //Vector a_des = m_q_prev_error/m_dt - m_v_prev;
      //double K_error = - v.transpose() * m_K.cwiseProduct(m_q_error);
      Vector a_des = m_ref.acc;
      Vector B = m_q_error*m_dt + v_error*(m_dt*m_dt)/2 + a_des*(m_dt*m_dt*m_dt)/2;

      // ENERGY MAX CONSTRAINT
      Matrix maxEnergyMatrix = m_maxEnergyConstraint.matrix();
      Vector A_maxEnergy = (v*m_dt + a_des * (m_dt * m_dt)/2);
      //Vector B_maxEnergy = (m_q_error*m_dt + m_q_prev_error * m_dt/2);
      //std::cout << "##################### TASK_ENERGY ################################" << std::endl;
      // std::cout << "size m_maxEnergyConstraint->matrix().leftCols(m_v): "  << maxEnergyMatrix.leftCols(m_robot.nv()).rows() << "x" << maxEnergyMatrix.leftCols(m_robot.nv()).cols() << std::endl;
      // std::cout << "size A_maxEnergy.transpose(): "  << (A_maxEnergy.transpose()).rows() << "x" << (A_maxEnergy.transpose()).cols()  << std::endl;
      // std::cout << "size A_maxEnergy.transpose()* M: "  << (A_maxEnergy.transpose()* M).rows() << "x" << (A_maxEnergy.transpose()* M).cols()  << std::endl;
      // std::cout << "size m_dt*B.cwiseProduct(m_K).transpose(): "  << (m_dt*B.cwiseProduct(m_K).transpose()).rows() << "x" << (m_dt*B.cwiseProduct(m_K).transpose()).cols() << std::endl;
        
      maxEnergyMatrix.leftCols(m_robot.nv()) = A_maxEnergy.transpose()* M + m_dt*B.cwiseProduct(m_K).transpose(); // B_maxEnergy
      m_maxEnergyConstraint.setMatrix(maxEnergyMatrix);

      double BKv = B.transpose() * m_K.cwiseProduct(v_error); //B_maxEnergy
      //double BKv = - K_error;
      Vector up_maxEnergy = (m_E_max - m_E_c - m_E_p - BKv)*Vector::Ones(m_dim);
      m_maxEnergyConstraint.upperBound() = up_maxEnergy;
      m_maxEnergyConstraint.lowerBound() = (- m_E_c - m_E_p - BKv)*Vector::Ones(m_dim); //B_maxEnergy.cwiseProduct(m_K);

      // ENERGY TASK
      m_energyTask.setMatrix(maxEnergyMatrix);//* M + B_maxEnergy.cwiseProduct(m_K); // 
      Vector up_energyTask = (m_E_d - m_E_c - m_E_p - BKv)*Vector::Ones(m_dim);
      m_energyTask.setVector(up_energyTask);

      
      // ENERGY DERIVATIVE (LYAPUNOV) CONSTRAINT
      //double time_ratio = (m_time_preview * m_time_preview)/(2*m_dt); // delta_t^2 / 2*delta_t_iter
      // std::cout << "time_ratio: "  << time_ratio << std::endl;
      Vector a_des_t = a_des * m_dt/2;
      // Vector a_des_t = m_ref.acc * time_ratio;
      // std::cout << "a_des_t: "  << a_des_t << std::endl;
      //Vector v_des_t = m_q_prev_error * 0.5;

      //Vector diff = a_des_t - v;
      // std::cout << "diff: "  << diff << std::endl;
      m_A = a_des_t.transpose(); //((1/m_dt) * (v * m_time_preview + diff)).transpose();
      // std::cout << "A : "  << A << std::endl;
      //Vector B = m_q_error + v_des_t;
      m_BK = B.cwiseProduct(m_K);
      double BK_error = B.transpose() * m_K.cwiseProduct(v_error/m_dt);
      // Vector v_tail = v.tail(m_robot.nv()-6);
      // Vector tau_ext = m_ref.acc;
      // double mult = v.transpose() * tau_ext;
      // double mult = v_tail.transpose() * tau_ext.tail(m_robot.nv()-6);
      // - mult
      // double K_error = - BK_error;
      // m_q_prev = q_rpy;
      m_v = v;
      //Vector preview_v = (m_p + m_v * m_time_preview).transpose();
      //Vector preview_a = (m_v * m_time_preview + 0.5 * m_ref.acc * m_time_preview * m_time_preview).transpose();
      
      //double E_p = m_p_error.transpose() * K_p_error;
      //double bound = -2*E_p - (m_v_error * m_time_preview).transpose() * K_p_error;
      //Vector v_bound = bound * Vector::Ones(m_dim);

      Matrix matrix = Matrix::Zero(1, 2*m_robot.nv());
      matrix.leftCols(m_robot.nv()) = m_A * M + m_BK.transpose();
      matrix.rightCols(m_robot.nv()) = v.transpose();
      m_lyapunovConstraint.setMatrix(matrix);
      m_b_upper = (- BK_error - d_s) * Vector::Ones(m_dim); //K_error
      m_lyapunovConstraint.upperBound() = m_b_upper;
      m_lyapunovConstraint.lowerBound() = m_b_lower;//

      return m_lyapunovConstraint;
    }
    
  }
}
