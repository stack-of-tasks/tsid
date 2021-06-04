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

#include "tsid/formulations/inverse-dynamics-formulation-acc-force.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/tasks/task-energy.hpp"
#include "tsid/robots/robot-wrapper.hpp"
//#include "sot/core/matrix-svd.hh"


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
      m_lyapunovConstraint(name, 1, robot.nv()),
      m_maxEnergyConstraint(name, 1, robot.nv()),
      m_energyTask(name, 1, robot.nv()),
      m_ref(robot.na())
    {
      m_q_init = q; //qQuatToRPY(q)
      // m_q_prev = qQuatToRPY(q);
      m_v = v;
      m_dim = 1;
      m_K.setZero(robot.nv());
      m_BK.setZero(robot.nv());
      m_q_error.setZero(robot.nv());
      // m_q_prev_error.setZero(robot.nv());
      m_E_max_tank = 5.0;
      m_E_min_tank = 0.1;
      m_E_max = 10.0;
      m_E_d = 2.0;
      m_E_m_ctrl = 0.0;
      m_b_lower = -1e10 * Vector::Ones(m_dim);
      m_b_upper = 1e10 * Vector::Ones(m_dim);
      m_first_iter = true;
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

    void TaskEnergy::setTasks(const std::vector<std::shared_ptr<TaskLevelMotion> >  taskMotions, 
                              const std::vector<std::shared_ptr<ContactLevel> >  taskContacts, Data & data){
      m_taskMotions = taskMotions;
      m_taskContacts = taskContacts;
      for (auto cl : m_taskContacts){
        TaskSE3Equality& contact_motion = cl->contact.getMotionTask();
        auto tl = std::make_shared<TaskLevelMotion>(contact_motion, 1);
        m_taskMotions.push_back(tl);
      }
      std::cout << "##################### size taskMotions: " << m_taskMotions.size() << "################################" << std::endl;
      if (m_first_iter){
        m_alpha = 0.0;
        m_beta = 1.0;
        m_gamma = 1.0;
        m_Plow = - 1.0; 
        m_E_tank = 5.0;
        m_H = m_E_tank;
        m_dH = 0.0;
        m_H_tot = 0.0;
        // pinocchio::Data data(m_robot.model());
        double V_g = data.potential_energy; //pinocchio::computePotentialEnergy(m_robot.model(), data, m_q_init);
        m_H_tot_prev = V_g;
        m_dH_tot = 0.0;
        m_dS.setZero(m_taskMotions.size());
        m_A.setZero(m_taskMotions.size());
        std::cout << "##################### size m_A: " << m_A.size() << "################################" << std::endl;
        m_S.setZero(m_taskMotions.size());
        m_S_prev.setZero(m_taskMotions.size());
        m_maked_Kp_prev.resize(m_taskMotions.size());
        m_first_iter = false;
      }
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

    const double & TaskEnergy::get_lowerBound() const
    {
      return m_lyapunovConstraint.lowerBound()[0];
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
    
    const double & TaskEnergy::get_H() const
    {
      return m_H;
    }
    const double & TaskEnergy::get_dH() const
    {
      return m_dH;
    }
    const double & TaskEnergy::get_E_tank() const
    {
      return m_E_tank;
    }
    const double & TaskEnergy::get_H_tot() const
    {
      return m_H_tot;
    }
    const double & TaskEnergy::get_dH_tot() const
    {
      return m_dH_tot;
    }
    const Vector & TaskEnergy::get_S() const
    {
      return m_S;
    }
    const Vector & TaskEnergy::get_dS() const
    {
      return m_dS;
    }
    const double & TaskEnergy::get_dt() const
    {
      return m_dt;
    }
    const Vector & TaskEnergy::get_v() const
    {
      return m_v;
    }    
    const double & TaskEnergy::get_alpha() const
    {
      return m_alpha;
    }
    const double & TaskEnergy::get_beta() const
    {
      return m_beta;
    }
    const double & TaskEnergy::get_gamma() const
    {
      return m_gamma;
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
      // Compute q
      Vector q_rpy = qQuatToRPY(q);

      // Compute dE_tank
      if (m_taskMotions.size() <= 0){
        std::cerr << "No motion tasks for energy calculation !" << std::endl;
        return m_lyapunovConstraint;
      }
      double dE_tank = 0.0;
      double E_c = 0.0;

      double non_linear_effect_term;
      non_linear_effect_term = v.transpose() * m_robot.nonLinearEffects(data);
      std::cout << "##################### non_linear_effect_term: " << non_linear_effect_term << " ################################" << std::endl;
      double contact_forces_term = 0.0;
      // std::cout << "##################### size m_taskContacts: " << m_taskContacts.size() << " ################################" << std::endl;
      for (auto cl : m_taskContacts){
        Matrix J_k = cl->motionConstraint->matrix().leftCols(m_robot.nv());
        // std::cout << "##################### J_k: " << J_k << " ################################" << std::endl;
        // std::cout << "##################### J_k size: " << J_k.size() << " ################################" << std::endl;
        // std::cout << J_k.rows() << std::endl;
        // std::cout << J_k.cols() << std::endl;
        Vector f_k_ref = cl->forceRegTask->vector();
        // std::cout << "##################### f_k_ref: " << f_k_ref << " ################################" << std::endl;
        contact_forces_term += v.transpose() * J_k.transpose() * f_k_ref;
      }
      std::cout << "##################### contact_forces_term: " << contact_forces_term << " ################################" << std::endl;
      
      int i = 0;
      for (auto& it : m_taskMotions){
        std::cout << "##################### For loop i: " << i << " ################################" << std::endl;
        std::string frame_name = it->task.getFrameName();
        double damping_term, acc_term;
        std::cout << "##################### Frame name: " << frame_name << " ################################" << std::endl;
        
        Matrix J = it->task.getJacobian();
        // std::cout << "##################### J i: " << i << " " << J << "################################" << std::endl;
        // std::cout << J.rows() << std::endl;
        // std::cout << J.cols() << std::endl;
        
        Matrix Jpinv, dJ;
        Jpinv.setZero(J.cols(), J.rows());
        dJ.setZero(6, m_robot.nv());
        pseudoInverse(J, Jpinv, 1e-6);
        // std::cout << "##################### pseudoInverse i: " << i << " " << Jpinv << "################################" << std::endl;
        // std::cout << Jpinv.rows() << std::endl;
        // std::cout << Jpinv.cols() << std::endl;

        Matrix Lambda;
        //Matrix Lambda_inv = it->task.getLambdaInv();
        Vector acc_error;
        Lambda = Jpinv.transpose() * m_robot.mass(data) * Jpinv;
        if (frame_name == "com"){
          //pinocchio::computeCentroidalMapTimeVariation(m_robot.model(), data, q, v);
          dJ = data.dAg.topRows(3);
          // std::cout << "##################### dJ i: " << i << " " << dJ << "################################" << std::endl;
          // std::cout << dJ.rows() << std::endl;
          // std::cout << dJ.cols() << std::endl;
          acc_error = it->task.acceleration_ref() - dJ * v;          
          //Matrix Minv = (m_robot.mass(data)).inverse();
          //Lambda_inv = J * Minv * J.transpose();
        } else if (frame_name == "am"){
          //pinocchio::computeCentroidalMapTimeVariation(m_robot.model(), data, q, v);
          dJ = data.dAg.bottomRows(3);
          // std::cout << "##################### dJ i: " << i << " " << dJ << "################################" << std::endl;
          // std::cout << dJ.rows() << std::endl;
          // std::cout << dJ.cols() << std::endl;
          acc_error = it->task.acceleration_ref() - dJ * v;
          // Matrix Minv = (m_robot.mass(data)).inverse();
          //Lambda_inv = J * Minv * J.transpose();
        } else if (frame_name == "posture"){
          //pinocchio::computeJointJacobiansTimeVariation(m_robot.model(), data, q, v);
          //dJ = Eigen::MatrixXd::Identity(32, 38);
          // // std::cout << "##################### dJ i: " << i << " " << dJ << "################################" << std::endl;
          // // std::cout << dJ.rows() << std::endl;
          // // std::cout << dJ.cols() << std::endl;
          acc_error = it->task.acceleration_ref() - v.tail(m_robot.nv() - 6);
          //Lambda_inv = Lambda.inverse();
        } else {
          // std::cout << "##################### SE3 calculation dJ ################################" << std::endl;
          pinocchio::computeJointJacobiansTimeVariation(m_robot.model(), data, q, v);
          Index frame_id = m_robot.model().getFrameId(frame_name);
          // std::cout << "##################### frame_id : " << frame_id << "################################" << std::endl;
          pinocchio::getFrameJacobianTimeVariation(m_robot.model(), data, frame_id, pinocchio::LOCAL, dJ);
          // std::cout << "##################### dJ i: " << i << " " << dJ << "################################" << std::endl;
          // std::cout << dJ.rows() << std::endl;
          // std::cout << dJ.cols() << std::endl;
          acc_error = it->task.acceleration_ref() - dJ * v;
          // Matrix Minv = (m_robot.mass(data)).inverse();
          // Lambda_inv = J * Minv * J.transpose();
        }
        
        // std::cout << "##################### Lambda i: " << i << " " << Lambda << "################################" << std::endl;
        // std::cout << "##################### size Lambda " << Lambda.size() << "################################" << std::endl;
        // std::cout << Lambda.rows() << std::endl;
        // std::cout << Lambda.cols() << std::endl;
      
        // std::cout << "##################### Compute Lambda_inv ok ################################" << std::endl;
        // std::cout << Lambda_inv << std::endl;

        Vector mask = it->task.getMask();
        // std::cout << "##################### mask i: " << i << " " << mask << "################################" << std::endl;
        // std::cout << mask.size() << std::endl;
        Vector masked_vel((int)mask.sum()), masked_Kd((int)mask.sum()), masked_Kp((int)mask.sum());
        // std::cout << masked_vel.size() << std::endl;
        // std::cout << masked_Kd.size() << std::endl;
        // std::cout << masked_Kp.size() << std::endl;
        Vector task_vel = it->task.velocity();
        // std::cout << task_vel.size() << std::endl;
        // std::cout << "##################### it->task.Kp(): " << it->task.Kp() << "################################" << std::endl;
        // std::cout << "##################### it->task.Kp(): " << it->task.Kd() << "################################" << std::endl;
        Vector task_Kp = Lambda * it->task.Kp();
        Vector task_Kd = Lambda * it->task.Kd();
        std::cout << "##################### task_Kp: " << task_Kp << "################################" << std::endl;
        std::cout << "##################### task_Kd: " << task_Kd << "################################" << std::endl;
        std::cout << task_Kp.size() << std::endl;
        std::cout << task_Kd.size() << std::endl;
        int idx = 0;
        for (int k = 0; k < mask.size(); k++) {
          if (mask(k) != 1.) continue;
          masked_vel[idx] = task_vel[k];
          masked_Kp[idx] = task_Kp[k];
          masked_Kd[idx] = task_Kd[k];
          idx ++;
        }
        std::cout << "##################### FOR loop mask ok ################################" << std::endl;
        std::cout << masked_vel.size() << std::endl;
        std::cout << masked_Kp.size() << std::endl;
        std::cout << masked_Kd.size() << std::endl;
        damping_term = masked_vel.transpose() * masked_Kd.cwiseProduct(it->task.velocity_error());
        std::cout << "##################### damping_term i: " << i << " " << damping_term << " ################################" << std::endl;

        // std::cout << "##################### acc_error i: " << i << " " << acc_error << "################################" << std::endl;
        Vector test;
        test = Lambda * acc_error;
        // std::cout << "##################### test i: " << i << " " << test << "################################" << std::endl;
        // std::cout << test.rows() << std::endl;
        // std::cout << test.cols() << std::endl;
        // std::cout << "##################### it->task.velocity() i: " << i << " " << it->task.velocity() << "################################" << std::endl;
        acc_term = (it->task.velocity()).transpose() * test;
        std::cout << "##################### acc_term i: " << i << " " << acc_term << " ################################" << std::endl;
        std::cout << "##################### size m_A: " << m_A.size() << " ################################" << std::endl;

        Vector dot_Lambda_Kp;
        if (m_maked_Kp_prev[i].size() == 0){
          std::cout << "##################### m_maked_Kp_prev[i].size() == 0 ################################" << std::endl;
          dot_Lambda_Kp = Vector::Zero(masked_Kp.size());
        } else {
          std::cout << "##################### m_maked_Kp_prev[i] : " << m_maked_Kp_prev[i] << " ################################" << std::endl;
          dot_Lambda_Kp = (masked_Kp - m_maked_Kp_prev[i]) * m_dt;
        } 
        double Lambda_dot_term;
        if (frame_name != "am"){
          m_S[i] = 0.5 * it->task.position_error().transpose() * masked_Kp.cwiseProduct(it->task.position_error());
          m_dS[i] = masked_vel.transpose() * masked_Kp.cwiseProduct(it->task.position_error());
          Lambda_dot_term = 0.5 * it->task.position_error().transpose() * dot_Lambda_Kp.cwiseProduct(it->task.position_error());
        } else {
          m_dS[i] = 0.0;
          m_S[i] = 0.0;
          Lambda_dot_term = 0.0;
        }
        
        std::cout << "##################### dS i: " << i << " " << m_dS[i] << " ################################" << std::endl;
        std::cout << "##################### S i: " << i << " " << m_S[i] << " ################################" << std::endl;
        m_S_prev[i] = m_S[i];
        m_maked_Kp_prev[i].resize(masked_Kp.size());
        m_maked_Kp_prev[i] = masked_Kp;
        std::cout << "##################### m_maked_Kp_prev[i][0] : " << m_maked_Kp_prev[i][0] << " ################################" << std::endl;
          
        m_A[i] = damping_term - acc_term - Lambda_dot_term;
        std::cout << "##################### A_i i: " << i << " " << m_A[i] << " ################################" << std::endl;

        E_c += 0.5 * (it->task.velocity()).transpose() * Lambda * (it->task.velocity());

        i++;
      }
      double A = m_A.sum();
      double S = m_S.sum();
      double dS = m_dS.sum();
      double B = non_linear_effect_term - contact_forces_term;

      if (A < m_Plow){
        m_gamma = m_Plow/A;
      } else {
        m_gamma = 1.0;
      }

      if ((m_E_tank <= m_E_min_tank) && ((m_gamma * A - B) < 0)) {
        m_beta = 0.0;
      } else {
        m_beta = 1.0;
      }

      if ((m_E_tank >= m_E_max_tank) && ((m_beta * m_gamma * A - B) > 0)){
        m_alpha = 1.0;
      } else {
        m_alpha = 0.0;
      }

      dE_tank = (1-m_alpha) * (m_beta * m_gamma * A);
      dE_tank -= (1-m_alpha) * B;
      std::cout << "##################### dE_tank : " << dE_tank << "################################" << std::endl;

      m_E_tank += dE_tank * m_dt;
      if (m_E_tank < m_E_min_tank){
        m_E_tank = m_E_min_tank;
      } else if (m_E_tank > m_E_max_tank){
        m_E_tank = m_E_max_tank;
      }
      std::cout << "##################### m_E_tank : " << m_E_tank << "################################" << std::endl;

      m_H = S + m_E_tank;
      std::cout << "##################### m_H : " << m_H << "################################" << std::endl;

      m_dH = dS + dE_tank;
      std::cout << "##################### m_dH : " << m_dH << "################################" << std::endl;
      
      double V_g_i = data.potential_energy; //pinocchio::computePotentialEnergy(m_robot.model(), data, q);
      // std::cout << "##################### V_g_i i: " << i << " " << V_g_i << "################################" << std::endl;

      m_H_tot = m_H + E_c + V_g_i;
      // std::cout << "##################### m_H_tot i: " << i << " " << m_H_tot[i] << "################################" << std::endl;

      m_dH_tot = (m_H_tot - m_H_tot_prev)/m_dt;
      // std::cout << "##################### m_dH_tot i: " << i << " " << m_dH_tot[i] << "################################" << std::endl;

      m_H_tot_prev = m_H_tot;

      // ENERGY DERIVATIVE (LYAPUNOV) CONSTRAINT
      Matrix matrix = Matrix::Zero(1, m_robot.nv());
      matrix = - v.transpose();
      m_lyapunovConstraint.setMatrix(matrix);
      m_b_lower = m_dH * Vector::Ones(m_dim);
      m_lyapunovConstraint.upperBound() = m_b_upper;
      m_lyapunovConstraint.lowerBound() = m_b_lower;
      

      // m_q_error = q_rpy - m_ref.pos;
      // Vector v_error = v - m_ref.vel;
      // //m_q_prev_error = m_ref.pos - m_q_prev;
      // Vector q_error_init = m_q_init - m_ref.pos;

      // // E_c and E_p
      // const Matrix & M = m_robot.mass(data);
      // m_E_c = 0.5 * v.transpose() * M * v;
      // m_E_p = 0.5 * m_q_error.transpose() * m_K.cwiseProduct(m_q_error);
      // m_E_p -= 0.5 * q_error_init.transpose() * m_K.cwiseProduct(q_error_init);

      // // Energy tank
      // double diff_E_tank_mech = m_E_tank - m_E_m_ctrl;
      // double E_tank_prev = m_E_tank;
      // if (diff_E_tank_mech <= 0.0){
      //   m_E_tank = 0.0;
      // } else if (diff_E_tank_mech >= m_E_max_tank){
      //   m_E_tank = m_E_max_tank;
      // } else {
      //   m_E_tank = diff_E_tank_mech;
      // }
      // // s dot -> "derivative" of Energy tank
      // double d_s;
      // if (m_E_tank == 0.0){
      //   d_s = 0.0;
      // } else {
      //   double diff_E_tank = m_E_tank - E_tank_prev;
      //   // double diff_E_des = m_E_c + m_E_p - m_E_d;
      //   // double diff_E_mech = m_E_c + m_E_p - m_E_m_ctrl;
      //   // if (diff_E_mech < 0.0) {
      //   //   d_s = - (m_E_tank + diff_E_mech) * exp(-(m_E_tank + diff_E_mech)*m_dt);
      //   // } else {
      //   //   d_s = - (m_E_tank) * exp(-(m_E_tank)*m_dt);
      //   // }
      //   d_s = - abs(diff_E_tank)/m_dt;
      //   //d_s = - abs(diff_E_des) * m_E_tank * exp(-abs(diff_E_des)*m_dt);
      // }

      // //Vector a_des = m_q_prev_error/m_dt - m_v_prev;
      // //double K_error = - v.transpose() * m_K.cwiseProduct(m_q_error);
      // Vector a_des = m_ref.acc;
      // Vector B = m_q_error*m_dt + v_error*(m_dt*m_dt)/2 + a_des*(m_dt*m_dt*m_dt)/2;

      // // ENERGY MAX CONSTRAINT
      // Matrix maxEnergyMatrix = m_maxEnergyConstraint.matrix();
      // Vector A_maxEnergy = (v*m_dt + a_des * (m_dt * m_dt)/2);
      // //Vector B_maxEnergy = (m_q_error*m_dt + m_q_prev_error * m_dt/2);
      // //std::cout << "##################### TASK_ENERGY ################################" << std::endl;
      // // std::cout << "size m_maxEnergyConstraint->matrix().leftCols(m_v): "  << maxEnergyMatrix.leftCols(m_robot.nv()).rows() << "x" << maxEnergyMatrix.leftCols(m_robot.nv()).cols() << std::endl;
      // // std::cout << "size A_maxEnergy.transpose(): "  << (A_maxEnergy.transpose()).rows() << "x" << (A_maxEnergy.transpose()).cols()  << std::endl;
      // // std::cout << "size A_maxEnergy.transpose()* M: "  << (A_maxEnergy.transpose()* M).rows() << "x" << (A_maxEnergy.transpose()* M).cols()  << std::endl;
      // // std::cout << "size m_dt*B.cwiseProduct(m_K).transpose(): "  << (m_dt*B.cwiseProduct(m_K).transpose()).rows() << "x" << (m_dt*B.cwiseProduct(m_K).transpose()).cols() << std::endl;
        
      // maxEnergyMatrix.leftCols(m_robot.nv()) = A_maxEnergy.transpose()* M + m_dt*B.cwiseProduct(m_K).transpose(); // B_maxEnergy
      // m_maxEnergyConstraint.setMatrix(maxEnergyMatrix);

      // double BKv = B.transpose() * m_K.cwiseProduct(v_error); //B_maxEnergy
      // //double BKv = - K_error;
      // Vector up_maxEnergy = (m_E_max - m_E_c - m_E_p - BKv)*Vector::Ones(m_dim);
      // m_maxEnergyConstraint.upperBound() = up_maxEnergy;
      // m_maxEnergyConstraint.lowerBound() = (- m_E_c - m_E_p - BKv)*Vector::Ones(m_dim); //B_maxEnergy.cwiseProduct(m_K);

      // // ENERGY TASK
      // m_energyTask.setMatrix(maxEnergyMatrix);//* M + B_maxEnergy.cwiseProduct(m_K); // 
      // Vector up_energyTask = (m_E_d - m_E_c - m_E_p - BKv)*Vector::Ones(m_dim);
      // m_energyTask.setVector(up_energyTask);

      
      // // ENERGY DERIVATIVE (LYAPUNOV) CONSTRAINT
      // //double time_ratio = (m_time_preview * m_time_preview)/(2*m_dt); // delta_t^2 / 2*delta_t_iter
      // // std::cout << "time_ratio: "  << time_ratio << std::endl;
      // Vector a_des_t = a_des * m_dt/2;
      // // Vector a_des_t = m_ref.acc * time_ratio;
      // // std::cout << "a_des_t: "  << a_des_t << std::endl;
      // //Vector v_des_t = m_q_prev_error * 0.5;

      // //Vector diff = a_des_t - v;
      // // std::cout << "diff: "  << diff << std::endl;
      // m_A = a_des_t.transpose(); //((1/m_dt) * (v * m_time_preview + diff)).transpose();
      // // std::cout << "A : "  << A << std::endl;
      // //Vector B = m_q_error + v_des_t;
      // m_BK = B.cwiseProduct(m_K);
      // double BK_error = B.transpose() * m_K.cwiseProduct(v_error/m_dt);
      // // Vector v_tail = v.tail(m_robot.nv()-6);
      // // Vector tau_ext = m_ref.acc;
      // // double mult = v.transpose() * tau_ext;
      // // double mult = v_tail.transpose() * tau_ext.tail(m_robot.nv()-6);
      // // - mult
      // // double K_error = - BK_error;
      // // m_q_prev = q_rpy;
      // m_v = v;
      // //Vector preview_v = (m_p + m_v * m_time_preview).transpose();
      // //Vector preview_a = (m_v * m_time_preview + 0.5 * m_ref.acc * m_time_preview * m_time_preview).transpose();
      
      // //double E_p = m_p_error.transpose() * K_p_error;
      // //double bound = -2*E_p - (m_v_error * m_time_preview).transpose() * K_p_error;
      // //Vector v_bound = bound * Vector::Ones(m_dim);

      // Matrix matrix = Matrix::Zero(1, 2*m_robot.nv());
      // matrix.leftCols(m_robot.nv()) = m_A * M + m_BK.transpose();
      // matrix.rightCols(m_robot.nv()) = v.transpose();
      // m_lyapunovConstraint.setMatrix(matrix);
      // m_b_upper = (- BK_error - d_s) * Vector::Ones(m_dim); //K_error
      // m_lyapunovConstraint.upperBound() = m_b_upper;
      // m_lyapunovConstraint.lowerBound() = m_b_lower;//

      return m_lyapunovConstraint;
    }
    
  }
}
