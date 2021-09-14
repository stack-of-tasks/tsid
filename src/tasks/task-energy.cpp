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

    double TaskEnergy::H_min(const double a, const double b, const double x, const double e_val){
      if ((x<a) && (e_val < 0)){
        return 0;
      } else if ((a <= x) && (x <= b)){
        double q = (x-a)/(b-a);
        double value = 6*pow(q,5) - 15*pow(q,4) + 10*pow(q,3);
        return value;
      } else {
        return 1;
      }
    }
    double TaskEnergy::H_max(const double a, const double b, const double x, const double e_val){
      if ((x>=b) && (e_val>0)){
        return 1;
      } else if ((a <= x) && (x < b) && (e_val>0)) {
        double q = (x-a)/(b-a);
        double value = 6*pow(q,5) - 15*pow(q,4) + 10*pow(q,3);
        return value;
      } else {
        return 0;
      }
    }

    double TaskEnergy::gammaFunction(const double A, const double P,
                                     const double delta, const double gamma_prev){
      if (A < P) {
        return P/A;
      } else if (A <= P + delta) {
        double x = (A - P)/delta;
        double value = 6*pow(x,5) - 15*pow(x,4) + 10*pow(x,3);
        double test = (P/A*exp(1 - 1/(1-value)) + exp(1 - 1/value));
        if (test > 1) {
          return 1;
        }
        return test;
      } else { 
        return 1;
      }
    }

    double TaskEnergy::lowPassFilter(const double& frequency, const double& signal,
                                     double& previous_signal) {
      // signal = alpha * previous_signal + (1-alpha) * signal_des
      double alpha = exp(-m_dt * 2 * M_PI * frequency);
      double output = alpha * previous_signal + signal * (1 - alpha);
      return output;
    }

    TaskEnergy::TaskEnergy(const std::string & name,
                           RobotWrapper & robot,
                           const double dt):
      TaskBase(name, robot),
      m_dt(dt),
      m_passivityConstraint(name, 1, robot.nv()),
      m_ref(robot.na())
    {
      m_dim = 1;
      m_E_max_tank = 5.0;
      m_E_min_tank = 0.1;
      m_E_tank = 3.0;
      m_b_lower = -1e10 * Vector::Ones(m_dim);
      m_b_upper = 1e10 * Vector::Ones(m_dim);
      m_first_iter = true;
      m_alpha = 0.0;
      m_beta = 1.0;
      m_gamma = 1.0;
      //m_prev_signal_filter = 0.0;
    }

    int TaskEnergy::dim() const
    {
      return m_dim;
    }

    void TaskEnergy::setTasks(const std::vector<std::shared_ptr<TaskLevelMotion> >  taskMotions, 
                              const std::vector<std::shared_ptr<ContactLevel> >  taskContacts, 
                              const std::vector<std::shared_ptr<TaskLevelForce> > taskForces, Data & data)
    {
      // TODO: Use dictionary instead of vector to keep track of tasks 
      // Then when adding/removing task reshape properly instead of setting all to 0.
      bool reshape = false;
      if (!m_first_iter) {
        if ((taskMotions.size() != m_taskMotions.size()) || (taskContacts.size() != m_taskContacts.size())){
          reshape = true;
        }
      }
      m_taskMotions = taskMotions;
      m_taskContacts = taskContacts;
      m_taskForces = taskForces;
      for (auto cl : m_taskContacts){
        TaskSE3Equality& contact_motion = cl->contact.getMotionTask();
        auto tl = std::make_shared<TaskLevelMotion>(contact_motion, 1);
        m_taskMotions.push_back(tl);
      }
      if (m_first_iter){
        m_alpha = 1.0;
        m_beta = 1.0;
        m_gamma = 1.0;
        m_Plow = -5.0; 
        m_E_tank = 3.0;
        m_dE_tank = 0.0;
        m_H = m_E_tank;
        m_dH = 0.0;
        m_H_tot = 0.0;
        double V_g = data.potential_energy;
        m_H_tot_prev = V_g;
        m_dH_tot = 0.0;
      } 
      if ((m_first_iter) || (reshape)) {
        m_dS.setZero(m_taskMotions.size());
        m_A.setZero(m_taskMotions.size());
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

    const Vector & TaskEnergy::get_A_vector() const
    {
      return m_A;
    }

    const double & TaskEnergy::get_lowerBound() const
    {
      return m_passivityConstraint.lowerBound()[0];
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
    void TaskEnergy::set_E_tank(const double & E_tank)
    {
      m_E_tank = E_tank;
    }
    const double & TaskEnergy::get_dE_tank() const
    {
      return m_dE_tank;
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

    const ConstraintBase & TaskEnergy::getConstraint() const
    {
      return m_passivityConstraint;
    }

    const ConstraintBase & TaskEnergy::compute(const double ,
                                               ConstRefVector q,
                                               ConstRefVector v,
                                               Data & data)
    {
      // Compute q
      Vector q_rpy = qQuatToRPY(q);

      if (m_taskMotions.size() <= 0){
        std::cerr << "No motion tasks for energy calculation !" << std::endl;
        return m_passivityConstraint;
      }
      double E_c = 0.0;
      double A_posture = 0.0;
      double S_posture = 0.0;
      double dS_posture = 0.0;

      double non_linear_effect_term;
      non_linear_effect_term = v.transpose() * m_robot.nonLinearEffects(data);
      double contact_forces_term = 0.0;
      double task_force_term = 0.0;
      for (auto cl : m_taskContacts){
        Matrix J_k = cl->motionConstraint->matrix().leftCols(m_robot.nv());
        Vector f_k_ref = cl->forceRegTask->vector();
        contact_forces_term += v.transpose() * J_k.transpose() * f_k_ref;
      }
      for (auto f : m_taskForces){
        Vector f_des = f->task.getConstraint().vector();
        Matrix J_f = Matrix::Zero(6, 38);
        for(auto cl_f : m_taskContacts) {
          if (f->task.getAssociatedContactName() == cl_f->contact.name()) {
            J_f = cl_f->motionConstraint->matrix().leftCols(m_robot.nv());
            break;
          }
        } 
        task_force_term += v.transpose() * J_f.transpose() * f_des;
      }
      
      int i = 0;
      for (auto& it : m_taskMotions){
        std::string frame_name = it->task.getFrameName();
        double damping_term, acc_term;        
        Matrix J = it->task.getJacobian();        
        Matrix Jpinv, dJ;
        Jpinv.setZero(J.cols(), J.rows());
        dJ.setZero(6, m_robot.nv());
        pseudoInverse(J, Jpinv, 1e-6);

        Matrix Lambda;
        Vector acc_error;
        Lambda = Jpinv.transpose() * m_robot.mass(data) * Jpinv;
        if (frame_name == "com"){
          dJ = data.dAg.topRows(3);
          acc_error = it->task.acceleration_ref() - dJ * v;    
        } else if (frame_name == "am"){
          dJ = data.dAg.bottomRows(3);
          acc_error = it->task.acceleration_ref() - dJ * v;
        } else if (frame_name == "posture"){
          acc_error = it->task.acceleration_ref() - v.tail(m_robot.nv() - 6);
        } else {
          pinocchio::computeJointJacobiansTimeVariation(m_robot.model(), data, q, v);
          Index frame_id = m_robot.model().getFrameId(frame_name);
          pinocchio::getFrameJacobianTimeVariation(m_robot.model(), data, frame_id, pinocchio::LOCAL, dJ);
          acc_error = it->task.acceleration_ref() - dJ * v;
        }

        Vector mask = it->task.getMask();
        Vector masked_vel((int)mask.sum()), masked_Kd((int)mask.sum()), masked_Kp((int)mask.sum());
        Vector vel_ref = it->task.velocity_ref();
        Vector masked_vel_ref((int)mask.sum());
        Vector task_vel = it->task.velocity();
        Vector task_Kp = Lambda * it->task.Kp();
        Vector task_Kd = Lambda * it->task.Kd();
        int idx = 0;
        for (int k = 0; k < mask.size(); k++) {
          if (mask(k) != 1.) continue;
          masked_vel[idx] = task_vel[k];
          masked_vel_ref[idx] = vel_ref[k];
          masked_Kp[idx] = task_Kp[k];
          masked_Kd[idx] = task_Kd[k];
          idx ++;
        }
        damping_term = masked_vel.transpose() * masked_Kd.cwiseProduct(it->task.velocity_error());
        Vector Lambda_acc;
        Lambda_acc = Lambda * acc_error;
        acc_term = (it->task.velocity()).transpose() * Lambda_acc;

        Vector dot_Lambda_Kp;
        if (m_maked_Kp_prev[i].size() == 0){
          dot_Lambda_Kp = Vector::Zero(masked_Kp.size());
        } else {
          dot_Lambda_Kp = (masked_Kp - m_maked_Kp_prev[i])/m_dt;
        } 
        double Lambda_dot_term;
        double vel_ref_term;
        if (frame_name != "am"){
          m_S[i] = 0.5 * it->task.position_error().transpose() * masked_Kp.cwiseProduct(it->task.position_error());          
          m_dS[i] = it->task.velocity_error().transpose() * masked_Kp.cwiseProduct(it->task.position_error());
          Lambda_dot_term = 0.5 * it->task.position_error().transpose() * dot_Lambda_Kp.cwiseProduct(it->task.position_error());
          vel_ref_term = masked_vel_ref.transpose() * masked_Kp.cwiseProduct(it->task.position_error());
          m_dS[i] += Lambda_dot_term;
        } else {
          m_dS[i] = 0.0;
          m_S[i] = 0.0;
          Lambda_dot_term = 0.0;
          vel_ref_term = 0.0;
        }
        
        m_S_prev[i] = m_S[i];
        m_maked_Kp_prev[i].resize(masked_Kp.size());
        m_maked_Kp_prev[i] = masked_Kp;
        
        double A = damping_term - acc_term - Lambda_dot_term + vel_ref_term;
        if (frame_name == "posture"){
          A_posture = A;
          S_posture = m_S[i];
          dS_posture = m_dS[i];
          m_dS[i] = 0.0;
          m_S[i] = 0.0;
        } else {
          m_A[i] = A;
        }

        E_c += 0.5 * (it->task.velocity()).transpose() * Lambda * (it->task.velocity());

        i++;
      }
      double A = m_A.sum();
      double B = non_linear_effect_term - contact_forces_term;
      double signal_to_filter = A + A_posture + task_force_term;
      //double signal_filter = lowPassFilter(1.0, signal_to_filter, m_prev_signal_filter);
      //m_prev_signal_filter = signal_filter;

      m_gamma = gammaFunction(signal_to_filter, m_Plow, 4.0, m_gamma);

      if ((m_E_tank <= m_E_min_tank) && ((m_gamma * (signal_to_filter) - B) < 0)) {
        m_beta = 0.0;
      } else {
        m_beta = 1.0;
      }

      m_alpha = H_max(0.0, m_E_max_tank, m_E_tank, m_beta * m_gamma * signal_to_filter - B);

      m_dE_tank = (1-m_alpha) * (m_beta * m_gamma * signal_to_filter);
      m_dE_tank -= (1-m_alpha) * B;

      m_E_tank += m_dE_tank * m_dt;
      if (m_E_tank < m_E_min_tank){
        m_E_tank = m_E_min_tank;
      } else if (m_E_tank > m_E_max_tank){
        m_E_tank = m_E_max_tank;
      }

      m_S = m_beta * m_gamma* m_S;
      m_dS = m_beta * m_gamma* m_dS;
      double S = m_S.sum() + m_beta *m_gamma* S_posture;
      double dS = m_dS.sum() + m_beta *m_gamma*dS_posture;
      m_H = S + m_E_tank;

      m_dH = dS + m_dE_tank;
      
      double V_g_i = data.potential_energy;

      m_H_tot = m_H + E_c + V_g_i;

      m_dH_tot = (m_H_tot - m_H_tot_prev)/m_dt;

      m_H_tot_prev = m_H_tot;

      // ENERGY DERIVATIVE PASSIVITY CONSTRAINT
      Matrix matrix = Matrix::Zero(1, m_robot.nv());
      matrix = - v.transpose();
      m_passivityConstraint.setMatrix(matrix);
      m_b_lower = m_dH * Vector::Ones(m_dim);
      m_passivityConstraint.upperBound() = m_b_upper;
      m_passivityConstraint.lowerBound() = m_b_lower;
      
      return m_passivityConstraint;
    }
    
  }
}
