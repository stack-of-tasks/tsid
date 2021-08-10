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

#ifndef __invdyn_task_energy_hpp__
#define __invdyn_task_energy_hpp__

#include "tsid/tasks/task-base.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/formulations/task-motion-level.hpp"
#include "tsid/formulations/contact-level.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/energy.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskEnergy : public TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Matrix Matrix;     
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;

      Vector qQuatToRPY(const Vector & q);
      double H_min(const double a, const double b, const double x, const double e_val);
      double H_max(const double a, const double b, const double x, const double e_val);
      double gammaFunction(const double A, const double P,
                           const double delta, const double gamma_prev);
      double lowPassFilter(const double& frequency, const double& signal,
                           double& previous_signal);

      TaskEnergy(const std::string & name,
                 RobotWrapper & robot,
                 const Vector & q,
                 const Vector & v,
                 const double dt,
                 const double timePreview);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;
      const ConstraintBase & getLyapunovConstraint() const;
      const ConstraintInequality & getMaxEnergyConstraint() const;
      const ConstraintEquality & getEnergyTask() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & position_ref() const;
      const Vector & get_A_vector() const;
      const double & get_lowerBound() const;
      const double & get_H() const;
      const double & get_dH() const;
      const double & get_E_tank() const;
      void set_E_tank(const double & E_tank);
      const double & get_dE_tank() const;
      const double & get_H_tot() const;
      const double & get_dH_tot() const;
      const Vector & get_S() const;
      const Vector & get_dS() const;
      const double & get_dt() const;
      const Vector & get_v() const;
      const double & get_alpha() const;
      const double & get_beta() const;
      const double & get_gamma() const;
      const double & get_upperBoundMaxEnergyCst() const;
      const double & get_lowerBoundMaxEnergyCst() const;
      const double & get_vectorEnergyTask() const;
      const Matrix & get_matrixEnergyTask() const;
      const Matrix & get_LyapunovMatrix() const;
      const Vector & K() const;
      void K(ConstRefVector K);
      const double & E_d() const;
      void setE_d(const double E);
      void setLyapunovMatrix(const Matrix M);
      void setE_m_ctrl(const double E_m);

      void setTasks(const std::vector<std::shared_ptr<TaskLevelMotion> >  taskMotions, 
                    const std::vector<std::shared_ptr<ContactLevel> >  taskContacts, Data & data);

    protected:
      Vector m_q_init;
      Vector m_v; //m_q_prev, 
      double m_dt;
      double m_time_preview;
      double m_alpha;
      double m_beta;
      double m_gamma;
      double m_Plow; 
      Vector m_A;     
      double m_H;
      double m_dH;
      double m_H_tot;
      double m_dH_tot;
      double m_H_tot_prev;
      Vector m_dS;
      Vector m_S;
      Vector m_S_prev;
      Vector m_K;
      std::vector<Vector> m_maked_Kp_prev;
      int m_dim;
      Matrix m_LyapMat;
      double m_E_tank;
      double m_dE_tank;
      double m_E_max;
      double m_E_d;
      double m_E_m_ctrl;
      double m_E_max_tank;
      double m_E_min_tank;
      Vector m_q_error; //, m_q_prev_error;
      Vector m_a_des;
      Vector m_BK;
      ConstraintInequality m_lyapunovConstraint;
      ConstraintInequality m_maxEnergyConstraint;
      ConstraintEquality m_energyTask;
      TrajectorySample m_ref;
      Vector m_b_lower;
      Vector m_b_upper;
      std::vector<std::shared_ptr<TaskLevelMotion> > m_taskMotions;
      std::vector<std::shared_ptr<ContactLevel> >   m_taskContacts;
      bool m_first_iter;
    };

  }
}

#endif // ifndef __invdyn_task_energy_hpp__
