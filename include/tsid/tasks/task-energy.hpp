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

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

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
      const Vector & get_BK_vector() const;
      const double & get_upperBound() const;
      const double & get_E_c() const;
      const double & get_E_p() const;
      const double & get_E_tank() const;
      const double & get_dt() const;
      const Vector & get_A() const;
      const Vector & get_v() const;
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

    protected:
      Vector m_q_init;
      Vector m_v; //m_q_prev, 
      double m_dt;
      double m_time_preview;
      Vector m_K;
      int m_dim;
      Vector m_A;
      Matrix m_LyapMat;
      double m_E_c;
      double m_E_p;
      double m_E_tank;
      double m_E_max;
      double m_E_d;
      double m_E_m_ctrl;
      double m_E_max_tank;
      Vector m_q_error; //, m_q_prev_error;
      Vector m_a_des;
      Vector m_BK;
      ConstraintInequality m_lyapunovConstraint;
      ConstraintInequality m_maxEnergyConstraint;
      ConstraintEquality m_energyTask;
      TrajectorySample m_ref;
      Vector m_b_lower;
      Vector m_b_upper;
    };

  }
}

#endif // ifndef __invdyn_task_energy_hpp__
