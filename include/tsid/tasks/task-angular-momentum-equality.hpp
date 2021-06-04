//
// Copyright (c) 2017 CNRS
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

#ifndef __invdyn_task_am_equality_hpp__
#define __invdyn_task_am_equality_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-equality.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskAMEquality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data::Matrix6x Matrix6x;
      typedef math::Matrix Matrix;


      TaskAMEquality(const std::string & name, 
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;
      const Matrix & getJacobian() const;
      const Vector3 & getDesiredMomentumDerivative() const;
      Vector3 getdMomentum(ConstRefVector dv) const;

      const Vector & momentum_error() const;
      const Vector & momentum() const;
      const Vector & momentum_ref() const;
      const Vector & dmomentum_ref() const;
      const Vector & velocity_error() const { return momentum_error(); }
      const Vector & velocity() const { return momentum(); }
      const Vector & velocity_ref() const { return momentum_ref(); }
      const Vector & acceleration_ref() const { return dmomentum_ref(); }
      const std::string getFrameName() { return "am"; }

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector m_Kp;
      Vector m_Kd;
      Vector m_L_error, m_dL_error;
      Vector3 m_dL_des;
      Matrix m_J_am;
      
      Vector3 m_drift;
      Vector m_L, m_dL;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif // ifndef __invdyn_task_am_equality_hpp__
