//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_task_com_equality_hpp__
#define __invdyn_task_com_equality_hpp__

#include <pininvdyn/tasks/task-motion.hpp>
#include <pininvdyn/trajectories/trajectory-base.hpp>
#include <pininvdyn/math/constraint-equality.hpp>

namespace pininvdyn
{
  namespace tasks
  {

    class TaskComEquality : public TaskMotion
    {
    public:
      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef se3::Data Data;
      typedef se3::Data::Matrix6x Matrix6x;
      typedef se3::Motion Motion;
      typedef se3::SE3 SE3;

      TaskComEquality(const std::string & name,
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     const Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);
      const TrajectorySample & getReference() const;

      const Vector & getDesiredAcceleration() const;
      Vector getAcceleration(ConstRefVector dv) const;

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector3 & Kp();
      const Vector3 & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector3 m_Kp;
      Vector3 m_Kd;
      Vector3 m_p_error, m_v_error;
      Vector3 m_a_des;
      Vector3 m_drift;
      Vector m_p_error_vec, m_v_error_vec;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif // ifndef __invdyn_task_com_equality_hpp__
