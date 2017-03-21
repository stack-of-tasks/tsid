//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_task_joint_posture_hpp__
#define __invdyn_task_joint_posture_hpp__

#include <pininvdyn/tasks/task-motion.hpp>
#include <pininvdyn/trajectories/trajectory-base.hpp>
#include <pininvdyn/math/constraint-equality.hpp>

namespace pininvdyn
{
  namespace tasks
  {

    class TaskJointPosture:
        public TaskMotion
    {
    public:
      typedef pininvdyn::RobotWrapper RobotWrapper;
      typedef pininvdyn::math::Index Index;
      typedef pininvdyn::trajectories::TrajectorySample TrajectorySample;
      typedef pininvdyn::math::Vector Vector;
      typedef pininvdyn::math::VectorXi VectorXi;
      typedef pininvdyn::math::ConstraintEquality ConstraintEquality;
      typedef se3::Data Data;

      TaskJointPosture(const std::string & name,
                      RobotWrapper & robot);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      void setReference(const TrajectorySample & ref);

      const Vector & mask() const;
      bool mask(const Vector & mask);

      const Vector & position_error() const;
      const Vector & velocity_error() const;
      const Vector & position() const;
      const Vector & velocity() const;
      const Vector & position_ref() const;
      const Vector & velocity_ref() const;

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      Vector m_Kp;
      Vector m_Kd;
      Vector m_p_error, m_v_error;
      Vector m_p, m_v;
      Vector m_mask;
      VectorXi m_activeAxes;
      TrajectorySample m_ref;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif // ifndef __invdyn_task_joint_posture_hpp__
