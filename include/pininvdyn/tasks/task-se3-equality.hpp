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

#ifndef __invdyn_task_se3_equality_hpp__
#define __invdyn_task_se3_equality_hpp__

#include <pininvdyn/tasks/task-motion.hpp>
#include <pininvdyn/trajectories/trajectory-base.hpp>
#include <pininvdyn/math/constraint-equality.hpp>

namespace pininvdyn
{
  namespace tasks
  {

    class TaskSE3Equality:
        public TaskMotion
    {
    public:
      typedef pininvdyn::RobotWrapper RobotWrapper;
      typedef pininvdyn::math::Index Index;
      typedef pininvdyn::trajectories::TrajectorySample TrajectorySample;
      typedef pininvdyn::math::Vector Vector;
      typedef pininvdyn::math::ConstraintEquality ConstraintEquality;
      typedef se3::Data Data;
      typedef se3::Data::Matrix6x Matrix6x;
      typedef se3::Motion Motion;
      typedef se3::SE3 SE3;

      TaskSE3Equality(const std::string & name,
                      RobotWrapper & robot,
                      const std::string & frameName);

      int dim() const;

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      void setReference(TrajectorySample & ref);

      const Vector & Kp();
      const Vector & Kd();
      void Kp(ConstRefVector Kp);
      void Kd(ConstRefVector Kp);

    protected:
      std::string m_frame_name;
      Index m_frame_id;
      Motion m_p_error, m_v_error;
      Motion m_v_ref, m_a_ref;
      SE3 m_M_ref, m_wMl;
      Vector m_Kp;
      Vector m_Kd;
      Vector m_a_des;
      Matrix6x m_J;
      ConstraintEquality m_constraint;
    };
    
  }
}

#endif // ifndef __invdyn_task_se3_equality_hpp__
