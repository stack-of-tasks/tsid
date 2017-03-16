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

#include <pininvdyn/tasks/task-joint-posture.hpp>

namespace pininvdyn
{
  namespace tasks
  {
    using namespace pininvdyn::math;
    using namespace se3;

    TaskJointPosture::TaskJointPosture(const std::string & name,
                                     RobotWrapper & robot):
      TaskMotion(name, robot),
      m_constraint(name, robot.nv()-6, robot.nv())
    {
      m_Kp.setZero(robot.nv()-6);
      m_Kd.setZero(robot.nv()-6);
      Matrix S(robot.nv()-6, robot.nv());
      S.leftCols(6).setZero();
      S.rightCols(robot.nv()-6).setIdentity();
      m_constraint.setMatrix(S);
    }

    int TaskJointPosture::dim() const
    {
      //return self._mask.sum ()
      return m_robot.nv()-6;
    }

    const Vector & TaskJointPosture::Kp(){ return m_Kp; }

    const Vector & TaskJointPosture::Kd(){ return m_Kd; }

    void TaskJointPosture::Kp(ConstRefVector Kp)
    {
      assert(Kp.size()==m_robot.nv()-6);
      m_Kp = Kp;
    }

    void TaskJointPosture::Kd(ConstRefVector Kd)
    {
      assert(Kd.size()==m_robot.nv()-6);
      m_Kd = Kd;
    }

    void TaskJointPosture::setReference(const TrajectorySample & ref)
    {
      assert(ref.pos.size()==m_robot.nv()-6);
      assert(ref.vel.size()==m_robot.nv()-6);
      assert(ref.acc.size()==m_robot.nv()-6);
      m_ref = ref;
    }

    const Vector & TaskJointPosture::position_error() const
    {
      return m_p_error;
    }

    const Vector & TaskJointPosture::velocity_error() const
    {
      return m_v_error;
    }

    const Vector & TaskJointPosture::position() const
    {
      return m_p;
    }

    const Vector & TaskJointPosture::velocity() const
    {
      return m_v;
    }

    const Vector & TaskJointPosture::position_ref() const
    {
      return m_ref.pos;
    }

    const Vector & TaskJointPosture::velocity_ref() const
    {
      return m_ref.vel;
    }

    const ConstraintBase & TaskJointPosture::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskJointPosture::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    Data & data)
    {
      // Compute errors
      m_p = q.tail(m_robot.nv()-6);
      m_v = v.tail(m_robot.nv()-6);
      m_p_error = m_p - m_ref.pos;
      m_v_error = m_v - m_ref.vel;
      Vector m_a_des = - m_Kp.cwiseProduct(m_p_error)
                       - m_Kd.cwiseProduct(m_v_error)
                       + m_ref.acc;

      m_constraint.setVector(m_a_des);
      return m_constraint;
    }
    
  }
}
