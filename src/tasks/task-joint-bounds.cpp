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

#include <tsid/tasks/task-joint-bounds.hpp>
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskJointBounds::TaskJointBounds(const std::string & name,
                                     RobotWrapper & robot,
                                     double dt):
      TaskMotion(name, robot),
      m_constraint(name, robot.nv()),
      m_dt(dt),
      m_nv(robot.nv()),
      m_na(robot.na())
    {
      assert(dt>0.0);
      m_v_lb = -1e10*Vector::Ones(m_na);
      m_v_ub = +1e10*Vector::Ones(m_na);
      m_a_lb = -1e10*Vector::Ones(m_na);
      m_a_ub = +1e10*Vector::Ones(m_na);
      m_ddq_max_due_to_vel.setZero(m_na);
      m_ddq_max_due_to_vel.setZero(m_na);

      int offset = m_nv-m_na;
      for(int i=0; i<offset; i++)
      {
        m_constraint.upperBound()(i) = 1e10;
        m_constraint.lowerBound()(i) = -1e10;
      }
    }

    int TaskJointBounds::dim() const
    { return m_nv; }

    const Vector & TaskJointBounds::getAccelerationLowerBounds() const
    { return m_a_lb; }

    const Vector & TaskJointBounds::getAccelerationUpperBounds() const
    { return m_a_ub; }

    const Vector & TaskJointBounds::getVelocityLowerBounds() const
    { return m_v_lb; }

    const Vector & TaskJointBounds::getVelocityUpperBounds() const
    { return m_v_ub; }

    void TaskJointBounds::setTimeStep(double dt)
    {
      assert(dt>0);
      m_dt = dt;
    }

    void TaskJointBounds::setVelocityBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_v_lb = lower;
      m_v_ub = upper;
    }

    void TaskJointBounds::setAccelerationBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==m_na);
      assert(upper.size()==m_na);
      m_a_lb = lower;
      m_a_ub = upper;
    }

    const ConstraintBase & TaskJointBounds::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskJointBounds::compute(const double ,
                                                    ConstRefVector ,
                                                    ConstRefVector v,
                                                    Data & )
    {
      // compute min/max joint acc imposed by velocity limits
      m_ddq_max_due_to_vel = (m_v_ub - v.tail(m_na))/m_dt;
      m_ddq_min_due_to_vel = (m_v_lb - v.tail(m_na))/m_dt;

      // take most conservative limit between vel and acc
      int offset = m_nv-m_na;
      for(int i=0; i<m_na; i++)
      {
        m_constraint.upperBound()(offset+i) = std::min(m_ddq_max_due_to_vel(i), m_a_ub(i));
        m_constraint.lowerBound()(offset+i) = std::max(m_ddq_min_due_to_vel(i), m_a_lb(i));
      }
      return m_constraint;
    }

  }
}
