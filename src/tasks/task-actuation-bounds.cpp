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

#include <tsid/tasks/task-actuation-bounds.hpp>
#include "tsid/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskActuationBounds::TaskActuationBounds(const std::string & name,
                                             RobotWrapper & robot):
      TaskActuation(name, robot),
      m_constraint(name, robot.na(), robot.na())
    {
      Vector m = Vector::Ones(robot.na());
      mask(m);
    }

    const Vector & TaskActuationBounds::mask() const
    {
      return m_mask;
    }

    void TaskActuationBounds::mask(const Vector & m)
    {
      assert(m.size()==m_robot.na());
      m_mask = m;
      const Vector::Index dim = static_cast<Vector::Index>(m.sum());
      Matrix S = Matrix::Zero(dim, m_robot.na());
      m_activeAxes.resize(dim);
      unsigned int j=0;
      for(unsigned int i=0; i<m.size(); i++)
        if(m(i)!=0.0)
        {
          assert(m(i)==1.0);
          S(j,i) = 1.0;
          m_activeAxes(j) = i;
          j++;
        }
      m_constraint.resize((unsigned int)dim, m_robot.na());
      m_constraint.setMatrix(S);
    }

    int TaskActuationBounds::dim() const
    {
      return (int)m_mask.sum();
    }

    const Vector & TaskActuationBounds::getLowerBounds() const { return m_constraint.lowerBound(); }

    const Vector & TaskActuationBounds::getUpperBounds() const { return m_constraint.upperBound(); }

    void TaskActuationBounds::setBounds(ConstRefVector lower, ConstRefVector upper)
    {
      assert(lower.size()==dim());
      assert(upper.size()==dim());
      m_constraint.setLowerBound(lower);
      m_constraint.setUpperBound(upper);
    }

    const ConstraintBase & TaskActuationBounds::getConstraint() const
    {
      return m_constraint;
    }

    const ConstraintBase & TaskActuationBounds::compute(const double ,
                                                        ConstRefVector ,
                                                        ConstRefVector ,
                                                        Data & )
    {
      return m_constraint;
    }

  }
}
