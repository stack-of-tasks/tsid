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

#ifndef __invdyn_task_base_hpp__
#define __invdyn_task_base_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/robots/fwd.hpp"
#include "tsid/math/constraint-base.hpp"

#include <pinocchio/multibody/fwd.hpp>

namespace tsid
{
  namespace tasks
  {
    
    ///
    /// \brief Base template of a Task.
    /// Each class is defined according to a constant model of a robot.
    ///
    class TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::ConstraintBase ConstraintBase;
      typedef math::ConstRefVector ConstRefVector;
      typedef pinocchio::Data Data;
      typedef robots::RobotWrapper RobotWrapper;

      TaskBase(const std::string & name,
               RobotWrapper & robot);

      const std::string & name() const;

      void name(const std::string & name);
      
      /// \brief Return the dimension of the task.
      /// \info should be overloaded in the child class.
      virtual int dim() const = 0;

      virtual const ConstraintBase & compute(const double t,
                                             ConstRefVector q,
                                             ConstRefVector v,
                                             const Data & data) = 0;

      virtual const ConstraintBase & getConstraint() const = 0;
      
    protected:
      std::string m_name;
      
      /// \brief Reference on the robot model.
      RobotWrapper & m_robot;
    };
    
  }
}

#endif // ifndef __invdyn_task_base_hpp__
