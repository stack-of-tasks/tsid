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

#ifndef __invdyn_tasks_task_base_hpp__
#define __invdyn_tasks_task_base_hpp__

namespace pininvdyn
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
      typedef double Scalar;
      typedef se3::Model Model;
      
      TaskBase(const Model & model)
      : m_model(model)
      {}
      
      ///
      /// \brief Return the dimension of the task.
      /// \info should be overloaded in the child class.
      virtual int dim() const = 0;
      virtual int nq() const = 0;
      virtual int nv() const = 0;
      
      ///
      /// \brief Accessor to model.
      ///
      /// \returns a const reference on the model.
      ///
      const Model & model() const { return m_model; };
      
    protected:
      
      
      ///
      /// \brief Reference on the robot model.
      ///
      const Model & m_model;
    };
    
  }
}

#endif // ifndef __invdyn_tasks_task_base_hpp__