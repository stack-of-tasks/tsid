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

#ifndef __invdyn_task_motion_hpp__
#define __invdyn_task_motion_hpp__

#include "tsid/tasks/task-base.hpp"
#include "tsid/trajectories/trajectory-base.hpp"

namespace tsid
{
  namespace tasks
  {
    class TaskMotion : public TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Vector Vector;
      typedef trajectories::TrajectorySample TrajectorySample;

      TaskMotion(const std::string & name,
                 RobotWrapper & robot);

      virtual const TrajectorySample & getReference() const = 0;

      virtual const Vector & getDesiredAcceleration() const = 0;

      virtual Vector getAcceleration(ConstRefVector dv) const = 0;

      virtual const Vector & position_error() const = 0;
      virtual const Vector & velocity_error() const = 0;
      virtual const Vector & position() const = 0;
      virtual const Vector & velocity() const = 0;
      virtual const Vector & position_ref() const = 0;
      virtual const Vector & velocity_ref() const = 0;

      virtual void setMask(math::ConstRefVector mask);
      virtual bool hasMask();

    protected:
      math::Vector m_mask;
    };
  }
}

#endif // ifndef __invdyn_task_motion_hpp__
