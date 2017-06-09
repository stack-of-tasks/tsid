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

#ifndef __invdyn_task_actuation_hpp__
#define __invdyn_task_actuation_hpp__

#include <pininvdyn/tasks/task-base.hpp>

namespace pininvdyn
{
  namespace tasks
  {
    class TaskActuation : public TaskBase
    {
    public:

      TaskActuation(const std::string & name,
                    RobotWrapper & robot);
    };
  }
}

#endif // ifndef __invdyn_task_actuation_hpp__
