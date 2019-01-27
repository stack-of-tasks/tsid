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

#include "tsid/tasks/task-base.hpp"

namespace tsid
{
  namespace tasks
  {
    TaskBase::TaskBase(const std::string & name,
                       RobotWrapper & robot):
      m_name(name),
      m_robot(robot)
    {}

    const std::string & TaskBase::name() const
    {
      return m_name;
    }

    void TaskBase::name(const std::string & name)
    {
      m_name = name;
    }
    
  }
}
