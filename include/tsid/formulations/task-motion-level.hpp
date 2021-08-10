//
// Copyright (c) 2021 University of Trento
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

#ifndef __tsid_task_motion_level_hpp__
#define __tsid_task_motion_level_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/tasks/task-motion.hpp"

namespace tsid
{
  struct TaskLevelMotion
  {
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    
    tasks::TaskMotion & task;
    std::shared_ptr<math::ConstraintBase> constraint;
    unsigned int priority;

    TaskLevelMotion(tasks::TaskMotion & task, unsigned int priority);
  };

}

#endif // ifndef __tsid_task_motion_level_hpp__
