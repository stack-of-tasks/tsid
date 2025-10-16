//
// Copyright (c) 2022 INRIA
//

#include "tsid/bindings/python/tasks/task-joint-posVelAcc-bounds.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskJointPosVelAccBounds() {
  TaskJointPosVelAccBoundsPythonVisitor<tsid::tasks::TaskJointPosVelAccBounds>::
      expose("TaskJointPosVelAccBounds");
}
}  // namespace python
}  // namespace tsid
