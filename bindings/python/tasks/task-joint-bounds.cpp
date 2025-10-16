//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/tasks/task-joint-bounds.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskJointBounds() {
  TaskJointBoundsPythonVisitor<tsid::tasks::TaskJointBounds>::expose(
      "TaskJointBounds");
}
}  // namespace python
}  // namespace tsid
