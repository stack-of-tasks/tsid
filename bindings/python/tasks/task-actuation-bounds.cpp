//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/tasks/task-actuation-bounds.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskActuationBounds() {
  TaskActuationBoundsPythonVisitor<tsid::tasks::TaskActuationBounds>::expose(
      "TaskActuationBounds");
}
}  // namespace python
}  // namespace tsid
