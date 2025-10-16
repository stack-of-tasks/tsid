//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/tasks/task-com-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskComEquality() {
  TaskCOMEqualityPythonVisitor<tsid::tasks::TaskComEquality>::expose(
      "TaskComEquality");
}
}  // namespace python
}  // namespace tsid
