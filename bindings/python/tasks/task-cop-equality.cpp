//
// Copyright (c) 2021 University of Trento
//

#include "tsid/bindings/python/tasks/task-cop-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskCopEquality() {
  TaskCOPEqualityPythonVisitor<tsid::tasks::TaskCopEquality>::expose(
      "TaskCopEquality");
}
}  // namespace python
}  // namespace tsid
