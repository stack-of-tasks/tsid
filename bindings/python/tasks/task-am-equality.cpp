//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/tasks/task-am-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskAMEquality() {
  TaskAMEqualityPythonVisitor<tsid::tasks::TaskAMEquality>::expose(
      "TaskAMEquality");
}
}  // namespace python
}  // namespace tsid
