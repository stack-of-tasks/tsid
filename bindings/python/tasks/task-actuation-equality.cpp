//
// Copyright (c) 2018 CNRS
//

#include "tsid/bindings/python/tasks/task-actuation-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskActuationEquality() {
  TaskActuationEqualityPythonVisitor<
      tsid::tasks::TaskActuationEquality>::expose("TaskActuationEquality");
}
}  // namespace python
}  // namespace tsid
