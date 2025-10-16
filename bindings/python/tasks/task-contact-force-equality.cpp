//
// Copyright (c) 2021 LAAS-CNRS, University of Trento
//

#include "tsid/bindings/python/tasks/task-contact-force-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskContactForceEquality() {
  TaskContactForceEqualityPythonVisitor<tsid::tasks::TaskContactForceEquality>::
      expose("TaskContactForceEquality");
}
}  // namespace python
}  // namespace tsid
