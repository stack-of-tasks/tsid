//
// Copyright (c) 2023 MIPT
//

#include "tsid/bindings/python/tasks/task-two-frames-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid {
namespace python {
void exposeTaskTwoFramesEquality() {
  TaskTwoFramesEqualityPythonVisitor<
      tsid::tasks::TaskTwoFramesEquality>::expose("TaskTwoFramesEquality");
}
}  // namespace python
}  // namespace tsid
