#include "tsid/bindings/python/tasks/task-se3-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTaskSE3Equality()
    {
      TaskSE3EqualityPythonVisitor<tsid::tasks::TaskSE3Equality>::expose("TaskSE3Equality");
    }
  }
}
