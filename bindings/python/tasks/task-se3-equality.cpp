#include "tsid/bindings/python/tasks/task-se3-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTaskSE3()
    {
      TaskSE3PythonVisitor<tsid::tasks::TaskSE3Equality>::expose("TaskSE3");
    }
  }
}
