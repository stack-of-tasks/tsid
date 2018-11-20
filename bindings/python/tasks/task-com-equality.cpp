#include "tsid/bindings/python/tasks/task-com-equality.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTaskCOM()
    {
      TaskCOMPythonVisitor<tsid::tasks::TaskComEquality>::expose("TaskCOM");
    }
  }
}
