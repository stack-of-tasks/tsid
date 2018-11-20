#ifndef __tsid_python_expose_tasks_hpp__
#define __tsid_python_expose_tasks_hpp__

#include "tsid/bindings/python/tasks/task-com-equality.hpp"
#include "tsid/bindings/python/tasks/task-se3-equality.hpp"
#include "tsid/bindings/python/tasks/task-joint-posture.hpp"


namespace tsid
{
  namespace python
  {
    void exposeTaskCOM();
    void exposeTaskSE3();
    void exposeTaskJointPosture();

    inline void exposeTasks()
    {
      exposeTaskCOM();
      exposeTaskSE3();
      exposeTaskJointPosture();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_tasks_hpp__
