#ifndef __tsid_python_expose_tasks_hpp__
#define __tsid_python_expose_tasks_hpp__

#include "tsid/bindings/python/tasks/task-com-equality.hpp"
#include "tsid/bindings/python/tasks/task-se3-equality.hpp"
#include "tsid/bindings/python/tasks/task-joint-posture.hpp"


namespace tsid
{
  namespace python
  {
    void exposeTaskComEquality();
    void exposeTaskSE3Equality();
    void exposeTaskJointPosture();

    inline void exposeTasks()
    {
      exposeTaskComEquality();
      exposeTaskSE3Equality();
      exposeTaskJointPosture();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_tasks_hpp__
