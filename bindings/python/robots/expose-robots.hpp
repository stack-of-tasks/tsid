#ifndef __tsid_python_expose_robot_hpp__
#define __tsid_python_expose_robot_hpp__


#include "tsid/bindings/python/robots/robot-wrapper.hpp"

namespace tsid
{
  namespace python
  {
    void exposeRobotWrapper();
    inline void exposeRobots()
    {
      exposeRobotWrapper();
    }
    
  } // namespace python
} // namespace tsid
#endif // ifndef __tsid_python_expose_robot_hpp__
