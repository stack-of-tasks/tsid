#include "tsid/bindings/python/trajectories/trajectory-se3.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTrajectorySE3Constant()
    {
      TrajSE3PythonVisitor<tsid::trajectories::TrajectorySE3Constant>::expose("TrajSE3Constant");
    }
  }
}
