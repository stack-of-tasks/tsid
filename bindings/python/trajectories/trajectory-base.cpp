#include "tsid/bindings/python/trajectories/trajectory-base.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTrajectorySample()
    {
      TrajectorySamplePythonVisitor<tsid::trajectories::TrajectorySample>::expose("TrajectorySample");
    }
  }
}