#include "tsid/bindings/python/trajectories/trajectory-base.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTrajectorySample()
    {
      TrajSamplePythonVisitor<tsid::trajectories::TrajectorySample>::expose("TrajSample");
    }
  }
}
