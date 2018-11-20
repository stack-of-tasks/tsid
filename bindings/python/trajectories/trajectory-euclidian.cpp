#include "tsid/bindings/python/trajectories/trajectory-euclidian.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"

namespace tsid
{
  namespace python
  {
    void exposeTrajectoryEuclidianConstant()
    {
      TrajEuclidianPythonVisitor<tsid::trajectories::TrajectoryEuclidianConstant>::expose("TrajEuclidianConstant");
    }
  }
}
