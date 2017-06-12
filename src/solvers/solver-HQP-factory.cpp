//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/solvers/solver-HQP-factory.hpp>
#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>
#include <pininvdyn/solvers/solver-HQP-eiquadprog-fast.h>

#ifdef QPOASES_FOUND
  #include <pininvdyn/solvers/solver-HQP-qpoases.hh>
#endif

namespace pininvdyn
{
  namespace solvers
  {
    
    SolverHQPBase* SolverHQPFactory::createNewSolver(const SolverHQP solverType,
                                                     const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG)
        return new SolverHQuadProg(name);
      
      if(solverType==SOLVER_HQP_EIQUADPROG_FAST)
        return new SolverHQuadProgFast(name);
      
#ifdef QPOASES_FOUND
      if(solverType==SOLVER_HQP_QPOASES)
        return new Solver_HQP_qpoases(name);
#endif
      
      assert(false && "Specified solver type not recognized");
      return NULL;
    }
    
  }
}
