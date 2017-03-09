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

#include <pininvdyn/solvers/solver-HQP-base.hpp>
#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>
#include <iostream>

#ifdef QPOASES_FOUND
#include <pininvdyn/solvers/solver-HQP-qpoases.hh>
#endif


using namespace pininvdyn;
using namespace std;

namespace pininvdyn
{
  namespace solvers
  {
          
    Solver_HQP_base* Solver_HQP_base::getNewSolver(SolverHQP solverType, const std::string & name)
    {
      if(solverType==SOLVER_HQP_EIQUADPROG)
        return new Solver_HQP_eiquadprog(name);

    #ifdef QPOASES_FOUND
      if(solverType==SOLVER_HQP_QPOASES)
        return new Solver_HQP_qpoases(name);
    #endif

      assert(false && "Specified solver type not recognized");
      return NULL;
    }

    Solver_HQP_base::Solver_HQP_base(const std::string & name)
    {
      m_name = name;
      m_maxIter = 1000;
      m_maxTime = 100.0;
      m_useWarmStart = true;
    }

    bool Solver_HQP_base::setMaximumIterations(unsigned int maxIter)
    {
      if(maxIter==0)
        return false;
      m_maxIter = maxIter;
      return true;
    }

    bool Solver_HQP_base::setMaximumTime(double seconds)
    {
      if(seconds<=0.0)
        return false;
      m_maxTime = seconds;
      return true;
    }
    
  }
}
