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

#ifndef __invdyn_solvers_fwd_hpp__
#define __invdyn_solvers_fwd_hpp__

#include "pininvdyn/config.hh"

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8

namespace pininvdyn
{
  namespace solvers
  {
    
    /**
     * Available HQP solvers.
     */
    enum PININVDYN_DLLAPI SolverHQP
    {
      SOLVER_HQP_EIQUADPROG = 0,
      SOLVER_HQP_EIQUADPROG_FAST = 1,
      SOLVER_HQP_EIQUADPROG_RT = 2
#ifdef QPOASES_FOUND
      ,SOLVER_HQP_OASES = 3
#endif
    };
    
    
    /**
     * Possible states of an HQP solver.
     */
    enum PININVDYN_DLLAPI HQP_status
    {
      HQP_STATUS_UNKNOWN=-1,
      HQP_STATUS_OPTIMAL=0,
      HQP_STATUS_INFEASIBLE=1,
      HQP_STATUS_UNBOUNDED=2,
      HQP_STATUS_MAX_ITER_REACHED=3,
      HQP_STATUS_ERROR=4
    };
    
    class PININVDYN_DLLAPI Solver_HQP_base;
    
    template<int nVars, int nEqCon, int nIneqCon>
    class PININVDYN_DLLAPI Solver_HQP_eiquadprog_rt;
    
    
  }
}

#endif // ifndef __invdyn_solvers_fwd_hpp__
