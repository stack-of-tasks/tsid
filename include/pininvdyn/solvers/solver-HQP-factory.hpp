//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_solvers_hqp_factory_hpp__
#define __invdyn_solvers_hqp_factory_hpp__

#include <pininvdyn/solvers/solver-HQP-base.hpp>
#include <pininvdyn/solvers/solver-HQP-eiquadprog-rt.h>


namespace pininvdyn
{
  namespace solvers
  {
    
    struct SolverHQPFactory
    {
      
      /**
       * @brief Create a new HQP solver of the specified type.
       *
       * @param solverType Type of HQP solver.
       * @param name Name of the solver.
       *
       * @return A pointer to the new solver.
       */
      static Solver_HQP_base * createNewSolver(const SolverHQP solverType,
                                               const std::string & name);
      
      /**
       * @brief Create a new HQP solver of the specified type.
       *
       * @param solverType Type of HQP solver.
       * @param name Name of the solver.
       *
       * @return A pointer to the new solver.
       */
      template<int nVars, int nEqCon, int nIneqCon>
      static Solver_HQP_base* createNewSolver(const SolverHQP solverType,
                                              const std::string & name);
    };
    
  }
}

#endif // ifndef __invdyn_solvers_hqp_factory_hpp__
