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

#ifndef __invdyn_solvers_hqp_eiquadprog_hpp__
#define __invdyn_solvers_hqp_eiquadprog_hpp__

#include <pininvdyn/solvers/solver-HQP-base.hpp>

namespace pininvdyn
{
  namespace solvers
  {
    /**
     * @brief Abstract interface for a Quadratic Program (HQP) solver.
     */
    class PININVDYN_DLLAPI Solver_HQP_eiquadprog:
        public Solver_HQP_base
    {
    public:
      typedef pininvdyn::math::RefVector RefVector;
      typedef pininvdyn::math::ConstRefVector ConstRefVector;
      typedef pininvdyn::math::ConstRefMatrix ConstRefMatrix;

      Solver_HQP_eiquadprog(const std::string & name);

      /** Solve the quadratic program
       *  minimize    ||A x + a||^2
       *  subject to  Aeq x = b
       *              Alb <= Ain x <= Aub
       *              lb <= x <= ub
       */
      HqpOutput solve(const HqpData & problemData,
                      RefVector sol);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

    protected:
      HqpOutput m_output;
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_hpp__
