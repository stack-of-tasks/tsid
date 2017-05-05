//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
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

#ifndef __invdyn_solvers_hqp_eiquadprog_fast_h__
#define __invdyn_solvers_hqp_eiquadprog_fast_h__

#include <pininvdyn/solvers/solver-HQP-base.h>
#include <pininvdyn/solvers/eiquadprog_fast.h>

#define DEFAULT_HESSIAN_REGULARIZATION 1e-8

namespace pininvdyn
{
  namespace solvers
  {
    /**
     * @brief
     */
    class PININVDYN_DLLAPI Solver_HQP_eiquadprog_fast:
        public Solver_HQP_base
    {
    public:
      typedef pininvdyn::math::Matrix Matrix;
      typedef pininvdyn::math::Vector Vector;
      typedef pininvdyn::math::RefVector RefVector;
      typedef pininvdyn::math::ConstRefVector ConstRefVector;
      typedef pininvdyn::math::ConstRefMatrix ConstRefMatrix;

      Solver_HQP_eiquadprog_fast(const std::string & name);

      void resize(unsigned int n, unsigned int neq, unsigned int nin);

      /** Solve the given Hierarchical Quadratic Program
       */
      const HqpOutput & solve(const HqpData & problemData);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

      /** Set the current maximum number of iterations performed by the solver. */
      bool setMaximumIterations(unsigned int maxIter);

    protected:

      void sendMsg(const std::string & s);

      EiquadprogFast m_solver; // <nVars, nEqCon, 2*nIneqCon>

      Matrix m_H;
      Vector m_g;
      Matrix m_CE;
      Vector m_ce0;
      Matrix m_CI;  /// twice the rows because inequality constraints are bilateral
      Vector m_ci0;
      double m_objValue;

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
      int m_activeSetSize;

      int m_neq;  /// number of equality constraints
      int m_nin;  /// number of inequality constraints
      int m_n;    /// number of variables
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_fast_h__
