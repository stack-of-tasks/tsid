//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_solvers_hqp_eiquadprog_rt_hpp__
#define __invdyn_solvers_hqp_eiquadprog_rt_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/solvers/fwd.hpp"
#include "tsid/solvers/solver-HQP-base.hpp"

#include "tsid/solvers/eiquadprog-rt.hpp"

namespace tsid
{
  namespace solvers
  {
    /**
     * @brief
     */
    template<int nVars, int nEqCon, int nIneqCon>
    class TSID_DLLAPI SolverHQuadProgRT : public SolverHQPBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Matrix Matrix;
      typedef math::Vector Vector;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::ConstRefMatrix ConstRefMatrix;

      SolverHQuadProgRT(const std::string & name);

      void resize(unsigned int n, unsigned int neq, unsigned int nin);

      /** Solve the given Hierarchical Quadratic Program
       */
      const HQPOutput & solve(const HQPData & problemData);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

      /** Set the current maximum number of iterations performed by the solver. */
      bool setMaximumIterations(unsigned int maxIter);

    protected:

      void sendMsg(const std::string & s);

      RtEiquadprog<nVars, nEqCon, 2*nIneqCon> m_solver;

      typename RtMatrixX<nVars, nVars>::d m_H;
      typename RtVectorX<nVars>::d m_g;
      typename RtMatrixX<nEqCon, nVars>::d m_CE;
      typename RtVectorX<nEqCon>::d m_ce0;
      typename RtMatrixX<2*nIneqCon, nVars>::d m_CI;  /// twice the rows because inequality constraints are bilateral
      typename RtVectorX<2*nIneqCon>::d m_ci0;
      double m_objValue;

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
      int m_activeSetSize;

//      Eigen::FullPivLU<template RtMatrixX<nEqCon, nVars> > m_CE_lu;
      // ColPivHouseholderQR

      int m_neq;  /// number of equality constraints
      int m_nin;  /// number of inequality constraints
      int m_n;    /// number of variables
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_rt_hpp__
