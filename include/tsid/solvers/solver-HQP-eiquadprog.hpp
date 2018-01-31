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

#ifndef __invdyn_solvers_hqp_eiquadprog_hpp__
#define __invdyn_solvers_hqp_eiquadprog_hpp__

#include <tsid/solvers/solver-HQP-base.hpp>

namespace tsid
{
  namespace solvers
  {
    /**
     * @brief Abstract interface for a Quadratic Program (HQP) solver.
     */
    class TSID_DLLAPI SolverHQuadProg:
        public SolverHQPBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Matrix Matrix;
      typedef math::Vector Vector;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::ConstRefMatrix ConstRefMatrix;

      SolverHQuadProg(const std::string & name);

      void resize(unsigned int n, unsigned int neq, unsigned int nin);

      /** Solve the given Hierarchical Quadratic Program
       */
      const HQPOutput & solve(const HQPData & problemData);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

    protected:

      void sendMsg(const std::string & s);

      Matrix m_H;
      Vector m_g;
      Matrix m_CE;
      Vector m_ce0;
      Matrix m_CI;
      Vector m_ci0;
      double m_objValue;

      double m_hessian_regularization;

      Eigen::VectorXi m_activeSet;  /// vector containing the indexes of the active inequalities
      tsid::math::Index m_activeSetSize;

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS
//      Eigen::FullPivLU<Matrix>                        m_CE_dec;
//	  Eigen::ColPivHouseholderQR<Matrix>              m_CE_dec; // fast, but difficult to retrieve null space basis
//      Eigen::FullPivHouseholderQR<Matrix>             m_CE_dec; // doc says it is slow 
      Eigen::CompleteOrthogonalDecomposition<Matrix>  m_CE_dec; // available from Eigen 3.3.0, 40 us for decomposition, 40 us to get null space basis, 40 us to project Hessian
//      Eigen::JacobiSVD<Matrix, Eigen::HouseholderQRPreconditioner> m_CE_dec; // too slow
      Matrix m_ZT_H_Z;
      Matrix m_CI_Z;
#endif

      unsigned int m_neq;  /// number of equality constraints
      unsigned int m_nin;  /// number of inequality constraints
      unsigned int m_n;    /// number of variables
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_eiquadprog_hpp__
