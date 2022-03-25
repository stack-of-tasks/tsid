//
// Copyright (c) 2022 Inria
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

#ifndef __invdyn_solvers_hqp_qpmad_hpp__
#define __invdyn_solvers_hqp_qpmad_hpp__

#include <tsid/solvers/solver-HQP-base.hpp>

#include <qpmad/solver.h>

namespace tsid
{
  namespace solvers
  {
    /**
     * @brief Implementation of Quadratic Program (HQP) solver using qpmad.
     */
    class TSID_DLLAPI SolverHQpmad:
        public SolverHQPBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW
      
      typedef math::Matrix Matrix;
      typedef math::Vector Vector;
      typedef math::RefVector RefVector;
      typedef math::ConstRefVector ConstRefVector;
      typedef math::ConstRefMatrix ConstRefMatrix;

      typedef qpmad::SolverParameters Settings;

      SolverHQpmad(const std::string & name);

      void resize(unsigned int n, unsigned int neq, unsigned int nin);

      /** Solve the given Hierarchical Quadratic Program
       */
      const HQPOutput & solve(const HQPData & problemData);

      /** Get the objective value of the last solved problem. */
      double getObjectiveValue();

      Settings& settings() { return m_settings; }

    protected:

      void sendMsg(const std::string & s);

      qpmad::Solver m_solver;
      Settings m_settings;

      bool m_has_bounds;
      
      Matrix m_H;   // hessian matrix
      Vector m_g;   // gradient vector
      Vector m_lb;  // gradient vector
      Vector m_ub;  // gradient vector
      Matrix m_C;   // constraint matrix
      Vector m_cl;  // constraints lower bound
      Vector m_cu;  // constraints upper bound

      double m_hessian_regularization;

      unsigned int m_nc;  /// number of equality-inequality constraints
      unsigned int m_n;    /// number of variables
    };
  }
}

#endif // ifndef __invdyn_solvers_hqp_qpmad_hpp__
