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

#ifndef __invdyn_solvers_hqp_eiquadprog_fast_hpp__
#define __invdyn_solvers_hqp_eiquadprog_fast_hpp__

#include "tsid/deprecated.hh"
#include "tsid/solvers/solver-HQP-base.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"

namespace tsid {
namespace solvers {
/**
 * @brief
 */
class TSID_DLLAPI SolverHQuadProgFast : public SolverHQPBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Matrix Matrix;
  typedef math::Vector Vector;
  typedef math::RefVector RefVector;
  typedef math::ConstRefVector ConstRefVector;
  typedef math::ConstRefMatrix ConstRefMatrix;

  SolverHQuadProgFast(const std::string& name);

  void resize(unsigned int n, unsigned int neq, unsigned int nin) override;

  /** Solve the given Hierarchical Quadratic Program
   */
  const HQPOutput& solve(const HQPData& problemData) override;

  /** Retrieve the matrices describing a QP problem from the problem data. */
  void retrieveQPData(const HQPData& problemData,
                      const bool hessianRegularization = true) override;

  /** Return the QP data object. */
  const QPDataQuadProg getQPData() const { return m_qpData; }

  /** Get the objective value of the last solved problem. */
  double getObjectiveValue() override;

  /** Set the current maximum number of iterations performed by the solver. */
  bool setMaximumIterations(unsigned int maxIter) override;

 protected:
  void sendMsg(const std::string& s);

  // <nVars, nEqCon, 2*nIneqCon>
  eiquadprog::solvers::EiquadprogFast m_solver;

  TSID_DEPRECATED Matrix m_H;
  TSID_DEPRECATED Vector m_g;
  TSID_DEPRECATED Matrix m_CE;
  TSID_DEPRECATED Vector m_ce0;
  TSID_DEPRECATED Matrix
      m_CI;  /// twice the rows because inequality constraints are bilateral
  TSID_DEPRECATED Vector m_ci0;
  double m_objValue;
  double m_hessian_regularization;

  Eigen::VectorXi
      m_activeSet;  /// vector containing the indexes of the active inequalities
  int m_activeSetSize;

  unsigned int m_neq;  /// number of equality constraints
  unsigned int m_nin;  /// number of inequality constraints
  unsigned int m_n;    /// number of variables

  QPDataQuadProgTpl<double> m_qpData;
};
}  // namespace solvers
}  // namespace tsid

#endif  // ifndef __invdyn_solvers_hqp_eiquadprog_fast_hpp__
