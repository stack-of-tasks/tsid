//
// Copyright (c) 2022 INRIA
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

#ifndef __solvers_proxqp_hpp__
#define __solvers_proxqp_hpp__

#include "tsid/solvers/solver-HQP-base.hpp"
#include <proxsuite/proxqp/dense/dense.hpp>
#include <proxsuite/proxqp/sparse/sparse.hpp>
#include <proxsuite/proxqp/results.hpp>

#ifdef PROFILE_PROXQP
#define START_PROFILER_PROXQP(x) START_PROFILER(x)
#define STOP_PROFILER_PROXQP(x) STOP_PROFILER(x)
#else
#define START_PROFILER_PROXQP(x)
#define STOP_PROFILER_PROXQP(x)
#endif

using namespace proxsuite;
using namespace proxsuite::proxqp;

namespace tsid {
namespace solvers {
/**
 * @brief
 */
class TSID_DLLAPI SolverProxQP : public SolverHQPBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Matrix Matrix;
  typedef math::Vector Vector;
  typedef math::RefVector RefVector;
  typedef math::ConstRefVector ConstRefVector;
  typedef math::ConstRefMatrix ConstRefMatrix;

  SolverProxQP(const std::string& name);

  void resize(unsigned int n, unsigned int neq, unsigned int nin) override;

  /** Retrieve the matrices describing a QP problem from the problem data. */
  void retrieveQPData(const HQPData& problemData,
                      const bool hessianRegularization = false) override;

  /** Return the QP data object. */
  const QPData getQPData() const { return m_qpData; }

  /** Solve the given Hierarchical Quadratic Program
   */
  const HQPOutput& solve(const HQPData& problemData) override;

  /** Get the objective value of the last solved problem. */
  double getObjectiveValue() override;

  /** Set the current maximum number of iterations performed by the solver. */
  bool setMaximumIterations(unsigned int maxIter) override;

  void setMuInequality(double muIn);
  void setMuEquality(double muEq);
  void setRho(double rho);
  void setEpsilonAbsolute(double epsAbs);
  void setEpsilonRelative(double epsRel);
  void setVerbose(bool isVerbose = false);

 protected:
  void sendMsg(const std::string& s);

  double m_objValue;
  double m_hessian_regularization;

  dense::QP<double> m_solver;

  unsigned int m_neq;  /// number of equality constraints
  unsigned int m_nin;  /// number of inequality constraints
  unsigned int m_n;    /// number of variables

  QPDataTpl<double> m_qpData;

  double m_rho;
  double m_muIn;
  double m_muEq;
  double m_epsAbs;
  double m_epsRel;
  bool m_isVerbose;
};
}  // namespace solvers
}  // namespace tsid

#endif  // ifndef __solvers_proxqp_hpp__
