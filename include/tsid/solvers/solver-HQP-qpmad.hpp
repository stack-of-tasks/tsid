//
// Copyright (c) 2022 Inria
//

#ifndef __invdyn_solvers_hqp_qpmad_hpp__
#define __invdyn_solvers_hqp_qpmad_hpp__

#include <tsid/solvers/solver-HQP-base.hpp>

#include <qpmad/solver.h>

namespace tsid {
namespace solvers {
/**
 * @brief Implementation of Quadratic Program (HQP) solver using qpmad.
 */
class TSID_DLLAPI SolverHQpmad : public SolverHQPBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Matrix Matrix;
  typedef math::Vector Vector;
  typedef math::RefVector RefVector;
  typedef math::ConstRefVector ConstRefVector;
  typedef math::ConstRefMatrix ConstRefMatrix;

  typedef qpmad::SolverParameters Settings;

  SolverHQpmad(const std::string& name);

  void resize(unsigned int n, unsigned int neq, unsigned int nin) override;

  /** Solve the given Hierarchical Quadratic Program
   */
  const HQPOutput& solve(const HQPData& problemData) override;

  /** Retrieve the matrices describing a QP problem from the problem data. */
  void retrieveQPData(const HQPData& problemData,
                      const bool hessianRegularization = true) override;

  /** Get the objective value of the last solved problem. */
  double getObjectiveValue() override;

  Settings& settings() { return m_settings; }

 protected:
  void sendMsg(const std::string& s);

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
  unsigned int m_n;   /// number of variables
};
}  // namespace solvers
}  // namespace tsid

#endif  // ifndef __invdyn_solvers_hqp_qpmad_hpp__
