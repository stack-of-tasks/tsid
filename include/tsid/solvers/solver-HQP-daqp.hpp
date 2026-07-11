//
// Copyright (c) 2026 CNRS
//

#ifndef __invdyn_solvers_hqp_daqp_hpp__
#define __invdyn_solvers_hqp_daqp_hpp__

#include <memory>

#include "tsid/solvers/solver-HQP-base.hpp"

namespace tsid {
namespace solvers {

/** DAQP-backed solver supporting an arbitrary number of HQP levels. */
class TSID_DLLAPI SolverHQPDAQP : public SolverHQPBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>
      RowMajorMatrix;
  typedef math::Vector Vector;
  typedef math::VectorXi VectorXi;

  explicit SolverHQPDAQP(const std::string& name);
  ~SolverHQPDAQP() override;

  void resize(unsigned int n, unsigned int neq, unsigned int nin) override;
  const HQPOutput& solve(const HQPData& problemData) override;
  void retrieveQPData(const HQPData& problemData,
                      bool hessianRegularization = true) override;
  double getObjectiveValue() override;
  bool setMaximumIterations(unsigned int maxIter) override;
  bool setMaximumTime(double seconds) override;

  /**
   * @brief Skip matrix comparisons and reuse the DAQP factorization between
   * solves. The caller must disable this before changing any constraint or
   * task matrix, task weight, hierarchy structure, or hard bound. Task
   * targets may still change. The default is false, which refreshes all
   * numerical DAQP data on every solve.
   */
  void setAssumeMatricesUnchanged(bool unchanged);
  bool getAssumeMatricesUnchanged() const {
    return m_assumeMatricesUnchanged;
  }

  const RowMajorMatrix& constraintMatrix() const { return m_A; }
  const Vector& lowerBounds() const { return m_lower; }
  const Vector& upperBounds() const { return m_upper; }
  const VectorXi& constraintSense() const { return m_sense; }
  const VectorXi& breakPoints() const { return m_breakPoints; }

 protected:
  struct WorkspaceHolder;

  void resetWorkspace();
  unsigned int maxRowsInLevel() const;

  RowMajorMatrix m_A;
  Vector m_lower;
  Vector m_upper;
  VectorXi m_sense;
  VectorXi m_breakPoints;
  Vector m_rowScales;
  RowMajorMatrix m_H;
  Vector m_f;
  double m_objValue;
  unsigned int m_n;
  unsigned int m_totalRows;
  bool m_useConventionalQP;
  bool m_hasSolution;
  std::unique_ptr<WorkspaceHolder> m_workspace;
  unsigned int m_workspaceRows;
  unsigned int m_workspaceLevels;
  unsigned int m_workspaceMaxSoftRows;
  bool m_workspaceUsesConventionalQP;
  Vector m_daqpLambda;
  bool m_assumeMatricesUnchanged;
};

}  // namespace solvers
}  // namespace tsid

#endif  // __invdyn_solvers_hqp_daqp_hpp__
