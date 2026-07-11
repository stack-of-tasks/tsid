//
// Copyright (c) 2026 CNRS
//

#ifndef __invdyn_solvers_hqp_daqp_hpp__
#define __invdyn_solvers_hqp_daqp_hpp__

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

  void resize(unsigned int n, unsigned int neq, unsigned int nin) override;
  const HQPOutput& solve(const HQPData& problemData) override;
  void retrieveQPData(const HQPData& problemData,
                      bool hessianRegularization = true) override;
  double getObjectiveValue() override;
  bool setMaximumIterations(unsigned int maxIter) override;
  bool setMaximumTime(double seconds) override;

  const RowMajorMatrix& constraintMatrix() const { return m_A; }
  const Vector& lowerBounds() const { return m_lower; }
  const Vector& upperBounds() const { return m_upper; }
  const VectorXi& constraintSense() const { return m_sense; }
  const VectorXi& breakPoints() const { return m_breakPoints; }

 protected:
  RowMajorMatrix m_A;
  Vector m_lower;
  Vector m_upper;
  VectorXi m_sense;
  VectorXi m_breakPoints;
  Vector m_rowScales;
  double m_objValue;
  unsigned int m_n;
  bool m_hasSolution;
};

}  // namespace solvers
}  // namespace tsid

#endif  // __invdyn_solvers_hqp_daqp_hpp__
