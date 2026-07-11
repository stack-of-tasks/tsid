//
// Copyright (c) 2026 CNRS
//

#include "tsid/solvers/solver-HQP-daqp.hpp"

#if __has_include(<daqp/api.h>)
#include <daqp/api.h>
#else
#include <api.h>
#endif

#include <cmath>

#include <pinocchio/macros.hpp>

namespace tsid {
namespace solvers {

namespace {

unsigned int constraintRows(const ConstraintLevel& level) {
  unsigned int rows = 0;
  for (const auto& item : level) {
    if (item.first != 0.) rows += item.second->rows();
  }
  return rows;
}

}  // namespace

SolverHQPDAQP::SolverHQPDAQP(const std::string& name)
    : SolverHQPBase(name), m_objValue(0.), m_n(0), m_hasSolution(false) {}

void SolverHQPDAQP::resize(unsigned int n, unsigned int, unsigned int) {
  if (n != m_n) m_hasSolution = false;
  m_n = n;
  m_output.x.resize(n);
}

void SolverHQPDAQP::retrieveQPData(const HQPData& problemData, bool) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(!problemData.empty(),
                                 "HQP data must contain at least one level");

  unsigned int n = 0;
  unsigned int rows = 0;
  for (const ConstraintLevel& level : problemData) {
    rows += constraintRows(level);
    for (const auto& item : level) {
      if (item.second->rows() == 0) continue;
      if (n == 0) n = item.second->cols();
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          item.second->cols() == n,
          "All HQP constraints must have the same number of columns");
      PINOCCHIO_CHECK_INPUT_ARGUMENT(
          item.first >= 0., "HQP constraint weights must be nonnegative");
    }
  }
  if (n == 0) n = m_n;
  PINOCCHIO_CHECK_INPUT_ARGUMENT(n > 0,
                                 "Cannot infer the number of HQP variables");

  resize(n, 0, rows);
  m_A.resize(rows, n);
  m_lower.resize(rows);
  m_upper.resize(rows);
  m_sense.setZero(rows);
  m_rowScales.setZero(rows);
  m_breakPoints.resize(problemData.size());

  unsigned int row = 0;
  for (std::size_t levelIndex = 0; levelIndex < problemData.size();
       ++levelIndex) {
    for (const auto& item : problemData[levelIndex]) {
      const double weight = item.first;
      const std::shared_ptr<math::ConstraintBase>& constraint = item.second;
      if (weight == 0.) continue;

      const unsigned int count = constraint->rows();
      const double scale = levelIndex == 0 ? 0. : std::sqrt(weight);
      if (constraint->isBound()) {
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
            count == n,
            "A bound constraint must have one row per HQP variable");
        m_A.middleRows(row, count).setIdentity();
      } else {
        m_A.middleRows(row, count) = constraint->matrix();
      }

      if (constraint->isEquality()) {
        m_lower.segment(row, count) = constraint->vector();
        m_upper.segment(row, count) = constraint->vector();
        m_sense.segment(row, count).setConstant(DAQP_ACTIVE + DAQP_IMMUTABLE);
      } else {
        m_lower.segment(row, count) = constraint->lowerBound();
        m_upper.segment(row, count) = constraint->upperBound();
      }
      m_rowScales.segment(row, count).setConstant(scale);
      row += count;
    }
    m_breakPoints[levelIndex] = static_cast<int>(row);
  }

  if (row != rows) {
    m_A.conservativeResize(row, n);
    m_lower.conservativeResize(row);
    m_upper.conservativeResize(row);
    m_sense.conservativeResize(row);
    m_rowScales.conservativeResize(row);
  }
  if (m_output.lambda.size() != static_cast<Eigen::Index>(row)) {
    m_hasSolution = false;
  }
  m_output.lambda.resize(row);
  m_output.activeSet.resize(row);
}

const HQPOutput& SolverHQPDAQP::solve(const HQPData& problemData) {
  retrieveQPData(problemData, false);

  DAQPSettings settings;
  daqp_default_settings(&settings);
  settings.iter_limit = static_cast<int>(m_maxIter);
  settings.time_limit = m_maxTime;

  DAQPProblem problem = {};
  problem.n = static_cast<int>(m_n);
  problem.m = static_cast<int>(m_lower.size());
  problem.ms = 0;
  problem.A = m_A.size() == 0 ? nullptr : m_A.data();
  problem.bupper = m_upper.size() == 0 ? nullptr : m_upper.data();
  problem.blower = m_lower.size() == 0 ? nullptr : m_lower.data();
  problem.sense = m_sense.size() == 0 ? nullptr : m_sense.data();
  problem.break_points =
      m_breakPoints.size() == 0 ? nullptr : m_breakPoints.data();
  problem.nh = static_cast<int>(m_breakPoints.size());

  if (m_useWarmStart && m_hasSolution) {
    daqp_primal_init_active(&problem, m_output.x.data());
    daqp_dual_init_active(&problem, m_output.lambda.data());
  }
  m_output.lambda.setZero();
  DAQPResult result = {};
  result.x = m_output.x.data();
  result.lam = m_output.lambda.size() == 0 ? nullptr : m_output.lambda.data();

  DAQPWorkspace workspace = {};
  workspace.settings = &settings;
  result.exitflag =
      setup_daqp_main(&problem, &workspace, &result.setup_time, 1);
  if (result.exitflag >= 0) {
    // DAQP normalizes constraint rows during setup. Undo that normalization
    // for soft task rows, then apply TSID's sqrt(weight) scaling so the
    // least-squares metric is preserved.
    for (int i = 0; i < problem.m; ++i) {
      const double desiredScale = m_rowScales[i];
      if (desiredScale == 0.) continue;
      const double scale = desiredScale / workspace.scaling[i];
      for (int j = 0; j < problem.n; ++j) {
        workspace.M[problem.n * i + j] *= scale;
      }
      workspace.dupper[i] *= scale;
      workspace.dlower[i] *= scale;
      workspace.scaling[i] *= scale;
    }
    daqp_solve(&result, &workspace);
  }
  workspace.settings = nullptr;
  free_daqp_workspace(&workspace);
  free_daqp_ldp(&workspace);

  m_objValue = result.fval;
  m_output.iterations = result.iter;
  if (result.exitflag > 0) {
    m_output.status = HQP_STATUS_OPTIMAL;
  } else if (result.exitflag == DAQP_EXIT_INFEASIBLE) {
    m_output.status = HQP_STATUS_INFEASIBLE;
  } else if (result.exitflag == DAQP_EXIT_UNBOUNDED) {
    m_output.status = HQP_STATUS_UNBOUNDED;
  } else if (result.exitflag == DAQP_EXIT_ITERLIMIT ||
             result.exitflag == DAQP_EXIT_TIMELIMIT) {
    m_output.status = HQP_STATUS_MAX_ITER_REACHED;
  } else {
    m_output.status = HQP_STATUS_ERROR;
  }
  m_hasSolution = m_output.status == HQP_STATUS_OPTIMAL;

  int activeCount = 0;
  for (Eigen::Index i = 0; i < m_output.lambda.size(); ++i) {
    if (std::abs(m_output.lambda[i]) > settings.dual_tol) {
      m_output.activeSet[activeCount++] = static_cast<int>(i);
    }
  }
  m_output.activeSet.conservativeResize(activeCount);
  return m_output;
}

double SolverHQPDAQP::getObjectiveValue() { return m_objValue; }

bool SolverHQPDAQP::setMaximumIterations(unsigned int maxIter) {
  return SolverHQPBase::setMaximumIterations(maxIter);
}

bool SolverHQPDAQP::setMaximumTime(double seconds) {
  return SolverHQPBase::setMaximumTime(seconds);
}

}  // namespace solvers
}  // namespace tsid
