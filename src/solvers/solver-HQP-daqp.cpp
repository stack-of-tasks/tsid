//
// Copyright (c) 2026 CNRS
//

#include "tsid/solvers/solver-HQP-daqp.hpp"

#if __has_include(<daqp/api.h>)
#include <daqp/api.h>
#include <daqp/utils.h>
#else
#include <api.h>
#include <utils.h>
#endif

#include <algorithm>
#include <cmath>

#include <pinocchio/macros.hpp>

namespace tsid {
namespace solvers {

struct SolverHQPDAQP::WorkspaceHolder {
  DAQPWorkspace workspace = {};
  DAQPSettings settings = {};
};

namespace {

unsigned int constraintRows(const ConstraintLevel& level) {
  unsigned int rows = 0;
  for (const auto& item : level) {
    if (item.first != 0.) rows += item.second->rows();
  }
  return rows;
}

bool supportsConventionalQP(const HQPData& problemData) {
  if (problemData.size() != 2) return false;
  for (const auto& item : problemData[1]) {
    if (item.first != 0. && !item.second->isEquality()) return false;
  }
  return true;
}

}  // namespace

SolverHQPDAQP::SolverHQPDAQP(const std::string& name)
    : SolverHQPBase(name),
      m_objValue(0.),
      m_n(0),
      m_totalRows(0),
      m_useConventionalQP(false),
      m_hasSolution(false),
      m_workspaceRows(0),
      m_workspaceLevels(0),
      m_workspaceMaxSoftRows(0),
      m_workspaceUsesConventionalQP(false),
      m_assumeMatricesUnchanged(false) {}

SolverHQPDAQP::~SolverHQPDAQP() { resetWorkspace(); }

void SolverHQPDAQP::resetWorkspace() {
  if (m_workspace) {
    m_workspace->workspace.settings = nullptr;
    free_daqp_workspace(&m_workspace->workspace);
    free_daqp_ldp(&m_workspace->workspace);
    m_workspace.reset();
  }
  m_workspaceRows = 0;
  m_workspaceLevels = 0;
  m_workspaceMaxSoftRows = 0;
  m_workspaceUsesConventionalQP = false;
}

void SolverHQPDAQP::setAssumeMatricesUnchanged(bool unchanged) {
  m_assumeMatricesUnchanged = unchanged;
}

unsigned int SolverHQPDAQP::maxRowsInLevel() const {
  unsigned int maxRows = 0;
  int previous = 0;
  for (Eigen::Index i = 0; i < m_breakPoints.size(); ++i) {
    const int rows = m_breakPoints[i] - previous;
    if (rows > 0) maxRows = std::max(maxRows, static_cast<unsigned int>(rows));
    previous = m_breakPoints[i];
  }
  return maxRows;
}

void SolverHQPDAQP::resize(unsigned int n, unsigned int, unsigned int) {
  if (n != m_n) {
    m_hasSolution = false;
    resetWorkspace();
  }
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

  m_useConventionalQP = supportsConventionalQP(problemData);
  m_totalRows = rows;
  const unsigned int activeRows =
      m_useConventionalQP ? constraintRows(problemData[0]) : rows;

  resize(n, 0, activeRows);
  m_A.resize(activeRows, n);
  m_lower.resize(activeRows);
  m_upper.resize(activeRows);
  m_sense.setZero(activeRows);
  m_rowScales.setZero(activeRows);
  m_breakPoints.resize(m_useConventionalQP ? 1 : problemData.size());

  unsigned int row = 0;
  const std::size_t constraintLevels =
      m_useConventionalQP ? 1 : problemData.size();
  for (std::size_t levelIndex = 0; levelIndex < constraintLevels;
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

  if (row != activeRows) {
    m_A.conservativeResize(row, n);
    m_lower.conservativeResize(row);
    m_upper.conservativeResize(row);
    m_sense.conservativeResize(row);
    m_rowScales.conservativeResize(row);
  }

  if (m_useConventionalQP) {
    const bool rebuildH = !m_assumeMatricesUnchanged || m_H.rows() != n ||
                          m_H.cols() != n;
    if (rebuildH) m_H.setZero(n, n);
    m_f.setZero(n);
    for (const auto& item : problemData[1]) {
      if (item.first == 0.) continue;
      const auto& constraint = item.second;
      const double weight = item.first;
      if (rebuildH)
        m_H.noalias() += weight * constraint->matrix().transpose() *
                          constraint->matrix();
      m_f.noalias() -= weight * constraint->matrix().transpose() *
                        constraint->vector();
    }
  } else {
    m_H.resize(0, 0);
    m_f.resize(0);
  }

  if (m_output.lambda.size() != static_cast<Eigen::Index>(m_totalRows) ||
      m_daqpLambda.size() != static_cast<Eigen::Index>(row)) {
    m_hasSolution = false;
  }
  m_output.lambda.resize(m_totalRows);
  m_output.activeSet.resize(m_totalRows);
  m_daqpLambda.resize(row);

  if (m_workspace &&
      (m_workspaceRows != row ||
       m_workspaceLevels != static_cast<unsigned int>(m_breakPoints.size()) ||
       m_workspaceMaxSoftRows < maxRowsInLevel() ||
       m_workspaceUsesConventionalQP != m_useConventionalQP)) {
    m_hasSolution = false;
    resetWorkspace();
  }
}

const HQPOutput& SolverHQPDAQP::solve(const HQPData& problemData) {
  retrieveQPData(problemData, false);

  DAQPProblem problem = {};
  problem.n = static_cast<int>(m_n);
  problem.m = static_cast<int>(m_lower.size());
  problem.ms = 0;
  problem.H = m_useConventionalQP ? m_H.data() : nullptr;
  problem.f = m_useConventionalQP ? m_f.data() : nullptr;
  problem.A = m_A.size() == 0 ? nullptr : m_A.data();
  problem.bupper = m_upper.size() == 0 ? nullptr : m_upper.data();
  problem.blower = m_lower.size() == 0 ? nullptr : m_lower.data();
  problem.sense = m_sense.size() == 0 ? nullptr : m_sense.data();
  problem.break_points =
      m_breakPoints.size() == 0 ? nullptr : m_breakPoints.data();
  problem.nh = m_useConventionalQP ? 1 : static_cast<int>(m_breakPoints.size());

  if (m_useWarmStart && m_hasSolution) {
    daqp_primal_init_active(&problem, m_output.x.data());
    daqp_dual_init_active(&problem, m_daqpLambda.data());
  }
  m_daqpLambda.setZero();
  DAQPResult result = {};
  result.x = m_output.x.data();
  result.lam = m_daqpLambda.size() == 0 ? nullptr : m_daqpLambda.data();

  bool normalizationChanged = false;

  if (!m_workspace) {
    m_workspace = std::make_unique<WorkspaceHolder>();
    daqp_default_settings(&m_workspace->settings);
    m_workspace->settings.iter_limit = static_cast<int>(m_maxIter);
    m_workspace->settings.time_limit = m_maxTime;
    m_workspace->workspace.settings = &m_workspace->settings;
    m_workspaceRows = static_cast<unsigned int>(problem.m);
    m_workspaceLevels = static_cast<unsigned int>(problem.nh);
    m_workspaceMaxSoftRows = maxRowsInLevel();
    m_workspaceUsesConventionalQP = m_useConventionalQP;
    result.exitflag = setup_daqp_main(&problem, &m_workspace->workspace,
                                      &result.setup_time, 1);
    normalizationChanged = !m_useConventionalQP;
  } else {
    m_workspace->settings.iter_limit = static_cast<int>(m_maxIter);
    m_workspace->settings.time_limit = m_maxTime;
    int updateMask = DAQP_UPDATE_sense;
    if (m_assumeMatricesUnchanged) {
      if (m_useConventionalQP) {
        updateMask |= DAQP_UPDATE_v | DAQP_UPDATE_d;
      } else {
        updateMask |= DAQP_UPDATE_hierarchy;
        updateMask |= DAQP_UPDATE_d;
      }
    } else if (m_useConventionalQP) {
      updateMask |= DAQP_UPDATE_Rinv | DAQP_UPDATE_M | DAQP_UPDATE_v |
                    DAQP_UPDATE_d;
    } else {
      updateMask |= DAQP_UPDATE_M | DAQP_UPDATE_d | DAQP_UPDATE_hierarchy;
      normalizationChanged = true;
    }
    result.exitflag =
        daqp_update_ldp(updateMask, &m_workspace->workspace, &problem);
  }

  const double dualTolerance = m_workspace->settings.dual_tol;
  if (result.exitflag >= 0) {
    if (!m_useConventionalQP && normalizationChanged) {
      // DAQP normalizes constraint rows during setup. Undo that normalization
      // for soft task rows, then apply TSID's sqrt(weight) scaling so the
      // least-squares metric is preserved. Retain the adjusted rows while the
      // caller promises that matrices are unchanged.
      for (int i = 0; i < problem.m; ++i) {
        const double desiredScale = m_rowScales[i];
        if (desiredScale == 0.) continue;
        const double scale = desiredScale / m_workspace->workspace.scaling[i];
        for (int j = 0; j < problem.n; ++j) {
          m_workspace->workspace.M[problem.n * i + j] *= scale;
        }
        m_workspace->workspace.dupper[i] *= scale;
        m_workspace->workspace.dlower[i] *= scale;
        m_workspace->workspace.scaling[i] *= scale;
      }
    }
    daqp_solve(&result, &m_workspace->workspace);
    // DAQP's public hierarchical result contains soft-level duals. TSID
    // needs the final active-set signs to warm-start the next hierarchy.
    if (!m_useConventionalQP)
      daqp_extract_active_duals(&result, &m_workspace->workspace);
  } else {
    resetWorkspace();
  }

  m_objValue = result.fval;
  m_output.iterations = result.iter;
  m_output.lambda.setZero();
  m_output.lambda.head(problem.m) = m_daqpLambda;
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
    if (std::abs(m_output.lambda[i]) > dualTolerance) {
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
