//
// Copyright (c) 2022 INRIA
//

#include "tsid/solvers/solver-osqp.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/utils/stop-watch.hpp"

namespace tsid {
namespace solvers {

using namespace math;
SolverOSQP::SolverOSQP(const std::string& name)
    : SolverHQPBase(name),
      m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION) {
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
  m_rho = 0.1;
  m_sigma = 1e-6;
  m_alpha = 1.6;
  m_epsAbs = 1e-5;
  m_epsRel = 0.;
  m_isVerbose = false;
  m_isDataInitialized = false;
}

SolverOSQP::SolverOSQP(const SolverOSQP& other)
    : SolverHQPBase(other.name()),
      m_hessian_regularization(other.m_hessian_regularization) {
  m_n = other.m_n;
  m_neq = other.m_neq;
  m_nin = other.m_nin;
  m_rho = other.m_rho;
  m_sigma = other.m_sigma;
  m_alpha = other.m_alpha;
  m_epsAbs = other.m_epsAbs;
  m_epsRel = other.m_epsRel;
  m_isVerbose = other.m_isVerbose;
  m_isDataInitialized = other.m_isDataInitialized;
}

void SolverOSQP::sendMsg(const std::string& s) {
  std::cout << "[SolverOSQP." << m_name << "] " << s << std::endl;
}

void SolverOSQP::resize(unsigned int n, unsigned int neq, unsigned int nin) {
  const bool resizeVar = n != m_n;
  const bool resizeEq = (resizeVar || neq != m_neq);
  const bool resizeIn = (resizeVar || nin != m_nin);

  if (resizeIn || resizeEq) {
    m_qpData.CI.resize(nin + neq, n);
    m_qpData.ci_lb.resize(nin + neq);
    m_qpData.ci_ub.resize(nin + neq);
  }
  if (resizeVar) {
#ifndef NDEBUG
    sendMsg("Resizing Hessian from " + toString(m_n) + " to " + toString(n));
#endif
    m_qpData.H.resize(n, n);
    m_qpData.g.resize(n);
    m_output.x.resize(n);
  }

  m_n = n;
  m_neq = neq;
  m_nin = nin;

  if (resizeVar || resizeEq || resizeIn) {
    if (m_solver.isInitialized()) {
      m_solver.clearSolverVariables();
      m_solver.data()->clearHessianMatrix();
      m_solver.data()->clearLinearConstraintsMatrix();
      m_solver.clearSolver();
    }
    m_solver.data()->setNumberOfVariables(int(m_n));
    m_solver.data()->setNumberOfConstraints(int(m_neq + m_nin));

    m_isDataInitialized = false;
#ifndef NDEBUG
    setVerbose(true);
#endif
    setMaximumIterations(m_maxIter);
    setSigma(m_sigma);
    setAlpha(m_alpha);
    setRho(m_rho);
    setEpsilonAbsolute(m_epsAbs);
    setEpsilonRelative(m_epsRel);
    setVerbose(m_isVerbose);
  }
}

void SolverOSQP::retrieveQPData(const HQPData& problemData,
                                const bool hessianRegularization) {
  if (problemData.size() > 2) {
    PINOCCHIO_CHECK_INPUT_ARGUMENT(
        false, "Solver not implemented for more than 2 hierarchical levels.");
  }

  // Compute the constraint matrix sizes
  unsigned int neq = 0, nin = 0;
  const ConstraintLevel& cl0 = problemData[0];

  if (cl0.size() > 0) {
    const unsigned int n = cl0[0].second->cols();
    for (ConstraintLevel::const_iterator it = cl0.begin(); it != cl0.end();
         it++) {
      auto constr = it->second;
      assert(n == constr->cols());
      if (constr->isEquality())
        neq += constr->rows();
      else
        nin += constr->rows();
    }
    // If necessary, resize the constraint matrices
    resize(n, neq, nin);

    unsigned int i_in = 0;
    for (ConstraintLevel::const_iterator it = cl0.begin(); it != cl0.end();
         it++) {
      auto constr = it->second;
      if (constr->isEquality()) {
        m_qpData.CI.middleRows(i_in, constr->rows()) = constr->matrix();
        m_qpData.ci_lb.segment(i_in, constr->rows()) = constr->vector();
        m_qpData.ci_ub.segment(i_in, constr->rows()) = constr->vector();
        i_in += constr->rows();
      } else if (constr->isInequality()) {
        m_qpData.CI.middleRows(i_in, constr->rows()) = constr->matrix();
        m_qpData.ci_lb.segment(i_in, constr->rows()) = constr->lowerBound();
        m_qpData.ci_ub.segment(i_in, constr->rows()) = constr->upperBound();
        i_in += constr->rows();
      } else if (constr->isBound()) {
        m_qpData.CI.middleRows(i_in, constr->rows()).setIdentity();
        m_qpData.ci_lb.segment(i_in, constr->rows()) = constr->lowerBound();
        m_qpData.ci_ub.segment(i_in, constr->rows()) = constr->upperBound();
        i_in += constr->rows();
      }
    }
  } else {
    resize(m_n, neq, nin);
  }

  EIGEN_MALLOC_NOT_ALLOWED;

  // Compute the cost
  if (problemData.size() > 1) {
    const ConstraintLevel& cl1 = problemData[1];
    m_qpData.H.setZero();
    m_qpData.g.setZero();

    for (ConstraintLevel::const_iterator it = cl1.begin(); it != cl1.end();
         it++) {
      const double& w = it->first;
      auto constr = it->second;
      if (!constr->isEquality())
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
            false, "Inequalities in the cost function are not implemented yet");

      EIGEN_MALLOC_ALLOWED;
      m_qpData.H.noalias() +=
          w * constr->matrix().transpose() * constr->matrix();
      EIGEN_MALLOC_NOT_ALLOWED;

      m_qpData.g.noalias() -=
          w * constr->matrix().transpose() * constr->vector();
    }

    if (hessianRegularization) {
      double m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION);
      m_qpData.H.diagonal().array() += m_hessian_regularization;
    }
  }
}

const HQPOutput& SolverOSQP::solve(const HQPData& problemData) {
  typedef Eigen::SparseMatrix<double> SpMat;

  SolverOSQP::retrieveQPData(problemData);

  START_PROFILER_OSQP("PROFILE_OSQP_SOLUTION");
  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  lb <= CI x <= ub (including equality constraints)

  EIGEN_MALLOC_ALLOWED

  if (!m_isDataInitialized) {
    m_solver.data()->setHessianMatrix(
        static_cast<SpMat>(m_qpData.H.sparseView()));
    m_solver.data()->setGradient(m_qpData.g);
    m_solver.data()->setLinearConstraintsMatrix(
        static_cast<SpMat>(m_qpData.CI.sparseView()));
    m_solver.data()->setBounds(m_qpData.ci_lb, m_qpData.ci_ub);
    m_isDataInitialized = true;
  } else {
    m_solver.updateHessianMatrix(static_cast<SpMat>(m_qpData.H.sparseView()));
    m_solver.updateGradient(m_qpData.g);
    m_solver.updateLinearConstraintsMatrix(
        static_cast<SpMat>(m_qpData.CI.sparseView()));
    m_solver.updateBounds(m_qpData.ci_lb, m_qpData.ci_ub);
  }

  if (!m_solver.isInitialized()) {
    m_solver.initSolver();
  }
  m_solver.solveProblem();
  STOP_PROFILER_OSQP("PROFILE_OSQP_SOLUTION");

  OsqpEigen::Status status = m_solver.getStatus();

  if (status == OsqpEigen::Status::Solved) {
    m_output.x = m_solver.getSolution();
    m_output.status = HQP_STATUS_OPTIMAL;
    m_output.lambda = m_solver.getDualSolution();

#ifndef NDEBUG
    const Vector& x = m_solver.getSolution();

    const ConstraintLevel& cl0 = problemData[0];

    if (cl0.size() > 0) {
      for (ConstraintLevel::const_iterator it = cl0.begin(); it != cl0.end();
           it++) {
        auto constr = it->second;
        if (constr->checkConstraint(x) == false) {
          // m_output.status = HQP_STATUS_ERROR;
          if (constr->isEquality()) {
            sendMsg("Equality " + constr->name() + " violated: " +
                    toString((constr->matrix() * x - constr->vector()).norm()));
          } else if (constr->isInequality()) {
            sendMsg(
                "Inequality " + constr->name() + " violated: " +
                toString(
                    (constr->matrix() * x - constr->lowerBound()).minCoeff()) +
                "\n" +
                toString(
                    (constr->upperBound() - constr->matrix() * x).minCoeff()));
          } else if (constr->isBound()) {
            sendMsg("Bound " + constr->name() + " violated: " +
                    toString((x - constr->lowerBound()).minCoeff()) + "\n" +
                    toString((constr->upperBound() - x).minCoeff()));
          }
        }
      }
    }
#endif
  } else if (status == OsqpEigen::Status::PrimalInfeasible)
    m_output.status = HQP_STATUS_INFEASIBLE;
  else if (status == OsqpEigen::Status::MaxIterReached)
    m_output.status = HQP_STATUS_MAX_ITER_REACHED;
  else if (status == OsqpEigen::Status::DualInfeasible)
    m_output.status = HQP_STATUS_INFEASIBLE;
  else if (status == OsqpEigen::Status::SolvedInaccurate)
    m_output.status = HQP_STATUS_MAX_ITER_REACHED;
  else
    m_output.status = HQP_STATUS_UNKNOWN;

  return m_output;
}

double SolverOSQP::getObjectiveValue() { return m_solver.getObjValue(); }

bool SolverOSQP::setMaximumIterations(unsigned int maxIter) {
  SolverHQPBase::setMaximumIterations(maxIter);
  m_solver.settings()->setMaxIteration(int(maxIter));
  return true;
}

void SolverOSQP::setSigma(double sigma) {
  m_sigma = sigma;
  m_solver.settings()->setSigma(m_sigma);
}
void SolverOSQP::setAlpha(double alpha) {
  m_alpha = alpha;
  m_solver.settings()->setAlpha(m_alpha);
}

void SolverOSQP::setRho(double rho) {
  m_rho = rho;
  m_solver.settings()->setRho(m_rho);
}
void SolverOSQP::setEpsilonAbsolute(double epsAbs) {
  m_epsAbs = epsAbs;
  m_solver.settings()->setAbsoluteTolerance(m_epsAbs);
}
void SolverOSQP::setEpsilonRelative(double epsRel) {
  m_epsRel = epsRel;
  m_solver.settings()->setRelativeTolerance(m_epsRel);
}
void SolverOSQP::setVerbose(bool isVerbose) {
  m_isVerbose = isVerbose;
  m_solver.settings()->setVerbosity(m_isVerbose);
}
}  // namespace solvers
}  // namespace tsid
