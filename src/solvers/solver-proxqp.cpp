//
// Copyright (c) 2022 INRIA
//

#include "tsid/solvers/solver-proxqp.hpp"
#include "tsid/math/utils.hpp"
#include "tsid/utils/stop-watch.hpp"

namespace tsid {
namespace solvers {

using namespace math;
SolverProxQP::SolverProxQP(const std::string& name)
    : SolverHQPBase(name),
      m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION),
      m_solver(1, 0, 0)  // dim of primal var needs to be strictly positiv
{
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
  m_rho = 1e-6;
  m_muIn = 1e-1;
  m_muEq = 1e-3;
  m_epsAbs = 1e-5;
  m_epsRel = 0.;
  m_isVerbose = false;
}

void SolverProxQP::sendMsg(const std::string& s) {
  std::cout << "[SolverProxQP." << m_name << "] " << s << std::endl;
}

void SolverProxQP::resize(unsigned int n, unsigned int neq, unsigned int nin) {
  const bool resizeVar = n != m_n;
  const bool resizeEq = (resizeVar || neq != m_neq);
  const bool resizeIn = (resizeVar || nin != m_nin);

  if (resizeEq) {
#ifndef NDEBUG
    sendMsg("Resizing equality constraints from " + toString(m_neq) + " to " +
            toString(neq));
#endif
    m_qpData.CE.resize(neq, n);
    m_qpData.ce0.resize(neq);
  }
  if (resizeIn) {
#ifndef NDEBUG
    sendMsg("Resizing inequality constraints from " + toString(m_nin) + " to " +
            toString(nin));
#endif
    m_qpData.CI.resize(nin, n);
    m_qpData.ci_lb.resize(nin);
    m_qpData.ci_ub.resize(nin);
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
    m_solver = dense::QP<double>(m_n, m_neq, m_nin);
    setMaximumIterations(m_maxIter);
    setMuInequality(m_muIn);
    setMuEquality(m_muEq);
    setRho(m_rho);
    setEpsilonAbsolute(m_epsAbs);
    setEpsilonRelative(m_epsRel);
    setVerbose(m_isVerbose);
#ifndef NDEBUG
    setVerbose(true);
#endif
  }
}

void SolverProxQP::retrieveQPData(const HQPData& problemData,
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

    unsigned int i_eq = 0, i_in = 0;
    for (ConstraintLevel::const_iterator it = cl0.begin(); it != cl0.end();
         it++) {
      auto constr = it->second;
      if (constr->isEquality()) {
        m_qpData.CE.middleRows(i_eq, constr->rows()) = constr->matrix();
        m_qpData.ce0.segment(i_eq, constr->rows()) = constr->vector();
        i_eq += constr->rows();
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

const HQPOutput& SolverProxQP::solve(const HQPData& problemData) {
  SolverProxQP::retrieveQPData(problemData);

  START_PROFILER_PROXQP("PROFILE_PROXQP_SOLUTION");
  //  min 0.5 * x^T H x + g^T x
  //  s.t.
  //  CE x + ce0 = 0
  //  ci_lb <= CI x <= ci_ub

  EIGEN_MALLOC_ALLOWED

  m_solver.init(m_qpData.H, m_qpData.g, m_qpData.CE, m_qpData.ce0, m_qpData.CI,
                m_qpData.ci_lb, m_qpData.ci_ub);

  m_solver.solve();
  STOP_PROFILER_PROXQP("PROFILE_PROXQP_SOLUTION");

  QPSolverOutput status = m_solver.results.info.status;

  if (status == QPSolverOutput::PROXQP_SOLVED) {
    m_output.x = m_solver.results.x;
    m_output.status = HQP_STATUS_OPTIMAL;
    m_output.lambda = m_solver.results.y;
    m_output.iterations = int(m_solver.results.info.iter);
    m_output.activeSet.setZero();

#ifndef NDEBUG
    const Vector& x = m_solver.results.x;

    const ConstraintLevel& cl0 = problemData[0];

    if (cl0.size() > 0) {
      for (ConstraintLevel::const_iterator it = cl0.begin(); it != cl0.end();
           it++) {
        auto constr = it->second;
        if (constr->checkConstraint(x) == false) {
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
  } else if (status == QPSolverOutput::PROXQP_PRIMAL_INFEASIBLE)
    m_output.status = HQP_STATUS_INFEASIBLE;
  else if (status == QPSolverOutput::PROXQP_MAX_ITER_REACHED)
    m_output.status = HQP_STATUS_MAX_ITER_REACHED;
  else if (status == QPSolverOutput::PROXQP_DUAL_INFEASIBLE)
    m_output.status = HQP_STATUS_INFEASIBLE;

  return m_output;
}

double SolverProxQP::getObjectiveValue() {
  return m_solver.results.info.objValue;
}

bool SolverProxQP::setMaximumIterations(unsigned int maxIter) {
  SolverHQPBase::setMaximumIterations(maxIter);
  m_solver.settings.max_iter = maxIter;
  return true;
}

void SolverProxQP::setMuInequality(double muIn) {
  m_muIn = muIn;
  m_solver.settings.default_mu_in = m_muIn;
}
void SolverProxQP::setMuEquality(double muEq) {
  m_muEq = muEq;
  m_solver.settings.default_mu_eq = m_muEq;
}
void SolverProxQP::setRho(double rho) {
  m_rho = rho;
  m_solver.settings.default_rho = m_rho;
}
void SolverProxQP::setEpsilonAbsolute(double epsAbs) {
  m_epsAbs = epsAbs;
  m_solver.settings.eps_abs = m_epsAbs;
}
void SolverProxQP::setEpsilonRelative(double epsRel) {
  m_epsRel = epsRel;
  m_solver.settings.eps_rel = m_epsRel;
}
void SolverProxQP::setVerbose(bool isVerbose) {
  m_isVerbose = isVerbose;
  m_solver.settings.verbose = m_isVerbose;
}
}  // namespace solvers
}  // namespace tsid
