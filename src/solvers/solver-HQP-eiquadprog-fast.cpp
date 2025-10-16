//
// Copyright (c) 2017 CNRS
//

#include "tsid/solvers/solver-HQP-eiquadprog-fast.hpp"
#include "tsid/math/utils.hpp"
#include "eiquadprog/eiquadprog-fast.hpp"
#include "tsid/utils/stop-watch.hpp"

// #define PROFILE_EIQUADPROG_FAST

using namespace eiquadprog::solvers;

namespace tsid {
namespace solvers {

using namespace math;
SolverHQuadProgFast::SolverHQuadProgFast(const std::string& name)
    : SolverHQPBase(name),
      m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION) {
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
}

void SolverHQuadProgFast::sendMsg(const std::string& s) {
  std::cout << "[SolverHQuadProgFast." << m_name << "] " << s << std::endl;
}

void SolverHQuadProgFast::resize(unsigned int n, unsigned int neq,
                                 unsigned int nin) {
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
    m_qpData.CI.resize(2 * nin, n);
    m_qpData.ci0.resize(2 * nin);
  }
  if (resizeVar) {
#ifndef NDEBUG
    sendMsg("Resizing Hessian from " + toString(m_n) + " to " + toString(n));
#endif
    m_qpData.H.resize(n, n);
    m_qpData.g.resize(n);
    m_output.x.resize(n);
  }

  if (resizeVar || resizeIn || resizeEq) {
    m_solver.reset(n, neq, nin * 2);
    m_output.resize(n, neq, 2 * nin);
  }

  m_n = n;
  m_neq = neq;
  m_nin = nin;
}

void SolverHQuadProgFast::retrieveQPData(const HQPData& problemData,
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
        m_qpData.ce0.segment(i_eq, constr->rows()) = -constr->vector();
        i_eq += constr->rows();

      } else if (constr->isInequality()) {
        m_qpData.CI.middleRows(i_in, constr->rows()) = constr->matrix();
        m_qpData.ci0.segment(i_in, constr->rows()) = -constr->lowerBound();
        i_in += constr->rows();
        m_qpData.CI.middleRows(i_in, constr->rows()) = -constr->matrix();
        m_qpData.ci0.segment(i_in, constr->rows()) = constr->upperBound();
        i_in += constr->rows();
      } else if (constr->isBound()) {
        m_qpData.CI.middleRows(i_in, constr->rows()).setIdentity();
        m_qpData.ci0.segment(i_in, constr->rows()) = -constr->lowerBound();
        i_in += constr->rows();
        m_qpData.CI.middleRows(i_in, constr->rows()) =
            -Matrix::Identity(m_n, m_n);
        m_qpData.ci0.segment(i_in, constr->rows()) = constr->upperBound();
        i_in += constr->rows();
      }
    }
  } else
    resize(m_n, neq, nin);

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

const HQPOutput& SolverHQuadProgFast::solve(const HQPData& problemData) {
  SolverHQuadProgFast::retrieveQPData(problemData);

  START_PROFILER_EIQUADPROG_FAST(PROFILE_EIQUADPROG_SOLUTION);
  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE x + ce0 = 0
  //  CI x + ci0 >= 0
  EIGEN_MALLOC_ALLOWED
  eiquadprog::solvers::EiquadprogFast_status status =
      m_solver.solve_quadprog(m_qpData.H, m_qpData.g, m_qpData.CE, m_qpData.ce0,
                              m_qpData.CI, m_qpData.ci0, m_output.x);

  STOP_PROFILER_EIQUADPROG_FAST(PROFILE_EIQUADPROG_SOLUTION);

  if (status == EIQUADPROG_FAST_OPTIMAL) {
    m_output.status = HQP_STATUS_OPTIMAL;
    m_output.lambda = m_solver.getLagrangeMultipliers();
    m_output.iterations = m_solver.getIteratios();
    //    m_output.activeSet =
    //    m_solver.getActiveSet().tail(2*m_nin).head(m_solver.getActiveSetSize()-m_neq);
    m_output.activeSet = m_solver.getActiveSet().segment(
        m_neq, m_solver.getActiveSetSize() - m_neq);
#ifndef NDEBUG
    const Vector& x = m_output.x;

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
  } else if (status == EIQUADPROG_FAST_UNBOUNDED)
    m_output.status = HQP_STATUS_INFEASIBLE;
  else if (status == EIQUADPROG_FAST_MAX_ITER_REACHED)
    m_output.status = HQP_STATUS_MAX_ITER_REACHED;
  else if (status == EIQUADPROG_FAST_REDUNDANT_EQUALITIES)
    m_output.status = HQP_STATUS_ERROR;

  return m_output;
}

double SolverHQuadProgFast::getObjectiveValue() {
  return m_solver.getObjValue();
}

bool SolverHQuadProgFast::setMaximumIterations(unsigned int maxIter) {
  SolverHQPBase::setMaximumIterations(maxIter);
  return m_solver.setMaxIter(maxIter);
}
}  // namespace solvers
}  // namespace tsid
