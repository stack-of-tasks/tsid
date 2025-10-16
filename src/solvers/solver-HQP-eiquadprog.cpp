//
// Copyright (c) 2017 CNRS
//

#include "tsid/solvers/solver-HQP-eiquadprog.hpp"
#include "tsid/math/utils.hpp"
#include "eiquadprog/eiquadprog.hpp"
#include "tsid/utils/stop-watch.hpp"

using namespace tsid::math;
using namespace tsid::solvers;
using namespace Eigen;

SolverHQuadProg::SolverHQuadProg(const std::string& name)
    : SolverHQPBase(name),
      m_hessian_regularization(DEFAULT_HESSIAN_REGULARIZATION) {
  m_n = 0;
  m_neq = 0;
  m_nin = 0;
}

void SolverHQuadProg::sendMsg(const std::string& s) {
  std::cout << "[SolverHQuadProg." << m_name << "] " << s << std::endl;
}

void SolverHQuadProg::resize(unsigned int n, unsigned int neq,
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

  m_n = n;
  m_neq = neq;
  m_nin = nin;
}

void SolverHQuadProg::retrieveQPData(const HQPData& problemData,
                                     const bool /*hessianRegularization*/) {
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

    int i_eq = 0, i_in = 0;
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

      m_qpData.H += w * constr->matrix().transpose() * constr->matrix();
      m_qpData.g -= w * (constr->matrix().transpose() * constr->vector());
    }
    m_qpData.H.diagonal() += m_hessian_regularization * Vector::Ones(m_n);
  }

#ifdef ELIMINATE_EQUALITY_CONSTRAINTS

  // eliminate equality constraints
  const int r = m_neq;
  Matrix Z(m_n, m_n - m_neq);
  Matrix ZT(m_n, m_n);
  m_ZT_H_Z.resize(m_n - r, m_n - r);

  START_PROFILER("Eiquadprog eliminate equalities");
  if (m_neq > 0) {
    START_PROFILER("Eiquadprog CE decomposition");
    //    m_qpData.CE_dec.compute(m_qpData.CE, ComputeThinU | ComputeThinV);
    m_qpData.CE_dec.compute(m_qpData.CE);
    STOP_PROFILER("Eiquadprog CE decomposition");

    START_PROFILER("Eiquadprog get CE null-space basis");
    // get nullspace basis from SVD
    //    const int r = m_qpData.CE_dec.nonzeroSingularValues();
    //    const Matrix Z = m_qpData.CE_dec.matrixV().rightCols(m_n-r);

    // get null space basis from ColPivHouseholderQR
    //	Matrix Z = m_qpData.CE_dec.householderQ();
    //	Z = Z.rightCols(m_n-r);

    // get null space basis from COD
    // P^{-1} * y => colsPermutation() * y;
    //	Z = m_qpData.CE_dec.matrixZ(); // * m_qpData.CE_dec.colsPermutation();
    ZT.setIdentity();
    // m_qpData.CE_dec.applyZAdjointOnTheLeftInPlace(ZT);
    typedef tsid::math::Index Index;
    const Index rank = m_qpData.CE_dec.rank();
    Vector temp(m_n);
    for (Index k = 0; k < rank; ++k) {
      if (k != rank - 1) ZT.row(k).swap(ZT.row(rank - 1));
      ZT.middleRows(rank - 1, m_n - rank + 1)
          .applyHouseholderOnTheLeft(
              m_qpData.CE_dec.matrixQTZ().row(k).tail(m_n - rank).adjoint(),
              m_qpData.CE_dec.zCoeffs()(k), &temp(0));
      if (k != rank - 1) ZT.row(k).swap(ZT.row(rank - 1));
    }
    STOP_PROFILER("Eiquadprog get CE null-space basis");

    // find a solution for the equalities
    Vector x0 = m_qpData.CE_dec.solve(m_qpData.ce0);
    x0 = -x0;

    //    START_PROFILER("Eiquadprog project Hessian full");
    //    m_ZT_H_Z.noalias() = Z.transpose()*m_qpData.H*Z; // this is too slow
    //	STOP_PROFILER("Eiquadprog project Hessian full");

    START_PROFILER("Eiquadprog project Hessian incremental");
    const ConstraintLevel& cl1 = problemData[1];
    m_ZT_H_Z.setZero();
    // m_qpData.g.setZero();
    Matrix AZ;
    for (ConstraintLevel::const_iterator it = cl1.begin(); it != cl1.end();
         it++) {
      const double& w = it->first;
      auto constr = it->second;
      if (!constr->isEquality())
        PINOCCHIO_CHECK_INPUT_ARGUMENT(
            false, "Inequalities in the cost function are not implemented yet");

      AZ.noalias() = constr->matrix() * Z.rightCols(m_n - r);
      m_ZT_H_Z += w * AZ.transpose() * AZ;
      // m_qpData.g -= w*(constr->matrix().transpose()*constr->vector());
    }
    // m_ZT_H_Z.diagonal() += 1e-8*Vector::Ones(m_n);
    m_qpData.CI_Z.noalias() = m_qpData.CI * Z.rightCols(m_n - r);
    STOP_PROFILER("Eiquadprog project Hessian incremental");
  }
  STOP_PROFILER("Eiquadprog eliminate equalities");
#endif
}

const HQPOutput& SolverHQuadProg::solve(const HQPData& problemData) {
  // #ifndef NDEBUG
  //   PRINT_MATRIX(m_qpData.H);
  //   PRINT_VECTOR(m_qpData.g);
  //   PRINT_MATRIX(m_qpData.CE);
  //   PRINT_VECTOR(m_qpData.ce0);
  //   PRINT_MATRIX(m_qpData.CI);
  //   PRINT_VECTOR(m_qpData.ci0);
  // #endif
  SolverHQuadProg::retrieveQPData(problemData);

  //  min 0.5 * x G x + g0 x
  //  s.t.
  //  CE^T x + ce0 = 0
  //  CI^T x + ci0 >= 0
  m_objValue = eiquadprog::solvers::solve_quadprog(
      m_qpData.H, m_qpData.g, m_qpData.CE.transpose(), m_qpData.ce0,
      m_qpData.CI.transpose(), m_qpData.ci0, m_output.x, m_activeSet,
      m_activeSetSize);

  if (m_objValue == std::numeric_limits<double>::infinity())
    m_output.status = HQP_STATUS_INFEASIBLE;
  else {
    m_output.status = HQP_STATUS_OPTIMAL;
#ifndef NDEBUG
    const Vector& x = m_output.x;
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
  }

  return m_output;
}

double SolverHQuadProg::getObjectiveValue() { return m_objValue; }
