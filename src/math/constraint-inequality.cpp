//
// Copyright (c) 2017 CNRS
//

#include <tsid/math/constraint-inequality.hpp>

using namespace tsid::math;

ConstraintInequality::ConstraintInequality(const std::string& name)
    : ConstraintBase(name) {}

ConstraintInequality::ConstraintInequality(const std::string& name,
                                           const unsigned int rows,
                                           const unsigned int cols)
    : ConstraintBase(name, rows, cols),
      m_lb(Vector::Zero(rows)),
      m_ub(Vector::Zero(rows)) {}

ConstraintInequality::ConstraintInequality(const std::string& name,
                                           ConstRefMatrix A, ConstRefVector lb,
                                           ConstRefVector ub)
    : ConstraintBase(name, A), m_lb(lb), m_ub(ub) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(A.rows() == lb.rows(),
                                 "The number of rows of A and lb do not match");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(A.rows() == ub.rows(),
                                 "The number of rows of A and ub do not match");
}

unsigned int ConstraintInequality::rows() const {
  assert(m_A.rows() == m_lb.rows());
  assert(m_A.rows() == m_ub.rows());
  return (unsigned int)m_A.rows();
}

unsigned int ConstraintInequality::cols() const {
  return (unsigned int)m_A.cols();
}

void ConstraintInequality::resize(const unsigned int r, const unsigned int c) {
  m_A.setZero(r, c);
  m_lb.setZero(r);
  m_ub.setZero(r);
}

bool ConstraintInequality::isEquality() const { return false; }
bool ConstraintInequality::isInequality() const { return true; }
bool ConstraintInequality::isBound() const { return false; }

const Vector& ConstraintInequality::vector() const {
  assert(false);
  return m_lb;
}
const Vector& ConstraintInequality::lowerBound() const { return m_lb; }
const Vector& ConstraintInequality::upperBound() const { return m_ub; }

Vector& ConstraintInequality::vector() {
  assert(false);
  return m_lb;
}
Vector& ConstraintInequality::lowerBound() { return m_lb; }
Vector& ConstraintInequality::upperBound() { return m_ub; }

bool ConstraintInequality::setVector(ConstRefVector) {
  assert(false);
  return false;
}
bool ConstraintInequality::setLowerBound(ConstRefVector lb) {
  m_lb = lb;
  return true;
}
bool ConstraintInequality::setUpperBound(ConstRefVector ub) {
  m_ub = ub;
  return true;
}

bool ConstraintInequality::checkConstraint(ConstRefVector x, double tol) const {
  return ((m_A * x).array() <= m_ub.array() + tol).all() &&
         ((m_A * x).array() >= m_lb.array() - tol).all();
}
