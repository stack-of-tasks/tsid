//
// Copyright (c) 2017 CNRS
//

#include <tsid/math/constraint-base.hpp>

using namespace tsid::math;

ConstraintBase::ConstraintBase(const std::string& name) : m_name(name) {}

ConstraintBase::ConstraintBase(const std::string& name, const unsigned int rows,
                               const unsigned int cols)
    : m_name(name) {
  m_A = Matrix::Zero(rows, cols);
}

ConstraintBase::ConstraintBase(const std::string& name, ConstRefMatrix A)
    : m_name(name), m_A(A) {}

const std::string& ConstraintBase::name() const { return m_name; }

const Matrix& ConstraintBase::matrix() const { return m_A; }

Matrix& ConstraintBase::matrix() { return m_A; }

bool ConstraintBase::setMatrix(ConstRefMatrix A) {
  PINOCCHIO_CHECK_INPUT_ARGUMENT(m_A.cols() == A.cols(),
                                 "cols do not match the constraint dimension");
  PINOCCHIO_CHECK_INPUT_ARGUMENT(m_A.rows() == A.rows(),
                                 "rows do not match the constraint dimension");
  m_A = A;
  return true;
}
