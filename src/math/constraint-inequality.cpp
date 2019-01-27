//
// Copyright (c) 2017 CNRS
//
// This file is part of tsid
// tsid is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// tsid is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// tsid If not, see
// <http://www.gnu.org/licenses/>.
//

#include <tsid/math/constraint-inequality.hpp>

using namespace tsid::math;

ConstraintInequality::ConstraintInequality(const std::string & name):
  ConstraintBase(name)
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
                                           const unsigned int rows,
                                           const unsigned int cols):
  ConstraintBase(name, rows, cols),
  m_lb(Vector::Zero(rows)),
  m_ub(Vector::Zero(rows))
{}

ConstraintInequality::ConstraintInequality(const std::string & name,
                                           ConstRefMatrix A,
                                           ConstRefVector lb,
                                           ConstRefVector ub):
  ConstraintBase(name, A),
  m_lb(lb),
  m_ub(ub)
{
  assert(A.rows()==lb.rows());
  assert(A.rows()==ub.rows());
}

unsigned int ConstraintInequality::rows() const
{
  assert(m_A.rows()==m_lb.rows());
  assert(m_A.rows()==m_ub.rows());
  return (unsigned int) m_A.rows();
}

unsigned int ConstraintInequality::cols() const
{
  return (unsigned int) m_A.cols();
}

void ConstraintInequality::resize(const unsigned int r, const unsigned int c)
{
  m_A.setZero(r, c);
  m_lb.setZero(r);
  m_ub.setZero(r);
}

bool ConstraintInequality::isEquality() const    { return false; }
bool ConstraintInequality::isInequality() const  { return true; }
bool ConstraintInequality::isBound() const       { return false; }

const Vector & ConstraintInequality::vector()     const { assert(false); return m_lb;}
const Vector & ConstraintInequality::lowerBound() const { return m_lb; }
const Vector & ConstraintInequality::upperBound() const { return m_ub; }

Vector & ConstraintInequality::vector()     { assert(false); return m_lb;}
Vector & ConstraintInequality::lowerBound() { return m_lb; }
Vector & ConstraintInequality::upperBound() { return m_ub; }

bool ConstraintInequality::setVector(ConstRefVector ) { assert(false); return false; }
bool ConstraintInequality::setLowerBound(ConstRefVector lb) { m_lb = lb; return true; }
bool ConstraintInequality::setUpperBound(ConstRefVector ub) { m_ub = ub; return true; }

bool ConstraintInequality::checkConstraint(ConstRefVector x, double tol) const
{
  return ((m_A*x).array() <= m_ub.array() + tol).all() &&
      ((m_A*x).array() >= m_lb.array() - tol).all();
}
