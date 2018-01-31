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

#include <tsid/math/constraint-bound.hpp>

using namespace tsid::math;


ConstraintBound::ConstraintBound(const std::string & name):
  ConstraintBase(name)
{}

ConstraintBound::ConstraintBound(const std::string & name,
                                 const unsigned int size):
  ConstraintBase(name, Matrix::Identity(size,size)),
  m_lb(Vector::Zero(size)),
  m_ub(Vector::Zero(size))
{}

ConstraintBound::ConstraintBound(const std::string & name,
                                 ConstRefVector lb,
                                 ConstRefVector ub):
  ConstraintBase(name, Matrix::Identity(lb.size(), lb.size())),
  m_lb(lb),
  m_ub(ub)
{
  assert(lb.size()==ub.size());
}

unsigned int ConstraintBound::rows() const
{
  assert(m_lb.rows()==m_ub.rows());
  return (unsigned int) m_lb.rows();
}

unsigned int ConstraintBound::cols() const
{
  assert(m_lb.rows()==m_ub.rows());
  return (unsigned int) m_lb.rows();
}

void ConstraintBound::resize(const unsigned int r, const unsigned int c)
{
  assert(r==c);
  m_A.setIdentity(r, c);
  m_lb.setZero(r);
  m_ub.setZero(r);
}

bool ConstraintBound::isEquality() const    { return false; }
bool ConstraintBound::isInequality() const  { return false; }
bool ConstraintBound::isBound() const       { return true; }

const Vector & ConstraintBound::vector()     const { assert(false); return m_lb;}
const Vector & ConstraintBound::lowerBound() const { return m_lb; }
const Vector & ConstraintBound::upperBound() const { return m_ub; }

Vector & ConstraintBound::vector()     { assert(false); return m_lb;}
Vector & ConstraintBound::lowerBound() { return m_lb; }
Vector & ConstraintBound::upperBound() { return m_ub; }

bool ConstraintBound::setVector(ConstRefVector ) { assert(false); return false; }
bool ConstraintBound::setLowerBound(ConstRefVector lb) { m_lb = lb; return true; }
bool ConstraintBound::setUpperBound(ConstRefVector ub) { m_ub = ub; return true; }

bool ConstraintBound::checkConstraint(ConstRefVector x, double tol) const
{
  return (x.array() <= m_ub.array() + tol).all() &&
      (x.array() >= m_lb.array() - tol).all();
}
