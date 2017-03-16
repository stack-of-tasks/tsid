//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/math/constraint-equality.hpp>

using namespace pininvdyn::math;

ConstraintEquality::ConstraintEquality(const std::string & name):
  ConstraintBase(name)
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
                                       const unsigned int rows,
                                       const unsigned int cols):
  ConstraintBase(name, rows, cols),
  m_b(Vector::Zero(rows))
{}

ConstraintEquality::ConstraintEquality(const std::string & name,
                                       ConstRefMatrix A,
                                       ConstRefVector b):
  ConstraintBase(name, A),
  m_b(b)
{
  assert(A.rows()==b.rows());
}

unsigned int ConstraintEquality::rows() const
{
  assert(m_A.rows()==m_b.rows());
  return (unsigned int) m_A.rows();
}

unsigned int ConstraintEquality::cols() const
{
  return (unsigned int) m_A.cols();
}

void ConstraintEquality::resize(const unsigned int r, const unsigned int c)
{
  m_A.setZero(r, c);
  m_b.setZero(r);
}

bool ConstraintEquality::isEquality() const    { return true; }
bool ConstraintEquality::isInequality() const  { return false; }
bool ConstraintEquality::isBound() const       { return false; }

const Vector & ConstraintEquality::vector()     const { return m_b; }
const Vector & ConstraintEquality::lowerBound() const { assert(false); }
const Vector & ConstraintEquality::upperBound() const { assert(false); }

Vector & ConstraintEquality::vector()     { return m_b; }
Vector & ConstraintEquality::lowerBound() { assert(false); }
Vector & ConstraintEquality::upperBound() { assert(false); }

bool ConstraintEquality::setVector(ConstRefVector b) { m_b = b; return true; }
bool ConstraintEquality::setLowerBound(ConstRefVector lb) { assert(false); return false; }
bool ConstraintEquality::setUpperBound(ConstRefVector ub) { assert(false); return false; }

bool ConstraintEquality::checkConstraint(ConstRefVector x, double tol) const
{
  return (m_A*x-m_b).norm() < tol;
}
