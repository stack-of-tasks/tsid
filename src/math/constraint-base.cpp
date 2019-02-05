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

#include <tsid/math/constraint-base.hpp>

using namespace tsid::math;

ConstraintBase::ConstraintBase(const std::string & name):
  m_name(name){}

ConstraintBase::ConstraintBase(const std::string & name,
                               const unsigned int rows,
                               const unsigned int cols):
  m_name(name)
{
  m_A = Matrix::Zero(rows, cols);
}

ConstraintBase::ConstraintBase(const std::string & name,
                               ConstRefMatrix A):
  m_name(name),
  m_A(A)
{}

const std::string & ConstraintBase::name() const
{
  return m_name;
}

const Matrix & ConstraintBase::matrix() const
{
  return m_A;
}

Matrix & ConstraintBase::matrix()
{
  return m_A;
}

bool ConstraintBase::setMatrix(ConstRefMatrix A)
{
  assert(m_A.cols()==A.cols());
  assert(m_A.rows()==A.rows());
  m_A = A;
  return true;
}
