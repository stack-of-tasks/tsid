//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn. If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_math_constraint_equality_hpp__
#define __invdyn_math_constraint_equality_hpp__

#include <Eigen/Core>
#include <pininvdyn/math/constraint-base.hpp>
#include <exception>

namespace pininvdyn
{
  namespace math
  {

    class ConstraintEquality
        :public pininvdyn::math::ConstraintBase
    {
    public:

      ConstraintEquality(std::string name):
        ConstraintBase(name)
      {}

      ConstraintEquality(std::string name, const unsigned int rows, const unsigned int cols):
        ConstraintBase(name, rows, cols),
        m_b(Vector::Zero(rows))
      {}

      ConstraintEquality(std::string name, const Matrix & A, const Vector & b):
        ConstraintBase(name, A),
        m_b(b)
      {
        assert(A.rows()==b.rows());
      }

      inline unsigned int rows() const
      {
        assert(m_A.rows()==m_b.rows());
        return (unsigned int) m_A.rows();
      }

      inline unsigned int cols() const
      {
        return (unsigned int) m_A.cols();
      }

      inline bool isEquality() const    { return true; }
      inline bool isInequality() const  { return false; }
      inline bool isBound() const       { return false; }

      inline const Vector & vector()     const { return m_b; }
      inline const Vector & lowerBound() const { assert(false); }
      inline const Vector & upperBound() const { assert(false); }

      inline bool setVector(ConstRefVector b) { m_b = b; return true; }
      inline bool setLowerBound(ConstRefVector lb) { assert(false); }
      inline bool setUpperBound(ConstRefVector ub) { assert(false); }

    protected:
      Vector m_b;
    };

  }
}

#endif // ifndef __invdyn_math_constraint_equality_hpp__
