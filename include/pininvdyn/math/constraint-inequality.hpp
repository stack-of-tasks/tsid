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

#ifndef __invdyn_math_constraint_inequality_hpp__
#define __invdyn_math_constraint_inequality_hpp__

#include <Eigen/Core>
#include <pininvdyn/math/constraint-base.hpp>
#include <exception>

namespace pininvdyn
{
  namespace math
  {

    class ConstraintInequality
        :public pininvdyn::math::ConstraintBase
    {
    public:

      ConstraintInequality(std::string name):
        ConstraintBase(name)
      {}

      ConstraintInequality(std::string name, const unsigned int rows, const unsigned int cols):
        ConstraintBase(name, rows, cols),
        m_lb(Vector::Zero(rows)),
        m_ub(Vector::Zero(rows))
      {}

      ConstraintInequality(std::string name, const Matrix & A,
                           const Vector & lb, const Vector & ub):
        ConstraintBase(name, A),
        m_lb(lb),
        m_ub(ub)
      {
        assert(A.rows()==lb.rows());
        assert(A.rows()==ub.rows());
      }

      inline unsigned int rows() const
      {
        assert(m_A.rows()==m_lb.rows());
        assert(m_A.rows()==m_ub.rows());
        return (unsigned int) m_A.rows();
      }

      inline unsigned int cols() const
      {
        return (unsigned int) m_A.cols();
      }

      inline bool isEquality() const    { return false; }
      inline bool isInequality() const  { return true; }
      inline bool isBound() const       { return false; }

      inline const Vector & vector()     const { assert(false); }
      inline const Vector & lowerBound() const { return m_lb; }
      inline const Vector & upperBound() const { return m_ub; }

      inline bool setVector(ConstRefVector b) { assert(false); return false; }
      inline bool setLowerBound(ConstRefVector lb) { m_lb = lb; return true; }
      inline bool setUpperBound(ConstRefVector ub) { m_ub = ub; return true; }

    protected:
      Vector m_lb;
      Vector m_ub;
    };

  }
}

#endif // ifndef __invdyn_math_constraint_equality_hpp__
