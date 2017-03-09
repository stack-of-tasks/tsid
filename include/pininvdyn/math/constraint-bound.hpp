//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_math_constraint_bound_hpp__
#define __invdyn_math_constraint_bound_hpp__

#include <Eigen/Core>
#include <pininvdyn/math/constraint-base.hpp>
#include <exception>

namespace pininvdyn
{
  namespace math
  {

    class ConstraintBound
        :public pininvdyn::math::ConstraintBase
    {
    public:

      ConstraintBound(std::string name):
        ConstraintBase(name)
      {}

      ConstraintBound(std::string name, const unsigned int size):
        ConstraintBase(name),
        m_lb(Vector::Zero(size)),
        m_ub(Vector::Zero(size))
      {}

      ConstraintBound(std::string name, const Vector & lb, const Vector & ub):
        ConstraintBase(name),
        m_lb(lb),
        m_ub(ub)
      {}

      inline unsigned int rows() const
      {
        assert(m_lb.rows()==m_ub.rows());
        return (unsigned int) m_lb.rows();
      }

      inline unsigned int cols() const
      {
        assert(m_lb.rows()==m_ub.rows());
        return (unsigned int) m_lb.rows();
      }

      inline bool isEquality() const    { return false; }
      inline bool isInequality() const  { return false; }
      inline bool isBound() const       { return true; }

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

#endif // ifndef __invdyn_math_constraint_bound_hpp__
