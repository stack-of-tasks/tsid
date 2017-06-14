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

#ifndef __invdyn_math_constraint_base_hpp__
#define __invdyn_math_constraint_base_hpp__

#include "tsid/math/fwd.hpp"
#include <string>

namespace tsid
{
  namespace math
  {

    /**
     * @brief Abstract class representing a linear equality/inequality constraint.
     * Equality constraints are represented by a matrix A and a vector b: A*x = b
     * Inequality constraints are represented by a matrix A and two vectors
     * lb and ub: lb <= A*x <= ub
     * Bounds are represented by two vectors lb and ub: lb <= x <= ub
     */
    class ConstraintBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ConstraintBase(const std::string & name);

      ConstraintBase(const std::string & name,
                     const unsigned int rows,
                     const unsigned int cols);

      ConstraintBase(const std::string & name,
                     ConstRefMatrix A);

      virtual const std::string & name() const;
      virtual unsigned int rows() const = 0;
      virtual unsigned int cols() const = 0;
      virtual void resize(const unsigned int r, const unsigned int c) = 0;

      virtual bool isEquality() const = 0;
      virtual bool isInequality() const = 0;
      virtual bool isBound() const = 0;

      virtual const Matrix & matrix() const;
      virtual const Vector & vector() const = 0;
      virtual const Vector & lowerBound() const = 0;
      virtual const Vector & upperBound() const = 0;

      virtual Matrix & matrix();
      virtual Vector & vector() = 0;
      virtual Vector & lowerBound() = 0;
      virtual Vector & upperBound() = 0;

      virtual bool setMatrix(ConstRefMatrix A);
      virtual bool setVector(ConstRefVector b) = 0;
      virtual bool setLowerBound(ConstRefVector lb) = 0;
      virtual bool setUpperBound(ConstRefVector ub) = 0;

      virtual bool checkConstraint(ConstRefVector x, double tol=1e-6) const = 0;

    protected:
      
      std::string m_name;
      Matrix m_A;
    };
    
  }
}

#endif // ifndef __invdyn_math_constraint_base_hpp__
