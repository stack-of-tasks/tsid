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

#ifndef __invdyn_math_constraint_base_hpp__
#define __invdyn_math_constraint_base_hpp__

#include <pininvdyn/math/utils.hpp>
#include <Eigen/Core>
#include <string>

namespace pininvdyn
{
  namespace math
  {

    class ConstraintBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ConstraintBase(std::string name):
        m_name(name){}

      ConstraintBase(std::string name, const unsigned int rows, const unsigned int cols):
        m_name(name)
      {
        m_A = Matrix::Zero(rows, cols);
      }

      ConstraintBase(std::string name, const Matrix & A):
        m_name(name),
        m_A(A)
      {}

      virtual unsigned int rows() const = 0;
      virtual unsigned int cols() const = 0;

      virtual bool isEquality() const = 0;
      virtual bool isInequality() const = 0;
      virtual bool isBound() const = 0;

      virtual const Matrix & matrix() const { return m_A; }
      virtual const Vector & vector() const = 0;
      virtual const Vector & lowerBound() const = 0;
      virtual const Vector & upperBound() const = 0;

      virtual bool setMatrix(ConstRefMatrix A) { m_A = A; return true; }
      virtual bool setVector(ConstRefVector b) = 0;
      virtual bool setLowerBound(ConstRefVector lb) = 0;
      virtual bool setUpperBound(ConstRefVector ub) = 0;

    protected:
      Matrix m_A;
      std::string m_name;
    };
    
  }
}

#endif // ifndef __invdyn_math_constraint_base_hpp__
