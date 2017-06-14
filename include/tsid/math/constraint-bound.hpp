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

#ifndef __invdyn_math_constraint_bound_hpp__
#define __invdyn_math_constraint_bound_hpp__

#include "tsid/math/constraint-base.hpp"

namespace tsid
{
  namespace math
  {

    class ConstraintBound : public ConstraintBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      ConstraintBound(const std::string & name);

      ConstraintBound(const std::string & name,
                      const unsigned int size);

      ConstraintBound(const std::string & name,
                      ConstRefVector lb,
                      ConstRefVector ub);

      unsigned int rows() const;
      unsigned int cols() const;
      void resize(const unsigned int r, const unsigned int c);

      bool isEquality() const;
      bool isInequality() const;
      bool isBound() const;

      const Vector & vector()     const;
      const Vector & lowerBound() const;
      const Vector & upperBound() const;

      Vector & vector();
      Vector & lowerBound();
      Vector & upperBound();

      bool setVector(ConstRefVector b);
      bool setLowerBound(ConstRefVector lb);
      bool setUpperBound(ConstRefVector ub);

      bool checkConstraint(ConstRefVector x, double tol=1e-6) const;

    protected:
      
      Vector m_lb;
      Vector m_ub;
    };

  }
}

#endif // ifndef __invdyn_math_constraint_bound_hpp__
