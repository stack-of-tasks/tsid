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
// tsid. If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_math_constraint_inequality_hpp__
#define __invdyn_math_constraint_inequality_hpp__

#include "tsid/math/constraint-base.hpp"

namespace tsid {
namespace math {

class ConstraintInequality : public ConstraintBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConstraintInequality(const std::string& name);

  ConstraintInequality(const std::string& name, const unsigned int rows,
                       const unsigned int cols);

  ConstraintInequality(const std::string& name, ConstRefMatrix A,
                       ConstRefVector lb, ConstRefVector ub);

  unsigned int rows() const override;
  unsigned int cols() const override;
  void resize(unsigned int r, unsigned int c) override;

  bool isEquality() const override;
  bool isInequality() const override;
  bool isBound() const override;

  const Vector& vector() const override;
  const Vector& lowerBound() const override;
  const Vector& upperBound() const override;

  Vector& vector() override;
  Vector& lowerBound() override;
  Vector& upperBound() override;

  bool setVector(ConstRefVector b) override;
  bool setLowerBound(ConstRefVector lb) override;
  bool setUpperBound(ConstRefVector ub) override;

  bool checkConstraint(ConstRefVector x, double tol = 1e-6) const override;

 protected:
  Vector m_lb;
  Vector m_ub;
};

}  // namespace math
}  // namespace tsid

#endif  // ifndef __invdyn_math_constraint_equality_hpp__
