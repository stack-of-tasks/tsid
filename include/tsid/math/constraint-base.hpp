//
// Copyright (c) 2017 CNRS
//

#ifndef __invdyn_math_constraint_base_hpp__
#define __invdyn_math_constraint_base_hpp__

#include "tsid/math/fwd.hpp"
#include <string>
#include <pinocchio/macros.hpp>

namespace tsid {
namespace math {

/**
 * @brief Abstract class representing a linear equality/inequality constraint.
 * Equality constraints are represented by a matrix A and a vector b: A*x = b
 * Inequality constraints are represented by a matrix A and two vectors
 * lb and ub: lb <= A*x <= ub
 * Bounds are represented by two vectors lb and ub: lb <= x <= ub
 */
class ConstraintBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  ConstraintBase(const std::string& name);

  ConstraintBase(const std::string& name, const unsigned int rows,
                 const unsigned int cols);

  ConstraintBase(const std::string& name, ConstRefMatrix A);
  virtual ~ConstraintBase() = default;

  virtual const std::string& name() const;
  virtual unsigned int rows() const = 0;
  virtual unsigned int cols() const = 0;
  virtual void resize(const unsigned int r, const unsigned int c) = 0;

  virtual bool isEquality() const = 0;
  virtual bool isInequality() const = 0;
  virtual bool isBound() const = 0;

  virtual const Matrix& matrix() const;
  virtual const Vector& vector() const = 0;
  virtual const Vector& lowerBound() const = 0;
  virtual const Vector& upperBound() const = 0;

  virtual Matrix& matrix();
  virtual Vector& vector() = 0;
  virtual Vector& lowerBound() = 0;
  virtual Vector& upperBound() = 0;

  virtual bool setMatrix(ConstRefMatrix A);
  virtual bool setVector(ConstRefVector b) = 0;
  virtual bool setLowerBound(ConstRefVector lb) = 0;
  virtual bool setUpperBound(ConstRefVector ub) = 0;

  virtual bool checkConstraint(ConstRefVector x, double tol = 1e-6) const = 0;

 protected:
  std::string m_name;
  Matrix m_A;
};

}  // namespace math
}  // namespace tsid

#endif  // ifndef __invdyn_math_constraint_base_hpp__
