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

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <tsid/math/constraint-bound.hpp>
#include <tsid/math/constraint-equality.hpp>
#include <tsid/math/constraint-inequality.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_constraint_bounds )
{
  std::cout << "test_constraint_bounds\n";
  using namespace tsid::math;
  using namespace Eigen;
  const unsigned int n = 5;

  VectorXd lb = -1.0*VectorXd::Ones(n);
  VectorXd ub = VectorXd::Ones(n);
  ConstraintBound bounds("bounds", lb, ub);

  BOOST_CHECK(bounds.isBound());
  BOOST_CHECK(!bounds.isEquality());
  BOOST_CHECK(!bounds.isInequality());

  BOOST_CHECK(bounds.rows()==n);
  BOOST_CHECK(bounds.cols()==n);

  BOOST_CHECK(lb.isApprox(bounds.lowerBound()));
  BOOST_CHECK(ub.isApprox(bounds.upperBound()));

  lb *= 2.0;
  BOOST_CHECK(! lb.isApprox(bounds.lowerBound()));
  bounds.setLowerBound(lb);
  BOOST_CHECK(lb.isApprox(bounds.lowerBound()));

  ub *= 2.0;
  BOOST_CHECK(! ub.isApprox(bounds.upperBound()));
  bounds.setUpperBound(ub);
  BOOST_CHECK(ub.isApprox(bounds.upperBound()));
}

BOOST_AUTO_TEST_CASE ( test_constraint_equality )
{
  using namespace tsid::math;
  using namespace Eigen;
  using namespace std;

  const unsigned int n = 5;
  const unsigned int m = 2;

  MatrixXd A = MatrixXd::Ones(m,n);
  VectorXd b = VectorXd::Ones(m);
  ConstraintEquality equality("equality", A, b);

  BOOST_CHECK(!equality.isBound());
  BOOST_CHECK(equality.isEquality());
  BOOST_CHECK(!equality.isInequality());

  BOOST_CHECK(equality.rows()==m);
  BOOST_CHECK(equality.cols()==n);

  BOOST_CHECK(A.isApprox(equality.matrix()));
  BOOST_CHECK(b.isApprox(equality.vector()));

  b *= 2.0;
  BOOST_CHECK(! b.isApprox(equality.vector()));
  equality.setVector(b);
  BOOST_CHECK(b.isApprox(equality.vector()));

  A *= 2.0;
  BOOST_CHECK(! A.isApprox(equality.matrix()));
  equality.setMatrix(A);
  BOOST_CHECK(A.isApprox(equality.matrix()));
}

BOOST_AUTO_TEST_CASE ( test_constraint_inequality )
{
  using namespace tsid::math;
  using namespace Eigen;
  using namespace std;

  const unsigned int n = 5;
  const unsigned int m = 2;

  MatrixXd A = MatrixXd::Ones(m,n);
  VectorXd lb = -1.0*VectorXd::Ones(m);
  VectorXd ub = VectorXd::Ones(m);
  ConstraintInequality inequality("inequality", A, lb, ub);

  BOOST_CHECK(!inequality.isBound());
  BOOST_CHECK(!inequality.isEquality());
  BOOST_CHECK(inequality.isInequality());

  BOOST_CHECK(inequality.rows()==m);
  BOOST_CHECK(inequality.cols()==n);

  BOOST_CHECK(A.isApprox(inequality.matrix()));
  BOOST_CHECK(lb.isApprox(inequality.lowerBound()));
  BOOST_CHECK(ub.isApprox(inequality.upperBound()));

  lb *= 2.0;
  BOOST_CHECK(! lb.isApprox(inequality.lowerBound()));
  inequality.setLowerBound(lb);
  BOOST_CHECK(lb.isApprox(inequality.lowerBound()));

  A *= 2.0;
  BOOST_CHECK(! A.isApprox(inequality.matrix()));
  inequality.setMatrix(A);
  BOOST_CHECK(A.isApprox(inequality.matrix()));
}

BOOST_AUTO_TEST_SUITE_END ()
