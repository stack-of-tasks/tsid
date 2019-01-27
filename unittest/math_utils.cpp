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

#include <tsid/math/utils.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_pseudoinverse)
{
  std::cout << "test_pseudoinverse\n";
  using namespace tsid::math;
  const unsigned int m = 3;
  const unsigned int n = 5;

  Matrix A = Matrix::Random(m,n);
  Matrix Apinv = Matrix::Zero(n,m);
  pseudoInverse(A, Apinv, 1e-5);

  BOOST_CHECK(Matrix::Identity(m,m).isApprox(A*Apinv));
}


BOOST_AUTO_TEST_SUITE_END ()
