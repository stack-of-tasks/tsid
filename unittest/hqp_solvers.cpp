//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>
#include <pininvdyn/math/constraint-equality.hpp>
#include <pininvdyn/math/constraint-inequality.hpp>
#include <pininvdyn/math/constraint-bound.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_eiquadprog_unconstrained)
{
  std::cout << "test_eiquadprog_unconstrained\n";
  using namespace pininvdyn;
  using namespace pininvdyn::math;
  using namespace pininvdyn::solvers;
  using namespace std;

  const unsigned int n = 5;
  const unsigned int m = 3;
  const unsigned int neq = 0;
  const unsigned int nin = 0;
  const double damping = 1e-4;
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG, "solver-eiquadprog");
  solver->resize(n, neq, nin);

  HqpData hqpData(2);
  Matrix A = Matrix::Random(m, n);
  Vector b = Vector::Random(m);
  ConstraintEquality constraint1("c1", A, b);
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

  const HqpOutput & output = solver->solve(hqpData);
  BOOST_CHECK_MESSAGE(output.status==HQP_STATUS_OPTIMAL, "Status "+toString(output.status));

  Vector x(n);
  svdSolveWithDamping(A, b, x, damping);
  BOOST_CHECK_MESSAGE(x.isApprox(output.x, 1e-3), "Solution error: "+toString(x-output.x));
}

BOOST_AUTO_TEST_CASE ( test_eiquadprog_equality_constrained)
{
  std::cout << "test_eiquadprog_equality_constrained\n";
  using namespace pininvdyn;
  using namespace pininvdyn::math;
  using namespace pininvdyn::solvers;
  using namespace std;

  const unsigned int n = 5;
  const unsigned int m = 3;
  const unsigned int neq = 2;
  const unsigned int nin = 0;
  const double damping = 1e-4;
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(n, neq, nin);

  HqpData hqpData(2);

  Matrix A_eq = Matrix::Random(neq, n);
  Vector b_eq = Vector::Random(neq);
  ConstraintEquality eq_constraint("eq1", A_eq, b_eq);
  hqpData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &eq_constraint));

  Matrix A = Matrix::Random(m, n);
  Vector b = Vector::Random(m);
  ConstraintEquality constraint1("c1", A, b);
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

  const HqpOutput & output = solver->solve(hqpData);
  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
                        "Status "+toString(output.status));
  BOOST_CHECK_MESSAGE(b_eq.isApprox(A_eq*output.x),
                      "Constraint error: "+toString(b_eq-A_eq*output.x));
}

BOOST_AUTO_TEST_CASE ( test_eiquadprog_inequality_constrained)
{
  std::cout << "test_eiquadprog_inequality_constrained\n";
  using namespace pininvdyn;
  using namespace pininvdyn::math;
  using namespace pininvdyn::solvers;
  using namespace std;

  const unsigned int n = 5;
  const unsigned int m = 3;
  const unsigned int neq = 0;
  const unsigned int nin = 3;
  const double damping = 1e-5;
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(n, neq, nin);

  HqpData hqpData(2);

  Matrix A = Matrix::Random(m, n);
  Vector b = Vector::Random(m);
  ConstraintEquality constraint1("c1", A, b);
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
  hqpData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

  Vector x(n);
  svdSolveWithDamping(A, b, x, damping);

  Matrix A_in = Matrix::Random(nin, n);
  Vector A_lb = A_in*x - Vector::Ones(nin) + Vector::Random(nin);
  Vector A_ub = A_in*x + Vector::Ones(nin) + Vector::Random(nin);
  ConstraintInequality in_constraint("in1", A_in, A_lb, A_ub);
  hqpData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &in_constraint));

  const HqpOutput & output = solver->solve(hqpData);
  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
                        "Status "+toString(output.status));
  BOOST_CHECK_MESSAGE(((A_in*output.x).array() <= A_ub.array()).all(),
                      "Upper bound error: "+toString(A_ub - A_in*output.x));
  BOOST_CHECK_MESSAGE(((A_in*output.x).array() >= A_lb.array()).all(),
                      "Lower bound error: "+toString(A_in*output.x-A_lb));

  A_lb[0] += 2.0;
  in_constraint.setLowerBound(A_lb);
  const HqpOutput & output2 = solver->solve(hqpData);
  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
                        "Status "+toString(output.status));
  BOOST_CHECK_MESSAGE((A_in.row(0)*output.x).isApproxToConstant(A_lb[0]),
                      "Active constraint error: "+toString(A_in.row(0)*output.x-A_lb.head<1>()));
}

BOOST_AUTO_TEST_SUITE_END ()
