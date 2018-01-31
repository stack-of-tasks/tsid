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

#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/solver-HQP-eiquadprog.hpp>
#include <tsid/solvers/solver-HQP-eiquadprog-rt.hpp>
#include <tsid/math/utils.hpp>
#include <tsid/math/constraint-equality.hpp>
#include <tsid/math/constraint-inequality.hpp>
#include <tsid/math/constraint-bound.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>

#define CHECK_LESS_THAN(A,B) BOOST_CHECK_MESSAGE(A<B, #A<<": "<<A<<">"<<B)
#define REQUIRE_FINITE(A) BOOST_REQUIRE_MESSAGE(isFinite(A), #A<<": "<<A)

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

//BOOST_AUTO_TEST_CASE ( test_eiquadprog_unconstrained)
//{
//  std::cout << "test_eiquadprog_unconstrained\n";
//  using namespace tsid;
//  using namespace math;
//  using namespace solvers;
//  using namespace std;

//  const unsigned int n = 5;
//  const unsigned int m = 3;
//  const unsigned int neq = 0;
//  const unsigned int nin = 0;
//  const double damping = 1e-4;
//  SolverHQPBase * solver = SolverHQPBase::getNewSolver(SOLVER_HQP_EIQUADPROG, "solver-eiquadprog");
//  solver->resize(n, neq, nin);

//  HQPData HQPData(2);
//  Matrix A = Matrix::Random(m, n);
//  Vector b = Vector::Random(m);
//  ConstraintEquality constraint1("c1", A, b);
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

//  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

//  const HQPOutput & output = solver->solve(HQPData);
//  BOOST_CHECK_MESSAGE(output.status==HQP_STATUS_OPTIMAL, "Status "+toString(output.status));

//  Vector x(n);
//  svdSolveWithDamping(A, b, x, damping);
//  BOOST_CHECK_MESSAGE(x.isApprox(output.x, 1e-3), "Solution error: "+toString(x-output.x));
//}

//BOOST_AUTO_TEST_CASE ( test_eiquadprog_equality_constrained)
//{
//  std::cout << "test_eiquadprog_equality_constrained\n";
//  using namespace tsid;
//  using namespace math;
//  using namespace solvers;
//  using namespace std;

//  const unsigned int n = 5;
//  const unsigned int m = 3;
//  const unsigned int neq = 2;
//  const unsigned int nin = 0;
//  const double damping = 1e-4;
//  SolverHQPBase * solver = SolverHQPBase::getNewSolver(SOLVER_HQP_EIQUADPROG,
//                                                           "solver-eiquadprog");
//  solver->resize(n, neq, nin);

//  HQPData HQPData(2);

//  Matrix A_eq = Matrix::Random(neq, n);
//  Vector b_eq = Vector::Random(neq);
//  ConstraintEquality eq_constraint("eq1", A_eq, b_eq);
//  HQPData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &eq_constraint));

//  Matrix A = Matrix::Random(m, n);
//  Vector b = Vector::Random(m);
//  ConstraintEquality constraint1("c1", A, b);
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

//  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

//  const HQPOutput & output = solver->solve(HQPData);
//  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
//                        "Status "+toString(output.status));
//  BOOST_CHECK_MESSAGE(b_eq.isApprox(A_eq*output.x),
//                      "Constraint error: "+toString(b_eq-A_eq*output.x));
//}

//BOOST_AUTO_TEST_CASE ( test_eiquadprog_inequality_constrained)
//{
//  std::cout << "test_eiquadprog_inequality_constrained\n";
//  using namespace tsid;
//  using namespace math;
//  using namespace solvers;
//  using namespace std;

//  const unsigned int n = 5;
//  const unsigned int m = 3;
//  const unsigned int neq = 0;
//  const unsigned int nin = 3;
//  const double damping = 1e-5;
//  SolverHQPBase * solver = SolverHQPBase::getNewSolver(SOLVER_HQP_EIQUADPROG,
//                                                           "solver-eiquadprog");
//  solver->resize(n, neq, nin);

//  HQPData HQPData(2);

//  Matrix A = Matrix::Random(m, n);
//  Vector b = Vector::Random(m);
//  ConstraintEquality constraint1("c1", A, b);
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &constraint1));

//  ConstraintEquality constraint2("c2", Matrix::Identity(n,n), Vector::Zero(n));
//  HQPData[1].push_back(make_pair<double, ConstraintBase*>(damping, &constraint2));

//  Vector x(n);
//  svdSolveWithDamping(A, b, x, damping);

//  Matrix A_in = Matrix::Random(nin, n);
//  Vector A_lb = A_in*x - Vector::Ones(nin) + Vector::Random(nin);
//  Vector A_ub = A_in*x + Vector::Ones(nin) + Vector::Random(nin);
//  ConstraintInequality in_constraint("in1", A_in, A_lb, A_ub);
//  HQPData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &in_constraint));

//  const HQPOutput & output = solver->solve(HQPData);
//  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
//                        "Status "+toString(output.status));
//  BOOST_CHECK_MESSAGE(((A_in*output.x).array() <= A_ub.array()).all(),
//                      "Upper bound error: "+toString(A_ub - A_in*output.x));
//  BOOST_CHECK_MESSAGE(((A_in*output.x).array() >= A_lb.array()).all(),
//                      "Lower bound error: "+toString(A_in*output.x-A_lb));

//  A_lb[0] += 2.0;
//  in_constraint.setLowerBound(A_lb);
//  const HQPOutput & output2 = solver->solve(HQPData);
//  BOOST_REQUIRE_MESSAGE(output.status==HQP_STATUS_OPTIMAL,
//                        "Status "+toString(output.status));
//  BOOST_CHECK_MESSAGE((A_in.row(0)*output.x).isApproxToConstant(A_lb[0]),
//      "Active constraint error: "+toString(A_in.row(0)*output.x-A_lb.head<1>()));
//}


#define PROFILE_EIQUADPROG "Eiquadprog"
#define PROFILE_EIQUADPROG_RT "Eiquadprog Real Time"
#define PROFILE_EIQUADPROG_FAST "Eiquadprog Fast"

BOOST_AUTO_TEST_CASE ( test_eiquadprog_classic_vs_rt_vs_fast)
{
  std::cout << "test_eiquadprog_classic_vs_rt_vs_fast\n";
  using namespace tsid;
  using namespace math;
  using namespace solvers;

  const double EPS = 1e-8;
#ifdef NDEBUG
  const unsigned int nTest = 1000;
#else
  const unsigned int nTest = 100;
#endif
  const unsigned int n = 60;
  const unsigned int neq = 36;
  const unsigned int nin = 40;
  const double damping = 1e-10;

  // variance of the normal distribution used for generating initial bounds
  const double NORMAL_DISTR_VAR = 10.0;
  // each cycle the gradient is perturbed by a Gaussian random variable with this covariance
  const double GRADIENT_PERTURBATION_VARIANCE = 1e-2;
  // each cycle the Hessian is perturbed by a Gaussian random variable with this covariance
  const double HESSIAN_PERTURBATION_VARIANCE = 1e-1;
  // min margin of activation of the inequality constraints for the first QP
  const double MARGIN_PERC = 1e-3;

  std::cout<<"Gonna perform "<<nTest<<" tests with "<<n<<" variables, "<<
	  neq<<" equalities, "<<nin<<" inequalities\n";

  // CREATE SOLVERS
  SolverHQPBase * solver_rt = SolverHQPFactory::createNewSolver<n,neq,nin>(SOLVER_HQP_EIQUADPROG_RT,
                                                                           "eiquadprog_rt");
  solver_rt->resize(n, neq, nin);

  SolverHQPBase * solver_fast = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                                  "eiquadprog_fast");
  solver_fast->resize(n, neq, nin);

  SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG,
                                                             "eiquadprog");
  solver->resize(n, neq, nin);

  // CREATE PROBLEM DATA
  HQPData HQPData(2);

  Matrix A1 = Matrix::Random(n, n);
  Vector b1 = Vector::Random(n);
  ConstraintEquality cost("c1", A1, b1);
  HQPData[1].push_back(make_pair<double, ConstraintBase*>(1.0, &cost));

  Vector x(n);
  svdSolveWithDamping(A1, b1, x, damping);

  Matrix A_in = Matrix::Random(nin, n);
  Vector A_lb = Vector::Random(nin)*NORMAL_DISTR_VAR;
  Vector A_ub = Vector::Random(nin)*NORMAL_DISTR_VAR;
  Vector constrVal = A_in*x;
  for(unsigned int i=0; i<nin; i++)
  {
      if(constrVal[i]>A_ub[i])
      {
//        std::cout<<"Inequality constraint "<<i<<" active at first iteration. UB="<<A_ub[i]<<", value="<<constrVal[i]<<endl;
        A_ub[i] = constrVal[i] + MARGIN_PERC*fabs(constrVal[i]);
      }
      if(constrVal[i]<A_lb[i])
      {
//        std::cout<<"Inequality constraint "<<i<<" active at first iteration. LB="<<A_lb[i]<<", value="<<constrVal[i]<<endl;
        A_lb[i] = constrVal[i] - MARGIN_PERC*fabs(constrVal[i]);
      }
  }
  ConstraintInequality in_constraint("in1", A_in, A_lb, A_ub);
  HQPData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &in_constraint));

  Matrix A_eq = Matrix::Random(neq, n);
  Vector b_eq = A_eq*x;
  ConstraintEquality eq_constraint("eq1", A_eq, b_eq);
  HQPData[0].push_back(make_pair<double, ConstraintBase*>(1.0, &eq_constraint));

  // Prepare random data to perturb initial QP
  std::vector<Vector> gradientPerturbations(nTest);
  std::vector<Matrix> hessianPerturbations(nTest);
  for(unsigned int i=0; i<nTest; i++)
  {
    gradientPerturbations[i] = Vector::Random(n)*GRADIENT_PERTURBATION_VARIANCE;
    hessianPerturbations[i] = Matrix::Random(n,n)*HESSIAN_PERTURBATION_VARIANCE;
  }

  // START COMPUTING
  for(unsigned int i=0; i<nTest; i++)
  {
    if(true || i==0)
    {
      cost.matrix() += hessianPerturbations[i];
      cost.vector() += gradientPerturbations[i];
    }

    getProfiler().start(PROFILE_EIQUADPROG_FAST);
    const HQPOutput & output_fast = solver_fast->solve(HQPData);
    getProfiler().stop(PROFILE_EIQUADPROG_FAST);

    getProfiler().start(PROFILE_EIQUADPROG_RT);
    const HQPOutput & output_rt = solver_rt->solve(HQPData);
    getProfiler().stop(PROFILE_EIQUADPROG_RT);

    getProfiler().start(PROFILE_EIQUADPROG);
    const HQPOutput & output    = solver->solve(HQPData);
    getProfiler().stop(PROFILE_EIQUADPROG);

    getStatistics().store("active inequalities", (double)output_rt.activeSet.size());
    getStatistics().store("solver iterations", output_rt.iterations);

    BOOST_REQUIRE_MESSAGE(output.status==output_rt.status,
                          "Status "+SolverHQPBase::HQP_status_string[output.status]+
                          " Status RT "+SolverHQPBase::HQP_status_string[output_rt.status]);
    BOOST_REQUIRE_MESSAGE(output.status==output_fast.status,
                          "Status "+SolverHQPBase::HQP_status_string[output.status]+
                          " Status FAST "+SolverHQPBase::HQP_status_string[output_fast.status]);

    if(output.status==HQP_STATUS_OPTIMAL)
    {
      CHECK_LESS_THAN((A_eq*output.x - b_eq).norm(), EPS);

      BOOST_CHECK_MESSAGE(((A_in*output.x).array() <= A_ub.array() + EPS).all(),
          "Lower bounds violated: "+toString((A_ub-A_in*output.x).transpose()));

      BOOST_CHECK_MESSAGE(((A_in*output.x).array() >= A_lb.array() - EPS).all(),
          "Upper bounds violated: "+toString((A_in*output.x - A_lb).transpose()));

      BOOST_CHECK_MESSAGE(output.x.isApprox(output_rt.x, EPS),
  //                        "Sol "+toString(output.x.transpose())+
  //                        "\nSol RT "+toString(output_rt.x.transpose())+
                          "\nDiff RT: "+toString((output.x-output_rt.x).norm()));

      BOOST_CHECK_MESSAGE(output_rt.x.isApprox(output_fast.x, EPS),
  //                        "Sol RT"+toString(output_rt.x.transpose())+
  //                        "\nSol FAST "+toString(output_fast.x.transpose())+
                          "\nDiff FAST: "+toString((output_rt.x-output_fast.x).norm()));
    }
  }

  std::cout<<"\n### TEST FINISHED ###\n";
  getProfiler().report_all(3, std::cout);
  getStatistics().report_all(1, std::cout);
}



BOOST_AUTO_TEST_SUITE_END ()
