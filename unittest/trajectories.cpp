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
#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_trajectory_se3 )
{
  using namespace tsid;
  using namespace trajectories;
  using namespace math;
  using namespace std;
  using namespace Eigen;
  using namespace pinocchio;

  SE3 M_ref = SE3::Identity();
  VectorXd M_vec(12);
  SE3ToVector(M_ref, M_vec);
  VectorXd zero = VectorXd::Zero(6);

  TrajectoryBase *traj = new TrajectorySE3Constant("traj_se3", M_ref);
  BOOST_CHECK(traj->has_trajectory_ended());
  BOOST_CHECK(traj->computeNext().pos.isApprox(M_vec));
  BOOST_CHECK(traj->operator ()(0.0).pos.isApprox(M_vec));

  TrajectorySample sample(12, 6);
  traj->getLastSample(sample);
  BOOST_CHECK(sample.pos.isApprox(M_vec));
  BOOST_CHECK(sample.vel.isApprox(zero));
  BOOST_CHECK(sample.acc.isApprox(zero));
}

BOOST_AUTO_TEST_CASE ( test_trajectory_euclidian )
{
  using namespace tsid;
  using namespace trajectories;
  using namespace std;
  using namespace Eigen;

  const unsigned int n = 5;
  VectorXd q_ref = VectorXd::Ones(n);
  VectorXd zero = VectorXd::Zero(n);
  TrajectoryBase *traj = new TrajectoryEuclidianConstant("traj_eucl", q_ref);

  BOOST_CHECK(traj->has_trajectory_ended());
  BOOST_CHECK(traj->computeNext().pos.isApprox(q_ref));
  BOOST_CHECK(traj->operator ()(0.0).pos.isApprox(q_ref));

  TrajectorySample sample(n);
  traj->getLastSample(sample);
  BOOST_CHECK(sample.pos.isApprox(q_ref));
  BOOST_CHECK(sample.vel.isApprox(zero));
  BOOST_CHECK(sample.acc.isApprox(zero));
}


BOOST_AUTO_TEST_SUITE_END ()
