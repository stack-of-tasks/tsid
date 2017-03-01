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

#include <pininvdyn/robot-wrapper.hpp>

#define HRP2_PKG_DIR "/home/adelpret/devel/sot_hydro/install/share"

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_robot_wrapper )
{
  using namespace pininvdyn;
  using namespace std;
  using namespace Eigen;
  using namespace se3;

  vector<string> package_dirs;
  package_dirs.push_back(HRP2_PKG_DIR);
  string urdfFileName = package_dirs[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     se3::JointModelFreeFlyer(),
                     false);
  VectorXd q = VectorXd::Ones(robot.nq());
  VectorXd v = VectorXd::Ones(robot.nv());
  se3::Data data(robot.model());
  robot.computeAllTerms(data, q, v);

  VectorXd com = robot.com(data);
//  cout << "Robot com:" << com.transpose() << endl;
}

BOOST_AUTO_TEST_SUITE_END ()
