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

#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace tsid;
using namespace tsid::math;
using namespace tsid::robots;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_robot_wrapper )
{

  using namespace std;
  using namespace pinocchio;
  
  const string romeo_model_path = TSID_SOURCE_DIR"/models/romeo";

  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
  
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);
  
  const Model & model = robot.model();
  
  // Update default config bounds to take into account the Free Flyer
  Vector lb(model.lowerPositionLimit);
  lb.head<3>().fill(-10.);
  lb.segment<4>(3).fill(-1.);
  
  Vector ub(model.upperPositionLimit);
  ub.head<3>().fill(10.);
  ub.segment<4>(3).fill(1.);
  
  Vector q = pinocchio::randomConfiguration(model,lb,ub);
  Vector v = Vector::Ones(robot.nv());
  Data data(robot.model());
  robot.computeAllTerms(data, q, v);

  Vector3 com = robot.com(data);
  std::cout << com << std::endl;
  BOOST_CHECK(robot.nq() == 38);
  BOOST_CHECK(robot.nv() == 37);
}

BOOST_AUTO_TEST_SUITE_END ()
