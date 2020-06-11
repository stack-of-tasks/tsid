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
#include "tsid/robots/robot-wrapper.hpp"


using namespace tsid;
using namespace tsid::robots;
using namespace std;
using namespace pinocchio;

typedef pinocchio::Motion Motion;



BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_set_gravity )
{
	
  const string romeo_model_path = TSID_SOURCE_DIR"/models/romeo";
	
  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);


  Motion g = pinocchio::Motion::Zero();
  Motion init_gravity = robot.model().gravity;
 
  robot.setGravity(g);
  
  Motion no_gravity = robot.model().gravity;
  
  
  BOOST_CHECK(no_gravity != init_gravity);

}


BOOST_AUTO_TEST_SUITE_END ()
