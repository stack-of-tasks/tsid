//
// Copyright (c) 2017 CNRS
//

#include <iostream>

#include <boost/test/unit_test.hpp>
#include <boost/utility/binary.hpp>

#include "tsid/robots/robot-wrapper.hpp"
#include <pinocchio/algorithm/joint-configuration.hpp>

using namespace tsid;
using namespace tsid::math;
using namespace tsid::robots;

BOOST_AUTO_TEST_SUITE(BOOST_TEST_MODULE)

BOOST_AUTO_TEST_CASE(test_robot_wrapper) {
  using namespace std;
  using namespace pinocchio;

  const string romeo_model_path = TSID_SOURCE_DIR "/models/romeo";

  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";

  RobotWrapper robot(urdfFileName, package_dirs,
                     pinocchio::JointModelFreeFlyer(), false);

  const Model& model = robot.model();

  // Update default config bounds to take into account the Free Flyer
  Vector lb(model.lowerPositionLimit);
  lb.head<3>().fill(-10.);
  lb.segment<4>(3).fill(-1.);

  Vector ub(model.upperPositionLimit);
  ub.head<3>().fill(10.);
  ub.segment<4>(3).fill(1.);

  Vector q = pinocchio::randomConfiguration(model, lb, ub);
  Vector v = Vector::Ones(robot.nv());
  Data data(robot.model());
  robot.computeAllTerms(data, q, v);

  Vector3 com = robot.com(data);
  std::cout << com << std::endl;
  BOOST_CHECK(robot.nq() == 38);
  BOOST_CHECK(robot.nv() == 37);
}

BOOST_AUTO_TEST_SUITE_END()
