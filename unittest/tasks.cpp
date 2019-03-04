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
#include <tsid/robots/robot-wrapper.hpp>

#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>

#include <tsid/trajectories/trajectory-se3.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>

#include <pinocchio/parsers/srdf.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/center-of-mass.hpp>
#include <Eigen/SVD>

using namespace tsid;
using namespace trajectories;
using namespace math;
using namespace tasks;
using namespace std;
using namespace Eigen;
using namespace tsid::robots;

#define REQUIRE_FINITE(A) BOOST_REQUIRE_MESSAGE(isFinite(A), #A<<": "<<A)

const string romeo_model_path = TSID_SOURCE_DIR"/models/romeo";

#ifndef NDEBUG
const int max_it = 100;
#else
const int max_it = 10000;
#endif

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

BOOST_AUTO_TEST_CASE ( test_task_se3_equality )
{
  cout<<"\n\n*********** TEST TASK SE3 EQUALITY ***********\n";
  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);

  TaskSE3Equality task("task-se3", robot, "RWristPitch");

  VectorXd Kp = VectorXd::Ones(6);
  VectorXd Kd = 2*VectorXd::Ones(6);
  task.Kp(Kp);
  task.Kd(Kd);
  BOOST_CHECK(task.Kp().isApprox(Kp));
  BOOST_CHECK(task.Kd().isApprox(Kd));

  pinocchio::SE3 M_ref = pinocchio::SE3::Random();
  TrajectoryBase *traj = new TrajectorySE3Constant("traj_SE3", M_ref);
  TrajectorySample sample;

  double t = 0.0;
  const double dt = 0.001;
  MatrixXd Jpinv(robot.nv(), 6);
  double error, error_past=1e100;
  VectorXd q = neutral(robot.model());
  VectorXd v = VectorXd::Zero(robot.nv());
  pinocchio::Data data(robot.model());
  for(int i=0; i<max_it; i++)
  {
    robot.computeAllTerms(data, q, v);
    sample = traj->computeNext();
    task.setReference(sample);
    const ConstraintBase & constraint = task.compute(t, q, v, data);
    BOOST_CHECK(constraint.rows()==6);
    BOOST_CHECK(static_cast<tsid::math::Index>(constraint.cols())==static_cast<tsid::math::Index>(robot.nv()));
    REQUIRE_FINITE(constraint.matrix());
    BOOST_REQUIRE(isFinite(constraint.vector()));

    pseudoInverse(constraint.matrix(), Jpinv, 1e-4);
    Vector dv = Jpinv * constraint.vector();
    BOOST_REQUIRE(isFinite(Jpinv));
    BOOST_CHECK(MatrixXd::Identity(6,6).isApprox(constraint.matrix()*Jpinv));
    if(!isFinite(dv))
    {
      cout<< "Jpinv" << Jpinv.transpose() <<endl;
      cout<< "b" << constraint.vector().transpose() <<endl;
    }
    REQUIRE_FINITE(dv.transpose());

    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
    BOOST_REQUIRE(isFinite(v));
    BOOST_REQUIRE(isFinite(q));
    t += dt;

    error = task.position_error().norm();
    BOOST_REQUIRE(isFinite(task.position_error()));
    BOOST_CHECK(error <= error_past);
    error_past = error;

    if(i%100==0)
      cout << "Time "<<t<<"\t Pos error "<<error<<
              "\t Vel error "<<task.velocity_error().norm()<<endl;
  }
}

BOOST_AUTO_TEST_CASE ( test_task_com_equality )
{
  cout<<"\n\n*********** TEST TASK COM EQUALITY ***********\n";
  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);
  
  pinocchio::Data data(robot.model());
  const string srdfFileName = package_dirs[0] + "/srdf/romeo_collision.srdf";

  pinocchio::srdf::loadReferenceConfigurations(robot.model(),srdfFileName,false);
  
  //  const unsigned int nv = robot.nv();
  VectorXd q = neutral(robot.model());
  std::cout << "q: " << q.transpose() << std::endl;
  q(2) += 0.84;
  
  pinocchio::centerOfMass(robot.model(),data,q);

  TaskComEquality task("task-com", robot);

  VectorXd Kp = VectorXd::Ones(3);
  VectorXd Kd = 2.0*VectorXd::Ones(3);
  task.Kp(Kp);
  task.Kd(Kd);
  BOOST_CHECK(task.Kp().isApprox(Kp));
  BOOST_CHECK(task.Kd().isApprox(Kd));

  Vector3 com_ref = data.com[0] + pinocchio::SE3::Vector3(0.02,0.02,0.02);
  TrajectoryBase *traj = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sample;

  double t = 0.0;
  const double dt = 0.001;
  MatrixXd Jpinv(robot.nv(), 3);
  double error, error_past=1e100;
  VectorXd v = VectorXd::Zero(robot.nv());
  for(int i=0; i<max_it; i++)
  {
    robot.computeAllTerms(data, q, v);
    sample = traj->computeNext();
    task.setReference(sample);
    const ConstraintBase & constraint = task.compute(t, q, v, data);
    BOOST_CHECK(constraint.rows()==3);
    BOOST_CHECK(static_cast<tsid::math::Index>(constraint.cols())==static_cast<tsid::math::Index>(robot.nv()));
    BOOST_REQUIRE(isFinite(constraint.matrix()));
    BOOST_REQUIRE(isFinite(constraint.vector()));

    pseudoInverse(constraint.matrix(), Jpinv, 1e-5);
    Vector dv = Jpinv * constraint.vector();
    BOOST_REQUIRE(isFinite(Jpinv));
    BOOST_CHECK(MatrixXd::Identity(constraint.rows(),constraint.rows()).isApprox(constraint.matrix()*Jpinv));
    BOOST_REQUIRE(isFinite(dv));

    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
    BOOST_REQUIRE(isFinite(v));
    BOOST_REQUIRE(isFinite(q));
    t += dt;

    error = task.position_error().norm();
    BOOST_REQUIRE(isFinite(task.position_error()));
    BOOST_CHECK((error - error_past) <= 1e-4);
    error_past = error;
    
    if(error < 1e-8) break;

    if(i%100==0)
      cout << "Time "<<t<<"\t CoM pos error "<<error<<
              "\t CoM vel error "<<task.velocity_error().norm()<<endl;
  }
}

BOOST_AUTO_TEST_CASE ( test_task_joint_posture )
{
  cout<<"\n\n*********** TEST TASK JOINT POSTURE ***********\n";
  vector<string> package_dirs;
  package_dirs.push_back(romeo_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);
  const unsigned int na = robot.nv()-6;

  cout<<"Gonna create task\n";
  TaskJointPosture task("task-posture", robot);

  cout<<"Gonna set gains\n"<<na<<endl;
  VectorXd Kp = VectorXd::Ones(na);
  VectorXd Kd = 2.0*Kp;
  task.Kp(Kp);
  task.Kd(Kd);
  BOOST_CHECK(task.Kp().isApprox(Kp));
  BOOST_CHECK(task.Kd().isApprox(Kd));

  cout<<"Gonna create reference trajectory\n";
  Vector q_ref = Vector::Random(na);
  TrajectoryBase *traj = new TrajectoryEuclidianConstant("traj_joint", q_ref);
  TrajectorySample sample;

  cout<<"Gonna set up for simulation\n";
  double t = 0.0;
  const double dt = 0.001;
  MatrixXd Jpinv(robot.nv(), na);
  double error, error_past=1e100;
  VectorXd q = neutral(robot.model());
  VectorXd v = VectorXd::Zero(robot.nv());
  pinocchio::Data data(robot.model());
  for(int i=0; i<max_it; i++)
  {
    robot.computeAllTerms(data, q, v);
    sample = traj->computeNext();
    task.setReference(sample);
    const ConstraintBase & constraint = task.compute(t, q, v, data);
    BOOST_CHECK(constraint.rows()==na);
    BOOST_CHECK(static_cast<tsid::math::Index>(constraint.cols())==static_cast<tsid::math::Index>(robot.nv()));
    BOOST_REQUIRE(isFinite(constraint.matrix()));
    BOOST_REQUIRE(isFinite(constraint.vector()));

    pseudoInverse(constraint.matrix(), Jpinv, 1e-5);
    Vector dv = Jpinv * constraint.vector();
    BOOST_REQUIRE(isFinite(Jpinv));
    BOOST_CHECK(MatrixXd::Identity(na,na).isApprox(constraint.matrix()*Jpinv));
    BOOST_REQUIRE(isFinite(dv));

    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
    BOOST_REQUIRE(isFinite(v));
    BOOST_REQUIRE(isFinite(q));
    t += dt;

    error = task.position_error().norm();
    BOOST_REQUIRE(isFinite(task.position_error()));
    BOOST_CHECK(error <= error_past);
    error_past = error;

    if(i%100==0)
      cout << "Time "<<t<<"\t pos error "<<error<<
              "\t vel error "<<task.velocity_error().norm()<<endl;
  }
}

BOOST_AUTO_TEST_SUITE_END ()
