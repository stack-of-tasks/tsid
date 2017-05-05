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

#include <pininvdyn/contacts/contact-6d.hpp>
#include <pininvdyn/inverse-dynamics-formulation-acc-force.hpp>
#include <pininvdyn/tasks/task-com-equality.hpp>
#include <pininvdyn/tasks/task-joint-posture.hpp>
#include <pininvdyn/trajectories/trajectory-euclidian.hpp>
#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>
#include <pininvdyn/utils/stop-watch.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp> // integrate

#define HRP2_PKG_DIR "/home/adelpret/devel/sot_hydro/install/share"

using namespace pininvdyn;
using namespace pininvdyn::trajectories;
using namespace pininvdyn::math;
using namespace pininvdyn::contacts;
using namespace pininvdyn::tasks;
using namespace pininvdyn::solvers;
//using namespace pininvdyn::utils;
using namespace std;

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

#define REQUIRE_FINITE(A) BOOST_REQUIRE_MESSAGE(is_finite(A), #A<<": "<<A)
#define CHECK_LESS_THAN(A,B) BOOST_CHECK_MESSAGE(A<B, #A<<": "<<A<<">"<<B)

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force )
{
  cout<<"\n*** test_invdyn_formulation_acc_force ***\n";
  const double lxp = 0.14;
  const double lxn = 0.077;
  const double lyp = 0.069;
  const double lyn = 0.069;
  const double lz = 0.105;
  const double mu = 0.3;
  const double fMin = 10.0;
  const std::string rf_frame_name = "RLEG_JOINT5";
  const std::string lf_frame_name = "LLEG_JOINT5";
  Vector3 contactNormal = Vector3::UnitZ();
  const double w_com = 1.0;
  const double w_posture = 1e-2;
  const double w_forceReg = 1e-5;
  const double kp_contact = 100.0;
  const double kp_com = 30.0;
  const double kp_posture = 30.0;
  const double dt = 0.001;
  const unsigned int N_DT = 3000;
  const unsigned int PRINT_N = 100;
  double t = 0.0;

  vector<string> package_dirs;
  package_dirs.push_back(HRP2_PKG_DIR);
  string urdfFileName = package_dirs[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     se3::JointModelFreeFlyer());
  const unsigned int nv = robot.nv();
  Vector q = robot.model().neutralConfiguration;
  q(2) += 0.6;
  Vector v = Vector::Zero(nv);
  BOOST_REQUIRE(robot.model().existFrame(rf_frame_name));
  BOOST_REQUIRE(robot.model().existFrame(lf_frame_name));

  // Create the inverse-dynamics formulation
  InverseDynamicsFormulationAccForce * invDyn;
  invDyn = new InverseDynamicsFormulationAccForce("invdyn", robot);
  invDyn->computeProblemData(t, q, v);
  const se3::Data & data = invDyn->data();

  // Add the contact constraints
  Matrix3x contactPoints(3,4);
  contactPoints << -lxn, -lxn, +lxp, +lxp,
                   -lyn, +lyp, -lyn, +lyp,
                    lz,  lz,  lz,  lz;
  Contact6d contactRF("contact_rfoot", robot, rf_frame_name,
                    contactPoints, contactNormal,
                    mu, fMin, w_forceReg);
  contactRF.Kp(kp_contact*Vector::Ones(6));
  contactRF.Kd(2.0*contactRF.Kp().cwiseSqrt());
  se3::SE3 H_rf_ref = robot.position(data,
                                  robot.model().getJointId(rf_frame_name));
  contactRF.setReference(H_rf_ref);
  invDyn->addRigidContact(contactRF);

  Contact6d contactLF("contact_lfoot", robot, lf_frame_name,
                    contactPoints, contactNormal,
                    mu, fMin, w_forceReg);
  contactLF.Kp(kp_contact*Vector::Ones(6));
  contactLF.Kd(2.0*contactLF.Kp().cwiseSqrt());
  se3::SE3 H_lf_ref = robot.position(data,
                                    robot.model().getJointId(lf_frame_name));
  contactLF.setReference(H_lf_ref);
  invDyn->addRigidContact(contactLF);

  // Add the com task
  TaskComEquality comTask("task-com", robot);
  comTask.Kp(kp_com*Vector::Ones(3));
  comTask.Kd(2.0*comTask.Kp().cwiseSqrt());
  Vector3 com_ref = robot.com(invDyn->data());
  com_ref(0) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);
  invDyn->addMotionTask(comTask, w_com, 1);

  // Add the posture task
  TaskJointPosture postureTask("task-posture", robot);
  postureTask.Kp(kp_posture*Vector::Ones(nv-6));
  postureTask.Kd(2.0*postureTask.Kp().cwiseSqrt());
  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);
  invDyn->addMotionTask(postureTask, w_posture, 1);

  // Create an HQP solver
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());
  cout<<"nVar "<<invDyn->nVar()<<endl;
  cout<<"nEq "<<invDyn->nEq()<<endl;
  cout<<"nIn "<<invDyn->nIn()<<endl;
  cout<<"Initial CoM position: "<<robot.com(invDyn->data()).transpose()<<endl;
  cout<<"Initial RF position: "<<H_rf_ref<<endl;
  cout<<"Initial LF position: "<<H_lf_ref<<endl;

  Vector dv = Vector::Zero(nv);
  Vector f_RF(12), f_LF(12), f(24);
  vector<ContactBase*> contacts;
  contacts.push_back(&contactRF);
  contacts.push_back(&contactLF);
  Matrix Jc(24, nv);
  for(int i=0; i<N_DT; i++)
  {
    sampleCom = trajCom->computeNext();
    comTask.setReference(sampleCom);
    samplePosture = trajPosture->computeNext();
    postureTask.setReference(samplePosture);

    const HqpData & hqpData = invDyn->computeProblemData(t, q, v);
    if(i==0)
      cout<< hqpDataToString(hqpData, false)<<endl;

    REQUIRE_FINITE(postureTask.getConstraint().matrix());
    REQUIRE_FINITE(postureTask.getConstraint().vector());
    REQUIRE_FINITE(comTask.getConstraint().matrix());
    REQUIRE_FINITE(comTask.getConstraint().vector());
    REQUIRE_FINITE(contactRF.getMotionConstraint().matrix());
    REQUIRE_FINITE(contactRF.getMotionConstraint().vector());
    REQUIRE_FINITE(contactRF.getForceConstraint().matrix());
    REQUIRE_FINITE(contactRF.getForceConstraint().lowerBound());
    REQUIRE_FINITE(contactRF.getForceConstraint().upperBound());
    REQUIRE_FINITE(contactRF.getForceRegularizationTask().matrix());
    REQUIRE_FINITE(contactRF.getForceRegularizationTask().vector());
    REQUIRE_FINITE(contactLF.getMotionConstraint().matrix());
    REQUIRE_FINITE(contactLF.getMotionConstraint().vector());
    REQUIRE_FINITE(contactLF.getForceConstraint().matrix());
    REQUIRE_FINITE(contactLF.getForceConstraint().lowerBound());
    REQUIRE_FINITE(contactLF.getForceConstraint().upperBound());
    REQUIRE_FINITE(contactLF.getForceRegularizationTask().matrix());
    REQUIRE_FINITE(contactLF.getForceRegularizationTask().vector());

    CHECK_LESS_THAN(contactRF.getMotionTask().position_error().norm(), 1e-3);
    CHECK_LESS_THAN(contactLF.getMotionTask().position_error().norm(), 1e-3);


    if(contactRF.getMotionTask().position_error().norm() > 1e-2)
    {
      PRINT_VECTOR(v);
      PRINT_VECTOR(dv);
      Vector rf_pos = contactRF.getMotionTask().position();
      Vector rf_pos_ref = contactRF.getMotionTask().position_ref();
      se3::SE3 M_rf, M_rf_ref;
      vectorToSE3(rf_pos, M_rf);
      vectorToSE3(rf_pos_ref, M_rf_ref);
      cout<<"RF pos:     "<<rf_pos.transpose()<<endl;
      cout<<"RF pos ref: "<<rf_pos_ref.transpose()<<endl;
    }

    if(i%PRINT_N==0)
    {
      cout<<"Time "<<i<<endl;
      cout<<"  "<<contactRF.name()<<" err: "<<contactRF.getMotionTask().position_error().norm()<<" \t";
      cout<<contactLF.name()<<" err: "<<contactLF.getMotionTask().position_error().norm()<<" \t";
      cout<<comTask.name()<<" err: "<<comTask.position_error().norm()<<" \t";
      cout<<postureTask.name()<<" err: "<<postureTask.position_error().norm()<<" \t";
      cout<<"v="<<v.norm()<<"\t dv="<<dv.norm()<<endl;
//      PRINT_VECTOR(postureTask.getConstraint().vector());
//      PRINT_VECTOR(postureTask.position_error());
      if(i<20)
        PRINT_VECTOR(dv);
    }

    const HqpOutput & sol = solver->solve(hqpData);

    BOOST_CHECK_MESSAGE(sol.status==HQP_STATUS_OPTIMAL, "Status "+toString(sol.status));

    for(ConstraintLevel::const_iterator it=hqpData[0].begin(); it!=hqpData[0].end(); it++)
    {
      const ConstraintBase* constr = it->second;
      if(constr->checkConstraint(sol.x)==false)
      {
        if(constr->isEquality())
        {
          BOOST_CHECK_MESSAGE(false, "Equality "+constr->name()+" violated: "+
                       toString((constr->matrix()*sol.x-constr->vector()).norm()));
        }
        else if(constr->isInequality())
        {
          BOOST_CHECK_MESSAGE(false, "Inequality "+constr->name()+" violated: "+
                  toString((constr->matrix()*sol.x-constr->lowerBound()).minCoeff())+"\n"+
                  toString((constr->upperBound()-constr->matrix()*sol.x).minCoeff()));
        }
        else if(constr->isBound())
        {
          BOOST_CHECK_MESSAGE(false, "Bound "+constr->name()+" violated: "+
                  toString((sol.x-constr->lowerBound()).minCoeff())+"\n"+
                  toString((constr->upperBound()-sol.x).minCoeff()));
        }
      }
    }

    dv = sol.x.head(nv);
    f_RF = sol.x.segment<12>(nv);
    f_LF = sol.x.segment<12>(nv+12);
    f = sol.x.tail(24);

    BOOST_CHECK(contactRF.getMotionConstraint().checkConstraint(dv));
    BOOST_CHECK(contactRF.getForceConstraint().checkConstraint(f_RF));
    BOOST_CHECK(contactLF.getMotionConstraint().checkConstraint(dv));
    BOOST_CHECK(contactLF.getForceConstraint().checkConstraint(f_LF));

//    unsigned int index = 0;
//    for(vector<ContactBase*>::iterator it=contacts.begin(); it!=contacts.end(); it++)
//    {
//      unsigned int m = (*it)->n_force();
//      const Matrix & T = (*it)->getForceGeneratorMatrix(); // 6x12
//      Jc.middleRows(index, m) = T.transpose()*(*it)->getMotionConstraint().matrix();
//      index += m;
//    }
//    const Matrix & M_u = robot.mass(data).topRows<6>();
//    const Vector & h_u = robot.nonLinearEffects(data).head<6>();
//    const Matrix & J_u = Jc.leftCols<6>();
//    CHECK_LESS_THAN((M_u*dv + h_u - J_u.transpose()*f).norm(), 1e-6);

    v += dt*dv;
    q = se3::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
    CHECK_LESS_THAN(dv.norm(), 1e6);
    CHECK_LESS_THAN(v.norm(), 1e6);
  }

  cout<<"\n### TEST FINISHED ###\n";
  PRINT_VECTOR(v);
  cout<<"Final   CoM position: "<<robot.com(invDyn->data()).transpose()<<endl;
  cout<<"Desired CoM position: "<<com_ref.transpose()<<endl;
}

#define PROFILE_CONTROL_CYCLE "Control cycle"
#define PROFILE_PROBLEM_FORMULATION "Problem formulation"
#define PROFILE_HQP "HQP"

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force_computation_time )
{
  cout<<"\n*** test_invdyn_formulation_acc_force_computation_time ***\n";
  const double lxp = 0.14;
  const double lxn = 0.077;
  const double lyp = 0.069;
  const double lyn = 0.069;
  const double lz = 0.105;
  const double mu = 0.3;
  const double fMin = 10.0;
  const std::string rf_frame_name = "RLEG_JOINT5";
  const std::string lf_frame_name = "LLEG_JOINT5";
  Vector3 contactNormal = Vector3::UnitZ();
  const double w_com = 1.0;
  const double w_posture = 1e-2;
  const double w_forceReg = 1e-5;
  const double kp_contact = 100.0;
  const double kp_com = 30.0;
  const double kp_posture = 30.0;
  const double dt = 0.001;
  const unsigned int N_DT = 3000;
  const unsigned int PRINT_N = 100;
  double t = 0.0;

  vector<string> package_dirs;
  package_dirs.push_back(HRP2_PKG_DIR);
  string urdfFileName = package_dirs[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     se3::JointModelFreeFlyer());
  const unsigned int nv = robot.nv();
  Vector q = robot.model().neutralConfiguration;
  q(2) += 0.6;
  Vector v = Vector::Zero(nv);
  BOOST_REQUIRE(robot.model().existFrame(rf_frame_name));
  BOOST_REQUIRE(robot.model().existFrame(lf_frame_name));

  // Create the inverse-dynamics formulation
  InverseDynamicsFormulationAccForce * invDyn;
  invDyn = new InverseDynamicsFormulationAccForce("invdyn", robot);
  invDyn->computeProblemData(t, q, v);
  const se3::Data & data = invDyn->data();

  // Add the contact constraints
  Matrix3x contactPoints(3,4);
  contactPoints << -lxn, -lxn, +lxp, +lxp,
                   -lyn, +lyp, -lyn, +lyp,
                    lz,  lz,  lz,  lz;
  Contact6d contactRF("contact_rfoot", robot, rf_frame_name,
                    contactPoints, contactNormal,
                    mu, fMin, w_forceReg);
  contactRF.Kp(kp_contact*Vector::Ones(6));
  contactRF.Kd(2.0*contactRF.Kp().cwiseSqrt());
  se3::SE3 H_rf_ref = robot.position(data,
                                  robot.model().getJointId(rf_frame_name));
  contactRF.setReference(H_rf_ref);
  invDyn->addRigidContact(contactRF);

  Contact6d contactLF("contact_lfoot", robot, lf_frame_name,
                    contactPoints, contactNormal,
                    mu, fMin, w_forceReg);
  contactLF.Kp(kp_contact*Vector::Ones(6));
  contactLF.Kd(2.0*contactLF.Kp().cwiseSqrt());
  se3::SE3 H_lf_ref = robot.position(data,
                                    robot.model().getJointId(lf_frame_name));
  contactLF.setReference(H_lf_ref);
  invDyn->addRigidContact(contactLF);

  // Add the com task
  TaskComEquality comTask("task-com", robot);
  comTask.Kp(kp_com*Vector::Ones(3));
  comTask.Kd(2.0*comTask.Kp().cwiseSqrt());
  Vector3 com_ref = robot.com(invDyn->data());
  com_ref(0) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);
  invDyn->addMotionTask(comTask, w_com, 1);

  // Add the posture task
  TaskJointPosture postureTask("task-posture", robot);
  postureTask.Kp(kp_posture*Vector::Ones(nv-6));
  postureTask.Kd(2.0*postureTask.Kp().cwiseSqrt());
  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);
  invDyn->addMotionTask(postureTask, w_posture, 1);

  // Create an HQP solver
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());

  Vector dv = Vector::Zero(nv);
  for(int i=0; i<N_DT; i++)
  {
    getProfiler().start(PROFILE_CONTROL_CYCLE);
    {
      sampleCom = trajCom->computeNext();
      comTask.setReference(sampleCom);
      samplePosture = trajPosture->computeNext();
      postureTask.setReference(samplePosture);

      getProfiler().start(PROFILE_PROBLEM_FORMULATION);
      const HqpData & hqpData = invDyn->computeProblemData(t, q, v);
      getProfiler().stop(PROFILE_PROBLEM_FORMULATION);

      getProfiler().start(PROFILE_HQP);
      const HqpOutput & sol = solver->solve(hqpData);
      getProfiler().stop(PROFILE_HQP);

      dv = sol.x.head(nv);
    }
    getProfiler().stop(PROFILE_CONTROL_CYCLE);

    v += dt*dv;
    q = se3::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
  }

  cout<<"\n### TEST FINISHED ###\n";
  getProfiler().report_all(3, cout);
}

BOOST_AUTO_TEST_SUITE_END ()
