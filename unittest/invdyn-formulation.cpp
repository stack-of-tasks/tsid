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
#include <pininvdyn/tasks/task-se3-equality.hpp>
#include <pininvdyn/tasks/task-joint-posture.hpp>
#include <pininvdyn/trajectories/trajectory-euclidian.hpp>
#include <pininvdyn/solvers/solver-HQP-base.hpp>
#include <pininvdyn/utils/stop-watch.hpp>
#include <pininvdyn/utils/statistics.hpp>

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

#define REQUIRE_TASK_FINITE(task) REQUIRE_FINITE(task.getConstraint().matrix()); \
                                  REQUIRE_FINITE(task.getConstraint().vector())

#define REQUIRE_CONTACT_FINITE(contact) REQUIRE_FINITE(contact.getMotionConstraint().matrix()); \
                                        REQUIRE_FINITE(contact.getMotionConstraint().vector()); \
                                        REQUIRE_FINITE(contact.getForceConstraint().matrix()); \
                                        REQUIRE_FINITE(contact.getForceConstraint().lowerBound()); \
                                        REQUIRE_FINITE(contact.getForceConstraint().upperBound()); \
                                        REQUIRE_FINITE(contact.getForceRegularizationTask().matrix()); \
                                        REQUIRE_FINITE(contact.getForceRegularizationTask().vector())

class StandardHrp2InvDynCtrl
{
  public:
  const double lxp = 0.14;
  const double lxn = 0.077;
  const double lyp = 0.069;
  const double lyn = 0.069;
  const double lz = 0.105;
  const double mu = 0.3;
  const double fMin = 5.0;
  const double fMax = 1000.0;
  const std::string rf_frame_name = "RLEG_JOINT5";
  const std::string lf_frame_name = "LLEG_JOINT5";
  const Vector3 contactNormal = Vector3::UnitZ();
  const double w_com = 1.0;
  const double w_posture = 1e-2;
  const double w_forceReg = 1e-5;
  const double kp_contact = 100.0;
  const double kp_com = 30.0;
  const double kp_posture = 30.0;
  double t = 0.0;

  RobotWrapper * robot;
  InverseDynamicsFormulationAccForce * invDyn;
  Contact6d * contactRF;
  Contact6d * contactLF;
  TaskComEquality * comTask;
  TaskJointPosture * postureTask;
  Vector q;
  Vector v;
  se3::SE3 H_rf_ref;
  se3::SE3 H_lf_ref;

  StandardHrp2InvDynCtrl()
  {
    vector<string> package_dirs;
    package_dirs.push_back(HRP2_PKG_DIR);
    string urdfFileName = package_dirs[0] + "/hrp2_14_description/urdf/hrp2_14_reduced.urdf";
    robot = new RobotWrapper(urdfFileName, package_dirs, se3::JointModelFreeFlyer());

    const unsigned int nv = robot->nv();
    q = robot->model().neutralConfiguration;
    q(2) += 0.6;
    v = Vector::Zero(nv);
    BOOST_REQUIRE(robot->model().existFrame(rf_frame_name));
    BOOST_REQUIRE(robot->model().existFrame(lf_frame_name));

    // Create the inverse-dynamics formulation
    invDyn = new InverseDynamicsFormulationAccForce("invdyn", *robot);
    invDyn->computeProblemData(t, q, v);
    const se3::Data & data = invDyn->data();

    // Add the contact constraints
    Matrix3x contactPoints(3,4);
    contactPoints << -lxn, -lxn, +lxp, +lxp,
                     -lyn, +lyp, -lyn, +lyp,
                      lz,  lz,  lz,  lz;
    contactRF = new Contact6d("contact_rfoot", *robot, rf_frame_name,
                              contactPoints, contactNormal,
                              mu, fMin, fMax, w_forceReg);
    contactRF->Kp(kp_contact*Vector::Ones(6));
    contactRF->Kd(2.0*contactRF->Kp().cwiseSqrt());
    H_rf_ref = robot->position(data, robot->model().getJointId(rf_frame_name));
    contactRF->setReference(H_rf_ref);
    invDyn->addRigidContact(*contactRF);

    contactLF = new Contact6d ("contact_lfoot", *robot, lf_frame_name,
                               contactPoints, contactNormal,
                               mu, fMin, fMax, w_forceReg);
    contactLF->Kp(kp_contact*Vector::Ones(6));
    contactLF->Kd(2.0*contactLF->Kp().cwiseSqrt());
    H_lf_ref = robot->position(data, robot->model().getJointId(lf_frame_name));
    contactLF->setReference(H_lf_ref);
    invDyn->addRigidContact(*contactLF);

    // Add the com task
    comTask = new TaskComEquality("task-com", *robot);
    comTask->Kp(kp_com*Vector::Ones(3));
    comTask->Kd(2.0*comTask->Kp().cwiseSqrt());
    invDyn->addMotionTask(*comTask, w_com, 1);

    // Add the posture task
    postureTask = new TaskJointPosture("task-posture", *robot);
    postureTask->Kp(kp_posture*Vector::Ones(nv-6));
    postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());
    invDyn->addMotionTask(*postureTask, w_posture, 1);
  }

};

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force_remove_contact )
{
  cout<<"\n*** test_invdyn_formulation_acc_force_remove_contact ***\n";
  const double dt = 0.01;
  const unsigned int N_DT = 300;
  const unsigned int PRINT_N = 10;
  const unsigned int REMOVE_CONTACT_N = 100;
  const double CONTACT_TRANSITION_TIME = 1.0;
  const double kp_RF = 100.0;
  const double w_RF = 1e3;
  double t = 0.0;

  StandardHrp2InvDynCtrl hrp2_inv_dyn;
  RobotWrapper & robot = *(hrp2_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * invDyn = hrp2_inv_dyn.invDyn;
  Contact6d & contactRF = *(hrp2_inv_dyn.contactRF);
  Contact6d & contactLF = *(hrp2_inv_dyn.contactLF);
  TaskComEquality & comTask = *(hrp2_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(hrp2_inv_dyn.postureTask);
  Vector q = hrp2_inv_dyn.q;
  Vector v = hrp2_inv_dyn.v;
  const int nv = robot.model().nv;

  // Add the right foot task
  TaskSE3Equality * rightFootTask = new TaskSE3Equality("task-right-foot",
                                                       robot,
                                                       hrp2_inv_dyn.rf_frame_name);
  rightFootTask->Kp(kp_RF*Vector::Ones(6));
  rightFootTask->Kd(2.0*rightFootTask->Kp().cwiseSqrt());
  se3::SE3 H_rf_ref = robot.position(invDyn->data(),
                                     robot.model().getJointId(hrp2_inv_dyn.rf_frame_name));
  invDyn->addMotionTask(*rightFootTask, w_RF, 1);

  TrajectorySample s(12, 6);
  SE3ToVector(H_rf_ref, s.pos);
  rightFootTask->setReference(s);

  Vector3 com_ref = robot.com(invDyn->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());

  Vector tau_old(nv-6);
  for(int i=0; i<N_DT; i++)
  {
    if(i==REMOVE_CONTACT_N)
    {
      cout<<"Start breaking contact right foot\n";
      invDyn->removeRigidContact(contactRF.name(), CONTACT_TRANSITION_TIME);
    }

    sampleCom = trajCom->computeNext();
    comTask.setReference(sampleCom);
    samplePosture = trajPosture->computeNext();
    postureTask.setReference(samplePosture);

    const HqpData & hqpData = invDyn->computeProblemData(t, q, v);
    if(i==0)
      cout<< hqpDataToString(hqpData, false)<<endl;

    REQUIRE_TASK_FINITE(postureTask);
    REQUIRE_TASK_FINITE(comTask);
    REQUIRE_CONTACT_FINITE(contactRF);
    REQUIRE_CONTACT_FINITE(contactLF);

    CHECK_LESS_THAN(contactRF.getMotionTask().position_error().norm(), 1e-3);
    CHECK_LESS_THAN(contactLF.getMotionTask().position_error().norm(), 1e-3);

    const HqpOutput & sol = solver->solve(hqpData);

    BOOST_CHECK_MESSAGE(sol.status==HQP_STATUS_OPTIMAL, "Status "+toString(sol.status));

    const Vector & tau = invDyn->getActuatorForces(sol);
    const Vector & dv = invDyn->getAccelerations(sol);

    if(i>0)
    {
      CHECK_LESS_THAN((tau-tau_old).norm(), 1e1);
      if((tau-tau_old).norm()>1e1) // || (i>=197 && i<=200))
      {
//        contactRF.computeMotionTask(t, q, v, invDyn->data());
//        rightFootTask->compute(t, q, v, invDyn->data());
        cout << "Time "<<i<<endl;
        cout<<"tau:\n"<<tau.transpose()<<"\ntauOld:\n"<<tau_old.transpose()<<"\n";
//        cout << "RF contact task des acc:   "<<contactRF.getMotionTask().getDesiredAcceleration().transpose()<<endl;
//        cout << "RF contact task acc:       "<<contactRF.getMotionTask().getAcceleration(dv).transpose()<<endl;
//        cout << "RF motion task des acc:    "<<rightFootTask->getDesiredAcceleration().transpose()<<endl;
        cout << endl;
      }
    }
    tau_old = tau;

    if(i%PRINT_N==0)
    {
      cout<<"Time "<<i<<endl;

      Eigen::Matrix<double, 12, 1> f;
      if(invDyn->getContactForces(contactRF.name(), sol, f))
        cout<<"  "<<contactRF.name()<<" force: "<<contactRF.getNormalForce(f)<<" \t";

      if(invDyn->getContactForces(contactLF.name(), sol, f))
        cout<<"  "<<contactLF.name()<<" force: "<<contactLF.getNormalForce(f)<<" \t";

      cout<<comTask.name()<<" err: "<<comTask.position_error().norm()<<" \t";
      cout<<"v="<<v.norm()<<"\t dv="<<dv.norm()<<endl;
    }

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

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force )
{
  cout<<"\n*** test_invdyn_formulation_acc_force ***\n";

  const double dt = 0.001;
  const unsigned int N_DT = 3000;
  const unsigned int PRINT_N = 100;
  double t = 0.0;

  StandardHrp2InvDynCtrl hrp2_inv_dyn;
  RobotWrapper & robot = *(hrp2_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * invDyn = hrp2_inv_dyn.invDyn;
  Contact6d & contactRF = *(hrp2_inv_dyn.contactRF);
  Contact6d & contactLF = *(hrp2_inv_dyn.contactLF);
  TaskComEquality & comTask = *(hrp2_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(hrp2_inv_dyn.postureTask);
  Vector q = hrp2_inv_dyn.q;
  Vector v = hrp2_inv_dyn.v;
  const int nv = robot.model().nv;

  Vector3 com_ref = robot.com(invDyn->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "solver-eiquadprog");
  solver->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());
  cout<<"nVar "<<invDyn->nVar()<<endl;
  cout<<"nEq "<<invDyn->nEq()<<endl;
  cout<<"nIn "<<invDyn->nIn()<<endl;
  cout<<"Initial CoM position: "<<robot.com(invDyn->data()).transpose()<<endl;
  cout<<"Initial RF position: "<<hrp2_inv_dyn.H_rf_ref<<endl;
  cout<<"Initial LF position: "<<hrp2_inv_dyn.H_lf_ref<<endl;

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

    REQUIRE_TASK_FINITE(postureTask);
    REQUIRE_TASK_FINITE(comTask);
    REQUIRE_CONTACT_FINITE(contactRF);
    REQUIRE_CONTACT_FINITE(contactLF);

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
#define PROFILE_HQP_FAST "HQP_FAST"
#define PROFILE_HQP_RT "HQP_RT"

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force_computation_time )
{
  cout<<"\n*** test_invdyn_formulation_acc_force_computation_time ***\n";

  const double dt = 0.001;
  const unsigned int N_DT = 3000;
  double t = 0.0;

  StandardHrp2InvDynCtrl hrp2_inv_dyn;
  RobotWrapper & robot = *(hrp2_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * invDyn = hrp2_inv_dyn.invDyn;
  TaskComEquality & comTask = *(hrp2_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(hrp2_inv_dyn.postureTask);
  Vector q = hrp2_inv_dyn.q;
  Vector v = hrp2_inv_dyn.v;
  const int nv = robot.model().nv;

  Vector3 com_ref = robot.com(invDyn->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  Solver_HQP_base * solver = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG,
                                                           "eiquadprog");
  Solver_HQP_base * solver_fast = Solver_HQP_base::getNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                                "eiquadprog-fast");
  Solver_HQP_base * solver_rt =
      Solver_HQP_base::getNewSolverFixedSize<60,18,34>(SOLVER_HQP_EIQUADPROG_RT,
                                                       "eiquadprog-rt");
  solver->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());
  solver_fast->resize(invDyn->nVar(), invDyn->nEq(), invDyn->nIn());

  Vector dv = Vector::Zero(nv);
  for(int i=0; i<N_DT; i++)
  {
    getProfiler().start(PROFILE_CONTROL_CYCLE);

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

    getProfiler().stop(PROFILE_CONTROL_CYCLE);

    getProfiler().start(PROFILE_HQP_FAST);
    const HqpOutput & sol_fast = solver_fast->solve(hqpData);
    getProfiler().stop(PROFILE_HQP_FAST);

    getProfiler().start(PROFILE_HQP_RT);
    solver_rt->solve(hqpData);
    getProfiler().stop(PROFILE_HQP_RT);

    getStatistics().store("active inequalities", sol_fast.activeSet.size());
    getStatistics().store("solver iterations", sol_fast.iterations);

    dv = sol.x.head(nv);
    v += dt*dv;
    q = se3::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
  }

  cout<<"\n### TEST FINISHED ###\n";
  getProfiler().report_all(3, cout);
  getStatistics().report_all(1, cout);
}



BOOST_AUTO_TEST_SUITE_END ()
