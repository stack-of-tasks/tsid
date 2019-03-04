//
// Copyright (c) 2017 CNRS, NYU, MPI Tübingen
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

#include <tsid/contacts/contact-6d.hpp>
#include <tsid/contacts/contact-point.hpp>
#include <tsid/formulations/inverse-dynamics-formulation-acc-force.hpp>
#include <tsid/tasks/task-com-equality.hpp>
#include <tsid/tasks/task-se3-equality.hpp>
#include <tsid/tasks/task-joint-posture.hpp>
#include <tsid/trajectories/trajectory-euclidian.hpp>
#include <tsid/solvers/solver-HQP-factory.hxx>
#include <tsid/solvers/utils.hpp>
#include <tsid/utils/stop-watch.hpp>
#include <tsid/utils/statistics.hpp>
#include <tsid/math/utils.hpp>

#include <pinocchio/algorithm/joint-configuration.hpp> // integrate
#include <pinocchio/parsers/srdf.hpp>

using namespace tsid;
using namespace tsid::trajectories;
using namespace tsid::math;
using namespace tsid::contacts;
using namespace tsid::tasks;
using namespace tsid::solvers;
using namespace tsid::robots;
using namespace std;


#define REQUIRE_FINITE(A) BOOST_REQUIRE_MESSAGE(isFinite(A), #A<<": "<<A)
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

const string romeo_model_path = TSID_SOURCE_DIR"/models/romeo";
const string quadruped_model_path = TSID_SOURCE_DIR"/models/quadruped";

#ifndef NDEBUG
const int max_it = 10;
#else
const int max_it = 1000;
#endif

BOOST_AUTO_TEST_SUITE ( BOOST_TEST_MODULE )

class StandardRomeoInvDynCtrl
{
  public:
  static const double lxp;
  static const double lxn;
  static const double lyp;
  static const double lyn;
  static const double lz;
  static const double mu;
  static const double fMin;
  static const double fMax;
  static const std::string rf_frame_name;
  static const std::string lf_frame_name;
  static const Vector3 contactNormal;
  static const double w_com;
  static const double w_posture;
  static const double w_forceReg;
  static const double kp_contact;
  static const double kp_com;
  static const double kp_posture;
  double t;

  RobotWrapper * robot;
  InverseDynamicsFormulationAccForce * tsid;
  Contact6d * contactRF;
  Contact6d * contactLF;
  TaskComEquality * comTask;
  TaskJointPosture * postureTask;
  Vector q;
  Vector v;
  pinocchio::SE3 H_rf_ref;
  pinocchio::SE3 H_lf_ref;

  StandardRomeoInvDynCtrl() : t(0.)
  {
    vector<string> package_dirs;
    package_dirs.push_back(romeo_model_path);
    const string urdfFileName = package_dirs[0] + "/urdf/romeo.urdf";
    robot = new RobotWrapper(urdfFileName, package_dirs, pinocchio::JointModelFreeFlyer());
    
    const string srdfFileName = package_dirs[0] + "/srdf/romeo_collision.srdf";

    pinocchio::srdf::loadReferenceConfigurations(robot->model(),srdfFileName,false);
    
    const unsigned int nv = robot->nv();
    q = neutral(robot->model());
    std::cout << "q: " << q.transpose() << std::endl;
    q(2) += 0.84;
    v = Vector::Zero(nv);
    BOOST_REQUIRE(robot->model().existFrame(rf_frame_name));
    BOOST_REQUIRE(robot->model().existFrame(lf_frame_name));

    // Create the inverse-dynamics formulation
    tsid = new InverseDynamicsFormulationAccForce("tsid", *robot);
    tsid->computeProblemData(t, q, v);
    const pinocchio::Data & data = tsid->data();

    // Add the contact constraints
    Matrix3x contactPoints(3,4);
    contactPoints << -lxn, -lxn, +lxp, +lxp,
                     -lyn, +lyp, -lyn, +lyp,
                      lz,  lz,  lz,  lz;
    contactRF = new Contact6d("contact_rfoot", *robot, rf_frame_name,
                              contactPoints, contactNormal,
                              mu, fMin, fMax);
    contactRF->Kp(kp_contact*Vector::Ones(6));
    contactRF->Kd(2.0*contactRF->Kp().cwiseSqrt());
    H_rf_ref = robot->position(data, robot->model().getJointId(rf_frame_name));
    contactRF->setReference(H_rf_ref);
    tsid->addRigidContact(*contactRF, w_forceReg);

    contactLF = new Contact6d ("contact_lfoot", *robot, lf_frame_name,
                               contactPoints, contactNormal,
                               mu, fMin, fMax);
    contactLF->Kp(kp_contact*Vector::Ones(6));
    contactLF->Kd(2.0*contactLF->Kp().cwiseSqrt());
    H_lf_ref = robot->position(data, robot->model().getJointId(lf_frame_name));
    contactLF->setReference(H_lf_ref);
    tsid->addRigidContact(*contactLF, w_forceReg);

    // Add the com task
    comTask = new TaskComEquality("task-com", *robot);
    comTask->Kp(kp_com*Vector::Ones(3));
    comTask->Kd(2.0*comTask->Kp().cwiseSqrt());
    tsid->addMotionTask(*comTask, w_com, 1);

    // Add the posture task
    postureTask = new TaskJointPosture("task-posture", *robot);
    postureTask->Kp(kp_posture*Vector::Ones(nv-6));
    postureTask->Kd(2.0*postureTask->Kp().cwiseSqrt());
    tsid->addMotionTask(*postureTask, w_posture, 1);
  }
};

const double StandardRomeoInvDynCtrl::lxp = 0.14;
const double StandardRomeoInvDynCtrl::lxn = 0.077;
const double StandardRomeoInvDynCtrl::lyp = 0.069;
const double StandardRomeoInvDynCtrl::lyn = 0.069;
const double StandardRomeoInvDynCtrl::lz = 0.105;
const double StandardRomeoInvDynCtrl::mu = 0.3;
const double StandardRomeoInvDynCtrl::fMin = 5.0;
const double StandardRomeoInvDynCtrl::fMax = 1000.0;
const std::string StandardRomeoInvDynCtrl::rf_frame_name = "RAnkleRoll";
const std::string StandardRomeoInvDynCtrl::lf_frame_name = "LAnkleRoll";
const Vector3 StandardRomeoInvDynCtrl::contactNormal = Vector3::UnitZ();
const double StandardRomeoInvDynCtrl::w_com = 1.0;
const double StandardRomeoInvDynCtrl::w_posture = 1e-2;
const double StandardRomeoInvDynCtrl::w_forceReg = 1e-5;
const double StandardRomeoInvDynCtrl::kp_contact = 100.0;
const double StandardRomeoInvDynCtrl::kp_com = 30.0;
const double StandardRomeoInvDynCtrl::kp_posture = 30.0;

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force_remove_contact )
{
  cout<<"\n*** test_invdyn_formulation_acc_force_remove_contact ***\n";
  const double dt = 0.01;
  const unsigned int PRINT_N = 10;
  const unsigned int REMOVE_CONTACT_N = 100;
  const double CONTACT_TRANSITION_TIME = 1.0;
  const double kp_RF = 100.0;
  const double w_RF = 1e3;
  double t = 0.0;

  StandardRomeoInvDynCtrl romeo_inv_dyn;
  RobotWrapper & robot = *(romeo_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * tsid = romeo_inv_dyn.tsid;
  Contact6d & contactRF = *(romeo_inv_dyn.contactRF);
  Contact6d & contactLF = *(romeo_inv_dyn.contactLF);
  TaskComEquality & comTask = *(romeo_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(romeo_inv_dyn.postureTask);
  Vector q = romeo_inv_dyn.q;
  Vector v = romeo_inv_dyn.v;
  const int nv = robot.model().nv;

  // Add the right foot task
  TaskSE3Equality * rightFootTask = new TaskSE3Equality("task-right-foot",
                                                       robot,
                                                       romeo_inv_dyn.rf_frame_name);
  rightFootTask->Kp(kp_RF*Vector::Ones(6));
  rightFootTask->Kd(2.0*rightFootTask->Kp().cwiseSqrt());
  pinocchio::SE3 H_rf_ref = robot.position(tsid->data(),
                                     robot.model().getJointId(romeo_inv_dyn.rf_frame_name));
  tsid->addMotionTask(*rightFootTask, w_RF, 1);

  TrajectorySample s(12, 6);
  SE3ToVector(H_rf_ref, s.pos);
  rightFootTask->setReference(s);

  Vector3 com_ref = robot.com(tsid->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG,
                                                               "solver-eiquadprog");
  solver->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());

  Vector tau_old(nv-6);
  for(int i=0; i<max_it; i++)
  {
    if(i==REMOVE_CONTACT_N)
    {
      cout<<"Start breaking contact right foot\n";
      tsid->removeRigidContact(contactRF.name(), CONTACT_TRANSITION_TIME);
    }

    sampleCom = trajCom->computeNext();
    comTask.setReference(sampleCom);
    samplePosture = trajPosture->computeNext();
    postureTask.setReference(samplePosture);

    const HQPData & HQPData = tsid->computeProblemData(t, q, v);
    if(i==0)
      cout<< HQPDataToString(HQPData, false)<<endl;

    REQUIRE_TASK_FINITE(postureTask);
    REQUIRE_TASK_FINITE(comTask);
    REQUIRE_CONTACT_FINITE(contactRF);
    REQUIRE_CONTACT_FINITE(contactLF);

    CHECK_LESS_THAN(contactRF.getMotionTask().position_error().norm(), 1e-3);
    CHECK_LESS_THAN(contactLF.getMotionTask().position_error().norm(), 1e-3);

    const HQPOutput & sol = solver->solve(HQPData);

    BOOST_CHECK_MESSAGE(sol.status==HQP_STATUS_OPTIMAL, "Status "+toString(sol.status));

    const Vector & tau = tsid->getActuatorForces(sol);
    const Vector & dv = tsid->getAccelerations(sol);

    if(i>0)
    {
      CHECK_LESS_THAN((tau-tau_old).norm(), 2e1);
      if((tau-tau_old).norm()>2e1) // || (i>=197 && i<=200))
      {
//        contactRF.computeMotionTask(t, q, v, tsid->data());
//        rightFootTask->compute(t, q, v, tsid->data());
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
      if(tsid->getContactForces(contactRF.name(), sol, f))
        cout<<"  "<<contactRF.name()<<" force: "<<contactRF.getNormalForce(f)<<" \t";

      if(tsid->getContactForces(contactLF.name(), sol, f))
        cout<<"  "<<contactLF.name()<<" force: "<<contactLF.getNormalForce(f)<<" \t";

      cout<<comTask.name()<<" err: "<<comTask.position_error().norm()<<" \t";
      cout<<"v="<<v.norm()<<"\t dv="<<dv.norm()<<endl;
    }

    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
    CHECK_LESS_THAN(dv.norm(), 1e6);
    CHECK_LESS_THAN(v.norm(), 1e6);
  }

  cout<<"\n### TEST FINISHED ###\n";
  PRINT_VECTOR(v);
  cout<<"Final   CoM position: "<<robot.com(tsid->data()).transpose()<<endl;
  cout<<"Desired CoM position: "<<com_ref.transpose()<<endl;
}

BOOST_AUTO_TEST_CASE ( test_invdyn_formulation_acc_force )
{
  cout<<"\n*** test_invdyn_formulation_acc_force ***\n";

  const double dt = 0.001;
  const unsigned int PRINT_N = 100;
  double t = 0.0;

  StandardRomeoInvDynCtrl romeo_inv_dyn;
  RobotWrapper & robot = *(romeo_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * tsid = romeo_inv_dyn.tsid;
  Contact6d & contactRF = *(romeo_inv_dyn.contactRF);
  Contact6d & contactLF = *(romeo_inv_dyn.contactLF);
  TaskComEquality & comTask = *(romeo_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(romeo_inv_dyn.postureTask);
  Vector q = romeo_inv_dyn.q;
  Vector v = romeo_inv_dyn.v;
  const int nv = robot.model().nv;

  Vector3 com_ref = robot.com(tsid->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG,
                                                               "solver-eiquadprog");

  solver->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());
  cout<<"nVar "<<tsid->nVar()<<endl;
  cout<<"nEq "<<tsid->nEq()<<endl;
  cout<<"nIn "<<tsid->nIn()<<endl;
  cout<<"Initial CoM position: "<<robot.com(tsid->data()).transpose()<<endl;
  cout<<"Initial RF position: "<<romeo_inv_dyn.H_rf_ref<<endl;
  cout<<"Initial LF position: "<<romeo_inv_dyn.H_lf_ref<<endl;

  Vector dv = Vector::Zero(nv);
  Vector f_RF(12), f_LF(12), f(24);
  vector<ContactBase*> contacts;
  contacts.push_back(&contactRF);
  contacts.push_back(&contactLF);
  Matrix Jc(24, nv);
  for(int i=0; i<max_it; i++)
  {
    sampleCom = trajCom->computeNext();
    comTask.setReference(sampleCom);
    samplePosture = trajPosture->computeNext();
    postureTask.setReference(samplePosture);

    const HQPData & HQPData = tsid->computeProblemData(t, q, v);
    if(i==0)
      cout<< HQPDataToString(HQPData, false)<<endl;

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
      pinocchio::SE3 M_rf, M_rf_ref;
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

    const HQPOutput & sol = solver->solve(HQPData);

    BOOST_CHECK_MESSAGE(sol.status==HQP_STATUS_OPTIMAL, "Status "+toString(sol.status));

    for(ConstraintLevel::const_iterator it=HQPData[0].begin(); it!=HQPData[0].end(); it++)
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
    q = pinocchio::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
    CHECK_LESS_THAN(dv.norm(), 1e6);
    CHECK_LESS_THAN(v.norm(), 1e6);
  }

  cout<<"\n### TEST FINISHED ###\n";
  PRINT_VECTOR(v);
  cout<<"Final   CoM position: "<<robot.com(tsid->data()).transpose()<<endl;
  cout<<"Desired CoM position: "<<com_ref.transpose()<<endl;
}


BOOST_AUTO_TEST_CASE ( test_contact_point_invdyn_formulation_acc_force )
{
  cout<<"\n*** test_contact_point_invdyn_formulation_acc_force ***\n";

  const double mu = 0.3;
  const double fMin = 0.0;
  const double fMax = 10.0;
  const std::string frameName = "base_link";
  const double dt = 1e-3;

  double t = 0.;

  double w_com = 1.0;                     // weight of center of mass task
  double w_forceReg = 1e-5;               // weight of force regularization task
  double kp_contact = 100.0;              // proportional gain of contact constraint
  double kp_com = 1.0;                    // proportional gain of center of mass task

  vector<string> package_dirs;
  package_dirs.push_back(quadruped_model_path);
  string urdfFileName = package_dirs[0] + "/urdf/quadruped.urdf";
  RobotWrapper robot(urdfFileName,
                     package_dirs,
                     pinocchio::JointModelFreeFlyer(),
                     false);

  BOOST_REQUIRE(robot.model().existFrame(frameName));

  Vector q = neutral(robot.model());
  Vector v = Vector::Zero(robot.nv());
  const unsigned int nv = robot.nv();

  // Create initial posture.
  q(0) = 0.1;
  q(2) = 0.5;
  q(6) = 1.;
  for (int i = 0; i < 4; i++) {
    q(7 + 2 * i) = -0.4;
    q(8 + 2 * i) = 0.8;
  }

  // Create the inverse-dynamics formulation
  InverseDynamicsFormulationAccForce *tsid =
      new InverseDynamicsFormulationAccForce("tsid", robot);
  tsid->computeProblemData(t, q, v);
  const pinocchio::Data & data = tsid->data();

  // Place the robot onto the ground.

  pinocchio::SE3 fl_contact = robot.framePosition(data, robot.model().getFrameId("FL_contact"));
  q[2] -= fl_contact.translation()(2);

  tsid->computeProblemData(t, q, v);

  // Add task for the COM.
  TaskComEquality *comTask = new TaskComEquality("task-com", robot);
  comTask->Kp(kp_com * Vector::Ones(3));
  comTask->Kd(2.0 * comTask->Kp().cwiseSqrt());
  tsid->addMotionTask(*comTask, w_com, 1);

  Vector3 com_ref = robot.com(data);
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);
  sampleCom = trajCom->computeNext();
  comTask->setReference(sampleCom);


  // Add contact constraints.
  std::string contactFrames[] = {
    "BL_contact", "BR_contact", "FL_contact", "FR_contact"
  };

  Vector3 contactNormal = Vector3::UnitZ();
  std::vector<ContactPoint*> contacts(4);

  for (int i = 0; i < 4; i++) {
    ContactPoint* cp = new ContactPoint("contact_" + contactFrames[i], robot,
        contactFrames[i], contactNormal, mu, fMin, fMax);
    cp->Kp(kp_contact*Vector::Ones(6));
    cp->Kd(2.0*cp->Kp().cwiseSqrt());
    cp->setReference(robot.framePosition(data, robot.model().getFrameId(contactFrames[i])));
    cp->useLocalFrame(false);
    tsid->addRigidContact(*cp, w_forceReg, 1.0, 1);

    contacts[i] = cp;
  }

  // Create an HQP solver
  SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG,
                                                               "solver-eiquadprog");
  solver->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());

  Vector dv = Vector::Zero(nv);
  const HQPOutput *sol;
  for (int i = 0; i < max_it; i++) {
    const HQPData & HQPData = tsid->computeProblemData(t, q, v);

    REQUIRE_TASK_FINITE((*comTask));

    for(unsigned int i=0; i<contacts.size(); i++) {
      REQUIRE_CONTACT_FINITE((*(contacts[i])));
    }

    sol = &(solver->solve(HQPData));


    BOOST_CHECK_MESSAGE(sol->status==HQP_STATUS_OPTIMAL, "Status "+toString(sol->status));

    for(ConstraintLevel::const_iterator it=HQPData[0].begin(); it!=HQPData[0].end(); it++)
    {
      const ConstraintBase* constr = it->second;
      if(constr->checkConstraint(sol->x)==false)
      {
        if(constr->isEquality())
        {
          BOOST_CHECK_MESSAGE(false, "Equality "+constr->name()+" violated: "+
                       toString((constr->matrix()*sol->x-constr->vector()).norm()));
        }
        else if(constr->isInequality())
        {
          BOOST_CHECK_MESSAGE(false, "Inequality "+constr->name()+" violated: "+
                  toString((constr->matrix()*sol->x-constr->lowerBound()).minCoeff())+"\n"+
                  toString((constr->upperBound()-constr->matrix()*sol->x).minCoeff()));
        }
        else if(constr->isBound())
        {
          BOOST_CHECK_MESSAGE(false, "Bound "+constr->name()+" violated: "+
                  toString((sol->x-constr->lowerBound()).minCoeff())+"\n"+
                  toString((constr->upperBound()-sol->x).minCoeff()));
        }
      }
    }

    dv = sol->x.head(nv);

    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
    t += dt;

    REQUIRE_FINITE(dv.transpose());
    REQUIRE_FINITE(v.transpose());
    REQUIRE_FINITE(q.transpose());
    CHECK_LESS_THAN(dv.norm(), 1e6);
    CHECK_LESS_THAN(v.norm(), 1e6);
  }

  for (int i = 0; i < 4; i++) {
    Eigen::Matrix<double, 3, 1> f;
    tsid->getContactForces(contacts[i]->name(), *sol, f);
    cout << contacts[i]->name() << " force:" << f.transpose() << endl;
  }

  cout<<"\n### TEST FINISHED ###\n";
  PRINT_VECTOR(v);
  cout<<"Final   CoM position: "<<robot.com(tsid->data()).transpose()<<endl;
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
  double t = 0.0;

  StandardRomeoInvDynCtrl romeo_inv_dyn;
  RobotWrapper & robot = *(romeo_inv_dyn.robot);
  InverseDynamicsFormulationAccForce * tsid = romeo_inv_dyn.tsid;
  TaskComEquality & comTask = *(romeo_inv_dyn.comTask);
  TaskJointPosture & postureTask = *(romeo_inv_dyn.postureTask);
  Vector q = romeo_inv_dyn.q;
  Vector v = romeo_inv_dyn.v;
  const int nv = robot.model().nv;

  Vector3 com_ref = robot.com(tsid->data());
  com_ref(1) += 0.1;
  TrajectoryBase *trajCom = new TrajectoryEuclidianConstant("traj_com", com_ref);
  TrajectorySample sampleCom(3);

  Vector q_ref = q.tail(nv-6);
  TrajectoryBase *trajPosture = new TrajectoryEuclidianConstant("traj_posture", q_ref);
  TrajectorySample samplePosture(nv-6);

  // Create an HQP solver
  SolverHQPBase * solver = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG,
                                                             "eiquadprog");
  SolverHQPBase * solver_fast = SolverHQPFactory::createNewSolver(SOLVER_HQP_EIQUADPROG_FAST,
                                                                  "eiquadprog-fast");
  SolverHQPBase * solver_rt =
      SolverHQPFactory::createNewSolver<61,18,34>(SOLVER_HQP_EIQUADPROG_RT,
                                                  "eiquadprog-rt");

  solver->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());
  solver_fast->resize(tsid->nVar(), tsid->nEq(), tsid->nIn());

  Vector dv = Vector::Zero(nv);
  for(int i=0; i<max_it; i++)
  {
    getProfiler().start(PROFILE_CONTROL_CYCLE);

    sampleCom = trajCom->computeNext();
    comTask.setReference(sampleCom);
    samplePosture = trajPosture->computeNext();
    postureTask.setReference(samplePosture);

    getProfiler().start(PROFILE_PROBLEM_FORMULATION);
    const HQPData & HQPData = tsid->computeProblemData(t, q, v);
    getProfiler().stop(PROFILE_PROBLEM_FORMULATION);

    getProfiler().start(PROFILE_HQP);
    const HQPOutput & sol = solver->solve(HQPData);
    getProfiler().stop(PROFILE_HQP);

    getProfiler().stop(PROFILE_CONTROL_CYCLE);

    getProfiler().start(PROFILE_HQP_FAST);
    const HQPOutput & sol_fast = solver_fast->solve(HQPData);
    getProfiler().stop(PROFILE_HQP_FAST);

    getProfiler().start(PROFILE_HQP_RT);
    solver_rt->solve(HQPData);
    getProfiler().stop(PROFILE_HQP_RT);

    getStatistics().store("active inequalities",
			  static_cast<double>(sol_fast.activeSet.size()));
    getStatistics().store("solver iterations", sol_fast.iterations);

    dv = sol.x.head(nv);
    v += dt*dv;
    q = pinocchio::integrate(robot.model(), q, dt*v);
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
