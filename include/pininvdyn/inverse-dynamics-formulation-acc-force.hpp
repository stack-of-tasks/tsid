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

#ifndef __invdyn_inverse_dynamics_formulation_acc_force_hpp__
#define __invdyn_inverse_dynamics_formulation_acc_force_hpp__

#include <pininvdyn/inverse-dynamics-formulation-base.hpp>
#include <vector>

namespace pininvdyn
{

  class TaskLevel
  {
  public:
    pininvdyn::tasks::TaskBase & task;
    pininvdyn::math::ConstraintBase * constraint;
    double weight;
    unsigned int priority;

    TaskLevel(pininvdyn::tasks::TaskBase & task,
              double weight,
              unsigned int priority);
  };

  class ContactLevel
  {
  public:
    pininvdyn::contacts::ContactBase & contact;
    pininvdyn::math::ConstraintBase * motionConstraint;
    pininvdyn::math::ConstraintInequality * forceConstraint;
    pininvdyn::math::ConstraintEquality * forceRegTask;
    unsigned int index; /// index of 1st element of associated force variable in the force vector

    ContactLevel(pininvdyn::contacts::ContactBase & contact);
  };

  class InverseDynamicsFormulationAccForce:
      public InverseDynamicsFormulationBase
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef se3::Data Data;
    typedef pininvdyn::math::Vector Vector;
    typedef pininvdyn::math::Matrix Matrix;
    typedef pininvdyn::math::ConstRefVector ConstRefVector;
    typedef pininvdyn::tasks::TaskBase TaskBase;
    typedef pininvdyn::tasks::TaskMotion TaskMotion;
    typedef pininvdyn::tasks::TaskContactForce TaskContactForce;
    typedef pininvdyn::tasks::TaskActuation TaskActuation;
    typedef pininvdyn::solvers::HqpOutput HqpOutput;


    InverseDynamicsFormulationAccForce(const std::string & name,
                                       RobotWrapper & robot,
                                       bool verbose=false);

    const Data & data() const;

    unsigned int nVar() const;
    unsigned int nEq() const;
    unsigned int nIn() const;

    bool addMotionTask(TaskMotion & task,
                       double weight,
                       unsigned int priorityLevel,
                       double transition_duration=0.0);

    bool addForceTask(TaskContactForce & task,
                      double weight,
                      unsigned int priorityLevel,
                      double transition_duration=0.0);

    bool addTorqueTask(TaskActuation & task,
                       double weight,
                       unsigned int priorityLevel,
                       double transition_duration=0.0);

    bool addRigidContact(ContactBase & contact);

    bool removeTask(const std::string & taskName,
                    double transition_duration=0.0);

    bool removeRigidContact(const std::string & contactName,
                            double transition_duration=0.0);

    const HqpData & computeProblemData(double time,
                                       ConstRefVector q,
                                       ConstRefVector v);

    const Vector & computeActuatorForces(const HqpOutput & sol);

  public:

    void addTask(TaskLevel* task,
                 double weight,
                 unsigned int priorityLevel);

    void resizeHqpData();

    bool removeFromHqpData(const std::string & name);

    Data m_data;
    HqpData m_hqpData;
    std::vector<TaskLevel*>     m_taskMotions;
    std::vector<TaskLevel*>     m_taskContactForces;
    std::vector<TaskLevel*>     m_taskActuations;
    std::vector<ContactLevel*>   m_contacts;
    double m_t;         /// time
    unsigned int m_k;   /// number of contact-force variables
    unsigned int m_v;   /// number of acceleration variables
    unsigned int m_eq;  /// number of equality constraints
    unsigned int m_in;  /// number of inequality constraints
    Matrix m_Jc;        /// contact force Jacobian
    pininvdyn::math::ConstraintEquality m_baseDynamics;

    Vector m_dv;
    Vector m_f;
    Vector m_tau;
  };

}

#endif // ifndef __invdyn_inverse_dynamics_formulation_acc_force_hpp__
