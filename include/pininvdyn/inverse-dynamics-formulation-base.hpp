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

#ifndef __invdyn_inverse_dynamics_formulation_base_hpp__
#define __invdyn_inverse_dynamics_formulation_base_hpp__

#include <pininvdyn/math/utils.hpp>
#include <pininvdyn/robot-wrapper.hpp>
#include <pininvdyn/tasks/task-actuation.hpp>
#include <pininvdyn/tasks/task-motion.hpp>
#include <pininvdyn/tasks/task-contact-force.hpp>
#include <pininvdyn/contacts/contact-base.hpp>
#include <pininvdyn/solvers/solver-HQP-base.hpp>

#include <string>

namespace pininvdyn
{

  ///
  /// \brief Wrapper for a robot based on pinocchio
  ///
  class InverseDynamicsFormulationBase
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef se3::Data Data;
    typedef pininvdyn::math::ConstRefVector ConstRefVector;
    typedef pininvdyn::tasks::TaskMotion TaskMotion;
    typedef pininvdyn::tasks::TaskContactForce TaskContactForce;
    typedef pininvdyn::tasks::TaskActuation TaskActuation;
    typedef pininvdyn::contacts::ContactBase ContactBase;
    typedef pininvdyn::solvers::HqpData HqpData;


    InverseDynamicsFormulationBase(const std::string & name,
                                   RobotWrapper & robot,
                                   bool verbose=false);

    virtual const Data & data() const = 0;

    virtual unsigned int nVar() const = 0;
    virtual unsigned int nEq() const = 0;
    virtual unsigned int nIn() const = 0;

    virtual bool addMotionTask(TaskMotion & task,
                               double weight,
                               unsigned int priorityLevel,
                               double transition_duration=0.0) = 0;

    virtual bool addForceTask(TaskContactForce & task,
                              double weight,
                              unsigned int priorityLevel,
                              double transition_duration=0.0) = 0;

    virtual bool addTorqueTask(TaskActuation & task,
                               double weight,
                               unsigned int priorityLevel,
                               double transition_duration=0.0) = 0;

    virtual bool addRigidContact(ContactBase & contact) = 0;

    virtual bool removeTask(const std::string & taskName,
                            double transition_duration=0.0) = 0;

    virtual bool removeRigidContact(const std::string & contactName,
                                    double transition_duration=0.0) = 0;

    virtual const HqpData & computeProblemData(double time,
                                               ConstRefVector q,
                                               ConstRefVector v) = 0;

  protected:
    std::string m_name;
    RobotWrapper m_robot;
    bool m_verbose;
  };

}

#endif // ifndef __invdyn_inverse_dynamics_formulation_base_hpp__
