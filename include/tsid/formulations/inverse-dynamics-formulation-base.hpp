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

#ifndef __invdyn_inverse_dynamics_formulation_base_hpp__
#define __invdyn_inverse_dynamics_formulation_base_hpp__

#include "tsid/math/fwd.hpp"
#include "tsid/robots/robot-wrapper.hpp"
#include "tsid/tasks/task-actuation.hpp"
#include "tsid/tasks/task-motion.hpp"
#include "tsid/tasks/task-contact-force.hpp"
#include "tsid/contacts/contact-base.hpp"
#include "tsid/solvers/solver-HQP-base.hpp"

#include <string>

namespace tsid
{

  ///
  /// \brief Wrapper for a robot based on pinocchio
  ///
  class InverseDynamicsFormulationBase
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef se3::Data Data;
    typedef math::Vector Vector;
    typedef math::RefVector RefVector;
    typedef math::ConstRefVector ConstRefVector;
    typedef tasks::TaskMotion TaskMotion;
    typedef tasks::TaskContactForce TaskContactForce;
    typedef tasks::TaskActuation TaskActuation;
    typedef tasks::TaskBase TaskBase;
    typedef contacts::ContactBase ContactBase;
    typedef solvers::HQPData HQPData;
    typedef solvers::HQPOutput HQPOutput;
    typedef robots::RobotWrapper RobotWrapper;


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

    virtual bool updateTaskWeight(const std::string & task_name,
                                  double weight) = 0;

    virtual bool addRigidContact(ContactBase & contact) = 0;

    virtual bool removeTask(const std::string & taskName,
                            double transition_duration=0.0) = 0;

    virtual bool removeRigidContact(const std::string & contactName,
                                    double transition_duration=0.0) = 0;

    virtual const HQPData & computeProblemData(double time,
                                               ConstRefVector q,
                                               ConstRefVector v) = 0;

    virtual const Vector & getActuatorForces(const HQPOutput & sol) = 0;
    virtual const Vector & getAccelerations(const HQPOutput & sol) = 0;
    virtual const Vector & getContactForces(const HQPOutput & sol) = 0;
    virtual bool getContactForces(const std::string & name,
                                  const HQPOutput & sol,
                                  RefVector f) = 0;

  protected:
    std::string m_name;
    RobotWrapper m_robot;
    bool m_verbose;
  };

}

#endif // ifndef __invdyn_inverse_dynamics_formulation_base_hpp__
