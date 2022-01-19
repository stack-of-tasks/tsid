//
// Copyright (c) 2022 CNRS, Noelie Ramuzat
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

#ifndef __invdyn_task_energy_hpp__
#define __invdyn_task_energy_hpp__

#include "tsid/tasks/task-base.hpp"
#include "tsid/trajectories/trajectory-base.hpp"
#include "tsid/math/constraint-inequality.hpp"
#include "tsid/math/constraint-equality.hpp"
#include "tsid/formulations/inverse-dynamics-formulation-base.hpp"
#include "tsid/formulations/task-motion-level.hpp"
#include "tsid/formulations/contact-level.hpp"

#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>
#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/energy.hpp>

namespace tsid
{
  namespace tasks
  {
    // The energy task is used to transform the classical whole-body controller using inverse dynamics
    // of TSID into a passive scheme. 
    // The passivity is a stability criterion based on the power flow exchanged by the components of the system. 
    // A storage function H(x) is chosen (often as the energy of the system) and to have a passive system,
    // the system internal power (d_H: derivative of the energy in the system) must be lesser or equal to the power 
    // transferred to the system through its port: 
    // For a system controller+robot (controlled in torque) it is the velocity and the torque.
    // 
    // Our energy task adds an energy tank to the controller to monitor the energy flow of the system. 
    // This tank regulates the task gains using coefficients (alpha, beta and gamma in [0, 1])
    // to respect the passivity of the system. These coefficients are multiplied to the task vectors in the QP formulation.
    // Thus, if one of the coefficient is lowered to zero (when there is no more energy in the tank), the tasks are penalized
    // because have a decreased desired acceleration and even a null one.
    // Moreover, as the tank is computed without taking into account the constraints of the QP, a passivity constraint is
    // added in the QP formulation in addition to the regulating coefficient.
    // For more details see the RA-L paper "Passive Inverse Dynamics Control using a Global Energy Tank for Torque-Controlled 
    // Humanoid Robots in Multi-Contact", by Noelie Ramuzat, Sébastien BORIA, Olivier Stasse.
    class TaskEnergy : public TaskBase
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Matrix Matrix; 
      typedef math::ConstraintInequality ConstraintInequality;
      typedef math::ConstraintEquality ConstraintEquality;
      typedef pinocchio::Data Data;

      // Smoothing functions used on the alpha and gamma coefficient to avoid unstabilities
      double alphaFunction(const double a, const double b, const double x, const double e_val);
      double gammaFunction(const double A, const double P, const double delta);

      TaskEnergy(const std::string & name,
                 RobotWrapper & robot,
                 const double dt);

      int dim() const; // dimension of the constraint, equal to 1

      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     Data & data);

      const ConstraintBase & getConstraint() const;

      // Getter and Setter of the different vectors and variables of the problem 
      const Vector & get_A_vector() const;
      const double & get_lowerBound() const;
      const double & get_H() const;
      const double & get_dH() const;
      const double & get_E_tank() const;
      void set_E_tank(const double & E_tank);
      const double & get_dE_tank() const;
      const double & get_H_tot() const;
      const double & get_dH_tot() const;
      const Vector & get_S() const;
      const Vector & get_dS() const;
      const double & get_dt() const;
      const double & get_alpha() const;
      const double & get_beta() const;
      const double & get_gamma() const;
      const Matrix & get_Lambda() const;
      const bool & is_S_positive_definite() const; // test of the semi-positive definition of S
      void enable_test_positive_def(const bool & isEnabled); // function to activate the test of the semi-positive definition of S

      // Set the motion, contact and force tasks of the QP to compute the energy flow
      void setTasks(const std::vector<std::shared_ptr<TaskLevelMotion> >&  taskMotions, 
                    const std::vector<std::shared_ptr<ContactLevel> >&  taskContacts, 
                    const std::vector<std::shared_ptr<TaskLevelForce> >& taskForces, Data & data);

    protected:
      double m_dt;
      double m_alpha; // is the overflow valve, when = 1 the tank can be filled, otherwise it is already full
      double m_beta;  // controls the output flow of the tank, if = 0 the tank is empty (or at its lowest value)
      double m_gamma; // regulates the energy flow transferred from the tank to the controller, avoids sharp release of the tank energy to the system
      double m_Plow;  // empirical power threshold defining when the transfer should be regulated
      Vector m_A;     
      double m_H;     // The storage function H
      double m_dH;    // The derivative of H which must respect the passivity constraint: d_H <= -v^T tau
      double m_H_tot; // The storage function H augmented by kinetic energy of the tasks and the gravity potential
      double m_dH_tot;  // Its derivative
      double m_H_tot_prev;
      Vector m_S;     // The potential energy of the tasks
      Vector m_dS;    // Its derivative
      Vector m_S_prev;
      std::vector<Matrix> m_masked_Kp_prev; // Previous potential gains masked in function of the selected DoFs 
      Matrix m_Lambda;                      // The Cartesian space inertia matrix in function of the tasks
      int m_dim;      // The dimension of the constraint: 1
      double m_E_tank;                      // The energy in the tank
      double m_dE_tank;                     // Its derivative
      double m_E_max_tank;                  // The maximum value of the tank
      double m_E_min_tank;                  // The minimum value of the tank
      ConstraintInequality m_passivityConstraint; // the passivity constraint: d_H <= -v^T tau
      Vector m_b_lower;                           // the passivity constraint lower bound: dH
      Vector m_b_upper;                           // the passivity constraint upper bound: set to 1e10 * Vector::Ones(m_dim) (useless)
      std::vector<std::shared_ptr<TaskLevelMotion> > m_taskMotions; // The motion tasks in the QP used to compute the energy flow
      std::vector<std::shared_ptr<ContactLevel> >   m_taskContacts; // The contact tasks in the QP used to compute the energy flow
      std::vector<std::shared_ptr<TaskLevelForce> > m_taskForces;   // The force tasks in the QP used to compute the energy flow
      bool m_first_iter;
      bool m_test_semi_def_pos;             // boolean to activate the test of the semi-positive definition of S
      bool m_Lambda_Kp_pos_def;             // boolean resulting from the test of the semi-positive definition of S
    };

  }
}

#endif // ifndef __invdyn_task_energy_hpp__
