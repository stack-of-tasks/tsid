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

#ifndef __invdyn_task_convex_hull_inequality_hpp__
#define __invdyn_task_convex_hull_inequality_hpp__

#include <tsid/tasks/task-motion.hpp>
#include <tsid/trajectories/trajectory-base.hpp>
#include <tsid/math/constraint-inequality.hpp>
#include <vector>
#include <pinocchio/multibody/model.hpp>
#include <pinocchio/multibody/data.hpp>

namespace tsid
{
  namespace tasks
  {

    class TaskConvexHullInequality : public TaskMotion
    {
    public:
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW

      typedef math::Index Index;
      // typedef trajectories::TrajectorySample TrajectorySample;
      typedef math::Vector Vector;
      typedef math::Matrix Matrix;
      typedef math::Vector3 Vector3;
      typedef math::ConstraintInequality ConstraintInequality;
      typedef pinocchio::Data Data;
      // typedef pinocchio::Motion Motion;
      typedef pinocchio::SE3 SE3;

      TaskConvexHullInequality(const std::string & name,
                               RobotWrapper & robot,
                               const std::vector<std::string> & links_in_contact,
                               const double safety_margin = 0.01 );

      int dim() const;
      //
      const ConstraintBase & compute(const double t,
                                     ConstRefVector q,
                                     ConstRefVector v,
                                     const Data & data);

      const ConstraintBase & getConstraint() const;

      // void setReference(TrajectorySample & ref);
      // const TrajectorySample & getReference() const;
      //
      // /** Return the desired task acceleration (after applying the specified mask).
      //  *  The value is expressed in local frame is the local_frame flag is true,
      //  *  otherwise it is expressed in a local world-oriented frame.
      // */
      // const Vector & getDesiredAcceleration() const;

      /** Return the task acceleration (after applying the specified mask).
       *  The value is expressed in local frame is the local_frame flag is true,
       *  otherwise it is expressed in a local world-oriented frame.
      */
      Vector getAcceleration(ConstRefVector dv) const;

      // const Vector & position_error() const;
      // const Vector & velocity_error() const;
      const Vector & position() const;
      // const Vector & velocity() const;
      // const Vector & position_ref() const;
      // const Vector & velocity_ref() const;
      //
      // const Vector3 & Kp();
      // const Vector3 & Kd();
      // void Kp(ConstRefVector Kp);
      // void Kd(ConstRefVector Kp);
      //
      bool getSupportPolygonPoints(const std::vector<std::string> & links_in_contact,
                                   const std::string & referenceFrame,
                                   const Data & data);


      bool computeConvexHull(std::vector<Vector> & points, std::vector<Vector> & ch);

      bool getConvexHull(std::vector<Vector> & ch, const Data & data, std::string & fame);

      void getLineCoefficients(const Vector & p0, const Vector & p1,
                               double & a, double & b, double & c);

      void setSafetyMargin(const double safetyMargin);

      std::vector<std::string> getLinksInContact() const;

      void setLinksInContact(const std::vector<std::string> & links_in_contact);

      void getConstraints(const std::vector<Vector> & convex_hull, Matrix & A, Vector & b,
                          const double safety_margin);
    protected:

      Vector3 m_Kp;
      Vector3 m_Kd;
      Vector3 m_p_error, m_v_error;
      Vector3 m_a_des;
      Vector m_a_des_vec;
      Vector m_drift_vec;
      Vector3 m_drift;
      Vector m_p_com, m_v_com;
      Vector m_p_error_vec, m_v_error_vec;
      TrajectorySample m_ref;
      ConstraintInequality m_constraint;
      SE3 m_M_com;

      std::vector<Vector> m_ch;
      std::vector<Vector> m_points;
      std::vector<std::string> m_links_in_contact;
      double m_safety_margin;
      double m_nv;

      SE3 com_M_point;
      SE3 w_M_point;
      SE3 w_M_com;
      Index frame_id;
      Vector com_p_point;

      Matrix A;

      Vector b;
    };

  }
}

#endif //ifndef __invdyn_task_convex_hull_inequality_hpp__
