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
#include <tsid/tasks/task-convex-hull-inequality.hpp>
#include "tsid/math/utils.hpp"
#include "tsid/robots/robot-wrapper.hpp"


namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskConvexHullInequality::TaskConvexHullInequality(const std::string & name,
                                                       RobotWrapper & robot,
                                                       const std::vector<std::string> & links_in_contact,
                                                       const double safety_margin):
      TaskMotion(name, robot),
      m_constraint(name, 3, robot.nv()),
      m_nv(robot.nv()),
      m_links_in_contact(links_in_contact),
      m_safety_margin(safety_margin)
    {
      assert(links_in_contact.size() > 0);
     // std::cout << " CONSTRUCTOR CONVEX_HULL BEGIN "<< std::endl;
      m_Kp.setZero(3);
      m_Kd.setZero(3);
      m_p_error_vec.setZero(3);
      m_v_error_vec.setZero(3);
      m_p_com.setZero(3);
      m_v_com.setZero(3);
      m_a_des_vec.setZero(3);
      Vector zero;
      zero.setZero(3);
      for(int i=0; i < links_in_contact.size(); i++)
      {
        m_ch.push_back(zero);
        m_points.push_back(zero);
      }

      w_M_com.setIdentity();
      com_p_point.setZero(3);
      
      A.setZero(4,2);
      b.setZero(4);

      
    //  std::cout << " CONSTRUCTOR CONVEX_HULL END"<< std::endl;
    }


    int TaskConvexHullInequality::dim() const
    {
      //return self._mask.sum ()
      return 2;
    }
//
//     const Vector3 & TaskConvexHullInequality::Kp(){ return m_Kp; }
//
//     const Vector3 & TaskConvexHullInequality::Kd(){ return m_Kd; }
//
//     void TaskConvexHullInequality::Kp(ConstRefVector Kp)
//     {
//       assert(Kp.size()==3);
//       m_Kp = Kp;
//     }
//
//     void TaskConvexHullInequality::Kd(ConstRefVector Kd)
//     {
//       assert(Kd.size()==3);
//       m_Kd = Kd;
//     }
//
//     void TaskConvexHullInequality::setReference(const TrajectorySample & ref)
//     {
//       m_ref = ref;
//     }
//
//     const TrajectorySample & TaskConvexHullInequality::getReference() const
//     {
//       return m_ref;
//     }
//
//     const Vector & TaskConvexHullInequality::getDesiredAcceleration() const
//     {
//       return m_a_des_vec;
//     }

    Vector TaskConvexHullInequality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv - m_drift;
    }

//     const Vector & TaskConvexHullInequality::position_error() const
//     {
//       return m_p_error_vec;
//     }
//
//     const Vector & TaskConvexHullInequality::velocity_error() const
//     {
//       return m_v_error_vec;
//     }
//
    const Vector & TaskConvexHullInequality::position() const
    {
      return m_p_com;
    }
//
//     const Vector & TaskConvexHullInequality::velocity() const
//     {
//       return m_v_com;
//     }
//
//     const Vector & TaskConvexHullInequality::position_ref() const
//     {
//       return m_ref.pos;
//     }
//
//     const Vector & TaskConvexHullInequality::velocity_ref() const
//     {
//       return m_ref.vel;
//     }

    const ConstraintBase & TaskConvexHullInequality::getConstraint() const
    {
      return m_constraint;
    }

    bool TaskConvexHullInequality::getSupportPolygonPoints(const std::vector<std::string>  & links_in_contact,
                                                           const std::string & referenceFrame,
                                                           const Data & data)
    {
     // std::cout << "SUPPORT POLYGON BEGIN"<< std::endl;


     //  std::cout << " ---init ok"<< std::endl;


      if(referenceFrame != "COM" && referenceFrame != "world")
        std::cerr << "ERROR: "
                  << "trying to get support polygon points in unknown reference frame "
                  << std::endl;
      if(links_in_contact.empty() ||
         (referenceFrame != "COM" &&
         referenceFrame != "world" ))
         return false;
      w_M_com.setIdentity();
      vectorToSE3(m_p_com,w_M_com);
      
     // std::cout << " Transform world to CoM :  " << w_M_com << std::endl;
     //   std::cout << " Transform CoM to world (rotation) :" << w_M_com.rotation().setIdentity() << std::endl;

     // std::cout << " ---vectorToSE3 ok "<< std::endl;
      for(int i=0; i< links_in_contact.size(); i++)
      {
        if(referenceFrame == "COM" || referenceFrame == "world")
        {
          //get points in world frame
           
          assert(m_robot.model().existFrame(links_in_contact[i]));
          frame_id = m_robot.model().getFrameId(links_in_contact[i]);
          m_robot.framePosition(data, frame_id, w_M_point);
      //    std::cout << " Transform world to points[" <<i<<"]: " << w_M_point << std::endl;
        }
        if(referenceFrame == "COM")
        {
        
       // std::cout << " ---get Transform "<< std::endl;
        com_M_point =  w_M_com.inverse() * w_M_point;
      //  std::cout << " Transform CoM to points[" <<i<<"]: " << com_M_point << std::endl;
     //   std::cout << " ---SE3ToVector"<< std::endl;
        SE3ToVector(com_M_point,com_p_point);
        
        m_points[i]= com_M_point.translation();//com_p_point;
     //   std::cout << "points[" << i << "] : " << m_points[i] << std::endl;
        }
      }
     // std::cout << " SUPPORT POLYGON END"<< std::endl;
      return true;
    }

     bool TaskConvexHullInequality::getConvexHull(std::vector<Vector> & ch,
                                                 const Data & data,
                                                 std::string & frame
                                               )
    {
     // std::cout << " GET CONVEX HULL BEGIN"<< std::endl;
      std::vector<Vector> points;

      if(getSupportPolygonPoints(m_links_in_contact,frame, data))
      {
       //  std::cout << "Points size.. ?  : "<< m_points.size() <<  std::endl;
        if(m_points.size()>2)
        {
        //  std::cout << "COMPUTE CONVEX HULL ?"<< std::endl;
          // if(computeConvexHull(m_points, m_ch)) //reoder points to be trigonometry rotation
          // {
            m_ch = m_points;
      //      std::cout << " GET_CONVEX_HULL END"<< std::endl;
            return true;
          // }
          // else
          //   std::cout<<"Problems computing Convex Hull, old Convex Hull will be used"<<std::endl;
        }
        else
            std::cout<<"Too few points for Convex Hull computation!, old Convex Hull will be used"<<std::endl;
      }
      else
        std::cout<<"Problems getting Points for Convex Hull computation!, old Convex Hull will be used"<<std::endl;
      return false;
    }

    bool TaskConvexHullInequality::computeConvexHull(std::vector<Vector> & points, std::vector<Vector> & ch)
    {
      //TODO :reodering points to be a trigonometry rotation
      ch = points;
    //  std::cout << "COMPUTE CONVEX_HULL END"<< std::endl;
      return true;
    }

    void TaskConvexHullInequality::getLineCoefficients(const Vector & p0,
                                                       const Vector & p1,
                                                       double & a, double & b,
                                                       double & c)
    {
    //  std::cout << "GET LINE COEF BEGIN "<< std::endl;
      double x1 = p0[0];
      double x2 = p1[0];
      double y1 = p0[1];
      double y2 = p1[1];

      a = y1 - y2;
      b = x2 - x1;
      c = -b*y1 -a*x1;
     // std::cout << "a=" << a << " ; b="<< b << " ; c=" << c << std::endl;
     // std::cout << " GET LINE COEF END "<< std::endl;
    }

    void TaskConvexHullInequality::setSafetyMargin(const double safetyMargin)
    {
      m_safety_margin = safetyMargin;
    }

    std::vector<std::string> TaskConvexHullInequality::getLinksInContact() const
    {
      return m_links_in_contact;
    }

    void TaskConvexHullInequality::setLinksInContact(const std::vector<std::string> & links_in_contact)
    {
      m_links_in_contact = links_in_contact;
      m_constraint.resize(links_in_contact.size(),m_nv);
    }
    void TaskConvexHullInequality::getConstraints(const std::vector<Vector> & convex_hull, Matrix & A,
                                                  Vector & b, const double boundScaling)
    {
     // std::cout << " GET CONSTRAINT BEGIN "<< std::endl;
      double _a, _b, _c;
      // A.resize(convex_hull.size(),2);
      // b.resize(convex_hull.size());
    //  std::cout << "convex_hull size : " << convex_hull.size() << std::endl;
      unsigned int z = 0;

      for(unsigned int j = 0; j < convex_hull.size(); ++j)
      {
          unsigned int k = (j + 1)%convex_hull.size();
          getLineCoefficients(convex_hull[j], convex_hull[k], _a, _b, _c);

          //Where is the line w.r.t. the robot?
          //We consider that the constraint is feasable at the beginning (the robot is in the convex hull)
          if(_c <= 0.0) { // c < 0 --> AJdq < -c w/ -c > 0
              A(z,0) = + _a;
              A(z,1) = + _b;
              b(z) =   - _c;
          } else { // c > 0 --> -AJdq < c
              A(z,0) = - _a;
              A(z,1) = - _b;
              b(z) =   + _c;
          }

          double normalizedBoundScaling = boundScaling * sqrt(_a*_a + _b*_b); //boundScaling Normalization
          if(fabs(_c) <= normalizedBoundScaling)
              b(z) = 0.0;
          else
              b(z) -= normalizedBoundScaling;
          z++;
      }
    //  std::cout << " GET CONSTRAINT END"<< std::endl;
    }

    const ConstraintBase & TaskConvexHullInequality::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    const Data & data)
    {
     // std::cout << " COMPUTE BEGIN"<< std::endl;
      m_robot.com(data, m_p_com, m_v_com, m_drift);
    // std::cout << "CoM position : " << m_p_com << std::endl;
      // Get CoM jacobian
       //   std::cout << " Get ComJac : "<< std::endl;
      const Matrix3x & Jcom = m_robot.Jcom(data);

      
     // std::cout << Jcom << std::endl; 
          //  std::cout << " Get ComJac (X and Y): " << std::endl; 
          //  std::cout << Jcom.block(0,0,2,m_nv) << std::endl;
      std::string frame = "COM";
      if(getConvexHull(m_ch, data, frame))
        getConstraints(m_ch, A, b, m_safety_margin);
      // std::cout << "Matrix A : "<< std::endl; 
      // std::cout << A << std::endl; 
      //     std::cout << "Vector b : "<< std::endl; 
      // std::cout << b << std::endl;
      
      m_drift_vec=m_drift.head(2);
      m_constraint.upperBound() = (b-A*m_drift_vec);


      A = A* Jcom.block(0,0,2,m_nv);

      m_constraint.setMatrix(A);
      //m_constraint.lowerBound() = -100000*(b-A*m_drift_vec);
      A.setZero(4,2);
      b.setZero(4);

//std::cout << " COMPUTE END"<< std::endl;
      return m_constraint;
    }

  }
}
