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
#include <tsid/tasks/task-capture-point-inequality.hpp>
#include "tsid/math/utils.hpp"
#include "tsid/robots/robot-wrapper.hpp"


namespace tsid
{
  namespace tasks
  {
    using namespace math;
    using namespace trajectories;
    using namespace pinocchio;

    TaskCapturePointInequality::TaskCapturePointInequality(const std::string & name,
                                                       RobotWrapper & robot,
                                                       const std::vector<std::string> & links_in_contact,
                                                       const double safety_margin,
                                                       const double timeStep):
      TaskMotion(name, robot),
      m_constraint(name, 4, robot.nv()),
      m_nv(robot.nv()),
      m_links_in_contact(links_in_contact),
      m_safety_margin(safety_margin),
      m_delta_t(timeStep)
    {
      assert(links_in_contact.size() > 0);
     // std::cout << " CONSTRUCTOR CONVEX_HULL BEGIN "<< std::endl;
      m_p_com.setZero(3);
      m_v_com.setZero(3);

      m_rp_max.setZero(3);
      m_rp_min.setZero(3);

      Vector zero;
      zero.setZero(3);
      for(int i=0; i < links_in_contact.size(); i++)
      {
        m_ch.push_back(zero);
        m_points.push_back(zero);
      }

      w_M_com.setIdentity();
      com_M_point.setIdentity();
      w_M_point.setIdentity();
      com_p_point.setZero(3);
      
      A.setZero(4,2);
      b.setZero(4);
      b_min.setZero(2);
      b_max.setZero(2);
      
    //  std::cout << " CONSTRUCTOR CONVEX_HULL END"<< std::endl;
    }


    int TaskCapturePointInequality::dim() const
    {
      //return self._mask.sum ()
      return 2;
    }

    Vector TaskCapturePointInequality::getAcceleration(ConstRefVector dv) const
    {
      return m_constraint.matrix()*dv - m_drift;
    }

    const Vector & TaskCapturePointInequality::position() const
    {
      return m_p_com;
    }
    const ConstraintBase & TaskCapturePointInequality::getConstraint() const
    {
      return m_constraint;
    }

    bool TaskCapturePointInequality::getSupportPolygonPoints(const std::vector<std::string>  & links_in_contact,
                                                           const std::string & referenceFrame,
                                                           const Data & data)
    {
      if(referenceFrame != "COM" && referenceFrame != "world")
        std::cerr << "ERROR: "
                  << "trying to get support polygon points in unknown reference frame "
                  << std::endl;
      if(links_in_contact.empty() ||
         (referenceFrame != "COM" &&
         referenceFrame != "world" ))
         return false;
      w_M_com.setIdentity();
      w_M_com.translation(m_p_com.head<3>());
     
      for(int i=0; i< links_in_contact.size(); i++)
      {
        if(referenceFrame == "world")
        {
          //get points in world frame
          assert(m_robot.model().existFrame(links_in_contact[i]));
          frame_id = m_robot.model().getFrameId(links_in_contact[i]);
          m_robot.framePosition(data, frame_id, w_M_point);

          m_points[i]= w_M_point.translation();
          //std::cout << " Transform world to points[" <<i<<"]: " << w_M_point << std::endl;
        }
        if(referenceFrame == "COM")
        {
        
        com_M_point =  w_M_com.inverse() * w_M_point;
        
        m_points[i]= com_M_point.translation();//com_p_point;
        // std::cout << "points[" << i << "] : " << m_points[i] << std::endl;
        }
      }
      
      return true;
    }

     bool TaskCapturePointInequality::getCapturePoint(std::vector<Vector> & ch,
                                                 const Data & data,
                                                 std::string & frame
                                               )
    {

      if(getSupportPolygonPoints(m_links_in_contact,frame, data))
      {
       //  std::cout << "Points size.. ?  : "<< m_points.size() <<  std::endl;
        if(m_points.size()>2)
        {
            return true;
        }
        else
            std::cout<<"Too few points for Convex Hull computation!, old Convex Hull will be used"<<std::endl;
      }
      else
        std::cout<<"Problems getting Points for Convex Hull computation!, old Convex Hull will be used"<<std::endl;
      return false;
    }

    bool TaskCapturePointInequality::computeCapturePoint(std::vector<Vector> & points, std::vector<Vector> & ch)
    {
      //TODO :reodering points to be a trigonometry rotation
      ch = points;
    //  std::cout << "COMPUTE CONVEX_HULL END"<< std::endl;
      return true;
    }

    void TaskCapturePointInequality::setSafetyMargin(const double safetyMargin)
    {
      m_safety_margin = safetyMargin;
    }

    std::vector<std::string> TaskCapturePointInequality::getLinksInContact() const
    {
      return m_links_in_contact;
    }

    void TaskCapturePointInequality::setLinksInContact(const std::vector<std::string> & links_in_contact)
    {
      m_links_in_contact = links_in_contact;
      m_constraint.resize(links_in_contact.size(),m_nv);
    }
    void TaskCapturePointInequality::getConstraints(const std::vector<Vector> & convex_hull, Matrix & A,
                                                  Vector & b, const double boundScaling, const double timeStep)
    {
    double m_g = 9.81;
    double w = sqrt(m_g/m_p_com(2));

    m_delta_t = timeStep;

    double Ka = (2*w)/((w*m_delta_t+2)*m_delta_t);
  //  std::cout << "Ka = " << Ka << std::endl;

    m_rp_min(0) = -0.127 +0.04; // x min support polygon
    m_rp_min(1) = -0.135 +0.04; // y min support polygon

    m_rp_max(0) = 0.07 -0.04;  // x max support polygon
    m_rp_max(1) = 0.135 -0.04; // y max support polygon

    b_min(0) = Ka*(m_rp_min(0) - m_p_com(0) - m_v_com(0)*(m_delta_t+ 1/w)); // x axe
    b_min(1) = Ka*(m_rp_min(1) - m_p_com(1) - m_v_com(1)*(m_delta_t+ 1/w)); // y axe

    b_max(0) = Ka*(m_rp_max(0) - m_p_com(0) - m_v_com(0)*(m_delta_t+ 1/w)); // x axe
    b_max(1) = Ka*(m_rp_max(1) - m_p_com(1) - m_v_com(1)*(m_delta_t+ 1/w)); // y axe

    // std::cout << "b_max(0) = " << b_max(0) << std::endl;
    // std::cout << "b_max(1) = " << b_max(1) << std::endl;

    // std::cout << "b_min(0) = " << b_min(0) << std::endl;
    // std::cout << "b_min(1) = " << b_min(1) << std::endl;
    
    //  std::cout << " GET CONSTRAINT END"<< std::endl;
    }

    const ConstraintBase & TaskCapturePointInequality::compute(const double t,
                                                    ConstRefVector q,
                                                    ConstRefVector v,
                                                    const Data & data)
    {
     // std::cout << " COMPUTE BEGIN"<< std::endl;
      m_robot.com(data, m_p_com, m_v_com, m_drift);

      const Matrix3x & Jcom = m_robot.Jcom(data);
      
      std::string frame = "COM";
      if(getCapturePoint(m_ch, data, frame))
        getConstraints(m_ch, A, b, m_safety_margin, m_delta_t);
 
       
      m_drift_vec=m_drift.head(2);


      m_constraint.upperBound() =  b_max - m_drift_vec;
     // std::cout << "b_max "<< b_max - m_drift_vec << std::endl;

      m_constraint.lowerBound() = b_min - m_drift_vec;
      // std::cout << "b_min - drift = "<< b_min - m_drift_vec << std::endl;
      
      //A = A* Jcom.block(0,0,2,m_nv);

    // double m_g = 9.81;
    // double w = sqrt(m_g/m_p_com(2));

    // double Ka = 0.5*0.1*0.1 + 0.1/w;
    // Matrix Ka_v;
    // Ka_v.setZero(2,2);
    // Ka_v(0,0) = Ka;
    // Ka_v(1,1)= Ka;
    // std::cout << "Jcom size: "<< std::endl;
    //   std::cout << Jcom.size() << std::endl; 
    //      std::cout << "m_nv size: "<< std::endl;
    //   std::cout << m_nv << std::endl; 

    // Matrix F;
    // F.setZero(2,36);
    // F = Ka_v*Jcom.block(0,0,2,m_nv);
      m_constraint.setMatrix(Jcom.block(0,0,2,m_nv));
      //m_constraint.lowerBound() = -100000*(b-A*m_drift_vec);
      A.setZero(4,2);
      b.setZero(4);
      b_min.setZero(2);
      b_max.setZero(2);

//std::cout << " COMPUTE END"<< std::endl;
      return m_constraint;
    }

  }
}
