//
// Copyright (c) 2025 CNRS INRIA LORIA
//

#ifndef __invdyn_measured_3d_force_hpp__
#define __invdyn_measured_3d_force_hpp__

#include <pinocchio/multibody/data.hpp>

#include "tsid/contacts/measured-force-base.hpp"

namespace tsid {
namespace contacts {
class Measured3Dforce : public MeasuredForceBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Index Index;
  typedef math::Vector3 Vector3;
  typedef robots::RobotWrapper RobotWrapper;
  typedef pinocchio::Data Data;
  typedef pinocchio::Data::Matrix3x Matrix3x;

  Measured3Dforce(const std::string& name, RobotWrapper& robot,
                  const std::string& frameName);

  const Vector& computeJointTorques(Data& data) override;

  /**
   *  Set the value of the external wrench applied by the environment on the
   * robot.
   */
  void setMeasuredContactForce(const Vector3& fext);

  const Vector3& getMeasuredContactForce() const;

  /**
   * @brief Specifies if the external force and jacobian is
   * expressed in the local frame or the local world-oriented frame.
   *
   * @param local_frame If true, represent external force and jacobian in the
   *   local frame. If false, represent them in the local world-oriented frame.
   */
  void useLocalFrame(bool local_frame);

 protected:
  std::string m_frame_name;
  Index m_frame_id;
  Vector3 m_fext;
  Matrix3x m_J;
  Matrix3x m_J_rotated;
  Vector m_computedTorques;
  bool m_local_frame;
};
}  // namespace contacts
}  // namespace tsid

#endif  // ifndef __invdyn_measured_3d_force_hpp__
