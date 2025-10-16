//
// Copyright (c) 2022 CNRS INRIA LORIA
//

#ifndef __invdyn_measured_force_base_hpp__
#define __invdyn_measured_force_base_hpp__

#include <pinocchio/multibody/fwd.hpp>

#include "tsid/math/fwd.hpp"
#include "tsid/robots/fwd.hpp"

namespace tsid {
namespace contacts {
class MeasuredForceBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  typedef math::Vector Vector;
  typedef pinocchio::Data Data;
  typedef robots::RobotWrapper RobotWrapper;

  MeasuredForceBase(const std::string& name, RobotWrapper& robot);

  virtual ~MeasuredForceBase() = default;

  const std::string& name() const;

  void name(const std::string& name);

  /**
   * Compute the bias force (J^t F) associated to the
   * external measured force.
   */
  virtual const Vector& computeJointTorques(Data& data) = 0;

 protected:
  std::string m_name;

  /// \brief Reference on the robot model.
  RobotWrapper& m_robot;
};
}  // namespace contacts
}  // namespace tsid

#endif  // ifndef __invdyn_measured_force_base_hpp__
