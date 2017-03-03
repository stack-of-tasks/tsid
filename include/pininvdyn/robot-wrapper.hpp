//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// pinocchio is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// pinocchio is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// pinocchio If not, see
// <http://www.gnu.org/licenses/>.
//

#ifndef __invdyn_tasks_task_base_hpp__
#define __invdyn_tasks_task_base_hpp__

#include "pinocchio/multibody/model.hpp"
#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/center-of-mass.hpp"
#include "pinocchio/algorithm/compute-all-terms.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/spatial/skew.hpp"
#include <pininvdyn/math/utils.hpp>

#include <string>

namespace pininvdyn
{

  ///
  /// \brief Wrapper for a robot based on pinocchio
  ///
  class RobotWrapper
  {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    typedef double Scalar;
    typedef se3::Model Model;
    typedef se3::Data Data;
    typedef se3::Motion Motion;
    typedef se3::Frame Frame;
    typedef se3::SE3 SE3;
    typedef Eigen::VectorXd Vector;
    typedef Eigen::Matrix<Scalar,3,1> Vector3;
    typedef Eigen::Matrix<Scalar,6,1> Vector6;
    typedef Eigen::MatrixXd Matrix;
    typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
    typedef pininvdyn::math::RefVector RefVector;


    RobotWrapper(const std::string & filename,
                 const std::vector<std::string> & package_dirs,
                 bool verbose=false);

    RobotWrapper(const std::string & filename,
                 const std::vector<std::string> & package_dirs,
                 const se3::JointModelVariant & rootJoint,
                 bool verbose=false);

    virtual int nq() const;
    virtual int nv() const;

    ///
    /// \brief Accessor to model.
    ///
    /// \returns a const reference on the model.
    ///
    const Model & model() const;

    void computeAllTerms(Data & data, const Vector & q, const Vector & v) const;

    const void com(const Data & data,
                   RefVector com_pos,
                   RefVector com_vel,
                   RefVector com_acc) const;

    const Vector3 & com(const Data & data) const;

    const Vector3 & com_vel(const Data & data) const;

    const Vector3 & com_acc(const Data & data) const;

    const Matrix3x & Jcom(const Data & data) const;

    const Matrix & mass(const Data & data) const;

    const Vector & nonLinearEffects(const Data & data) const;

    const SE3 & position(const Data & data,
                         const Model::JointIndex index) const;

    const Motion & velocity(const Data & data,
                            const Model::JointIndex index) const;

    const Motion & acceleration(const Data & data,
                                const Model::JointIndex index) const;

    void jacobianWorld(const Data & data,
                       const Model::JointIndex index,
                       Data::Matrix6x & J) const;

    void jacobianLocal(const Data & data,
                       const Model::JointIndex index,
                       Data::Matrix6x & J) const;

    SE3 framePosition(const Data & data,
                      const Model::FrameIndex index) const;

    void framePosition(const Data & data,
                       const Model::FrameIndex index,
                       SE3 & framePosition) const;

    Motion frameVelocity(const Data & data,
                         const Model::FrameIndex index) const;

    void frameVelocity(const Data & data,
                       const Model::FrameIndex index,
                       Motion & frameVelocity) const;

    Motion frameAcceleration(const Data & data,
                             const Model::FrameIndex index) const;

    void frameAcceleration(const Data & data,
                           const Model::FrameIndex index,
                           Motion & frameAcceleration) const;

    Motion frameClassicAcceleration(const Data & data,
                                    const Model::FrameIndex index) const;

    void frameClassicAcceleration(const Data & data,
                                  const Model::FrameIndex index,
                                  Motion & frameAcceleration) const;

    void frameJacobianWorld(const Data & data,
                            const Model::FrameIndex index,
                            Data::Matrix6x & J) const;

    void frameJacobianLocal(const Data & data,
                            const Model::FrameIndex index,
                            Data::Matrix6x & J) const;


  protected:


    /// \brief Robot model.
    Model m_model;
    std::string m_model_filename;
    bool m_verbose;
  };

}

#endif // ifndef __invdyn_tasks_task_base_hpp__
