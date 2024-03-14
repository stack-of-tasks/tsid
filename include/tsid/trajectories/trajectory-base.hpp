//
// Copyright (c) 2017-2021 CNRS
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

#ifndef __invdyn_trajectory_base_hpp__
#define __invdyn_trajectory_base_hpp__

#include "tsid/deprecated.hh"
#include "tsid/macros.hpp"
#include "tsid/math/fwd.hpp"
#include "tsid/math/utils.hpp"

#include <string>

namespace tsid {
namespace trajectories {

typedef Eigen::Map<const Eigen::Matrix<double, 3, 3>> MapMatrix3;

class TrajectorySample {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  // TODO rename pos, vel, acc → value, derivative, second_derivative
  TSID_DEPRECATED math::Vector pos, vel, acc;

  TSID_DISABLE_WARNING_PUSH
  TSID_DISABLE_WARNING_DEPRECATED
  // getters / setters with updated names for math::Vector
  const math::Vector& getValue() const { return pos; }
  const math::Vector& getDerivative() const { return vel; }
  const math::Vector& getSecondDerivative() const { return acc; }
  void setValue(const math::Vector& value) { pos = value; }
  void setDerivative(const math::Vector& derivative) { vel = derivative; }
  void setSecondDerivative(const math::Vector& second_derivative) {
    acc = second_derivative;
  }

  TrajectorySample(unsigned int size = 0) { resize(size); }

  TrajectorySample(unsigned int size_value, unsigned int size_derivative) {
    resize(size_value, size_derivative);
  }

  void resize(unsigned int size) { resize(size, size); }

  void resize(unsigned int size_value, unsigned int size_derivative) {
    pos.setZero(size_value);
    vel.setZero(size_derivative);
    acc.setZero(size_derivative);
  }

  // declare default constructors / destructors to disable the deprecation
  // message for them. TODO: Remove this after the
  // pos/vel/acc → value/derivative/second_derivative rename
  ~TrajectorySample() = default;
  TrajectorySample(const TrajectorySample&) = default;
  TSID_DISABLE_WARNING_POP
};

class TrajectoryBase {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  TrajectoryBase(const std::string& name) : m_name(name) {}

  virtual ~TrajectoryBase() = default;

  virtual unsigned int size() const = 0;

  virtual const TrajectorySample& operator()(double time) = 0;

  virtual const TrajectorySample& computeNext() = 0;

  virtual const TrajectorySample& getLastSample() const { return m_sample; }

  virtual void getLastSample(TrajectorySample& sample) const = 0;

  virtual bool has_trajectory_ended() const = 0;

 protected:
  std::string m_name;
  TrajectorySample m_sample;
};
}  // namespace trajectories
}  // namespace tsid

#endif  // ifndef __invdyn_trajectory_base_hpp__
