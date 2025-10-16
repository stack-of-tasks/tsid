//
// Copyright (c) 2017-2021 CNRS
//

#include "tsid/math/utils.hpp"
#include "tsid/trajectories/trajectory-se3.hpp"

using namespace tsid::math;

namespace tsid {
namespace trajectories {

TrajectorySE3Constant::TrajectorySE3Constant(const std::string& name)
    : TrajectoryBase(name) {
  m_sample.resize(12, 6);
}

TrajectorySE3Constant::TrajectorySE3Constant(const std::string& name,
                                             const SE3& M)
    : TrajectoryBase(name) {
  m_sample.resize(12, 6);
  TSID_DISABLE_WARNING_PUSH
  TSID_DISABLE_WARNING_DEPRECATED
  tsid::math::SE3ToVector(M, m_sample.pos);
  TSID_DISABLE_WARNING_POP
}

unsigned int TrajectorySE3Constant::size() const { return 6; }

void TrajectorySE3Constant::setReference(const pinocchio::SE3& ref) {
  m_sample.resize(12, 6);
  TSID_DISABLE_WARNING_PUSH
  TSID_DISABLE_WARNING_DEPRECATED
  tsid::math::SE3ToVector(ref, m_sample.pos);
  TSID_DISABLE_WARNING_POP
}

const TrajectorySample& TrajectorySE3Constant::operator()(double) {
  return m_sample;
}

const TrajectorySample& TrajectorySE3Constant::computeNext() {
  return m_sample;
}

void TrajectorySE3Constant::getLastSample(TrajectorySample& sample) const {
  sample = m_sample;
}

bool TrajectorySE3Constant::has_trajectory_ended() const { return true; }

}  // namespace trajectories
}  // namespace tsid
