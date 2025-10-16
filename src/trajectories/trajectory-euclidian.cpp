//
// Copyright (c) 2017 CNRS
//

#include <tsid/trajectories/trajectory-euclidian.hpp>

namespace tsid {
namespace trajectories {

TrajectoryEuclidianConstant::TrajectoryEuclidianConstant(
    const std::string& name)
    : TrajectoryBase(name) {}

TrajectoryEuclidianConstant::TrajectoryEuclidianConstant(
    const std::string& name, ConstRefVector ref)
    : TrajectoryBase(name) {
  setReference(ref);
}

void TrajectoryEuclidianConstant::setReference(ConstRefVector ref) {
  m_sample.resize((unsigned int)ref.size());
  m_sample.setValue(ref);
}

unsigned int TrajectoryEuclidianConstant::size() const {
  return (unsigned int)m_sample.getValue().size();
}

const TrajectorySample& TrajectoryEuclidianConstant::operator()(double) {
  return m_sample;
}

const TrajectorySample& TrajectoryEuclidianConstant::computeNext() {
  return m_sample;
}

void TrajectoryEuclidianConstant::getLastSample(
    TrajectorySample& sample) const {
  sample = m_sample;
}

bool TrajectoryEuclidianConstant::has_trajectory_ended() const { return true; }

}  // namespace trajectories
}  // namespace tsid
