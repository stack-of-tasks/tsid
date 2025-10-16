//
// Copyright (c) 2017 CNRS
//

#include "tsid/solvers/solver-HQP-base.hpp"

#include <iostream>

namespace tsid {
namespace solvers {

std::string const SolverHQPBase::HQP_status_string[] = {
    "HQP_STATUS_OPTIMAL", "HQP_STATUS_INFEASIBLE", "HQP_STATUS_UNBOUNDED",
    "HQP_STATUS_MAX_ITER_REACHED", "HQP_STATUS_ERROR"};

SolverHQPBase::SolverHQPBase(const std::string& name) {
  m_name = name;
  m_maxIter = 1000;
  m_maxTime = 100.0;
  m_useWarmStart = true;
}

bool SolverHQPBase::setMaximumIterations(unsigned int maxIter) {
  if (maxIter == 0) return false;
  m_maxIter = maxIter;
  return true;
}

bool SolverHQPBase::setMaximumTime(double seconds) {
  if (seconds <= 0.0) return false;
  m_maxTime = seconds;
  return true;
}

}  // namespace solvers
}  // namespace tsid
