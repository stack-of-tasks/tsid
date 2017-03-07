//
// Copyright (c) 2017 CNRS
//
// This file is part of PinInvDyn
// PinInvDyn is free software: you can redistribute it
// and/or modify it under the terms of the GNU Lesser General Public
// License as published by the Free Software Foundation, either version
// 3 of the License, or (at your option) any later version.
// PinInvDyn is distributed in the hope that it will be
// useful, but WITHOUT ANY WARRANTY; without even the implied warranty
// of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
// General Lesser Public License for more details. You should have
// received a copy of the GNU Lesser General Public License along with
// PinInvDyn If not, see
// <http://www.gnu.org/licenses/>.
//

#include <pininvdyn/solvers/solver-HQP-eiquadprog.hpp>

using namespace pininvdyn::math;
using namespace pininvdyn::solvers;

Solver_HQP_eiquadprog::Solver_HQP_eiquadprog(const std::string & name):
  Solver_HQP_base(name)
{

}

HqpOutput Solver_HQP_eiquadprog::solve(const HqpData & problemData,
                                       RefVector sol)
{
  return m_output;
}

double Solver_HQP_eiquadprog::getObjectiveValue()
{
  return 0.0;
}
