//
// Copyright (c) 2018 CNRS
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

#include <eigenpy/eigenpy.hpp>
#include <eigenpy/geometry.hpp>

#include "tsid/bindings/python/robots/expose-robots.hpp"
#include "tsid/bindings/python/constraint/expose-constraints.hpp"
#include "tsid/bindings/python/contacts/expose-contact.hpp"
#include "tsid/bindings/python/trajectories/expose-trajectories.hpp"
#include "tsid/bindings/python/tasks/expose-tasks.hpp"
#include "tsid/bindings/python/solvers/expose-solvers.hpp"
#include "tsid/bindings/python/formulations/expose-formulations.hpp"

#include <boost/python/module.hpp>
#include <boost/python/def.hpp>
#include <boost/python/tuple.hpp>
#include <boost/python/to_python_converter.hpp>

namespace bp = boost::python;
using namespace tsid::python;

BOOST_PYTHON_MODULE(libtsid_pywrap)
{
  eigenpy::enableEigenPy();
  eigenpy::exposeAngleAxis();
  eigenpy::exposeQuaternion();

  typedef Eigen::Matrix<double,6,6> Matrix6d;
  typedef Eigen::Matrix<double,6,1> Vector6d;
  typedef Eigen::Matrix<double,6,Eigen::Dynamic> Matrix6x;
  typedef Eigen::Matrix<double,3,Eigen::Dynamic> Matrix3x;
  
  eigenpy::enableEigenPySpecific<Matrix6d>();
  eigenpy::enableEigenPySpecific<Vector6d>();
  eigenpy::enableEigenPySpecific<Matrix6x>();
  eigenpy::enableEigenPySpecific<Matrix3x>();
  eigenpy::enableEigenPySpecific<Eigen::MatrixXd>();
  eigenpy::enableEigenPySpecific<Eigen::Vector3d>();
  
  exposeRobots();  
  exposeConstraints();
  exposeContact();
  exposeTrajectories();
  exposeTasks();
  exposeSolvers();
  exposeFormulations();

}
 
